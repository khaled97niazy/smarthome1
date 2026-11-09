#include "cam_net.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstring>

namespace {

constexpr unsigned long HUB_CONNECT_GRACE_MS = 12000;
constexpr unsigned long STA_RETRY_INTERVAL_MS = 10000;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 5000;

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

SmartHomeNet::CamSource s_source = SmartHomeNet::CAM_SOURCE_FACE;
SemaphoreHandle_t s_netMutex = nullptr;

bool s_espNowReady = false;
bool s_hubPeerAdded = false;
uint8_t s_hubBssid[6] = {0};

bool s_fallbackApActive = false;
unsigned long s_bootMs = 0;
unsigned long s_lastStaAttemptMs = 0;
unsigned long s_lastHeartbeatMs = 0;

volatile bool s_lastSendOk = false;
uint32_t s_seq = 0;
uint32_t s_sentCount = 0;

void onEspNowSent(const uint8_t * /*mac_addr*/, esp_now_send_status_t status) {
  s_lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

bool isNonZeroMac(const uint8_t *mac) {
  if (!mac) return false;
  for (int i = 0; i < 6; ++i) {
    if (mac[i] != 0) return true;
  }
  return false;
}

bool ensurePeer(const uint8_t *mac, wifi_interface_t ifidx) {
  if (!s_espNowReady || !mac) return false;
  if (esp_now_is_peer_exist(mac)) {
    return true;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0; // Use current home channel
  peer.ifidx = ifidx;
  peer.encrypt = false;
  esp_err_t err = esp_now_add_peer(&peer);
  return err == ESP_OK;
}

void syncHubPeerLocked() {
  if (!s_espNowReady) return;
  if (WiFi.status() != WL_CONNECTED) return;

  const uint8_t *bssid = WiFi.BSSID();
  if (!isNonZeroMac(bssid)) return;

  if (!s_hubPeerAdded || memcmp(s_hubBssid, bssid, 6) != 0) {
    if (s_hubPeerAdded && esp_now_is_peer_exist(s_hubBssid)) {
      esp_now_del_peer(s_hubBssid);
    }
    memcpy(s_hubBssid, bssid, 6);
    s_hubPeerAdded = ensurePeer(s_hubBssid, WIFI_IF_STA);
    if (s_hubPeerAdded) {
      Serial.printf("ESP-NOW Hub peer registered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    s_hubBssid[0], s_hubBssid[1], s_hubBssid[2],
                    s_hubBssid[3], s_hubBssid[4], s_hubBssid[5]);
    }
  }
}

void startFallbackApLocked() {
  if (s_fallbackApActive) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);

  IPAddress apIp = (s_source == SmartHomeNet::CAM_SOURCE_FACE)
                       ? IPAddress(192, 168, 11, 1)
                       : IPAddress(192, 168, 12, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, apIp, subnet);

  String apSsid = camNetFallbackApSsid();
  if (WiFi.softAP(apSsid.c_str(), SmartHomeNet::AP_PASS, SmartHomeNet::WIFI_CHANNEL, 0, 2)) {
    s_fallbackApActive = true;
    Serial.print("Fallback setup AP active: ");
    Serial.print(apSsid);
    Serial.print(" at http://");
    Serial.println(WiFi.softAPIP());
  }
}

void stopFallbackApLocked() {
  if (!s_fallbackApActive) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  s_fallbackApActive = false;
  Serial.println("Main Hub reachable; fallback setup AP disabled.");
}

} // namespace

void camNetInit(SmartHomeNet::CamSource source) {
  s_source = source;
  s_bootMs = millis();
  if (!s_netMutex) {
    s_netMutex = xSemaphoreCreateMutex();
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  const uint8_t *ipArr = (source == SmartHomeNet::CAM_SOURCE_FACE)
                             ? SmartHomeNet::FACE_CAM_IP
                             : SmartHomeNet::PLATE_CAM_IP;
  IPAddress localIp(ipArr[0], ipArr[1], ipArr[2], ipArr[3]);
  IPAddress hubIp(SmartHomeNet::HUB_IP[0], SmartHomeNet::HUB_IP[1],
                  SmartHomeNet::HUB_IP[2], SmartHomeNet::HUB_IP[3]);
  IPAddress subnet(SmartHomeNet::SUBNET_MASK[0], SmartHomeNet::SUBNET_MASK[1],
                   SmartHomeNet::SUBNET_MASK[2], SmartHomeNet::SUBNET_MASK[3]);
  WiFi.config(localIp, hubIp, subnet);

  WiFi.begin(SmartHomeNet::AP_SSID, SmartHomeNet::AP_PASS, SmartHomeNet::WIFI_CHANNEL);
  s_lastStaAttemptMs = s_bootMs;

  if (esp_now_init() == ESP_OK) {
    s_espNowReady = true;
    esp_now_register_send_cb(onEspNowSent);
    ensurePeer(BROADCAST_MAC, WIFI_IF_STA);
    Serial.println("ESP-NOW initialized on STA interface.");
  } else {
    s_espNowReady = false;
    Serial.println("ERROR: esp_now_init failed!");
  }
}

void camNetLoop() {
  unsigned long now = millis();
  if (s_netMutex && xSemaphoreTake(s_netMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }

  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected) {
    stopFallbackApLocked();
    syncHubPeerLocked();
  } else {
    if (!s_fallbackApActive && (now - s_bootMs >= HUB_CONNECT_GRACE_MS)) {
      startFallbackApLocked();
    }
    if (now - s_lastStaAttemptMs >= STA_RETRY_INTERVAL_MS) {
      s_lastStaAttemptMs = now;
      WiFi.begin(SmartHomeNet::AP_SSID, SmartHomeNet::AP_PASS, SmartHomeNet::WIFI_CHANNEL);
    }
  }

  if (s_netMutex) {
    xSemaphoreGive(s_netMutex);
  }

  if (now - s_lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    s_lastHeartbeatMs = now;
    camNetSendEvent(SmartHomeNet::CAM_EVENT_HEARTBEAT, 0, 100,
                    s_source == SmartHomeNet::CAM_SOURCE_FACE ? "FACE-CAM" : "PLATE-CAM");
  }
}

bool camNetSendEvent(SmartHomeNet::CamEventType event,
                     int16_t matchId,
                     uint8_t confidence,
                     const char *label) {
  if (!s_espNowReady) return false;

  if (s_netMutex && xSemaphoreTake(s_netMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return false;
  }

  syncHubPeerLocked();

  SmartHomeNet::CamEventPacket pkt = {};
  pkt.magic = SmartHomeNet::PACKET_MAGIC;
  pkt.source = static_cast<uint8_t>(s_source);
  pkt.event = static_cast<uint8_t>(event);
  pkt.matchId = matchId;
  pkt.confidence = confidence;
  pkt.seq = ++s_seq;
  if (label) {
    strncpy(pkt.label, label, sizeof(pkt.label) - 1);
    pkt.label[sizeof(pkt.label) - 1] = '\0';
  }

  const uint8_t *targetMac = (WiFi.status() == WL_CONNECTED && s_hubPeerAdded)
                                 ? s_hubBssid
                                 : BROADCAST_MAC;
  if (targetMac == BROADCAST_MAC) {
    ensurePeer(BROADCAST_MAC, WIFI_IF_STA);
  }

  esp_err_t err = esp_now_send(targetMac,
                               reinterpret_cast<const uint8_t *>(&pkt),
                               sizeof(pkt));
  if (err == ESP_OK && event != SmartHomeNet::CAM_EVENT_HEARTBEAT) {
    s_sentCount++;
  }

  if (s_netMutex) {
    xSemaphoreGive(s_netMutex);
  }
  return err == ESP_OK;
}

bool camNetHubConnected() {
  return WiFi.status() == WL_CONNECTED;
}

bool camNetFallbackApActive() {
  return s_fallbackApActive;
}

bool camNetLastSendOk() {
  return s_lastSendOk;
}

uint32_t camNetSentCount() {
  return s_sentCount;
}

String camNetIpString() {
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.localIP().toString();
  }
  if (s_fallbackApActive) {
    return WiFi.softAPIP().toString();
  }
  return String("0.0.0.0");
}

String camNetFallbackApSsid() {
  return (s_source == SmartHomeNet::CAM_SOURCE_FACE)
             ? String("SmartHome-FaceCam")
             : String("SmartHome-PlateCam");
}

String camNetFallbackApIp() {
  return (s_source == SmartHomeNet::CAM_SOURCE_FACE)
             ? String("192.168.11.1")
             : String("192.168.12.1");
}

String camNetMacString() {
  return WiFi.macAddress();
}
