// Smart Home Camera 1 — On-Device Face Detection & Recognition (HW-297 / ESP32-CAM)
// Runs 2-stage ESP-DL face detection (MSR01 + MNP01) and MobileFaceNet S8 recognition
// locally on the ESP32-CAM, persists enrolled faces to the "fr" Flash partition, hosts a
// management Web UI, and sends wireless ESP-NOW events to the Main ESP32 Hub.

#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <fb_gfx.h>
#include <img_converters.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cstring>
#include <list>
#include <string>
#include <vector>

#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "face_recognition_tool.hpp"
#include "face_recognition_112_v1_s8.hpp"

#include "cam_net.h"
#include "smart_home_protocol.h"

// AI-Thinker ESP32-CAM (HW-297) Pinout
#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN      4
#define STATUS_LED_PIN    33
#else
#error "CAMERA_MODEL_AI_THINKER must be defined"
#endif

namespace {

constexpr int FLASH_LEDC_CHANNEL = 2; // Avoid LEDC channel 0 used by camera XCLK
constexpr unsigned long RECOGNIZED_COOLDOWN_MS = 2500;
constexpr unsigned long UNKNOWN_COOLDOWN_MS = 4000;
constexpr unsigned long ENROLL_TIMEOUT_MS = 12000;

struct EnrolledEntry {
  int id;
  char name[24];
};

struct FaceConfig {
  float thresh = 0.55f;
  int unknownConfirm = 2;
  bool armed = true;
  uint8_t flashDuty = 0;
  bool hmirror = false;
  bool vflip = false;
};

struct FaceStatusSnapshot {
  bool cameraReady = false;
  bool modelReady = false;
  bool flashOk = false;
  bool faceDetected = false;
  bool enrollPending = false;
  char enrollTargetName[24] = "";
  char enrollMessage[64] = "Idle";
  char lastResult[24] = "NONE";
  char lastMatchName[24] = "---";
  int lastMatchId = -1;
  float lastSimilarity = 0.0f;
  uint32_t inferenceMs = 0;
  uint32_t frameCount = 0;
  int enrolledCount = 0;
  std::vector<EnrolledEntry> enrolled;
  FaceConfig cfg;
};

Preferences prefs;
WebServer server(80);

SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t previewMutex = nullptr;

FaceStatusSnapshot g_status;
FaceConfig g_pendingCfg;
bool g_applyCfgPending = false;
bool g_enrollPending = false;
unsigned long g_enrollStartMs = 0;
char g_enrollName[24] = "User";
bool g_deletePending = false;
int g_deleteId = -1;
bool g_clearPending = false;

uint8_t *g_previewJpg = nullptr;
size_t g_previewJpgLen = 0;
size_t g_previewJpgCap = 0;

void setStatusLed(bool on) {
  // AI-Thinker red status LED on GPIO33 is active-LOW
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
}

void setFlashLed(uint8_t duty) {
  ledcWrite(FLASH_LEDC_CHANNEL, duty);
}

uint16_t packRgb565Color(uint32_t bgr888) {
  return static_cast<uint16_t>(((bgr888 >> 16) & 0x001F) |
                               ((bgr888 >> 3) & 0x07E0) |
                               ((bgr888 << 8) & 0xF800));
}

void drawFaceOverlay(camera_fb_t *fb,
                     const std::list<dl::detect::result_t> &results,
                     int matchId,
                     bool enrollActive) {
  fb_data_t gfx = {};
  gfx.width = fb->width;
  gfx.height = fb->height;
  gfx.bytes_per_pixel = 2;
  gfx.format = FB_RGB565;
  gfx.data = fb->buf;

  uint32_t bgr = 0x0000FFFF; // Yellow = detected / calibrating
  if (enrollActive) {
    bgr = 0x00FFFF00; // Cyan = enrolling
  } else if (matchId >= 0) {
    bgr = 0x0000FF00; // Green = recognized
  } else if (matchId == -2) {
    bgr = 0x000000FF; // Red = unknown
  }
  uint16_t color = packRgb565Color(bgr);

  for (const auto &res : results) {
    if (res.box.size() < 4) continue;
    int x = constrain(res.box[0], 0, fb->width - 1);
    int y = constrain(res.box[1], 0, fb->height - 1);
    int x2 = constrain(res.box[2], 0, fb->width - 1);
    int y2 = constrain(res.box[3], 0, fb->height - 1);
    int w = max(1, x2 - x + 1);
    int h = max(1, y2 - y + 1);
    if (x + w > fb->width) w = fb->width - x;
    if (y + h > fb->height) h = fb->height - y;

    fb_gfx_drawFastHLine(&gfx, x, y, w, color);
    fb_gfx_drawFastHLine(&gfx, x, y + h - 1, w, color);
    fb_gfx_drawFastVLine(&gfx, x, y, h, color);
    fb_gfx_drawFastVLine(&gfx, x + w - 1, y, h, color);

    for (size_t j = 0; j + 1 < res.keypoint.size(); j += 2) {
      int kx = constrain(res.keypoint[j], 0, fb->width - 3);
      int ky = constrain(res.keypoint[j + 1], 0, fb->height - 3);
      fb_gfx_fillRect(&gfx, kx, ky, 3, 3, color);
    }
  }
}

void updatePreviewJpeg(camera_fb_t *fb) {
  uint8_t *jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &jpgBuf, &jpgLen)) {
    return;
  }

  if (previewMutex && xSemaphoreTake(previewMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (jpgLen > g_previewJpgCap) {
      uint8_t *newBuf = static_cast<uint8_t *>(ps_realloc(g_previewJpg, jpgLen));
      if (!newBuf) {
        newBuf = static_cast<uint8_t *>(realloc(g_previewJpg, jpgLen));
      }
      if (newBuf) {
        g_previewJpg = newBuf;
        g_previewJpgCap = jpgLen;
      }
    }
    if (g_previewJpg && jpgLen <= g_previewJpgCap) {
      memcpy(g_previewJpg, jpgBuf, jpgLen);
      g_previewJpgLen = jpgLen;
    }
    xSemaphoreGive(previewMutex);
  }
  free(jpgBuf);
}

void loadConfigFromNvs(FaceConfig &cfg) {
  prefs.begin("facecfg", true);
  cfg.thresh = prefs.getFloat("thresh", 0.55f);
  cfg.unknownConfirm = prefs.getInt("uconf", 2);
  cfg.armed = prefs.getBool("armed", true);
  cfg.flashDuty = prefs.getUChar("flash", 0);
  cfg.hmirror = prefs.getBool("hmir", false);
  cfg.vflip = prefs.getBool("vflip", false);
  prefs.end();

  cfg.thresh = constrain(cfg.thresh, 0.30f, 0.85f);
  cfg.unknownConfirm = constrain(cfg.unknownConfirm, 1, 5);
}

void saveConfigToNvs(const FaceConfig &cfg) {
  prefs.begin("facecfg", false);
  prefs.putFloat("thresh", cfg.thresh);
  prefs.putInt("uconf", cfg.unknownConfirm);
  prefs.putBool("armed", cfg.armed);
  prefs.putUChar("flash", cfg.flashDuty);
  prefs.putBool("hmir", cfg.hmirror);
  prefs.putBool("vflip", cfg.vflip);
  prefs.end();
}

void applySensorSettings(const FaceConfig &cfg) {
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, cfg.hmirror ? 1 : 0);
    s->set_vflip(s, cfg.vflip ? 1 : 0);
  }
  setFlashLed(cfg.flashDuty);
}

bool initCamera(const FaceConfig &cfg) {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_240X240;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: esp_camera_init failed (0x%x)\n", err);
    return false;
  }

  applySensorSettings(cfg);
  return true;
}

void syncEnrolledSnapshot(FaceRecognition112V1S8 &recognizer, FaceStatusSnapshot &snap) {
  std::vector<face_info_t> ids = recognizer.get_enrolled_ids();
  snap.enrolledCount = static_cast<int>(ids.size());
  snap.enrolled.clear();
  snap.enrolled.reserve(ids.size());
  for (const auto &item : ids) {
    EnrolledEntry e = {};
    e.id = item.id;
    if (item.name.empty()) {
      snprintf(e.name, sizeof(e.name), "Face-%d", item.id);
    } else {
      strncpy(e.name, item.name.c_str(), sizeof(e.name) - 1);
      e.name[sizeof(e.name) - 1] = '\0';
    }
    snap.enrolled.push_back(e);
  }
}

void sanitizeName(const String &raw, char *out, size_t outSize) {
  if (outSize == 0) return;
  size_t pos = 0;
  for (size_t i = 0; i < raw.length() && pos + 1 < outSize; ++i) {
    char c = raw[i];
    if (isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == ' ') {
      out[pos++] = c;
    }
  }
  while (pos > 0 && out[pos - 1] == ' ') pos--;
  out[pos] = '\0';
  if (pos == 0) {
    strncpy(out, "Person", outSize - 1);
    out[outSize - 1] = '\0';
  }
}

void faceWorkerTask(void * /*param*/) {
  FaceConfig activeCfg;
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    activeCfg = g_status.cfg;
    xSemaphoreGive(stateMutex);
  }

  HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
  HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
  FaceRecognition112V1S8 recognizer;

  bool flashOk = false;
  if (recognizer.set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr") == 1) {
    int partStatus = recognizer.check_partition();
    if (partStatus >= 0) {
      recognizer.set_ids_from_flash();
    } else {
      Serial.printf("Initializing blank 'fr' flash partition (status=%d)\n", partStatus);
      recognizer.clear_id(true);
    }
    flashOk = (recognizer.check_partition() >= 0);
  } else {
    Serial.println("WARNING: 'fr' flash partition not found; faces will not persist.");
  }

  recognizer.set_thresh(activeCfg.thresh);

  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    g_status.modelReady = true;
    g_status.flashOk = flashOk;
    syncEnrolledSnapshot(recognizer, g_status);
    xSemaphoreGive(stateMutex);
  }

  Serial.printf("Face recognition ready: flashOk=%s enrolled=%d thresh=%.2f\n",
                flashOk ? "YES" : "NO",
                recognizer.get_enrolled_id_num(),
                recognizer.get_thresh());

  int unknownStreak = 0;
  unsigned long lastRecognizedSentMs = 0;
  unsigned long lastUnknownSentMs = 0;
  bool discardStaleNext = false;

  while (true) {
    // Service pending commands without holding recognizer across HTTP calls
    bool doDelete = false;
    int deleteId = -1;
    bool doClear = false;
    bool doApplyCfg = false;
    bool enrollActive = false;
    char enrollTarget[24] = "";

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (g_clearPending) {
        doClear = true;
        g_clearPending = false;
      }
      if (g_deletePending) {
        doDelete = true;
        deleteId = g_deleteId;
        g_deletePending = false;
      }
      if (g_applyCfgPending) {
        activeCfg = g_pendingCfg;
        g_status.cfg = activeCfg;
        g_applyCfgPending = false;
        doApplyCfg = true;
      }
      if (g_enrollPending) {
        if (millis() - g_enrollStartMs > ENROLL_TIMEOUT_MS) {
          g_enrollPending = false;
          g_status.enrollPending = false;
          snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                   "Enroll timed out: no face detected");
        } else {
          enrollActive = true;
          strncpy(enrollTarget, g_enrollName, sizeof(enrollTarget) - 1);
        }
      }
      xSemaphoreGive(stateMutex);
    }

    if (doClear) {
      recognizer.clear_id(true);
      unknownStreak = 0;
      Serial.println("Cleared all enrolled faces from Flash.");
      if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        syncEnrolledSnapshot(recognizer, g_status);
        snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage), "Cleared all enrolled faces");
        xSemaphoreGive(stateMutex);
      }
    }

    if (doDelete) {
      recognizer.delete_id(deleteId, true);
      unknownStreak = 0;
      Serial.printf("Deleted face ID #%d from Flash.\n", deleteId);
      if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
        syncEnrolledSnapshot(recognizer, g_status);
        snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage), "Deleted face ID #%d", deleteId);
        xSemaphoreGive(stateMutex);
      }
    }

    if (doApplyCfg) {
      recognizer.set_thresh(activeCfg.thresh);
      applySensorSettings(activeCfg);
      saveConfigToNvs(activeCfg);
      Serial.printf("Updated settings: thresh=%.2f armed=%d uconf=%d flash=%u\n",
                    activeCfg.thresh, activeCfg.armed, activeCfg.unknownConfirm, activeCfg.flashDuty);
    }

    if (discardStaleNext) {
      camera_fb_t *stale = esp_camera_fb_get();
      if (stale) esp_camera_fb_return(stale);
      discardStaleNext = false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    unsigned long tStart = millis();
    std::vector<int> imgShape = {static_cast<int>(fb->height), static_cast<int>(fb->width), 3};
    std::list<dl::detect::result_t> &candidates =
        s1.infer(reinterpret_cast<uint16_t *>(fb->buf), imgShape);
    std::list<dl::detect::result_t> &results =
        s2.infer(reinterpret_cast<uint16_t *>(fb->buf), imgShape, candidates);

    // Pick largest face with 5-point facial landmarks
    const dl::detect::result_t *bestFace = nullptr;
    long bestArea = -1;
    for (const auto &res : results) {
      if (res.box.size() < 4 || res.keypoint.size() < 10) continue;
      long area = static_cast<long>(res.box[2] - res.box[0]) * static_cast<long>(res.box[3] - res.box[1]);
      if (area > bestArea) {
        bestArea = area;
        bestFace = &res;
      }
    }

    int overlayState = 0;
    bool detected = (bestFace != nullptr);
    int matchedId = -1;
    float similarity = 0.0f;
    char matchedName[24] = "---";
    char resultLabel[24] = "NO FACE";
    bool enrolledNow = false;
    int enrolledNewId = -1;

    if (bestFace) {
      std::vector<int> landmarks = bestFace->keypoint;
      if (enrollActive) {
        enrolledNewId = recognizer.enroll_id(reinterpret_cast<uint16_t *>(fb->buf),
                                             imgShape,
                                             landmarks,
                                             std::string(enrollTarget),
                                             true);
        enrolledNow = (enrolledNewId >= 0);
        overlayState = enrolledNewId;
        snprintf(resultLabel, sizeof(resultLabel), enrolledNow ? "ENROLLED" : "ENROLL ERR");
        if (enrolledNow) {
          matchedId = enrolledNewId;
          strncpy(matchedName, enrollTarget, sizeof(matchedName) - 1);
          similarity = 1.0f;
        }
        unknownStreak = 0;
        discardStaleNext = true;
      } else {
        face_info_t info = recognizer.recognize(reinterpret_cast<uint16_t *>(fb->buf),
                                                imgShape,
                                                landmarks);
        matchedId = info.id;
        similarity = info.similarity;
        discardStaleNext = true;

        unsigned long now = millis();
        uint8_t confPct = static_cast<uint8_t>(constrain(lroundf(similarity * 100.0f), 0L, 100L));

        if (info.id >= 0) {
          unknownStreak = 0;
          overlayState = info.id;
          snprintf(resultLabel, sizeof(resultLabel), "RECOGNIZED");
          if (info.name.empty()) {
            snprintf(matchedName, sizeof(matchedName), "ID-%d", info.id);
          } else {
            strncpy(matchedName, info.name.c_str(), sizeof(matchedName) - 1);
            matchedName[sizeof(matchedName) - 1] = '\0';
          }
          setStatusLed(true);

          if (now - lastRecognizedSentMs >= RECOGNIZED_COOLDOWN_MS) {
            lastRecognizedSentMs = now;
            camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED,
                            static_cast<int16_t>(info.id),
                            confPct,
                            matchedName);
            Serial.printf("[FACE] RECOGNIZED id=%d name=\"%s\" sim=%.2f\n",
                          info.id, matchedName, similarity);
          }
        } else {
          overlayState = -2;
          snprintf(resultLabel, sizeof(resultLabel), "UNKNOWN");
          snprintf(matchedName, sizeof(matchedName), "Unknown");
          unknownStreak++;

          if (activeCfg.armed &&
              unknownStreak >= activeCfg.unknownConfirm &&
              (now - lastUnknownSentMs >= UNKNOWN_COOLDOWN_MS)) {
            lastUnknownSentMs = now;
            camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN, -1, confPct, "UNKNOWN");
            Serial.printf("[FACE] UNKNOWN INTRUDER ALERT (sim=%.2f streak=%d)\n",
                          similarity, unknownStreak);
          }
        }
      }
    } else {
      unknownStreak = 0;
      setStatusLed(false);
    }

    unsigned long elapsedMs = millis() - tStart;
    if (elapsedMs > 250) {
      discardStaleNext = true;
    }

    if (!results.empty()) {
      drawFaceOverlay(fb, results, overlayState, enrollActive);
    }
    updatePreviewJpeg(fb);
    esp_camera_fb_return(fb);

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_status.faceDetected = detected;
      g_status.lastMatchId = matchedId;
      g_status.lastSimilarity = similarity;
      g_status.inferenceMs = static_cast<uint32_t>(elapsedMs);
      g_status.frameCount++;
      strncpy(g_status.lastResult, resultLabel, sizeof(g_status.lastResult) - 1);
      strncpy(g_status.lastMatchName, matchedName, sizeof(g_status.lastMatchName) - 1);

      if (enrollActive) {
        g_enrollPending = false;
        g_status.enrollPending = false;
        if (enrolledNow) {
          snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                   "Saved \"%s\" as ID #%d in Flash", enrollTarget, enrolledNewId);
          syncEnrolledSnapshot(recognizer, g_status);
          Serial.println(g_status.enrollMessage);
        } else {
          snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
                   "Enroll failed (Flash full or bad alignment)");
        }
      }
      xSemaphoreGive(stateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

String jsonEscape(const char *s) {
  String out;
  if (!s) return out;
  for (size_t i = 0; s[i] != '\0'; ++i) {
    char c = s[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<uint8_t>(c) >= 0x20) {
      out += c;
    }
  }
  return out;
}

void handleRoot() {
  static const char HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Camera 1 — Face Recognition</title>
<style>
  body{font-family:system-ui,-apple-system,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:16px}
  .wrap{max-width:920px;margin:0 auto}
  h1{font-size:1.35rem;margin:0 0 12px}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:14px}
  .card{background:#1e293b;border:1px solid #334155;border-radius:10px;padding:14px}
  img{width:100%;max-width:320px;border-radius:8px;border:1px solid #475569;display:block;margin:0 auto 10px;background:#020617}
  .row{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid #334155;font-size:.92rem}
  .row:last-child{border-bottom:0}
  .badge{padding:2px 8px;border-radius:999px;font-size:.78rem;font-weight:700}
  .ok{background:#14532d;color:#86efac}.bad{background:#7f1d1d;color:#fca5a5}.warn{background:#7c2d12;color:#fdba74}.off{background:#334155;color:#cbd5e1}
  input,select{background:#0f172a;color:#e2e8f0;border:1px solid #475569;border-radius:6px;padding:6px 8px;font-size:.9rem}
  button{background:#2563eb;color:#fff;border:0;border-radius:7px;padding:7px 12px;font-weight:600;cursor:pointer;font-size:.88rem}
  button.red{background:#b91c1c}
  button.gray{background:#475569}
  button.green{background:#15803d}
  .formrow{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}
  table{width:100%;border-collapse:collapse;margin-top:8px;font-size:.9rem}
  th,td{padding:6px;border-bottom:1px solid #334155;text-align:left}
</style>
</head>
<body>
<div class="wrap">
  <h1>Camera 1 — Local Face Recognition (192.168.4.101)</h1>
  <div class="grid">
    <div class="card">
      <img id="cam" src="/preview.jpg" alt="Camera Preview">
      <div class="row"><span>Recognition State</span><span id="res" class="badge off">---</span></div>
      <div class="row"><span>Identity / Score</span><span id="who">---</span></div>
      <div class="row"><span>Hub Wireless Link</span><span id="hub" class="badge off">---</span></div>
      <div class="row"><span>Cycle Time / Flash DB</span><span id="perf">---</span></div>
      <div class="formrow">
        <button class="green" onclick="testTrigger(1)">Test Valid Door Open</button>
        <button class="red" onclick="testTrigger(0)">Test Intruder Siren</button>
      </div>
    </div>
    <div class="card">
      <strong>Enroll New Authorized Face</strong>
      <p style="font-size:.84rem;color:#94a3b8;margin:4px 0 8px">Look directly at the camera (~40–80 cm), enter a name, then click Enroll.</p>
      <div class="formrow">
        <input id="ename" type="text" maxlength="20" placeholder="Name (e.g. Alice)" value="User">
        <button onclick="enrollFace()">Enroll Next Face</button>
      </div>
      <div id="emsg" style="font-size:.85rem;color:#93c5fd;margin-bottom:10px">Idle</div>

      <div style="display:flex;justify-content:space-between;align-items:center">
        <strong>Saved Faces in Flash (<span id="ecnt">0</span>)</strong>
        <button class="red" onclick="clearFaces()">Clear All</button>
      </div>
      <table><thead><tr><th>ID</th><th>Name</th><th>Action</th></tr></thead><tbody id="elist"></tbody></table>

      <hr style="border-color:#334155;margin:12px 0">
      <strong>Settings (Saved to NVS)</strong>
      <div class="row"><span>Intruder Siren Armed</span><input id="armed" type="checkbox"></div>
      <div class="row"><span>Match Threshold (0.35–0.80)</span><input id="thresh" type="number" step="0.05" min="0.30" max="0.85" style="width:78px"></div>
      <div class="row"><span>Unknown Confirm Frames</span><input id="uconf" type="number" min="1" max="5" style="width:78px"></div>
      <div class="row"><span>Flash LED Brightness (0–180)</span><input id="flash" type="number" min="0" max="180" style="width:78px"></div>
      <div class="row"><span>Flip Image (H / V)</span><span><label><input id="hmir" type="checkbox"> H</label> <label><input id="vflip" type="checkbox"> V</label></span></div>
      <div class="formrow"><button onclick="saveCfg()">Save Settings</button></div>
    </div>
  </div>
</div>
<script>
let cfgLoaded = false;
function refreshImg(){
  const img = document.getElementById('cam');
  const nxt = new Image();
  nxt.onload = ()=>{ img.src = nxt.src; setTimeout(refreshImg, 450); };
  nxt.onerror = ()=>{ setTimeout(refreshImg, 1200); };
  nxt.src = '/preview.jpg?t=' + Date.now();
}
async function refreshStatus(){
  try{
    const r = await fetch('/api/status');
    const d = await r.json();
    const resEl = document.getElementById('res');
    resEl.textContent = d.enrollPending ? 'ENROLLING...' : d.lastResult;
    resEl.className = 'badge ' + (d.lastResult==='RECOGNIZED'?'ok':(d.lastResult==='UNKNOWN'?'bad':'off'));
    document.getElementById('who').textContent = d.lastMatchName + ' (' + Math.round(d.lastSimilarity*100) + '%)';
    const hubEl = document.getElementById('hub');
    hubEl.textContent = d.hubConnected ? ('CONNECTED ('+d.ip+')') : 'FALLBACK AP';
    hubEl.className = 'badge ' + (d.hubConnected ? 'ok' : 'warn');
    document.getElementById('perf').textContent = d.inferenceMs + ' ms | Flash=' + (d.flashOk?'OK':'ERR');
    document.getElementById('emsg').textContent = d.enrollMessage;
    document.getElementById('ecnt').textContent = d.enrolledCount;

    const tb = document.getElementById('elist');
    tb.innerHTML = '';
    d.enrolled.forEach(item=>{
      const tr = document.createElement('tr');
      tr.innerHTML = '<td>#'+item.id+'</td><td>'+item.name+'</td><td><button class="red" onclick="delFace('+item.id+')">Delete</button></td>';
      tb.appendChild(tr);
    });

    if(!cfgLoaded){
      cfgLoaded = true;
      document.getElementById('armed').checked = d.cfg.armed;
      document.getElementById('thresh').value = d.cfg.thresh.toFixed(2);
      document.getElementById('uconf').value = d.cfg.unknownConfirm;
      document.getElementById('flash').value = d.cfg.flashDuty;
      document.getElementById('hmir').checked = d.cfg.hmirror;
      document.getElementById('vflip').checked = d.cfg.vflip;
    }
  }catch(e){}
}
async function enrollFace(){
  const name = encodeURIComponent(document.getElementById('ename').value || 'User');
  await fetch('/api/enroll?name=' + name, {method:'POST'});
  refreshStatus();
}
async function delFace(id){ await fetch('/api/delete?id=' + id, {method:'POST'}); refreshStatus(); }
async function clearFaces(){ if(confirm('Delete ALL enrolled faces?')){ await fetch('/api/clear', {method:'POST'}); refreshStatus(); } }
async function saveCfg(){
  const q = new URLSearchParams({
    armed: document.getElementById('armed').checked ? 1 : 0,
    thresh: document.getElementById('thresh').value,
    uconf: document.getElementById('uconf').value,
    flash: document.getElementById('flash').value,
    hmir: document.getElementById('hmir').checked ? 1 : 0,
    vflip: document.getElementById('vflip').checked ? 1 : 0
  });
  await fetch('/api/config?' + q.toString(), {method:'POST'});
  cfgLoaded = false;
  refreshStatus();
}
async function testTrigger(ok){ await fetch('/api/test?ok=' + ok, {method:'POST'}); }
refreshImg(); refreshStatus(); setInterval(refreshStatus, 1000);
</script>
</body>
</html>)HTML";
  server.send_P(200, "text/html", HTML);
}

void handlePreview() {
  if (!previewMutex || xSemaphoreTake(previewMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    server.send(503, "text/plain", "Preview busy");
    return;
  }
  if (!g_previewJpg || g_previewJpgLen == 0) {
    xSemaphoreGive(previewMutex);
    server.send(503, "text/plain", "Preview not ready");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(g_previewJpgLen);
  server.send(200, "image/jpeg", "");
  server.sendContent(reinterpret_cast<const char *>(g_previewJpg), g_previewJpgLen);
  xSemaphoreGive(previewMutex);
}

void handleStatus() {
  FaceStatusSnapshot snap;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snap = g_status;
    xSemaphoreGive(stateMutex);
  }

  String json = "{";
  json += "\"cameraReady\":" + String(snap.cameraReady ? "true" : "false") + ",";
  json += "\"modelReady\":" + String(snap.modelReady ? "true" : "false") + ",";
  json += "\"flashOk\":" + String(snap.flashOk ? "true" : "false") + ",";
  json += "\"hubConnected\":" + String(camNetHubConnected() ? "true" : "false") + ",";
  json += "\"fallbackAp\":" + String(camNetFallbackApActive() ? "true" : "false") + ",";
  json += "\"ip\":\"" + camNetIpString() + "\",";
  json += "\"faceDetected\":" + String(snap.faceDetected ? "true" : "false") + ",";
  json += "\"enrollPending\":" + String(snap.enrollPending ? "true" : "false") + ",";
  json += "\"enrollMessage\":\"" + jsonEscape(snap.enrollMessage) + "\",";
  json += "\"lastResult\":\"" + jsonEscape(snap.lastResult) + "\",";
  json += "\"lastMatchName\":\"" + jsonEscape(snap.lastMatchName) + "\",";
  json += "\"lastMatchId\":" + String(snap.lastMatchId) + ",";
  json += "\"lastSimilarity\":" + String(snap.lastSimilarity, 3) + ",";
  json += "\"inferenceMs\":" + String(snap.inferenceMs) + ",";
  json += "\"frameCount\":" + String(snap.frameCount) + ",";
  json += "\"enrolledCount\":" + String(snap.enrolledCount) + ",";
  json += "\"enrolled\":[";
  for (size_t i = 0; i < snap.enrolled.size(); ++i) {
    if (i > 0) json += ",";
    json += "{\"id\":" + String(snap.enrolled[i].id) + ",\"name\":\"" + jsonEscape(snap.enrolled[i].name) + "\"}";
  }
  json += "],\"cfg\":{";
  json += "\"armed\":" + String(snap.cfg.armed ? "true" : "false") + ",";
  json += "\"thresh\":" + String(snap.cfg.thresh, 2) + ",";
  json += "\"unknownConfirm\":" + String(snap.cfg.unknownConfirm) + ",";
  json += "\"flashDuty\":" + String(snap.cfg.flashDuty) + ",";
  json += "\"hmirror\":" + String(snap.cfg.hmirror ? "true" : "false") + ",";
  json += "\"vflip\":" + String(snap.cfg.vflip ? "true" : "false");
  json += "}}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleEnroll() {
  String rawName = server.hasArg("name") ? server.arg("name") : String("User");
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    sanitizeName(rawName, g_enrollName, sizeof(g_enrollName));
    g_enrollPending = true;
    g_enrollStartMs = millis();
    g_status.enrollPending = true;
    snprintf(g_status.enrollMessage, sizeof(g_status.enrollMessage),
             "Waiting for face to enroll \"%s\"...", g_enrollName);
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"queued\":true}");
  } else {
    server.send(503, "application/json", "{\"queued\":false}");
  }
}

void handleDelete() {
  int id = server.hasArg("id") ? server.arg("id").toInt() : -1;
  if (id >= 0 && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_deleteId = id;
    g_deletePending = true;
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"queued\":true}");
  } else {
    server.send(400, "application/json", "{\"error\":\"invalid id\"}");
  }
}

void handleClear() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_clearPending = true;
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"queued\":true}");
  } else {
    server.send(503, "application/json", "{\"queued\":false}");
  }
}

void handleConfig() {
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    FaceConfig next = g_applyCfgPending ? g_pendingCfg : g_status.cfg;
    if (server.hasArg("armed")) next.armed = (server.arg("armed").toInt() != 0);
    if (server.hasArg("thresh")) next.thresh = constrain(server.arg("thresh").toFloat(), 0.30f, 0.85f);
    if (server.hasArg("uconf")) next.unknownConfirm = constrain(server.arg("uconf").toInt(), 1, 5);
    if (server.hasArg("flash")) next.flashDuty = static_cast<uint8_t>(constrain(server.arg("flash").toInt(), 0, 180));
    if (server.hasArg("hmir")) next.hmirror = (server.arg("hmir").toInt() != 0);
    if (server.hasArg("vflip")) next.vflip = (server.arg("vflip").toInt() != 0);
    g_pendingCfg = next;
    g_applyCfgPending = true;
    xSemaphoreGive(stateMutex);
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(503, "application/json", "{\"ok\":false}");
  }
}

void handleTest() {
  bool ok = server.hasArg("ok") && (server.arg("ok").toInt() != 0);
  bool sent = ok ? camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED, 0, 100, "TEST-USER")
                 : camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN, -1, 0, "TEST-UNKNOWN");
  server.send(200, "application/json", sent ? "{\"sent\":true}" : "{\"sent\":false}");
}

} // namespace

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  setStatusLed(false);

  ledcSetup(FLASH_LEDC_CHANNEL, 5000, 8);
  ledcAttachPin(FLASH_LED_PIN, FLASH_LEDC_CHANNEL);
  setFlashLed(0);

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Smart Home Camera 1: Face Recognition (HW-297) ===");

  stateMutex = xSemaphoreCreateMutex();
  previewMutex = xSemaphoreCreateMutex();

  FaceConfig cfg;
  loadConfigFromNvs(cfg);
  bool camOk = initCamera(cfg);
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    g_status.cfg = cfg;
    g_status.cameraReady = camOk;
    xSemaphoreGive(stateMutex);
  }

  camNetInit(SmartHomeNet::CAM_SOURCE_FACE);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/preview.jpg", HTTP_GET, handlePreview);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/enroll", HTTP_POST, handleEnroll);
  server.on("/api/delete", HTTP_POST, handleDelete);
  server.on("/api/clear", HTTP_POST, handleClear);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/test", HTTP_POST, handleTest);
  server.begin();

  // Elevate loopTask above faceWorkerTask on Core 1 so HTTP + ESP-NOW remain snappy
  vTaskPrioritySet(nullptr, 2);

  if (camOk) {
    xTaskCreatePinnedToCore(faceWorkerTask,
                            "face_worker",
                            16384,
                            nullptr,
                            1,
                            nullptr,
                            1); // Pin to Core 1 (IDLE1 WDT is disabled, Core 0 handles Wi-Fi)
  }
}

void loop() {
  camNetLoop();
  server.handleClient();
  delay(10);
}
