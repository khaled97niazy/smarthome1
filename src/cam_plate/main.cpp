// Smart Home Camera 2 — On-Device Car License Plate, QR Code & Barcode Recognition (HW-297 / ESP32-CAM)
// Runs 100% locally on the ESP32-CAM (no PC/cloud):
//   - QR Code decoder (PSRAM-backed quirc) + 1D Barcode decoder (Code 128 & Code 39)
//   - Fallback License Plate OCR (edge-energy ROI, Otsu, CCL, 16x16 NCC)
// Sends ESP-NOW recognized / intruder events to the Main Hub.

#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_heap_caps.h>
#include <img_converters.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cstring>
#include <string>
#include <vector>

#include "cam_net.h"
#include "plate_code.h"
#include "plate_ocr.h"
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
constexpr size_t MAX_PLATES = 20;
constexpr unsigned long RECOGNIZED_COOLDOWN_MS = 3500;
constexpr unsigned long UNKNOWN_COOLDOWN_MS = 4000;

enum PlateRecognitionMode : uint8_t {
  PLATE_MODE_AUTO = 0,      // Try QR / Barcode first, fallback to License Plate OCR
  PLATE_MODE_CODE_ONLY = 1, // QR Code & Barcode (Code 128 / Code 39) only
  PLATE_MODE_OCR_ONLY = 2,  // License Plate OCR only
};

#ifndef PLATE_DEFAULT_MODE
#define PLATE_DEFAULT_MODE PLATE_MODE_AUTO
#endif

struct PlateConfig {
  uint8_t mode = PLATE_DEFAULT_MODE;
  int minConf = 55;
  int confirmReads = 2;
  uint8_t polarity = PLATE_POLARITY_AUTO;
  bool allowConfusable = true;
  bool armed = true;
  uint8_t flashDuty = 0;
  bool hmirror = false;
  bool vflip = false;
};

struct PlateStatusSnapshot {
  bool cameraReady = false;
  bool ocrReady = false;
  bool codeReady = false;
  bool plateLocalized = false;
  bool validRead = false;
  bool matched = false;
  char lastResult[24] = "NO PLATE";
  char lastSource[16] = "---";
  char lastReadText[16] = "---";
  char lastMatchedPlate[16] = "---";
  int lastConf = 0;
  int lastCharCount = 0;
  uint32_t processMs = 0;
  uint32_t frameCount = 0;
  std::vector<String> plates;
  PlateConfig cfg;
};

Preferences prefs;
WebServer server(80);

SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t previewMutex = nullptr;

PlateStatusSnapshot g_status;
std::vector<String> g_plates;
PlateConfig g_cfg;

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

void loadFromNvs(std::vector<String> &plates, PlateConfig &cfg) {
  prefs.begin("platecfg", true);
  uint8_t defMode = (PLATE_DEFAULT_MODE <= PLATE_MODE_OCR_ONLY)
                        ? static_cast<uint8_t>(PLATE_DEFAULT_MODE)
                        : static_cast<uint8_t>(PLATE_MODE_AUTO);
  cfg.mode = prefs.getUChar("mode", defMode);
  if (cfg.mode > PLATE_MODE_OCR_ONLY) cfg.mode = defMode;
  cfg.minConf = constrain(prefs.getInt("minc", 55), 35, 85);
  cfg.confirmReads = constrain(prefs.getInt("confn", 2), 1, 5);
  cfg.polarity = prefs.getUChar("pol", PLATE_POLARITY_AUTO);
  if (cfg.polarity > PLATE_POLARITY_LIGHT_ON_DARK) cfg.polarity = PLATE_POLARITY_AUTO;
  cfg.allowConfusable = prefs.getBool("fuzzy", true);
  cfg.armed = prefs.getBool("armed", true);
  cfg.flashDuty = prefs.getUChar("flash", 0);
  cfg.hmirror = prefs.getBool("hmir", false);
  cfg.vflip = prefs.getBool("vflip", false);

  size_t count = min<size_t>(MAX_PLATES, prefs.getUInt("cnt", 0));
  plates.clear();
  for (size_t i = 0; i < count; ++i) {
    char key[8];
    snprintf(key, sizeof(key), "p%u", static_cast<unsigned>(i));
    String norm = plateNormalizeString(prefs.getString(key, ""));
    if (norm.length() >= 3) {
      plates.push_back(norm);
    }
  }
  prefs.end();
}

void savePlatesToNvs(const std::vector<String> &plates) {
  prefs.begin("platecfg", false);
  size_t count = min<size_t>(MAX_PLATES, plates.size());
  prefs.putUInt("cnt", static_cast<uint32_t>(count));
  for (size_t i = 0; i < count; ++i) {
    char key[8];
    snprintf(key, sizeof(key), "p%u", static_cast<unsigned>(i));
    prefs.putString(key, plates[i]);
  }
  prefs.end();
}

void saveConfigToNvs(const PlateConfig &cfg) {
  prefs.begin("platecfg", false);
  prefs.putUChar("mode", cfg.mode);
  prefs.putInt("minc", cfg.minConf);
  prefs.putInt("confn", cfg.confirmReads);
  prefs.putUChar("pol", cfg.polarity);
  prefs.putBool("fuzzy", cfg.allowConfusable);
  prefs.putBool("armed", cfg.armed);
  prefs.putUChar("flash", cfg.flashDuty);
  prefs.putBool("hmir", cfg.hmirror);
  prefs.putBool("vflip", cfg.vflip);
  prefs.end();
}

void applySensorSettings(const PlateConfig &cfg) {
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, cfg.hmirror ? 1 : 0);
    s->set_vflip(s, cfg.vflip ? 1 : 0);
    s->set_contrast(s, 1);
  }
  setFlashLed(cfg.flashDuty);
}

bool initCamera(const PlateConfig &cfg) {
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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA; // 320x240 grayscale (76.8 KB)
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

void updatePreviewJpeg(camera_fb_t *fb) {
  uint8_t *jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_GRAYSCALE, 80, &jpgBuf, &jpgLen)) {
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

void plateWorkerTask(void * /*param*/) {
  int recognizedStreak = 0;
  int unknownStreak = 0;
  int missFrames = 0;
  unsigned long lastRecognizedSentMs = 0;
  unsigned long lastUnknownSentMs = 0;

  while (true) {
    PlateConfig cfg;
    std::vector<String> authorized;
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      cfg = g_cfg;
      authorized = g_plates;
      xSemaphoreGive(stateMutex);
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    unsigned long tStart = millis();
    PlateCodeResult codeRes = {};
    PlateOcrResult ocr = {};

    const bool tryCode = (cfg.mode != PLATE_MODE_OCR_ONLY);
    const bool tryOcr = (cfg.mode != PLATE_MODE_CODE_ONLY);

    if (tryCode) {
      plateCodeAnalyze(fb->buf,
                       static_cast<int>(fb->width),
                       static_cast<int>(fb->height),
                       true,
                       true,
                       codeRes);
    }
    if (!codeRes.validRead && tryOcr) {
      plateOcrAnalyze(fb->buf,
                      static_cast<int>(fb->width),
                      static_cast<int>(fb->height),
                      static_cast<PlatePolarity>(cfg.polarity),
                      cfg.minConf,
                      ocr);
    }
    unsigned long elapsedMs = millis() - tStart;

    const bool usedCode = codeRes.validRead;
    const bool localized = usedCode || ocr.plateLocalized || codeRes.detected;
    const bool validRead = usedCode || ocr.validRead;
    const char *readText = usedCode ? codeRes.text : ocr.text;
    const uint8_t readConf = usedCode ? codeRes.confidence : ocr.avgConfidence;
    const int charCount = usedCode ? static_cast<int>(strlen(codeRes.text))
                                   : static_cast<int>(ocr.charCount);
    const char *srcLabel = "---";
    if (usedCode || (codeRes.detected && !ocr.plateLocalized)) {
      srcLabel = plateSymbologyName(codeRes.symbology);
    } else if (ocr.validRead || ocr.plateLocalized) {
      srcLabel = "OCR";
    }

    int matchedIdx = -1;
    String matchedStr = "---";
    if (validRead) {
      const bool allowFuzzy = (!usedCode) && cfg.allowConfusable;
      for (size_t i = 0; i < authorized.size(); ++i) {
        if (plateMatchesAuthorized(readText, authorized[i].c_str(), allowFuzzy)) {
          matchedIdx = static_cast<int>(i);
          matchedStr = authorized[i];
          break;
        }
      }
    }

    unsigned long now = millis();
    const char *stateLabel = "NO PLATE";
    if (validRead) {
      missFrames = 0;
      if (matchedIdx >= 0) {
        stateLabel = "AUTHORIZED";
        unknownStreak = 0;
        recognizedStreak++;
        setStatusLed(true);
        if (recognizedStreak >= cfg.confirmReads &&
            (now - lastRecognizedSentMs >= RECOGNIZED_COOLDOWN_MS)) {
          lastRecognizedSentMs = now;
          camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED,
                          static_cast<int16_t>(matchedIdx),
                          readConf,
                          matchedStr.c_str());
          Serial.printf("[PLATE/%s] AUTHORIZED read=\"%s\" matched=\"%s\" conf=%u%%\n",
                        srcLabel, readText, matchedStr.c_str(), readConf);
        }
      } else {
        stateLabel = "UNKNOWN";
        recognizedStreak = 0;
        unknownStreak++;
        setStatusLed(false);
        if (cfg.armed &&
            unknownStreak >= cfg.confirmReads &&
            (now - lastUnknownSentMs >= UNKNOWN_COOLDOWN_MS)) {
          lastUnknownSentMs = now;
          camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN,
                          -1,
                          readConf,
                          readText);
          Serial.printf("[PLATE/%s] UNKNOWN INTRUDER read=\"%s\" conf=%u%%\n",
                        srcLabel, readText, readConf);
        }
      }
    } else {
      setStatusLed(false);
      if (localized) {
        stateLabel = "CANDIDATE";
      }
      if (!localized || ++missFrames >= 2) {
        recognizedStreak = 0;
        unknownStreak = 0;
      }
    }

    if (usedCode || (codeRes.detected && !ocr.plateLocalized)) {
      plateCodeDrawOverlayGray(fb->buf,
                               static_cast<int>(fb->width),
                               static_cast<int>(fb->height),
                               codeRes,
                               matchedIdx >= 0);
    } else {
      plateDrawOverlayGray(fb->buf,
                           static_cast<int>(fb->width),
                           static_cast<int>(fb->height),
                           ocr,
                           matchedIdx >= 0);
    }
    updatePreviewJpeg(fb);
    esp_camera_fb_return(fb);

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_status.plateLocalized = localized;
      g_status.validRead = validRead;
      g_status.matched = (matchedIdx >= 0);
      g_status.lastConf = readConf;
      g_status.lastCharCount = charCount;
      g_status.processMs = static_cast<uint32_t>(elapsedMs);
      g_status.frameCount++;
      snprintf(g_status.lastResult, sizeof(g_status.lastResult), "%s", stateLabel);
      snprintf(g_status.lastSource, sizeof(g_status.lastSource), "%s", srcLabel);
      if (charCount > 0 && readText[0] != '\0') {
        snprintf(g_status.lastReadText, sizeof(g_status.lastReadText), "%s", readText);
      } else {
        snprintf(g_status.lastReadText, sizeof(g_status.lastReadText), "%s", "---");
      }
      snprintf(g_status.lastMatchedPlate, sizeof(g_status.lastMatchedPlate), "%s", matchedStr.c_str());
      xSemaphoreGive(stateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(35));
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
<title>Camera 2 — Plate, QR &amp; Barcode</title>
<style>
  body{font-family:system-ui,-apple-system,sans-serif;background:#0f172a;color:#e2e8f0;margin:0;padding:16px}
  .wrap{max-width:940px;margin:0 auto}
  h1{font-size:1.35rem;margin:0 0 12px}
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}
  .card{background:#1e293b;border:1px solid #334155;border-radius:10px;padding:14px}
  img{width:100%;max-width:340px;border-radius:8px;border:1px solid #475569;display:block;margin:0 auto 10px;background:#020617}
  .row{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid #334155;font-size:.92rem}
  .row:last-child{border-bottom:0}
  .badge{padding:2px 8px;border-radius:999px;font-size:.78rem;font-weight:700}
  .ok{background:#14532d;color:#86efac}.bad{background:#7f1d1d;color:#fca5a5}.warn{background:#7c2d12;color:#fdba74}.off{background:#334155;color:#cbd5e1}
  input,select{background:#0f172a;color:#e2e8f0;border:1px solid #475569;border-radius:6px;padding:6px 8px;font-size:.9rem}
  button{background:#2563eb;color:#fff;border:0;border-radius:7px;padding:7px 12px;font-weight:600;cursor:pointer;font-size:.88rem}
  button.red{background:#b91c1c}
  button.green{background:#15803d}
  .formrow{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}
  table{width:100%;border-collapse:collapse;margin-top:8px;font-size:.9rem}
  th,td{padding:6px;border-bottom:1px solid #334155;text-align:left}
  code{font-size:1rem;color:#93c5fd;font-weight:700}
</style>
</head>
<body>
<div class="wrap">
  <h1>Camera 2 — Plate OCR, QR &amp; Barcode Recognition (192.168.4.102)</h1>
  <div class="grid">
    <div class="card">
      <img id="cam" src="/preview.jpg" alt="Plate Camera Preview">
      <div class="row"><span>Recognition State</span><span id="res" class="badge off">---</span></div>
      <div class="row"><span>Detection Source</span><span id="src" class="badge off">---</span></div>
      <div class="row"><span>Detected Plate</span><code id="txt">---</code></div>
      <div class="row"><span>Confidence / Chars</span><span id="conf">---</span></div>
      <div class="row"><span>Matched Plate</span><span id="match">---</span></div>
      <div class="row"><span>Hub Wireless Link</span><span id="hub" class="badge off">---</span></div>
      <div class="row"><span>Processing Time</span><span id="perf">---</span></div>
      <div class="formrow">
        <button class="green" onclick="testTrigger(1)">Test Gate Open</button>
        <button class="red" onclick="testTrigger(0)">Test Intruder Siren</button>
      </div>
    </div>
    <div class="card">
      <strong>Authorized Car Plates (Saved in Flash NVS)</strong>
      <p style="font-size:.84rem;color:#94a3b8;margin:4px 0 8px">Alphanumeric uppercase (A–Z, 0–9). Works with physical license plates, QR codes, or Code 128 / Code 39 barcodes carrying the plate number.</p>
      <div class="formrow">
        <input id="pinput" type="text" maxlength="12" placeholder="Plate (e.g. ABC1234)" style="text-transform:uppercase;width:150px">
        <button onclick="addPlate()">Add Plate</button>
        <button class="green" onclick="useLastRead()">Use Last Read</button>
      </div>

      <div style="display:flex;justify-content:space-between;align-items:center;margin-top:8px">
        <strong>Saved Plates (<span id="pcnt">0</span> / 20)</strong>
        <button class="red" onclick="clearPlates()">Clear All</button>
      </div>
      <table><thead><tr><th>#</th><th>Plate</th><th>Action</th></tr></thead><tbody id="plist"></tbody></table>

      <hr style="border-color:#334155;margin:12px 0">
      <strong>Recognition &amp; Camera Calibration</strong>
      <div class="row"><span>Recognition Mode</span>
        <select id="mode">
          <option value="0">Auto (QR / Barcode &rarr; OCR)</option>
          <option value="1">QR Code / Barcode Only</option>
          <option value="2">Plate OCR Only</option>
        </select>
      </div>
      <div class="row"><span>Intruder Siren Armed</span><input id="armed" type="checkbox"></div>
      <div class="row"><span>Min OCR Confidence (%)</span><input id="minc" type="number" min="35" max="85" style="width:78px"></div>
      <div class="row"><span>Confirm Frames (1–5)</span><input id="confn" type="number" min="1" max="5" style="width:78px"></div>
      <div class="row"><span>Polarity</span>
        <select id="pol">
          <option value="0">Auto</option>
          <option value="1">Dark text on Light plate</option>
          <option value="2">Light text on Dark plate</option>
        </select>
      </div>
      <div class="row"><span>1-Char Fuzzy (0/O, 1/I, 5/S, 8/B)</span><input id="fuzzy" type="checkbox"></div>
      <div class="row"><span>Flash LED Brightness (0–120)</span><input id="flash" type="number" min="0" max="120" style="width:78px"></div>
      <div class="row"><span>Flip Image (H / V)</span><span><label><input id="hmir" type="checkbox"> H</label> <label><input id="vflip" type="checkbox"> V</label></span></div>
      <div class="formrow"><button onclick="saveCfg()">Save Settings</button></div>
    </div>
  </div>
</div>
<script>
let cfgLoaded = false;
let lastReadCache = '';
function refreshImg(){
  const img = document.getElementById('cam');
  const nxt = new Image();
  nxt.onload = ()=>{ img.src = nxt.src; setTimeout(refreshImg, 350); };
  nxt.onerror = ()=>{ setTimeout(refreshImg, 1200); };
  nxt.src = '/preview.jpg?t=' + Date.now();
}
async function refreshStatus(){
  try{
    const r = await fetch('/api/status');
    const d = await r.json();
    const resEl = document.getElementById('res');
    resEl.textContent = d.lastResult;
    resEl.className = 'badge ' + (d.lastResult==='AUTHORIZED'?'ok':(d.lastResult==='UNKNOWN'?'bad':(d.lastResult==='CANDIDATE'?'warn':'off')));
    const srcEl = document.getElementById('src');
    srcEl.textContent = d.lastSource || '---';
    srcEl.className = 'badge ' + (d.lastSource==='OCR'?'warn':(d.lastSource && d.lastSource!=='---'?'ok':'off'));
    document.getElementById('txt').textContent = d.lastReadText;
    if(d.lastReadText && d.lastReadText !== '---') lastReadCache = d.lastReadText;
    document.getElementById('conf').textContent = d.lastConf + '% (' + d.lastCharCount + ' chars)';
    document.getElementById('match').textContent = d.lastMatchedPlate;
    const hubEl = document.getElementById('hub');
    hubEl.textContent = d.hubConnected ? ('CONNECTED ('+d.ip+')') : 'FALLBACK AP';
    hubEl.className = 'badge ' + (d.hubConnected ? 'ok' : 'warn');
    document.getElementById('perf').textContent = d.processMs + ' ms';

    document.getElementById('pcnt').textContent = d.plates.length;
    const tb = document.getElementById('plist');
    tb.innerHTML = '';
    d.plates.forEach((p, idx)=>{
      const tr = document.createElement('tr');
      tr.innerHTML = '<td>'+(idx+1)+'</td><td><code>'+p+'</code></td><td><button class="red" onclick="delPlate(\''+p+'\')">Delete</button></td>';
      tb.appendChild(tr);
    });

    if(!cfgLoaded){
      cfgLoaded = true;
      document.getElementById('mode').value = d.cfg.mode;
      document.getElementById('armed').checked = d.cfg.armed;
      document.getElementById('minc').value = d.cfg.minConf;
      document.getElementById('confn').value = d.cfg.confirmReads;
      document.getElementById('pol').value = d.cfg.polarity;
      document.getElementById('fuzzy').checked = d.cfg.allowConfusable;
      document.getElementById('flash').value = d.cfg.flashDuty;
      document.getElementById('hmir').checked = d.cfg.hmirror;
      document.getElementById('vflip').checked = d.cfg.vflip;
    }
  }catch(e){}
}
async function addPlate(){
  const p = document.getElementById('pinput').value.trim();
  if(!p) return;
  await fetch('/api/add?plate=' + encodeURIComponent(p), {method:'POST'});
  document.getElementById('pinput').value = '';
  refreshStatus();
}
function useLastRead(){
  if(lastReadCache) document.getElementById('pinput').value = lastReadCache;
}
async function delPlate(p){ await fetch('/api/delete?plate=' + encodeURIComponent(p), {method:'POST'}); refreshStatus(); }
async function clearPlates(){ if(confirm('Delete ALL saved plates?')){ await fetch('/api/clear', {method:'POST'}); refreshStatus(); } }
async function saveCfg(){
  const q = new URLSearchParams({
    mode: document.getElementById('mode').value,
    armed: document.getElementById('armed').checked ? 1 : 0,
    minc: document.getElementById('minc').value,
    confn: document.getElementById('confn').value,
    pol: document.getElementById('pol').value,
    fuzzy: document.getElementById('fuzzy').checked ? 1 : 0,
    flash: document.getElementById('flash').value,
    hmir: document.getElementById('hmir').checked ? 1 : 0,
    vflip: document.getElementById('vflip').checked ? 1 : 0
  });
  await fetch('/api/config?' + q.toString(), {method:'POST'});
  cfgLoaded = false;
  refreshStatus();
}
async function testTrigger(ok){ await fetch('/api/test?ok=' + ok, {method:'POST'}); }
refreshImg(); refreshStatus(); setInterval(refreshStatus, 900);
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
  PlateStatusSnapshot snap;
  std::vector<String> platesCopy;
  PlateConfig cfgCopy;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snap = g_status;
    platesCopy = g_plates;
    cfgCopy = g_cfg;
    xSemaphoreGive(stateMutex);
  }

  String json = "{";
  json += "\"cameraReady\":" + String(snap.cameraReady ? "true" : "false") + ",";
  json += "\"ocrReady\":" + String(snap.ocrReady ? "true" : "false") + ",";
  json += "\"codeReady\":" + String(snap.codeReady ? "true" : "false") + ",";
  json += "\"hubConnected\":" + String(camNetHubConnected() ? "true" : "false") + ",";
  json += "\"ip\":\"" + camNetIpString() + "\",";
  json += "\"plateLocalized\":" + String(snap.plateLocalized ? "true" : "false") + ",";
  json += "\"validRead\":" + String(snap.validRead ? "true" : "false") + ",";
  json += "\"lastResult\":\"" + jsonEscape(snap.lastResult) + "\",";
  json += "\"lastSource\":\"" + jsonEscape(snap.lastSource) + "\",";
  json += "\"lastReadText\":\"" + jsonEscape(snap.lastReadText) + "\",";
  json += "\"lastMatchedPlate\":\"" + jsonEscape(snap.lastMatchedPlate) + "\",";
  json += "\"lastConf\":" + String(snap.lastConf) + ",";
  json += "\"lastCharCount\":" + String(snap.lastCharCount) + ",";
  json += "\"processMs\":" + String(snap.processMs) + ",";
  json += "\"frameCount\":" + String(snap.frameCount) + ",";
  json += "\"plates\":[";
  for (size_t i = 0; i < platesCopy.size(); ++i) {
    if (i > 0) json += ",";
    json += "\"" + jsonEscape(platesCopy[i].c_str()) + "\"";
  }
  json += "],\"cfg\":{";
  json += "\"mode\":" + String(cfgCopy.mode) + ",";
  json += "\"armed\":" + String(cfgCopy.armed ? "true" : "false") + ",";
  json += "\"minConf\":" + String(cfgCopy.minConf) + ",";
  json += "\"confirmReads\":" + String(cfgCopy.confirmReads) + ",";
  json += "\"polarity\":" + String(cfgCopy.polarity) + ",";
  json += "\"allowConfusable\":" + String(cfgCopy.allowConfusable ? "true" : "false") + ",";
  json += "\"flashDuty\":" + String(cfgCopy.flashDuty) + ",";
  json += "\"hmirror\":" + String(cfgCopy.hmirror ? "true" : "false") + ",";
  json += "\"vflip\":" + String(cfgCopy.vflip ? "true" : "false");
  json += "}}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleAddPlate() {
  String raw = server.hasArg("plate") ? server.arg("plate") : String("");
  String norm = plateNormalizeString(raw);
  if (norm.length() < 3) {
    server.send(400, "application/json", "{\"error\":\"plate needs >=3 alphanumeric chars\"}");
    return;
  }

  std::vector<String> saved;
  bool added = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    bool exists = false;
    for (const auto &p : g_plates) {
      if (p == norm) {
        exists = true;
        break;
      }
    }
    if (!exists && g_plates.size() < MAX_PLATES) {
      g_plates.push_back(norm);
      saved = g_plates;
      added = true;
    }
    xSemaphoreGive(stateMutex);
  }
  if (added) {
    savePlatesToNvs(saved);
    Serial.printf("Saved plate \"%s\" to NVS Flash (total=%u)\n",
                  norm.c_str(), static_cast<unsigned>(saved.size()));
  }
  server.send(200, "application/json", added ? "{\"added\":true}" : "{\"added\":false}");
}

void handleDeletePlate() {
  String norm = plateNormalizeString(server.hasArg("plate") ? server.arg("plate") : String(""));
  std::vector<String> saved;
  bool removed = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (auto it = g_plates.begin(); it != g_plates.end(); ++it) {
      if (*it == norm) {
        g_plates.erase(it);
        removed = true;
        break;
      }
    }
    if (removed) saved = g_plates;
    xSemaphoreGive(stateMutex);
  }
  if (removed) {
    savePlatesToNvs(saved);
    Serial.printf("Deleted plate \"%s\" from NVS Flash\n", norm.c_str());
  }
  server.send(200, "application/json", removed ? "{\"deleted\":true}" : "{\"deleted\":false}");
}

void handleClearPlates() {
  std::vector<String> empty;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    g_plates.clear();
    xSemaphoreGive(stateMutex);
  }
  savePlatesToNvs(empty);
  Serial.println("Cleared all plates from NVS Flash.");
  server.send(200, "application/json", "{\"cleared\":true}");
}

void handleConfig() {
  PlateConfig next;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    next = g_cfg;
    if (server.hasArg("mode")) {
      int m = server.arg("mode").toInt();
      next.mode = (m >= 0 && m <= 2) ? static_cast<uint8_t>(m) : PLATE_MODE_AUTO;
    }
    if (server.hasArg("armed")) next.armed = (server.arg("armed").toInt() != 0);
    if (server.hasArg("minc")) next.minConf = constrain(server.arg("minc").toInt(), 35, 85);
    if (server.hasArg("confn")) next.confirmReads = constrain(server.arg("confn").toInt(), 1, 5);
    if (server.hasArg("pol")) {
      int pol = server.arg("pol").toInt();
      next.polarity = (pol >= 0 && pol <= 2) ? static_cast<uint8_t>(pol) : PLATE_POLARITY_AUTO;
    }
    if (server.hasArg("fuzzy")) next.allowConfusable = (server.arg("fuzzy").toInt() != 0);
    if (server.hasArg("flash")) next.flashDuty = static_cast<uint8_t>(constrain(server.arg("flash").toInt(), 0, 120));
    if (server.hasArg("hmir")) next.hmirror = (server.arg("hmir").toInt() != 0);
    if (server.hasArg("vflip")) next.vflip = (server.arg("vflip").toInt() != 0);
    g_cfg = next;
    xSemaphoreGive(stateMutex);
  }
  saveConfigToNvs(next);
  applySensorSettings(next);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTest() {
  bool ok = server.hasArg("ok") && (server.arg("ok").toInt() != 0);
  bool sent = ok ? camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED, 0, 100, "TEST123")
                 : camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN, -1, 0, "BAD999");
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
  Serial.println("\n=== Smart Home Camera 2: Plate OCR + QR/Barcode (HW-297) ===");

  stateMutex = xSemaphoreCreateMutex();
  previewMutex = xSemaphoreCreateMutex();

  loadFromNvs(g_plates, g_cfg);
  bool ocrOk = plateOcrInit();
  bool codeOk = plateCodeInit();
  bool camOk = initCamera(g_cfg);

  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    g_status.cameraReady = camOk;
    g_status.ocrReady = ocrOk;
    g_status.codeReady = codeOk;
    xSemaphoreGive(stateMutex);
  }

  Serial.printf("Plate Cam init: camOk=%s ocrOk=%s codeOk=%s mode=%u savedPlates=%u\n",
                camOk ? "YES" : "NO",
                ocrOk ? "YES" : "NO",
                codeOk ? "YES" : "NO",
                static_cast<unsigned>(g_cfg.mode),
                static_cast<unsigned>(g_plates.size()));

  camNetInit(SmartHomeNet::CAM_SOURCE_PLATE);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/preview.jpg", HTTP_GET, handlePreview);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/add", HTTP_POST, handleAddPlate);
  server.on("/api/delete", HTTP_POST, handleDeletePlate);
  server.on("/api/clear", HTTP_POST, handleClearPlates);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.on("/api/test", HTTP_POST, handleTest);
  server.begin();

  vTaskPrioritySet(nullptr, 2);

  if (camOk && (ocrOk || codeOk)) {
    xTaskCreatePinnedToCore(plateWorkerTask,
                            "plate_worker",
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
