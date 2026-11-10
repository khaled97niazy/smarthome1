// Smart Home Camera 2 — On-Device Car License Plate, QR Code & Barcode Recognition (HW-297 / ESP32-CAM)
// Runs 100% locally on the ESP32-CAM (no PC/cloud):
//   - QR Code decoder (PSRAM-backed quirc) + 1D Barcode decoder (Code 128 & Code 39)
//   - Fallback License Plate OCR (edge-energy ROI, Otsu, CCL, 16x16 NCC)
// Hosts real-time MJPEG live streaming and management Web UI, and sends ESP-NOW events to Main Hub.

#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <cstring>
#include <vector>

#include "cam_hw.h"
#include "cam_net.h"
#include "plate_code.h"
#include "plate_ocr.h"
#include "smart_home_json.h"
#include "smart_home_protocol.h"

namespace {

constexpr size_t MAX_PLATES = 20;
constexpr unsigned long RECOGNIZED_COOLDOWN_MS = 3500;
constexpr unsigned long UNKNOWN_COOLDOWN_MS = 4000;
constexpr uint16_t STREAM_PORT = 81;

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
  float fps = 0.0f;
  uint32_t frameCount = 0;
  PlateConfig cfg;
};

Preferences prefs;
WebServer server(80);

SemaphoreHandle_t stateMutex = nullptr;

PlateStatusSnapshot g_status;
std::vector<String> g_plates;
PlateConfig g_cfg;
PlateConfig g_pendingCfg;
bool g_applyCfgPending = false;

void loadFromNvs(std::vector<String> &plates, PlateConfig &cfg) {
  if (!prefs.begin("platecfg", true)) return;
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
  if (!prefs.begin("platecfg", false)) return;
  size_t count = min<size_t>(MAX_PLATES, plates.size());
  prefs.putUInt("cnt", static_cast<uint32_t>(count));
  for (size_t i = 0; i < count; ++i) {
    char key[8];
    snprintf(key, sizeof(key), "p%u", static_cast<unsigned>(i));
    prefs.putString(key, plates[i]);
  }
  for (size_t i = count; i < MAX_PLATES; ++i) {
    char key[8];
    snprintf(key, sizeof(key), "p%u", static_cast<unsigned>(i));
    if (prefs.isKey(key)) {
      prefs.remove(key);
    }
  }
  prefs.end();
}

void saveConfigToNvs(const PlateConfig &cfg) {
  if (!prefs.begin("platecfg", false)) return;
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

void plateWorkerTask(void * /*param*/) {
  int recognizedStreak = 0;
  int unknownStreak = 0;
  int missFrames = 0;
  char lastStreakText[16] = "";
  unsigned long lastRecognizedSentMs = 0;
  unsigned long lastUnknownSentMs = 0;

  uint32_t fpsFrames = 0;
  unsigned long fpsLastMs = millis();
  float currentFps = 0.0f;

  while (true) {
    PlateConfig cfg;
    bool applyCfg = false;
    std::vector<String> authorized;
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      if (g_applyCfgPending) {
        g_cfg = g_pendingCfg;
        g_applyCfgPending = false;
        applyCfg = true;
      }
      cfg = g_cfg;
      authorized = g_plates;
      xSemaphoreGive(stateMutex);
    }

    if (applyCfg) {
      camHwApplySensorSettings(cfg.hmirror, cfg.vflip, cfg.flashDuty, 1);
      saveConfigToNvs(cfg);
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      vTaskDelay(pdMS_TO_TICKS(40));
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
      const char *streakKey = (matchedIdx >= 0) ? matchedStr.c_str() : readText;
      if (strcmp(streakKey, lastStreakText) != 0) {
        recognizedStreak = 0;
        unknownStreak = 0;
        snprintf(lastStreakText, sizeof(lastStreakText), "%s", streakKey);
      }

      if (matchedIdx >= 0) {
        stateLabel = "AUTHORIZED";
        unknownStreak = 0;
        recognizedStreak++;
        camHwSetStatusLed(true);
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
        camHwSetStatusLed(false);
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
      camHwSetStatusLed(false);
      if (localized) {
        stateLabel = "CANDIDATE";
      }
      if (!localized || ++missFrames >= 2) {
        recognizedStreak = 0;
        unknownStreak = 0;
        lastStreakText[0] = '\0';
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

    camHwUpdatePreviewJpeg(fb, PIXFORMAT_GRAYSCALE, 68);
    esp_camera_fb_return(fb);

    // Track real-time FPS
    fpsFrames++;
    unsigned long nowMs = millis();
    if (nowMs - fpsLastMs >= 1000) {
      currentFps = (fpsFrames * 1000.0f) / (nowMs - fpsLastMs);
      fpsFrames = 0;
      fpsLastMs = nowMs;
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_status.plateLocalized = localized;
      g_status.validRead = validRead;
      g_status.matched = (matchedIdx >= 0);
      g_status.lastConf = readConf;
      g_status.lastCharCount = charCount;
      g_status.processMs = static_cast<uint32_t>(elapsedMs);
      g_status.fps = currentFps;
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

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void handleRoot() {
  static const char HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Smart Camera 2 — License Plate, QR &amp; Barcode Live</title>
<style>
  :root {
    --bg: #090d16;
    --card: #131c2e;
    --card-border: #23324d;
    --card-hover: #19263e;
    --text: #f1f5f9;
    --text-muted: #94a3b8;
    --primary: #3b82f6;
    --primary-hover: #2563eb;
    --success: #10b981;
    --danger: #ef4444;
    --warning: #f59e0b;
    --cyan: #06b6d4;
    --radius: 12px;
  }
  * { box-sizing: border-box; }
  body {
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: var(--bg);
    color: var(--text);
    margin: 0;
    padding: 16px;
    line-height: 1.45;
  }
  .wrap { max-width: 1040px; margin: 0 auto; }
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 10px;
    margin-bottom: 16px;
    padding-bottom: 12px;
    border-bottom: 1px solid var(--card-border);
  }
  h1 { font-size: 1.25rem; font-weight: 700; margin: 0; display: flex; align-items: center; gap: 8px; }
  .pulse-dot {
    width: 10px; height: 10px; border-radius: 50%; background: var(--success);
    box-shadow: 0 0 10px var(--success); animation: pulse 1.8s infinite;
  }
  @keyframes pulse { 0%, 100% { opacity: 1; transform: scale(1); } 50% { opacity: .4; transform: scale(.85); } }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); gap: 16px; }
  .card {
    background: var(--card);
    border: 1px solid var(--card-border);
    border-radius: var(--radius);
    padding: 16px;
    box-shadow: 0 4px 14px rgba(0,0,0,.35);
  }
  .card-title {
    font-size: .95rem; font-weight: 700; color: #cbd5e1;
    margin: 0 0 12px; display: flex; justify-content: space-between; align-items: center;
  }

  /* Video Player Box */
  .player-box {
    position: relative;
    width: 100%;
    max-width: 440px;
    margin: 0 auto 12px;
    border-radius: var(--radius);
    overflow: hidden;
    background: #020617;
    border: 1px solid var(--card-border);
    aspect-ratio: 4 / 3;
    box-shadow: 0 8px 24px rgba(0,0,0,.5);
  }
  .player-box img {
    width: 100%;
    height: 100%;
    object-fit: cover;
    display: block;
    background: #000;
  }

  /* License Plate Aspect Reticle Guide */
  .reticle {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    width: 68%;
    height: 38%;
    border-radius: 8px;
    border: 2px dashed rgba(148, 163, 184, 0.35);
    pointer-events: none;
    transition: all .25s ease;
  }
  .reticle.authorized {
    border: 3px solid var(--success);
    box-shadow: 0 0 18px rgba(16, 185, 129, 0.5);
  }
  .reticle.unknown {
    border: 3px solid var(--danger);
    box-shadow: 0 0 18px rgba(239, 68, 68, 0.5);
  }
  .reticle.code {
    border: 3px solid var(--cyan);
    box-shadow: 0 0 18px rgba(6, 182, 212, 0.5);
  }

  /* HUD Overlays */
  .hud-top {
    position: absolute;
    top: 10px;
    left: 10px;
    right: 10px;
    display: flex;
    justify-content: space-between;
    pointer-events: none;
  }
  .hud-pill {
    background: rgba(15, 23, 42, 0.85);
    backdrop-filter: blur(6px);
    border: 1px solid rgba(255, 255, 255, 0.12);
    border-radius: 999px;
    padding: 3px 9px;
    font-size: .75rem;
    font-weight: 700;
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }
  .hud-bottom {
    position: absolute;
    bottom: 0;
    left: 0;
    right: 0;
    background: linear-gradient(to top, rgba(2, 6, 23, 0.95), rgba(2, 6, 23, 0));
    padding: 24px 12px 10px;
    display: flex;
    justify-content: space-between;
    align-items: flex-end;
    pointer-events: none;
  }
  .hud-banner-title { font-size: 1.15rem; font-weight: 800; font-family: monospace; letter-spacing: 1px; }
  .hud-banner-sub { font-size: .75rem; color: var(--text-muted); }

  /* Quick Floating Add Plate Button on HUD */
  #hud-quick-add {
    display: none;
    pointer-events: auto;
    background: var(--success);
    color: #fff;
    border: 0;
    border-radius: 6px;
    padding: 4px 10px;
    font-size: .76rem;
    font-weight: 700;
    cursor: pointer;
    margin-top: 4px;
    box-shadow: 0 2px 8px rgba(0,0,0,.4);
  }
  #hud-quick-add:hover { background: #059669; }

  /* Badges */
  .badge {
    padding: 2px 8px;
    border-radius: 999px;
    font-size: .75rem;
    font-weight: 700;
    letter-spacing: .3px;
  }
  .badge.ok { background: rgba(16, 185, 129, 0.2); color: #6ee7b7; border: 1px solid rgba(16, 185, 129, 0.35); }
  .badge.bad { background: rgba(239, 68, 68, 0.2); color: #fca5a5; border: 1px solid rgba(239, 68, 68, 0.35); }
  .badge.warn { background: rgba(245, 158, 11, 0.2); color: #fde68a; border: 1px solid rgba(245, 158, 11, 0.35); }
  .badge.info { background: rgba(6, 182, 212, 0.2); color: #a5f3fc; border: 1px solid rgba(6, 182, 212, 0.35); }
  .badge.off { background: rgba(148, 163, 184, 0.15); color: #94a3b8; }

  /* Buttons & Controls */
  button {
    background: var(--primary);
    color: #fff;
    border: 0;
    border-radius: 8px;
    padding: 8px 14px;
    font-size: .88rem;
    font-weight: 600;
    cursor: pointer;
    transition: all .15s ease;
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }
  button:hover { background: var(--primary-hover); transform: translateY(-1px); }
  button:active { transform: translateY(0); }
  button.danger { background: var(--danger); }
  button.danger:hover { background: #dc2626; }
  button.success { background: var(--success); }
  button.success:hover { background: #059669; }
  button.outline { background: transparent; border: 1px solid var(--card-border); color: var(--text-muted); }
  button.outline:hover { background: var(--card-hover); color: var(--text); }

  input[type="text"], input[type="number"], select {
    background: #0b1322;
    color: var(--text);
    border: 1px solid var(--card-border);
    border-radius: 8px;
    padding: 8px 12px;
    font-size: .9rem;
    width: 100%;
    outline: none;
  }
  input[type="text"]:focus, input[type="number"]:focus {
    border-color: var(--primary);
    box-shadow: 0 0 0 2px rgba(59, 130, 246, 0.25);
  }
  .row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 7px 0;
    border-bottom: 1px solid rgba(255, 255, 255, 0.05);
    font-size: .88rem;
  }
  .row:last-child { border-bottom: 0; }

  /* Mode selection cards */
  .mode-grid {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 6px;
    margin-bottom: 12px;
  }
  .mode-btn {
    background: #0b1322;
    border: 1px solid var(--card-border);
    border-radius: 8px;
    padding: 8px 6px;
    color: var(--text-muted);
    font-size: .78rem;
    font-weight: 600;
    text-align: center;
    cursor: pointer;
    transition: all .15s ease;
  }
  .mode-btn.active {
    background: var(--card);
    border-color: var(--primary);
    color: #fff;
    box-shadow: 0 2px 8px rgba(59,130,246,.25);
  }

  table { width: 100%; border-collapse: collapse; margin-top: 8px; font-size: .88rem; }
  th, td { padding: 8px; text-align: left; border-bottom: 1px solid var(--card-border); }
  th { color: var(--text-muted); font-weight: 600; font-size: .75rem; text-transform: uppercase; }
  code { font-size: 1rem; color: #93c5fd; font-weight: 700; font-family: monospace; }

  .switch { position: relative; display: inline-block; width: 42px; height: 22px; }
  .switch input { opacity: 0; width: 0; height: 0; }
  .slider {
    position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0;
    background-color: #334155; transition: .2s; border-radius: 22px;
  }
  .slider:before {
    position: absolute; content: ""; height: 16px; width: 16px; left: 3px; bottom: 3px;
    background-color: white; transition: .2s; border-radius: 50%;
  }
  input:checked + .slider { background-color: var(--primary); }
  input:checked + .slider:before { transform: translateX(20px); }

  #toast {
    position: fixed;
    bottom: 20px;
    right: 20px;
    background: #1e293b;
    border: 1px solid var(--card-border);
    color: #fff;
    padding: 10px 18px;
    border-radius: 8px;
    box-shadow: 0 6px 18px rgba(0,0,0,.5);
    transform: translateY(100px);
    opacity: 0;
    transition: all .25s ease;
    z-index: 999;
    font-size: .88rem;
  }
  #toast.show { transform: translateY(0); opacity: 1; }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <h1><span class="pulse-dot"></span> Camera 2 — License Plate &amp; Code Live</h1>
    <div style="display:flex;gap:8px;align-items:center">
      <span id="hub-badge" class="badge off">CHECKING LINK</span>
      <span id="fps-badge" class="badge info">0.0 FPS</span>
    </div>
  </header>

  <div class="grid">
    <!-- LEFT CARD: LIVE STREAM & PLATE HUD -->
    <div class="card">
      <div class="card-title">
        <span>Live Recognition Feed</span>
        <button class="outline" style="padding:4px 8px;font-size:.78rem" onclick="toggleStreamMode()" id="stream-mode-btn">MJPEG Live</button>
      </div>

      <div class="player-box">
        <img id="cam" alt="Camera Feed" src="/preview.jpg">
        <div id="reticle" class="reticle"></div>

        <div class="hud-top">
          <div class="hud-pill" id="hud-stream-pill">
            <span style="display:inline-block;width:7px;height:7px;border-radius:50%;background:var(--success)"></span>
            <span id="hud-stream-text">STREAMING :81</span>
          </div>
          <div class="hud-pill" id="hud-src-pill">---</div>
        </div>

        <div class="hud-bottom">
          <div>
            <div class="hud-banner-title" id="hud-txt">---</div>
            <div class="hud-banner-sub" id="hud-state">Position plate or QR in view</div>
            <button id="hud-quick-add" onclick="quickAddCurrentPlate()">+ Authorize This Plate</button>
          </div>
          <div style="text-align:right">
            <div id="hud-conf" style="font-weight:700;font-size:1.15rem;color:#fff">0%</div>
            <div style="font-size:.72rem;color:var(--text-muted)" id="hud-ms">0 ms</div>
          </div>
        </div>
      </div>

      <!-- Quick LED & Gate Test Controls -->
      <div class="row">
        <span>Flash LED Brightness</span>
        <div style="display:flex;align-items:center;gap:8px;width:55%">
          <input id="flash-slider" type="range" min="0" max="120" value="0" style="flex:1" oninput="updateFlashPreview(this.value)">
          <span id="flash-val" style="font-size:.8rem;width:28px;text-align:right">0</span>
        </div>
      </div>

      <div class="row">
        <span>Processing Diagnostics</span>
        <span id="status-diag" style="font-size:.82rem;color:var(--text-muted)">OCR &amp; QR: Ready | ESP-NOW Active</span>
      </div>

      <div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px">
        <button class="success" onclick="testGate(1)" style="justify-content:center">Test Gate Open</button>
        <button class="danger" onclick="testGate(0)" style="justify-content:center">Test Intruder Siren</button>
      </div>
    </div>

    <!-- RIGHT CARD: AUTHORIZED PLATES & CALIBRATION -->
    <div class="card">
      <div class="card-title">
        <span>Authorized Plates (NVS Flash)</span>
        <span id="plate-count-badge" class="badge off">0 / 20</span>
      </div>

      <!-- Quick Add Plate -->
      <div style="margin-bottom:12px">
        <label style="font-size:.82rem;color:var(--text-muted);display:block;margin-bottom:4px">License Plate / Code Value (A–Z, 0–9)</label>
        <div style="display:flex;gap:8px">
          <input id="pinput" type="text" maxlength="12" placeholder="e.g. ABC1234" style="text-transform:uppercase;font-family:monospace;font-weight:700">
          <button onclick="addPlate()" style="white-space:nowrap">Add Plate</button>
          <button class="outline" onclick="useLastRead()" style="white-space:nowrap">Use Detected</button>
        </div>
      </div>

      <!-- Saved Plates Table -->
      <div style="display:flex;justify-content:space-between;align-items:center;margin-top:12px">
        <span style="font-weight:700;font-size:.88rem">Authorized Car Plates</span>
        <button class="outline danger" style="padding:4px 8px;font-size:.78rem" onclick="clearAllPlates()">Clear All</button>
      </div>

      <div style="max-height:220px;overflow-y:auto;margin-top:6px">
        <table>
          <thead><tr><th>#</th><th>Plate Number</th><th style="text-align:right">Action</th></tr></thead>
          <tbody id="plate-table-body">
            <tr><td colspan="3" style="text-align:center;color:var(--text-muted)">Loading authorized plates...</td></tr>
          </tbody>
        </table>
      </div>

      <hr style="border:0;border-top:1px solid var(--card-border);margin:16px 0 12px">

      <!-- Recognition Mode Cards -->
      <div class="card-title" style="margin-bottom:8px">
        <span>Recognition Mode</span>
      </div>
      <div class="mode-grid">
        <div class="mode-btn active" id="mbtn-0" onclick="setMode(0)">Auto (QR+OCR)</div>
        <div class="mode-btn" id="mbtn-1" onclick="setMode(1)">QR / Barcode</div>
        <div class="mode-btn" id="mbtn-2" onclick="setMode(2)">Plate OCR</div>
      </div>

      <!-- Calibration & Settings -->
      <div class="row">
        <span>Intruder Siren Armed</span>
        <label class="switch"><input id="cfg-armed" type="checkbox"><span class="slider"></span></label>
      </div>

      <div class="row">
        <span>Min OCR Confidence (<span id="minc-val">55%</span>)</span>
        <div style="display:flex;align-items:center;gap:6px;width:55%">
          <input id="cfg-minc" type="range" min="35" max="85" step="5" value="55" style="flex:1" oninput="document.getElementById('minc-val').textContent=this.value+'%'">
        </div>
      </div>

      <div class="row">
        <span>Confirmation Frames</span>
        <input id="cfg-confn" type="number" min="1" max="5" value="2" style="width:65px">
      </div>

      <div class="row">
        <span>Plate Polarity</span>
        <select id="cfg-pol" style="width:180px">
          <option value="0">Auto Detect</option>
          <option value="1">Dark text on Light</option>
          <option value="2">Light text on Dark</option>
        </select>
      </div>

      <div class="row">
        <span>1-Char Fuzzy Match (0/O, 1/I, 5/S, 8/B)</span>
        <input id="cfg-fuzzy" type="checkbox">
      </div>

      <div class="row">
        <span>Orientation</span>
        <span>
          <label style="margin-right:12px"><input id="cfg-hmir" type="checkbox"> Mirror H</label>
          <label><input id="cfg-vflip" type="checkbox"> Flip V</label>
        </span>
      </div>

      <div style="margin-top:10px">
        <button onclick="saveSettings()" style="width:100%;justify-content:center">Save Settings to NVS</button>
      </div>
    </div>
  </div>
</div>

<div id="toast">Settings Saved</div>

<script>
let currentMode = 'mjpeg';
let mjpegFails = 0;
let snapshotActive = false;
let cfgLoaded = false;
let clientFrames = 0;
let lastFpsTs = performance.now();
let lastListSig = '';
let lastDetectedText = '';

function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2600);
}

function initCamera() {
  const img = document.getElementById('cam');
  img.onload = () => {
    clientFrames++;
    const now = performance.now();
    if (now - lastFpsTs >= 1000) {
      const fps = Math.round((clientFrames * 1000) / (now - lastFpsTs) * 10) / 10;
      document.getElementById('fps-badge').textContent = fps + ' FPS';
      clientFrames = 0;
      lastFpsTs = now;
    }
  };

  img.onerror = () => {
    if (currentMode === 'mjpeg') {
      mjpegFails++;
      if (mjpegFails >= 2) {
        console.warn('Switching to fast snapshot polling fallback');
        setStreamMode('snapshot');
      }
    }
  };

  setStreamMode('mjpeg');
}

function setStreamMode(mode) {
  currentMode = mode;
  const img = document.getElementById('cam');
  const btn = document.getElementById('stream-mode-btn');
  const pill = document.getElementById('hud-stream-text');

  if (mode === 'mjpeg') {
    snapshotActive = false;
    btn.textContent = 'Mode: MJPEG';
    pill.textContent = 'STREAMING :81';
    img.src = window.location.protocol + '//' + window.location.hostname + ':81/stream?t=' + Date.now();
  } else {
    btn.textContent = 'Mode: Snapshots';
    pill.textContent = 'FAST SNAPSHOT';
    snapshotActive = true;
    pollNextSnapshot();
  }
}

function toggleStreamMode() {
  setStreamMode(currentMode === 'mjpeg' ? 'snapshot' : 'mjpeg');
}

function pollNextSnapshot() {
  if (!snapshotActive || currentMode !== 'snapshot') return;
  const img = document.getElementById('cam');
  const next = new Image();
  next.onload = () => {
    if (snapshotActive) {
      img.src = next.src;
      requestAnimationFrame(pollNextSnapshot);
    }
  };
  next.onerror = () => {
    if (snapshotActive) setTimeout(pollNextSnapshot, 400);
  };
  next.src = '/preview.jpg?t=' + Date.now();
}

let flashTimer = null;
function updateFlashPreview(val) {
  document.getElementById('flash-val').textContent = val;
  clearTimeout(flashTimer);
  flashTimer = setTimeout(async () => {
    const q = new URLSearchParams({ flash: val });
    await fetch('/api/config?' + q.toString(), { method: 'POST' });
  }, 120);
}

function setMode(m) {
  for (let i = 0; i <= 2; ++i) {
    document.getElementById('mbtn-' + i).className = 'mode-btn' + (i === m ? ' active' : '');
  }
}

function getSelectedMode() {
  for (let i = 0; i <= 2; ++i) {
    if (document.getElementById('mbtn-' + i).classList.contains('active')) return i;
  }
  return 0;
}

async function refreshStatus() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();

    const hub = document.getElementById('hub-badge');
    hub.textContent = d.hubConnected ? ('HUB: ' + d.ip) : 'STANDALONE AP';
    hub.className = 'badge ' + (d.hubConnected ? 'ok' : 'warn');

    document.getElementById('status-diag').textContent =
      'Latency: ' + d.processMs + ' ms | ' + d.plates.length + ' Authorized Plates | Frame #' + d.frameCount;
    document.getElementById('plate-count-badge').textContent = d.plates.length + ' / 20';

    const reticle = document.getElementById('reticle');
    const txtEl = document.getElementById('hud-txt');
    const stateEl = document.getElementById('hud-state');
    const srcEl = document.getElementById('hud-src-pill');
    const confEl = document.getElementById('hud-conf');
    const qAddBtn = document.getElementById('hud-quick-add');
    document.getElementById('hud-ms').textContent = d.processMs + ' ms';

    srcEl.textContent = d.lastSource || '---';
    srcEl.className = 'hud-pill badge ' + (d.lastSource === 'OCR' ? 'warn' : (d.lastSource && d.lastSource !== '---' ? 'info' : 'off'));

    if (d.lastReadText && d.lastReadText !== '---') {
      lastDetectedText = d.lastReadText;
      txtEl.textContent = d.lastReadText;
      confEl.textContent = d.lastConf + '%';

      if (d.lastResult === 'AUTHORIZED') {
        reticle.className = 'reticle authorized';
        stateEl.textContent = 'Authorized Car (' + d.lastMatchedPlate + ')';
        confEl.style.color = 'var(--success)';
        qAddBtn.style.display = 'none';
      } else if (d.lastResult === 'UNKNOWN') {
        reticle.className = 'reticle unknown';
        stateEl.textContent = d.cfg.armed ? 'Unknown Plate — Intruder Siren Armed' : 'Unregistered Plate';
        confEl.style.color = 'var(--danger)';
        qAddBtn.style.display = 'inline-block';
        qAddBtn.textContent = '+ Authorize "' + d.lastReadText + '"';
      } else {
        reticle.className = 'reticle code';
        stateEl.textContent = 'Detected Code / Plate Candidate';
        confEl.style.color = 'var(--cyan)';
        qAddBtn.style.display = 'none';
      }
    } else {
      txtEl.textContent = '---';
      confEl.textContent = '0%';
      confEl.style.color = '#fff';
      stateEl.textContent = 'Position plate or QR in view';
      reticle.className = 'reticle';
      qAddBtn.style.display = 'none';
    }

    renderPlatesTable(d.plates, d.lastMatchedPlate);

    if (!cfgLoaded && d.cfg) {
      cfgLoaded = true;
      setMode(d.cfg.mode);
      document.getElementById('cfg-armed').checked = d.cfg.armed;
      document.getElementById('cfg-minc').value = d.cfg.minConf;
      document.getElementById('minc-val').textContent = d.cfg.minConf + '%';
      document.getElementById('cfg-confn').value = d.cfg.confirmReads;
      document.getElementById('cfg-pol').value = d.cfg.polarity;
      document.getElementById('cfg-fuzzy').checked = d.cfg.allowConfusable;
      document.getElementById('flash-slider').value = d.cfg.flashDuty;
      document.getElementById('flash-val').textContent = d.cfg.flashDuty;
      document.getElementById('cfg-hmir').checked = d.cfg.hmirror;
      document.getElementById('cfg-vflip').checked = d.cfg.vflip;
    }
  } catch (err) {}
}

function renderPlatesTable(list, activeMatchPlate) {
  const sig = list.join('|') + '@' + activeMatchPlate;
  if (sig === lastListSig) return;
  lastListSig = sig;

  const tbody = document.getElementById('plate-table-body');
  tbody.innerHTML = '';

  if (list.length === 0) {
    tbody.innerHTML = '<tr><td colspan="3" style="text-align:center;color:var(--text-muted);padding:14px">No authorized plates stored. Add one above!</td></tr>';
    return;
  }

  list.forEach((plate, idx) => {
    const tr = document.createElement('tr');
    const isCurrent = (plate === activeMatchPlate);

    const tdIdx = document.createElement('td');
    tdIdx.innerHTML = '<span class="badge ' + (isCurrent ? 'ok' : 'off') + '">#' + (idx + 1) + '</span>';

    const tdPlate = document.createElement('td');
    tdPlate.innerHTML = '<code>' + escapeHtml(plate) + '</code>' + (isCurrent ? ' <span style="font-size:.72rem;color:var(--success)">[IN VIEW]</span>' : '');

    const tdAct = document.createElement('td');
    tdAct.style.textAlign = 'right';
    const btn = document.createElement('button');
    btn.className = 'danger';
    btn.style.padding = '3px 8px';
    btn.style.fontSize = '.75rem';
    btn.textContent = 'Delete';
    btn.onclick = () => deletePlate(plate);
    tdAct.appendChild(btn);

    tr.appendChild(tdIdx);
    tr.appendChild(tdPlate);
    tr.appendChild(tdAct);
    tbody.appendChild(tr);
  });
}

function escapeHtml(str) {
  return str.replace(/[&<>'"]/g, tag => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' }[tag] || tag));
}

async function addPlate() {
  const p = document.getElementById('pinput').value.trim().toUpperCase();
  if (p.length < 3) {
    showToast('Plate needs at least 3 alphanumeric characters');
    return;
  }
  await fetch('/api/add?plate=' + encodeURIComponent(p), { method: 'POST' });
  document.getElementById('pinput').value = '';
  showToast('Added plate "' + p + '"');
  refreshStatus();
}

function useLastRead() {
  if (lastDetectedText) {
    document.getElementById('pinput').value = lastDetectedText;
  }
}

async function quickAddCurrentPlate() {
  if (lastDetectedText) {
    await fetch('/api/add?plate=' + encodeURIComponent(lastDetectedText), { method: 'POST' });
    showToast('Authorized plate "' + lastDetectedText + '"');
    refreshStatus();
  }
}

async function deletePlate(plate) {
  if (confirm('Delete authorized plate "' + plate + '"?')) {
    await fetch('/api/delete?plate=' + encodeURIComponent(plate), { method: 'POST' });
    showToast('Deleted plate "' + plate + '"');
    refreshStatus();
  }
}

async function clearAllPlates() {
  if (confirm('Delete ALL authorized car plates from Flash memory?')) {
    await fetch('/api/clear', { method: 'POST' });
    showToast('All plates cleared');
    refreshStatus();
  }
}

async function saveSettings() {
  const q = new URLSearchParams({
    mode: getSelectedMode(),
    armed: document.getElementById('cfg-armed').checked ? 1 : 0,
    minc: document.getElementById('cfg-minc').value,
    confn: document.getElementById('cfg-confn').value,
    pol: document.getElementById('cfg-pol').value,
    fuzzy: document.getElementById('cfg-fuzzy').checked ? 1 : 0,
    flash: document.getElementById('flash-slider').value,
    hmir: document.getElementById('cfg-hmir').checked ? 1 : 0,
    vflip: document.getElementById('cfg-vflip').checked ? 1 : 0
  });
  await fetch('/api/config?' + q.toString(), { method: 'POST' });
  showToast('Settings saved to NVS Flash');
  cfgLoaded = false;
  refreshStatus();
}

async function testGate(ok) {
  await fetch('/api/test?ok=' + ok, { method: 'POST' });
  showToast(ok ? 'Triggered Gate Opening (Authorized Car)' : 'Triggered Intruder Siren Alert');
}

// Start live systems
initCamera();
refreshStatus();
setInterval(refreshStatus, 800);
</script>
</body>
</html>)HTML";
  server.send_P(200, "text/html", HTML);
}

void handlePreview() {
  camHwHandlePreviewRequest(server);
}

void handleStatus() {
  PlateStatusSnapshot snap;
  std::vector<String> platesCopy;
  PlateConfig cfgCopy;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    snap = g_status;
    platesCopy = g_plates;
    cfgCopy = g_applyCfgPending ? g_pendingCfg : g_cfg;
    xSemaphoreGive(stateMutex);
  }

  String json = "{";
  json += "\"cameraReady\":" + String(snap.cameraReady ? "true" : "false") + ",";
  json += "\"ocrReady\":" + String(snap.ocrReady ? "true" : "false") + ",";
  json += "\"codeReady\":" + String(snap.codeReady ? "true" : "false") + ",";
  json += "\"hubConnected\":" + String(camNetHubConnected() ? "true" : "false") + ",";
  json += "\"fallbackAp\":" + String(camNetFallbackApActive() ? "true" : "false") + ",";
  json += "\"ip\":\"" + camNetIpString() + "\",";
  json += "\"streamPort\":" + String(STREAM_PORT) + ",";
  json += "\"plateLocalized\":" + String(snap.plateLocalized ? "true" : "false") + ",";
  json += "\"validRead\":" + String(snap.validRead ? "true" : "false") + ",";
  json += "\"lastResult\":\"" + smartHomeJsonEscape(snap.lastResult) + "\",";
  json += "\"lastSource\":\"" + smartHomeJsonEscape(snap.lastSource) + "\",";
  json += "\"lastReadText\":\"" + smartHomeJsonEscape(snap.lastReadText) + "\",";
  json += "\"lastMatchedPlate\":\"" + smartHomeJsonEscape(snap.lastMatchedPlate) + "\",";
  json += "\"lastConf\":" + String(snap.lastConf) + ",";
  json += "\"lastCharCount\":" + String(snap.lastCharCount) + ",";
  json += "\"processMs\":" + String(snap.processMs) + ",";
  json += "\"fps\":" + String(snap.fps, 1) + ",";
  json += "\"frameCount\":" + String(snap.frameCount) + ",";
  json += "\"plates\":[";
  for (size_t i = 0; i < platesCopy.size(); ++i) {
    if (i > 0) json += ",";
    json += "\"" + smartHomeJsonEscape(platesCopy[i].c_str()) + "\"";
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

  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
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
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    PlateConfig next = g_applyCfgPending ? g_pendingCfg : g_cfg;
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
  bool sent = ok ? camNetSendEvent(SmartHomeNet::CAM_EVENT_RECOGNIZED, 0, 100, "TEST123")
                 : camNetSendEvent(SmartHomeNet::CAM_EVENT_UNKNOWN, -1, 0, "BAD999");
  server.send(200, "application/json", sent ? "{\"sent\":true}" : "{\"sent\":false}");
}

} // namespace

void setup() {
  camHwInitLeds();

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Smart Home Camera 2: Plate OCR + QR/Barcode Live (HW-297) ===");

  stateMutex = xSemaphoreCreateMutex();
  camHwInitPreview();

  loadFromNvs(g_plates, g_cfg);
  bool ocrOk = plateOcrInit();
  bool codeOk = plateCodeInit();
  bool camOk = camHwInitCamera(PIXFORMAT_GRAYSCALE,
                               FRAMESIZE_QVGA,
                               12,
                               g_cfg.hmirror,
                               g_cfg.vflip,
                               1);
  if (camOk) {
    camHwSetFlashLed(g_cfg.flashDuty);
  }

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

  // Dedicated real-time MJPEG streaming task on port 81
  camHwStartStreamServer(STREAM_PORT);

  // HTTP Web Server routes on port 80
  server.on("/", HTTP_GET, handleRoot);
  server.on("/preview.jpg", HTTP_GET, handlePreview);
  server.on("/stream", HTTP_GET, []() {
    String host = server.hostHeader();
    int colon = host.indexOf(':');
    if (colon > 0) host = host.substring(0, colon);
    server.sendHeader("Location", "http://" + host + ":81/stream");
    server.send(302, "text/plain", "Redirecting to :81/stream");
  });
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
                            1); // Pin to Core 1 (IDLE1 WDT is disabled, Core 0 handles Wi-Fi + Stream)
  }
}

void loop() {
  camNetLoop();
  server.handleClient();
  delay(10);
}
