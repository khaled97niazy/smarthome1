#include "cam_hw.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <img_converters.h>
#include <cstring>

namespace {

constexpr int FLASH_LEDC_CHANNEL = 2; // Avoid LEDC channel 0 used by camera XCLK

SemaphoreHandle_t s_previewMutex = nullptr;
uint8_t *s_previewJpg = nullptr;
size_t s_previewJpgLen = 0;
size_t s_previewJpgCap = 0;

} // namespace

void camHwInitLeds() {
  pinMode(CAM_HW_STATUS_LED_PIN, OUTPUT);
  camHwSetStatusLed(false);

  ledcSetup(FLASH_LEDC_CHANNEL, 5000, 8);
  ledcAttachPin(CAM_HW_FLASH_LED_PIN, FLASH_LEDC_CHANNEL);
  camHwSetFlashLed(0);
}

void camHwSetStatusLed(bool on) {
  // AI-Thinker red status LED on GPIO33 is active-LOW
  digitalWrite(CAM_HW_STATUS_LED_PIN, on ? LOW : HIGH);
}

void camHwSetFlashLed(uint8_t duty) {
  ledcWrite(FLASH_LEDC_CHANNEL, duty);
}

void camHwApplySensorSettings(bool hmirror,
                              bool vflip,
                              uint8_t flashDuty,
                              int8_t contrast) {
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, hmirror ? 1 : 0);
    s->set_vflip(s, vflip ? 1 : 0);
    s->set_contrast(s, contrast);
  }
  camHwSetFlashLed(flashDuty);
}

bool camHwInitCamera(pixformat_t pixelFormat,
                     framesize_t frameSize,
                     int jpegQuality,
                     bool hmirror,
                     bool vflip,
                     int8_t contrast) {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_HW_Y2_GPIO_NUM;
  config.pin_d1 = CAM_HW_Y3_GPIO_NUM;
  config.pin_d2 = CAM_HW_Y4_GPIO_NUM;
  config.pin_d3 = CAM_HW_Y5_GPIO_NUM;
  config.pin_d4 = CAM_HW_Y6_GPIO_NUM;
  config.pin_d5 = CAM_HW_Y7_GPIO_NUM;
  config.pin_d6 = CAM_HW_Y8_GPIO_NUM;
  config.pin_d7 = CAM_HW_Y9_GPIO_NUM;
  config.pin_xclk = CAM_HW_XCLK_GPIO_NUM;
  config.pin_pclk = CAM_HW_PCLK_GPIO_NUM;
  config.pin_vsync = CAM_HW_VSYNC_GPIO_NUM;
  config.pin_href = CAM_HW_HREF_GPIO_NUM;
  config.pin_sccb_sda = CAM_HW_SIOD_GPIO_NUM;
  config.pin_sccb_scl = CAM_HW_SIOC_GPIO_NUM;
  config.pin_pwdn = CAM_HW_PWDN_GPIO_NUM;
  config.pin_reset = CAM_HW_RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = pixelFormat;
  config.frame_size = frameSize;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.jpeg_quality = jpegQuality;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: esp_camera_init failed (0x%x)\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_hmirror(s, hmirror ? 1 : 0);
    s->set_vflip(s, vflip ? 1 : 0);
    s->set_contrast(s, contrast);
  }
  return true;
}

void camHwInitPreview() {
  if (!s_previewMutex) {
    s_previewMutex = xSemaphoreCreateMutex();
  }
}

void camHwUpdatePreviewJpeg(const camera_fb_t *fb,
                            pixformat_t srcFormat,
                            uint8_t quality) {
  if (!fb || !fb->buf || fb->len == 0) return;

  uint8_t *jpgBuf = nullptr;
  size_t jpgLen = 0;
  if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height, srcFormat, quality, &jpgBuf, &jpgLen)) {
    return;
  }

  if (s_previewMutex && xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (jpgLen > s_previewJpgCap) {
      uint8_t *newBuf = static_cast<uint8_t *>(ps_realloc(s_previewJpg, jpgLen));
      if (!newBuf) {
        newBuf = static_cast<uint8_t *>(realloc(s_previewJpg, jpgLen));
      }
      if (newBuf) {
        s_previewJpg = newBuf;
        s_previewJpgCap = jpgLen;
      }
    }
    if (s_previewJpg && jpgLen <= s_previewJpgCap) {
      memcpy(s_previewJpg, jpgBuf, jpgLen);
      s_previewJpgLen = jpgLen;
    }
    xSemaphoreGive(s_previewMutex);
  }
  free(jpgBuf);
}

void camHwHandlePreviewRequest(WebServer &server) {
  if (!s_previewMutex || xSemaphoreTake(s_previewMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    server.send(503, "text/plain", "Preview busy");
    return;
  }
  if (!s_previewJpg || s_previewJpgLen == 0) {
    xSemaphoreGive(s_previewMutex);
    server.send(503, "text/plain", "Preview not ready");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(s_previewJpgLen);
  server.send(200, "image/jpeg", "");
  server.sendContent(reinterpret_cast<const char *>(s_previewJpg), s_previewJpgLen);
  xSemaphoreGive(s_previewMutex);
}
