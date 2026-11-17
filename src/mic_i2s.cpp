#include "mic_i2s.h"
#include "audio_voice.h"

#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>
#include <cstdlib>

namespace {

constexpr int SAMPLE_RATE_HZ = 16000;
constexpr size_t DMA_BUF_COUNT = 4;
constexpr size_t DMA_BUF_LEN   = 256;

// DMA read buffer: DMA_BUF_LEN frames * 2 channels (stereo interleaved)
static int32_t s_rawI2sSamples[DMA_BUF_LEN * 2];

constexpr size_t LOOPBACK_MAX_SAMPLES = 32000; // 2.0s buffer at 16 kHz
uint8_t s_loopbackBuf[LOOPBACK_MAX_SAMPLES];
volatile size_t s_loopbackLen = 0;
volatile size_t s_loopbackTarget = 0;
volatile bool s_loopbackRecording = false;

bool s_micReady = false;
volatile uint8_t s_currentRmsPct = 0;
volatile uint8_t s_currentPeakPct = 0;
volatile int8_t  s_currentRmsDb  = -60;
volatile bool    s_vadActive     = false;

VoiceCommandCallback s_cmdCallback = nullptr;
TaskHandle_t s_micTaskHandle = nullptr;

void micReaderTask(void *pvParameters) {
  size_t bytesRead = 0;

  float smoothedRms = 0.0f;
  float peakDecay = 0.0f;
  unsigned long vadActiveUntilMs = 0;

  // DC-blocking filter states (fc ≈ 12.7 Hz at 16 kHz)
  float dcPrevIn = 0.0f;
  float dcPrevOut = 0.0f;

  // Active channel tracker with hysteresis: 0 = Left (L/R grounded), 1 = Right (L/R high or floating)
  int activeChannel = 0;

  for (;;) {
    esp_err_t err = i2s_read(I2S_NUM_0, s_rawI2sSamples, sizeof(s_rawI2sSamples), &bytesRead, pdMS_TO_TICKS(100));
    if (err != ESP_OK || bytesRead == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    size_t totalWords = bytesRead / sizeof(int32_t);
    size_t frameCount = totalWords / 2;
    if (frameCount == 0) continue;

    // Detect which channel has active audio energy across this block
    int64_t magL = 0;
    int64_t magR = 0;
    for (size_t j = 0; j < frameCount; ++j) {
      int32_t l = s_rawI2sSamples[j * 2] >> 16;
      int32_t r = s_rawI2sSamples[j * 2 + 1] >> 16;
      magL += std::labs(l);
      magR += std::labs(r);
    }

    // Auto-select channel with hysteresis so it never flip-flops mid-sample
    if (magR > (magL * 2 + 200)) {
      activeChannel = 1; // Right channel is active (L/R pin wired to 3.3V or floating with pull-up)
    } else if (magL > (magR * 2 + 200)) {
      activeChannel = 0; // Left channel is active (L/R pin grounded)
    }

    int64_t sumSq = 0;
    int32_t maxSample = 0;

    for (size_t j = 0; j < frameCount; ++j) {
      // Pick the active channel sample
      int32_t rawWord = s_rawI2sSamples[j * 2 + activeChannel];

      // INMP441 outputs 24-bit audio MSB-aligned in 32-bit slot:
      // Arithmetic shift >> 16 maps directly to standard signed 16-bit [-32768, 32767]
      int32_t s16 = rawWord >> 16;

      // 1st-order IIR DC-blocking filter (fc ≈ 12.7 Hz)
      float inF = (float)s16;
      float cleanF = inF - dcPrevIn + 0.995f * dcPrevOut;
      dcPrevIn = inF;
      dcPrevOut = cleanF;

      if (std::isnan(cleanF) || std::isinf(cleanF)) {
        cleanF = 0.0f;
        dcPrevOut = 0.0f;
      }

      // Voice digital pre-amplification (+18 dB / 8x gain for clean speech dynamic range)
      float gainedF = cleanF * 8.0f;
      int32_t gainedSample = (int32_t)gainedF;
      if (gainedSample < -32768) gainedSample = -32768;
      if (gainedSample > 32767) gainedSample = 32767;

      sumSq += (int64_t)gainedSample * gainedSample;
      int32_t absG = std::labs(gainedSample);
      if (absG > maxSample) {
        maxSample = absG;
      }

      // Record to loopback buffer if test is active
      if (s_loopbackRecording && s_loopbackLen < s_loopbackTarget) {
        // Linear mapping to [16..240] centered at 128 (prevents clipping the PAM8403 input stage)
        int u8 = 128 + (int)(gainedF / 250.0f);
        if (u8 < 16) u8 = 16;
        if (u8 > 240) u8 = 240;
        s_loopbackBuf[s_loopbackLen++] = (uint8_t)u8;
      }
    }

    // Check if loopback recording completed
    if (s_loopbackRecording && s_loopbackLen >= s_loopbackTarget) {
      s_loopbackRecording = false;
      // Replay recorded audio out of HW-104 speaker asynchronously (non-blocking) on Core 1!
      playRawPcmAsync(s_loopbackBuf, s_loopbackLen, SAMPLE_RATE_HZ);
    }

    float meanSq = (float)sumSq / (float)frameCount;
    float currentRms = std::sqrt(meanSq);

    // Fast attack, smooth decay (EMA filter)
    if (currentRms > smoothedRms) {
      smoothedRms = (0.50f * currentRms) + (0.50f * smoothedRms);
    } else {
      smoothedRms = (0.15f * currentRms) + (0.85f * smoothedRms);
    }

    // Map to 0-100 percentage based on gained voice dynamic range (0 to 12000 RMS)
    int pct = (int)((smoothedRms / 12000.0f) * 100.0f);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    s_currentRmsPct = (uint8_t)pct;

    // Peak meter with decay
    float peakPct = ((float)maxSample / 24000.0f) * 100.0f;
    if (peakPct > 100.0f) peakPct = 100.0f;
    if (peakPct > peakDecay) {
      peakDecay = peakPct;
    } else {
      peakDecay = (0.92f * peakDecay);
    }
    s_currentPeakPct = (uint8_t)peakDecay;

    // Approximate dBFS (referenced to 32768 full scale)
    float ratio = smoothedRms / 32768.0f;
    if (ratio < 0.0001f) ratio = 0.0001f;
    int db = (int)(20.0f * std::log10(ratio));
    if (db < -60) db = -60;
    if (db > 0) db = 0;
    s_currentRmsDb = (int8_t)db;

    // Voice Activity Detection (VAD) thresholding (~8% or -35 dB)
    unsigned long now = millis();
    if (pct >= 8 || db >= -35) {
      vadActiveUntilMs = now + 400; // Hold VAD for 400ms after speech burst
      s_vadActive = true;
    } else {
      if (now >= vadActiveUntilMs) {
        s_vadActive = false;
      }
    }
  }
}

} // namespace

bool initMicI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE_HZ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo capture to support both L and R pin wiring
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SCK_PIN,
    .ws_io_num = I2S_MIC_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD_PIN
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[MIC] ERROR: i2s_driver_install failed (%d)\n", err);
    return false;
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[MIC] ERROR: i2s_set_pin failed (%d)\n", err);
    return false;
  }

  if (!s_micTaskHandle) {
    xTaskCreatePinnedToCore(
        micReaderTask,
        "micReaderTask",
        4096,
        nullptr,
        1, // priority
        &s_micTaskHandle,
        0  // Pin to Core 0 (system/networking core)
    );
  }

  s_micReady = true;
  Serial.printf("[MIC] INMP441 I2S initialized (SCK=%d, WS=%d, SD=%d)\n",
                I2S_MIC_SCK_PIN, I2S_MIC_WS_PIN, I2S_MIC_SD_PIN);
  return true;
}

uint8_t getMicRmsLevel() {
  return s_currentRmsPct;
}

uint8_t getMicPeakLevel() {
  return s_currentPeakPct;
}

int8_t getMicRmsDb() {
  return s_currentRmsDb;
}

bool isVoiceActivityDetected() {
  return s_vadActive;
}

bool isMicOnline() {
  return s_micReady;
}

bool startMicLoopbackTest(uint16_t durationMs) {
  if (!s_micReady || s_loopbackRecording || isVoicePlaying()) return false;
  size_t target = ((size_t)SAMPLE_RATE_HZ * durationMs) / 1000;
  if (target > LOOPBACK_MAX_SAMPLES) {
    target = LOOPBACK_MAX_SAMPLES;
  }
  s_loopbackTarget = target;
  s_loopbackLen = 0;
  s_loopbackRecording = true;
  return true;
}

bool isMicLoopbackRecording() {
  return s_loopbackRecording;
}

uint8_t getMicLoopbackProgressPct() {
  if (s_loopbackTarget == 0) return 0;
  return (uint8_t)((s_loopbackLen * 100) / s_loopbackTarget);
}

void registerVoiceCommandCallback(VoiceCommandCallback cb) {
  s_cmdCallback = cb;
}

void dispatchVoiceCommand(VoiceCommand cmd) {
  if (s_cmdCallback && cmd != CMD_NONE) {
    s_cmdCallback(cmd);
  }
}

const char *voiceCommandName(VoiceCommand cmd) {
  switch (cmd) {
    case CMD_LIGHT_ON:     return "LIGHT_ON";
    case CMD_LIGHT_OFF:    return "LIGHT_OFF";
    case CMD_LIGHT_TOGGLE: return "LIGHT_TOGGLE";
    case CMD_OPEN_DOOR:    return "OPEN_DOOR";
    case CMD_OPEN_GATE:    return "OPEN_GATE";
    case CMD_SIREN_MUTE:   return "SIREN_MUTE";
    default:               return "NONE";
  }
}
