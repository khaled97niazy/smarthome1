#include "audio_voice.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <cmath>

namespace {

constexpr int AUDIO_SAMPLE_RATE_HZ = 16000;
constexpr int PWM_CARRIER_FREQ_HZ  = 312500; // Ultra-high 312.5 kHz carrier (clean 80MHz APB division; zero audible whine)
constexpr int PWM_RESOLUTION_BITS  = 8;

enum AudioItemType : uint8_t {
  AUDIO_ITEM_NONE = 0,
  AUDIO_ITEM_CHIME,
  AUDIO_ITEM_SENTENCE,
  AUDIO_ITEM_RAW_PCM
};

struct AudioQueueItem {
  AudioItemType type;
  uint8_t id;
  const uint8_t *rawSamples;
  size_t rawCount;
  int sampleRate;
};

int s_audioPin = 4;
int s_ledcChannel = 6; // Dedicated Timer 3 Channel 6
bool s_initialized = false;
volatile bool s_isPlaying = false;
volatile int s_sirenToneHz = 0;
const char* s_currentAudioLabel = "Idle";

QueueHandle_t s_voiceQueue = nullptr;
TaskHandle_t s_voiceTaskHandle = nullptr;

// Procedural sound synthesis: fundamental + harmonic overtone + ADSR envelope
void renderNote(float freq, float overtoneFreq, float overtoneGain, int durationMs, float attackPct, float decayPct) {
  int totalSamples = (AUDIO_SAMPLE_RATE_HZ * durationMs) / 1000;
  if (totalSamples <= 0) return;

  float dt = 1.0f / (float)AUDIO_SAMPLE_RATE_HZ;
  float phase = 0.0f;
  float phaseOvertone = 0.0f;

  int attackSamples = (int)(totalSamples * attackPct);
  int decaySamples  = (int)(totalSamples * decayPct);
  if (attackSamples < 1) attackSamples = 1;
  if (decaySamples < 1) decaySamples = 1;
  int sustainSamples = totalSamples - attackSamples - decaySamples;
  if (sustainSamples < 0) sustainSamples = 0;

  constexpr uint32_t noteIntervalUs = 1000000 / AUDIO_SAMPLE_RATE_HZ;
  int64_t targetTimeUs = esp_timer_get_time();

  for (int i = 0; i < totalSamples; ++i) {
    if (!s_isPlaying) break;

    // Amplitude envelope: ADSR
    float amp = 1.0f;
    if (i < attackSamples) {
      amp = (float)i / (float)attackSamples;
    } else if (i >= attackSamples + sustainSamples) {
      int decayIdx = i - (attackSamples + sustainSamples);
      amp = 1.0f - ((float)decayIdx / (float)decaySamples);
      if (amp < 0.0f) amp = 0.0f;
    }

    // Synthesis: Fundamental + harmonic overtone
    float wave = 0.0f;
    if (freq > 0.0f) {
      wave = std::sin(phase);
      if (overtoneFreq > 0.0f && overtoneGain > 0.0f) {
        wave += overtoneGain * std::sin(phaseOvertone);
        wave /= (1.0f + overtoneGain);
      }
      phase += 2.0f * (float)M_PI * freq * dt;
      if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;

      if (overtoneFreq > 0.0f) {
        phaseOvertone += 2.0f * (float)M_PI * overtoneFreq * dt;
        if (phaseOvertone >= 2.0f * (float)M_PI) phaseOvertone -= 2.0f * (float)M_PI;
      }
    }

    // Keep cleanly between 32 and 224 for distortion-free PAM8403 input
    float sampleVal = 128.0f + (wave * amp * 90.0f);
    if (sampleVal < 16.0f) sampleVal = 16.0f;
    if (sampleVal > 240.0f) sampleVal = 240.0f;

    ledcWrite(s_ledcChannel, (uint32_t)sampleVal);

    targetTimeUs += noteIntervalUs;
    int64_t diffUs = targetTimeUs - esp_timer_get_time();
    if (diffUs > 0) {
      ets_delay_us(static_cast<uint32_t>(diffUs));
    }
  }
}

void playPromptSequence(VoicePromptId id) {
  // Ensure timer is configured for ultra-high 312.5 kHz PWM carrier
  ledcSetup(s_ledcChannel, PWM_CARRIER_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(s_audioPin, s_ledcChannel);

  // Soft sinusoidal fade-in to center baseline (128) over 20ms to avoid pops
  constexpr int RAMP_STEPS = 64;
  for (int step = 0; step <= RAMP_STEPS; ++step) {
    float angle = ((float)step / (float)RAMP_STEPS) * (float)M_PI * 0.5f;
    uint32_t val = (uint32_t)(std::sin(angle) * 128.0f);
    if (val > 128) val = 128;
    ledcWrite(s_ledcChannel, val);
    ets_delay_us(300);
  }

  switch (id) {
    case VOICE_LIGHT_ON: {
      // Ascending bright chime (C5 -> E5 -> G5)
      renderNote(523.25f, 1046.50f, 0.25f, 120, 0.05f, 0.60f);
      renderNote(659.25f, 1318.51f, 0.25f, 120, 0.05f, 0.60f);
      renderNote(783.99f, 1567.98f, 0.30f, 220, 0.05f, 0.80f);
      break;
    }

    case VOICE_LIGHT_OFF: {
      // Descending soft chime (G5 -> E5 -> C5)
      renderNote(783.99f, 1567.98f, 0.20f, 110, 0.05f, 0.60f);
      renderNote(659.25f, 1318.51f, 0.20f, 110, 0.05f, 0.60f);
      renderNote(523.25f, 1046.50f, 0.25f, 200, 0.05f, 0.80f);
      break;
    }

    case VOICE_DOOR_OPEN: {
      // Pedestrian door resonant chime (E5 -> C5)
      renderNote(659.25f, 1318.51f, 0.35f, 180, 0.04f, 0.65f);
      renderNote(0.0f,   0.0f,     0.00f,  30, 0.00f, 0.00f); // gap
      renderNote(523.25f, 1046.50f, 0.40f, 320, 0.04f, 0.85f);
      break;
    }

    case VOICE_GATE_OPEN: {
      // Automotive gate access fanfare (G4 -> C5 -> E5 -> G5)
      renderNote(392.00f, 784.00f,  0.20f,  90, 0.05f, 0.50f);
      renderNote(523.25f, 1046.50f, 0.20f,  90, 0.05f, 0.50f);
      renderNote(659.25f, 1318.51f, 0.25f, 100, 0.05f, 0.50f);
      renderNote(783.99f, 1567.98f, 0.35f, 280, 0.05f, 0.80f);
      break;
    }

    case VOICE_CONFIRMED: {
      // Sharp modern smart-speaker double-pip (A5 -> D6)
      renderNote(880.00f, 1760.00f, 0.25f,  70, 0.05f, 0.50f);
      renderNote(0.0f,    0.0f,     0.00f,  30, 0.00f, 0.00f);
      renderNote(1174.66f, 2349.32f, 0.25f, 110, 0.05f, 0.70f);
      break;
    }

    case VOICE_ERROR: {
      // Dual low warning buzz (240Hz -> 180Hz)
      renderNote(240.00f, 480.00f, 0.15f, 120, 0.05f, 0.60f);
      renderNote(0.0f,    0.0f,    0.00f,  40, 0.00f, 0.00f);
      renderNote(180.00f, 360.00f, 0.15f, 180, 0.05f, 0.70f);
      break;
    }

    case VOICE_SIREN_PULSE: {
      // 1-second rising & falling siren pulse
      for (float f = 700.0f; f <= 1600.0f; f += 25.0f) {
        renderNote(f, 0.0f, 0.0f, 12, 0.1f, 0.1f);
      }
      for (float f = 1600.0f; f >= 700.0f; f -= 25.0f) {
        renderNote(f, 0.0f, 0.0f, 12, 0.1f, 0.1f);
      }
      break;
    }

    default:
      break;
  }

  // Soft sinusoidal fade-down from 128 to 0 over 20ms to eliminate trailing clicks
  for (int step = RAMP_STEPS; step >= 0; --step) {
    float angle = ((float)step / (float)RAMP_STEPS) * (float)M_PI * 0.5f;
    uint32_t val = (uint32_t)(std::sin(angle) * 128.0f);
    ledcWrite(s_ledcChannel, val);
    ets_delay_us(300);
  }
  ledcWrite(s_ledcChannel, 0);

  // Restore siren tone if siren was active
  if (s_sirenToneHz > 0) {
    ledcWriteTone(s_ledcChannel, s_sirenToneHz);
  }
}

void playSentenceSequence(VoiceSentenceId id) {
  size_t length = 0;
  const uint8_t* pcm = getVoiceSentenceData(id, length);
  if (!pcm || length == 0) return;

  // Ultra-high 312.5 kHz carrier frequency (maximum 8-bit APB speed; eliminates PWM ripple and beat tones)
  ledcSetup(s_ledcChannel, PWM_CARRIER_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(s_audioPin, s_ledcChannel);

  // Soft sinusoidal raised-cosine fade-in from 0 to 128 over 20ms to prevent pops
  constexpr int RAMP_STEPS = 64;
  for (int step = 0; step <= RAMP_STEPS; ++step) {
    float angle = ((float)step / (float)RAMP_STEPS) * (float)M_PI * 0.5f;
    uint32_t val = (uint32_t)(std::sin(angle) * 128.0f);
    if (val > 128) val = 128;
    ledcWrite(s_ledcChannel, val);
    ets_delay_us(300);
  }

  // 4x Linear Interpolation Oversampling:
  // Base 8,000 Hz samples -> 32,000 updates/second (~31.25 µs intervals).
  // Linear interpolation smooths out the 8 kHz steps, completely removing 8 kHz aliasing whine!
  constexpr int OVERSAMPLE_FACTOR = 4;
  constexpr uint32_t SUB_INTERVAL_US = 1000000 / (VOICE_SENTENCE_SAMPLE_RATE_HZ * OVERSAMPLE_FACTOR);

  // Headroom gain scaling (75%) ensures the PAM8403 input is not saturated/clipped
  constexpr int GAIN_NUM = 192; // 192 / 256 = 0.75

  int64_t targetTimeUs = esp_timer_get_time();

  for (size_t i = 0; i < length; ++i) {
    if (!s_isPlaying) break;

    uint8_t currRaw = pgm_read_byte(&pcm[i]);
    uint8_t nextRaw = (i + 1 < length) ? pgm_read_byte(&pcm[i + 1]) : currRaw;

    int currSample = 128 + (((int)currRaw - 128) * GAIN_NUM) / 256;
    int nextSample = 128 + (((int)nextRaw - 128) * GAIN_NUM) / 256;

    for (int sub = 0; sub < OVERSAMPLE_FACTOR; ++sub) {
      int interp = currSample + ((nextSample - currSample) * sub) / OVERSAMPLE_FACTOR;
      if (interp < 8) interp = 8;
      if (interp > 248) interp = 248;

      ledcWrite(s_ledcChannel, static_cast<uint32_t>(interp));

      targetTimeUs += SUB_INTERVAL_US;
      int64_t diffUs = targetTimeUs - esp_timer_get_time();
      if (diffUs > 0) {
        ets_delay_us(static_cast<uint32_t>(diffUs));
      }
    }
  }

  // Soft sinusoidal fade-down from 128 to 0 over 20ms to eliminate trailing clicks
  for (int step = RAMP_STEPS; step >= 0; --step) {
    float angle = ((float)step / (float)RAMP_STEPS) * (float)M_PI * 0.5f;
    uint32_t val = (uint32_t)(std::sin(angle) * 128.0f);
    ledcWrite(s_ledcChannel, val);
    ets_delay_us(300);
  }
  ledcWrite(s_ledcChannel, 0);

  // Restore siren tone if siren was active
  if (s_sirenToneHz > 0) {
    ledcWriteTone(s_ledcChannel, s_sirenToneHz);
  }
}

void audioVoiceWorkerTask(void *pvParameters) {
  AudioQueueItem item;
  for (;;) {
    if (xQueueReceive(s_voiceQueue, &item, portMAX_DELAY) == pdTRUE) {
      if (item.type == AUDIO_ITEM_CHIME && item.id != VOICE_PROMPT_NONE) {
        s_isPlaying = true;
        s_currentAudioLabel = "Chime";
        playPromptSequence(static_cast<VoicePromptId>(item.id));
        s_isPlaying = false;
        s_currentAudioLabel = "Idle";
      } else if (item.type == AUDIO_ITEM_SENTENCE && item.id != VOICE_SENT_NONE) {
        s_isPlaying = true;
        s_currentAudioLabel = getVoiceSentenceLabel(static_cast<VoiceSentenceId>(item.id));
        playSentenceSequence(static_cast<VoiceSentenceId>(item.id));
        s_isPlaying = false;
        s_currentAudioLabel = "Idle";
      } else if (item.type == AUDIO_ITEM_RAW_PCM && item.rawSamples && item.rawCount > 0) {
        s_isPlaying = true;
        s_currentAudioLabel = "Mic Playback";
        playRawPcm(item.rawSamples, item.rawCount, item.sampleRate > 0 ? item.sampleRate : 16000);
        s_isPlaying = false;
        s_currentAudioLabel = "Idle";
      }
    }
  }
}

} // namespace

void initVoiceEngine(int pin, int ledcChannel) {
  s_audioPin = pin;
  s_ledcChannel = ledcChannel;

  pinMode(s_audioPin, OUTPUT);
  digitalWrite(s_audioPin, LOW);

  ledcSetup(s_ledcChannel, PWM_CARRIER_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(s_audioPin, s_ledcChannel);
  ledcWrite(s_ledcChannel, 0);

  if (!s_voiceQueue) {
    s_voiceQueue = xQueueCreate(6, sizeof(AudioQueueItem));
  }

  if (!s_voiceTaskHandle) {
    xTaskCreatePinnedToCore(
        audioVoiceWorkerTask,
        "audioVoiceTask",
        3072,
        nullptr,
        2, // priority
        &s_voiceTaskHandle,
        1  // Pin to Core 1 (app core)
    );
  }

  s_initialized = true;
}

void playVoicePrompt(VoicePromptId id) {
  if (!s_initialized || !s_voiceQueue) return;
  AudioQueueItem item = { AUDIO_ITEM_CHIME, static_cast<uint8_t>(id), nullptr, 0, 0 };
  xQueueSend(s_voiceQueue, &item, 0);
}

void speakSentence(VoiceSentenceId id) {
  if (!s_initialized || !s_voiceQueue || id == VOICE_SENT_NONE) return;
  AudioQueueItem item = { AUDIO_ITEM_SENTENCE, static_cast<uint8_t>(id), nullptr, 0, 0 };
  xQueueSend(s_voiceQueue, &item, 0);
}

void playRawPcmAsync(const uint8_t *samples, size_t count, int sampleRateHz) {
  if (!s_initialized || !s_voiceQueue || !samples || count == 0) return;
  AudioQueueItem item = { AUDIO_ITEM_RAW_PCM, 0, samples, count, sampleRateHz };
  xQueueSend(s_voiceQueue, &item, 0);
}

bool isVoicePlaying() {
  return s_isPlaying;
}

const char* getVoiceCurrentText() {
  return s_isPlaying ? s_currentAudioLabel : "Idle";
}

void stopVoicePrompt() {
  s_isPlaying = false;
  s_currentAudioLabel = "Idle";
  if (s_initialized) {
    ledcWrite(s_ledcChannel, 0);
  }
}

void playRawPcm(const uint8_t *samples, size_t count, int sampleRateHz) {
  if (!s_initialized || !samples || count == 0) return;
  s_isPlaying = true;

  ledcSetup(s_ledcChannel, PWM_CARRIER_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(s_audioPin, s_ledcChannel);

  // Soft sinusoidal raised-cosine fade-in to start sample value over 15ms
  constexpr int RAMP_STEPS = 64;
  uint8_t firstVal = samples[0];
  for (int step = 0; step <= RAMP_STEPS; ++step) {
    float angle = ((float)step / (float)RAMP_STEPS) * (float)M_PI * 0.5f;
    uint32_t val = (uint32_t)(std::sin(angle) * (float)firstVal);
    if (val > firstVal) val = firstVal;
    ledcWrite(s_ledcChannel, val);
    ets_delay_us(250);
  }

  // 2x Linear Interpolation Oversampling for 16 kHz audio (32,000 updates/sec):
  // Eliminates PWM step edges and high-frequency aliasing buzzing
  constexpr int OVERSAMPLE_FACTOR = 2;
  uint32_t subIntervalUs = 1000000 / (sampleRateHz * OVERSAMPLE_FACTOR);
  int64_t targetTimeUs = esp_timer_get_time();

  for (size_t i = 0; i < count; ++i) {
    if (!s_isPlaying) break;

    int currSample = samples[i];
    int nextSample = (i + 1 < count) ? samples[i + 1] : currSample;

    for (int sub = 0; sub < OVERSAMPLE_FACTOR; ++sub) {
      int interp = currSample + ((nextSample - currSample) * sub) / OVERSAMPLE_FACTOR;
      if (interp < 8) interp = 8;
      if (interp > 248) interp = 248;

      ledcWrite(s_ledcChannel, static_cast<uint32_t>(interp));

      targetTimeUs += subIntervalUs;
      int64_t diffUs = targetTimeUs - esp_timer_get_time();
      if (diffUs > 0) {
        ets_delay_us(static_cast<uint32_t>(diffUs));
      }
    }
  }

  // Soft sinusoidal fade-down from last sample to 0 over 15ms to eliminate trailing clicks
  uint8_t lastVal = (count > 0 ? samples[count - 1] : 128);
  for (int step = RAMP_STEPS; step >= 0; --step) {
    float angle = ((float)step / (float)RAMP_STEPS) * (float)M_PI * 0.5f;
    uint32_t val = (uint32_t)(std::sin(angle) * (float)lastVal);
    ledcWrite(s_ledcChannel, val);
    ets_delay_us(250);
  }
  ledcWrite(s_ledcChannel, 0);

  s_isPlaying = false;
  if (s_sirenToneHz > 0) {
    ledcWriteTone(s_ledcChannel, s_sirenToneHz);
  }
}

void setVoiceSirenTone(int freqHz) {
  if (!s_initialized) return;
  s_sirenToneHz = freqHz;
  if (!s_isPlaying) {
    if (freqHz > 0) {
      ledcWriteTone(s_ledcChannel, freqHz);
    } else {
      ledcWriteTone(s_ledcChannel, 0);
      ledcWrite(s_ledcChannel, 0);
    }
  }
}
