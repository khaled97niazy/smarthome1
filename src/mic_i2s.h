#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// INMP441 I2S Digital Microphone Subsystem
// ---------------------------------------------------------------------------

constexpr int I2S_MIC_SCK_PIN = 17; // Bit Clock (BCLK) -> INMP441 SCK
constexpr int I2S_MIC_WS_PIN  = 16; // Word Select (LRCLK) -> INMP441 WS
constexpr int I2S_MIC_SD_PIN  = 23; // Serial Data In -> INMP441 SD

enum VoiceCommand : uint8_t {
  CMD_NONE = 0,
  CMD_LIGHT_ON,
  CMD_LIGHT_OFF,
  CMD_LIGHT_TOGGLE,
  CMD_OPEN_DOOR,
  CMD_OPEN_GATE,
  CMD_SIREN_MUTE
};

typedef void (*VoiceCommandCallback)(VoiceCommand cmd);

// Initialize the I2S microphone driver and background processing task
bool initMicI2S();

// Current RMS audio level (0 - 100%)
uint8_t getMicRmsLevel();

// Current RMS audio level in approximate dBFS (-60 dB to 0 dB)
int8_t getMicRmsDb();

// Current peak audio level (0 - 100%) with decay
uint8_t getMicPeakLevel();

// Returns true if voice speech activity is currently detected (VAD)
bool isVoiceActivityDetected();

// Returns true if I2S hardware driver initialized successfully
bool isMicOnline();

// Start a microphone loopback test: records for durationMs (default 2000ms),
// then immediately replays the recorded audio through the HW-104 speaker on GPIO4!
bool startMicLoopbackTest(uint16_t durationMs = 2000);

// Status of the loopback test
bool isMicLoopbackRecording();
uint8_t getMicLoopbackProgressPct();

// Register a callback triggered when a voice command is recognized
void registerVoiceCommandCallback(VoiceCommandCallback cb);

// Trigger a voice command directly (used by simulation API and speech recognizer)
void dispatchVoiceCommand(VoiceCommand cmd);

// Convert a VoiceCommand enum to a human-readable string
const char *voiceCommandName(VoiceCommand cmd);
