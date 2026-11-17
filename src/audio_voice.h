#pragma once

#include <Arduino.h>
#include "voice_data.h"

// ---------------------------------------------------------------------------
// Audio Voice & Verification Prompt Engine for HW-104 (PAM8403) Amplifier
// ---------------------------------------------------------------------------

enum VoicePromptId : uint8_t {
  VOICE_PROMPT_NONE = 0,
  VOICE_LIGHT_ON,     // Ascending bright confirmation chord
  VOICE_LIGHT_OFF,    // Descending soft chime
  VOICE_DOOR_OPEN,    // Pedestrian door resonant double-chime
  VOICE_GATE_OPEN,    // Automotive gate access fanfare
  VOICE_CONFIRMED,    // Crisp double-pip acknowledgment
  VOICE_ERROR,        // Low dual warning tone
  VOICE_SIREN_PULSE   // Swept alarm burst
};

// Initialize the voice engine on the specified pin (e.g. GPIO4) and LEDC channel.
void initVoiceEngine(int pin = 4, int ledcChannel = 5);

// Play a musical synthesizer verification prompt asynchronously (non-blocking).
void playVoicePrompt(VoicePromptId id);

// Play a pre-synthesized spoken sentence asynchronously (non-blocking).
void speakSentence(VoiceSentenceId id);

// Check if an audio prompt or spoken sentence is currently playing.
bool isVoicePlaying();

// Get the label of the currently playing sentence or prompt (or "Idle")
const char* getVoiceCurrentText();

// Stop current voice playback and gracefully ramp down to silence.
void stopVoicePrompt();

// Play raw 8-bit unsigned PCM audio buffer (e.g. from mic loopback test)
void playRawPcm(const uint8_t *samples, size_t count, int sampleRateHz = 16000);

// Play raw 8-bit unsigned PCM audio buffer asynchronously (non-blocking)
void playRawPcmAsync(const uint8_t *samples, size_t count, int sampleRateHz = 16000);

// Optional: Mirror siren frequency to HW-104 during master alarm.
void setVoiceSirenTone(int freqHz);
