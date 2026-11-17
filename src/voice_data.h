#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pre-Synthesized Natural Spoken Sentences for HW-104 (PAM8403) Amplifier
// Sample Rate: 8,000 Hz, 8-bit Unsigned Mono PCM (Stored in Flash PROGMEM)
// ---------------------------------------------------------------------------

enum VoiceSentenceId : uint8_t {
  VOICE_SENT_NONE = 0,
  VOICE_SENT_LIGHT_ON,
  VOICE_SENT_LIGHT_OFF,
  VOICE_SENT_DOOR_OPEN,
  VOICE_SENT_DOOR_CLOSE,
  VOICE_SENT_GATE_OPEN,
  VOICE_SENT_GATE_CLOSE,
  VOICE_SENT_ACCESS_GRANTED,
  VOICE_SENT_ACCESS_DENIED,
  VOICE_SENT_SMOKE_ALERT,
  VOICE_SENT_FLAME_ALERT,
  VOICE_SENT_CONFIRMED,
  VOICE_SENT_ERROR,
  VOICE_SENT_COUNT
};

constexpr int VOICE_SENTENCE_SAMPLE_RATE_HZ = 8000;

// Look up Flash pointer to sentence PCM data and byte length
const uint8_t* getVoiceSentenceData(VoiceSentenceId id, size_t &outLength);

// Look up human-readable sentence transcription
const char* getVoiceSentenceLabel(VoiceSentenceId id);

// Look up sentence identifier tag for API/JSON
const char* getVoiceSentenceTag(VoiceSentenceId id);

// Look up enum by tag name
VoiceSentenceId getVoiceSentenceIdByTag(const char* tag);
