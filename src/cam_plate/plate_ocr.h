#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

enum PlatePolarity : uint8_t {
  PLATE_POLARITY_AUTO = 0,
  PLATE_POLARITY_DARK_ON_LIGHT = 1,
  PLATE_POLARITY_LIGHT_ON_DARK = 2,
};

struct PlateCharBox {
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
  char ch;
  uint8_t confPct;
};

struct PlateOcrResult {
  bool plateLocalized;
  bool validRead;
  uint16_t roiX;
  uint16_t roiY;
  uint16_t roiW;
  uint16_t roiH;
  uint8_t charCount;
  uint8_t avgConfidence;
  uint8_t usedDarkOnLight;
  char text[16];
  PlateCharBox chars[12];
};

bool plateOcrInit();

void plateOcrAnalyze(const uint8_t *gray,
                     int width,
                     int height,
                     PlatePolarity polarity,
                     int minConfidencePct,
                     PlateOcrResult &out);

String plateNormalizeString(const String &raw);

// Checks exact match, or when allowConfusable is true permits at most one
// classic plate-OCR substitution (0<->O, 1<->I, 2<->Z, 5<->S, 6<->G, 8<->B).
bool plateMatchesAuthorized(const char *readPlate,
                            const char *authorizedPlate,
                            bool allowConfusable);

void plateDrawOverlayGray(uint8_t *gray,
                          int width,
                          int height,
                          const PlateOcrResult &res,
                          bool matched);
