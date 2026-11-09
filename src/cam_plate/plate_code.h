#pragma once

#include <stddef.h>
#include <stdint.h>

enum PlateCodeSymbology : uint8_t {
  PLATE_SYM_NONE = 0,
  PLATE_SYM_QR = 1,
  PLATE_SYM_CODE128 = 2,
  PLATE_SYM_CODE39 = 3,
};

struct PlateCodeResult {
  bool detected;       // A QR grid or valid symbology region was found
  bool validRead;      // Payload decoded cleanly to >= 3 alphanumeric plate characters
  uint8_t symbology;   // PlateCodeSymbology
  uint8_t confidence;  // 0-100% (100 for ECC/checksum verified QR & Code 128)
  int16_t cornersX[4]; // 4 polygon corners in image pixel coordinates
  int16_t cornersY[4];
  char text[16];       // Normalized uppercase A-Z, 0-9 plate string (<= 12 chars)
  char rawPayload[32]; // Trimmed raw payload snippet for logs/diagnostics
};

bool plateCodeInit();

void plateCodeAnalyze(const uint8_t *gray,
                      int width,
                      int height,
                      bool enableQr,
                      bool enableBarcode,
                      PlateCodeResult &out);

void plateCodeDrawOverlayGray(uint8_t *gray,
                              int width,
                              int height,
                              const PlateCodeResult &res,
                              bool matched);

const char *plateSymbologyName(uint8_t symbology);
