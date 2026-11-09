#include "plate_code.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

extern "C" {
#include "quirc/quirc.h"
}

#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

namespace {

constexpr int MAX_BARCODE_SCAN_LEN = 640;
constexpr int MAX_BARCODE_RUNS = 240;

struct BarcodeScratch {
  uint8_t colBuf[MAX_BARCODE_SCAN_LEN];
  uint8_t localMin[MAX_BARCODE_SCAN_LEN];
  uint8_t localMax[MAX_BARCODE_SCAN_LEN];
  float diff[MAX_BARCODE_SCAN_LEN];
  float runW[MAX_BARCODE_RUNS];
  uint8_t runIsBar[MAX_BARCODE_RUNS];
  float runStart[MAX_BARCODE_RUNS];
};

struct quirc *s_quirc = nullptr;
struct quirc_code *s_qrCode = nullptr;
struct quirc_data *s_qrData = nullptr;
BarcodeScratch *s_barcodeScratch = nullptr;
int s_quircW = 0;
int s_quircH = 0;
bool s_codeReady = false;

void *codeAllocPsram(size_t size) {
#ifdef ESP_PLATFORM
  void *ptr = heap_caps_calloc(1, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ptr) return ptr;
#endif
  return calloc(1, size);
}

// ---------------------------------------------------------------------------
// Payload parsing & normalization (matches plateNormalizeString rules)
// ---------------------------------------------------------------------------

bool startsWithIgnoreCase(const char *s, const char *prefix) {
  size_t i = 0;
  while (prefix[i] != '\0') {
    if (s[i] == '\0') return false;
    if (toupper(static_cast<unsigned char>(s[i])) !=
        toupper(static_cast<unsigned char>(prefix[i]))) {
      return false;
    }
    ++i;
  }
  return true;
}

void extractRawAndPlateText(const char *rawBytes,
                            int rawLen,
                            char outText[16],
                            char outRaw[32]) {
  memset(outText, 0, 16);
  memset(outRaw, 0, 32);
  if (!rawBytes || rawLen <= 0) return;

  int start = 0;
  while (start < rawLen &&
         isspace(static_cast<unsigned char>(rawBytes[start]))) {
    ++start;
  }
  int end = rawLen;
  while (end > start &&
         (isspace(static_cast<unsigned char>(rawBytes[end - 1])) ||
          rawBytes[end - 1] == '\0')) {
    --end;
  }
  if (end <= start) return;

  int copyLen = end - start;
  if (copyLen > 31) copyLen = 31;
  for (int i = 0; i < copyLen; ++i) {
    char c = rawBytes[start + i];
    outRaw[i] = (static_cast<unsigned char>(c) >= 0x20 &&
                 static_cast<unsigned char>(c) < 0x7F)
                    ? c
                    : ' ';
  }
  outRaw[copyLen] = '\0';

  // Strip optional key prefixes such as "PLATE:ABC1234" or "CAR=ABC-1234"
  const char *scan = outRaw;
  static const char *const kPrefixes[] = {
      "PLATE:", "PLATE=", "CAR:", "CAR=", "LIC:", "LIC=",
      "TAG:", "TAG=", "REG:", "REG=", "ID:", "ID="};
  for (const char *pfx : kPrefixes) {
    if (startsWithIgnoreCase(scan, pfx)) {
      const char *candidate = scan + strlen(pfx);
      int alnumCount = 0;
      for (size_t i = 0; candidate[i] != '\0'; ++i) {
        if (isalnum(static_cast<unsigned char>(candidate[i]))) {
          ++alnumCount;
        }
      }
      if (alnumCount >= 3) {
        scan = candidate;
      }
      break;
    }
  }

  size_t outLen = 0;
  for (size_t i = 0; scan[i] != '\0' && outLen < 12; ++i) {
    unsigned char ch = static_cast<unsigned char>(scan[i]);
    if (isalnum(ch)) {
      outText[outLen++] = static_cast<char>(toupper(ch));
    }
  }
  outText[outLen] = '\0';
}

// ---------------------------------------------------------------------------
// ISO/IEC 15417 Code 128 Table (0..105: 3 bars + 3 spaces, sum = 11)
// Stop (106): {2, 3, 3, 1, 1, 1, 2} (4 bars + 3 spaces, sum = 13)
// ---------------------------------------------------------------------------
const uint8_t C128_PATTERNS[106][6] = {
    {2, 1, 2, 2, 2, 2}, {2, 2, 2, 1, 2, 2}, {2, 2, 2, 2, 2, 1}, {1, 2, 1, 2, 2, 3}, {1, 2, 1, 3, 2, 2},
    {1, 3, 1, 2, 2, 2}, {1, 2, 2, 2, 1, 3}, {1, 2, 2, 3, 1, 2}, {1, 3, 2, 2, 1, 2}, {2, 2, 1, 2, 1, 3},
    {2, 2, 1, 3, 1, 2}, {2, 3, 1, 2, 1, 2}, {1, 1, 2, 2, 3, 2}, {1, 2, 2, 1, 3, 2}, {1, 2, 2, 2, 3, 1},
    {1, 1, 3, 2, 2, 2}, {1, 2, 3, 1, 2, 2}, {1, 2, 3, 2, 2, 1}, {2, 2, 3, 2, 1, 1}, {2, 2, 1, 1, 3, 2},
    {2, 2, 1, 2, 3, 1}, {2, 1, 3, 2, 1, 2}, {2, 2, 3, 1, 1, 2}, {3, 1, 2, 1, 3, 1}, {3, 1, 1, 2, 2, 2},
    {3, 2, 1, 1, 2, 2}, {3, 2, 1, 2, 2, 1}, {3, 1, 2, 2, 1, 2}, {3, 2, 2, 1, 1, 2}, {3, 2, 2, 2, 1, 1},
    {2, 1, 2, 1, 2, 3}, {2, 1, 2, 3, 2, 1}, {2, 3, 2, 1, 2, 1}, {1, 1, 1, 3, 2, 3}, {1, 3, 1, 1, 2, 3},
    {1, 3, 1, 3, 2, 1}, {1, 1, 2, 3, 1, 3}, {1, 3, 2, 1, 1, 3}, {1, 3, 2, 3, 1, 1}, {2, 1, 1, 3, 1, 3},
    {2, 3, 1, 1, 1, 3}, {2, 3, 1, 3, 1, 1}, {1, 1, 2, 1, 3, 3}, {1, 1, 2, 3, 3, 1}, {1, 3, 2, 1, 3, 1},
    {1, 1, 3, 1, 2, 3}, {1, 1, 3, 3, 2, 1}, {1, 3, 3, 1, 2, 1}, {3, 1, 3, 1, 2, 1}, {2, 1, 1, 3, 3, 1},
    {2, 3, 1, 1, 3, 1}, {2, 1, 3, 1, 1, 3}, {2, 1, 3, 3, 1, 1}, {2, 1, 3, 1, 3, 1}, {3, 1, 1, 1, 2, 3},
    {3, 1, 1, 3, 2, 1}, {3, 3, 1, 1, 2, 1}, {3, 1, 2, 1, 1, 3}, {3, 1, 2, 3, 1, 1}, {3, 3, 2, 1, 1, 1},
    {3, 1, 4, 1, 1, 1}, {2, 2, 1, 4, 1, 1}, {4, 3, 1, 1, 1, 1}, {1, 1, 1, 2, 2, 4}, {1, 1, 1, 4, 2, 2},
    {1, 2, 1, 1, 2, 4}, {1, 2, 1, 4, 2, 1}, {1, 4, 1, 1, 2, 2}, {1, 4, 1, 2, 2, 1}, {1, 1, 2, 2, 1, 4},
    {1, 1, 2, 4, 1, 2}, {1, 2, 2, 1, 1, 4}, {1, 2, 2, 4, 1, 1}, {1, 4, 2, 1, 1, 2}, {1, 4, 2, 2, 1, 1},
    {2, 4, 1, 2, 1, 1}, {2, 2, 1, 1, 1, 4}, {4, 1, 3, 1, 1, 1}, {2, 4, 1, 1, 1, 2}, {1, 3, 4, 1, 1, 1},
    {1, 1, 1, 2, 4, 2}, {1, 2, 1, 1, 4, 2}, {1, 2, 1, 2, 4, 1}, {1, 1, 4, 2, 1, 2}, {1, 2, 4, 1, 1, 2},
    {1, 2, 4, 2, 1, 1}, {4, 1, 1, 2, 1, 2}, {4, 2, 1, 1, 1, 2}, {4, 2, 1, 2, 1, 1}, {2, 1, 2, 1, 4, 1},
    {2, 1, 4, 1, 2, 1}, {4, 1, 2, 1, 2, 1}, {1, 1, 1, 1, 4, 3}, {1, 1, 1, 3, 4, 1}, {1, 3, 1, 1, 4, 1},
    {1, 1, 4, 1, 1, 3}, {1, 1, 4, 3, 1, 1}, {4, 1, 1, 1, 1, 3}, {4, 1, 1, 3, 1, 1}, {1, 1, 3, 1, 4, 1},
    {1, 1, 4, 1, 3, 1}, {3, 1, 1, 1, 4, 1}, {4, 1, 1, 1, 3, 1},
    {2, 1, 1, 4, 1, 2}, // 103: Start A
    {2, 1, 1, 2, 1, 4}, // 104: Start B
    {2, 1, 1, 2, 3, 2}  // 105: Start C
};

int matchCode128Symbol(const float w[6], float *outMod) {
  float sum = w[0] + w[1] + w[2] + w[3] + w[4] + w[5];
  if (sum < 7.0f) return -1;

  float mod = sum / 11.0f;
  float invMod = 1.0f / mod;

  // Adjacent bar+space edge-to-edge sums cancel uniform edge blur / ink spread
  float e1 = (w[0] + w[1]) * invMod;
  float e2 = (w[1] + w[2]) * invMod;
  float e3 = (w[2] + w[3]) * invMod;
  float e4 = (w[3] + w[4]) * invMod;

  float wn[6];
  for (int i = 0; i < 6; ++i) wn[i] = w[i] * invMod;

  int bestSym = -1;
  float bestScore = 1e9f;

  for (int sym = 0; sym < 106; ++sym) {
    const uint8_t *p = C128_PATTERNS[sym];
    float d1 = fabsf(e1 - static_cast<float>(p[0] + p[1]));
    if (d1 > 0.72f) continue;
    float d2 = fabsf(e2 - static_cast<float>(p[1] + p[2]));
    if (d2 > 0.72f) continue;
    float d3 = fabsf(e3 - static_cast<float>(p[2] + p[3]));
    if (d3 > 0.72f) continue;
    float d4 = fabsf(e4 - static_cast<float>(p[3] + p[4]));
    if (d4 > 0.72f) continue;

    bool elemOk = true;
    float elemErr = 0.0f;
    for (int i = 0; i < 6; ++i) {
      float de = fabsf(wn[i] - static_cast<float>(p[i]));
      if (de > 1.50f) {
        elemOk = false;
        break;
      }
      elemErr += de;
    }
    if (!elemOk) continue;

    float score = (d1 + d2 + d3 + d4) * 2.0f + elemErr * 0.15f;
    if (score < bestScore) {
      bestScore = score;
      bestSym = sym;
    }
  }

  if (bestSym >= 0 && bestScore > 2.6f) return -1;
  if (outMod) *outMod = mod;
  return bestSym;
}

bool isCode128Stop(const float w[7], float *outMod) {
  static const uint8_t STOP_PAT[7] = {2, 3, 3, 1, 1, 1, 2};
  float sum = w[0] + w[1] + w[2] + w[3] + w[4] + w[5] + w[6];
  if (sum < 8.5f) return false;

  float mod = sum / 13.0f;
  float invMod = 1.0f / mod;

  for (int i = 0; i < 6; ++i) {
    float pairMeas = (w[i] + w[i + 1]) * invMod;
    float pairExp = static_cast<float>(STOP_PAT[i] + STOP_PAT[i + 1]);
    if (fabsf(pairMeas - pairExp) > 0.75f) return false;
  }
  for (int i = 0; i < 7; ++i) {
    if (fabsf(w[i] * invMod - static_cast<float>(STOP_PAT[i])) > 0.95f) return false;
  }
  if (outMod) *outMod = mod;
  return true;
}

bool decodeCode128Sequence(const int *syms,
                           int count,
                           char *rawOut,
                           int rawCap) {
  if (count < 3) return false; // Start + >=1 payload + Checksum

  int chk = syms[0];
  for (int i = 1; i < count - 1; ++i) {
    chk = (chk + i * syms[i]) % 103;
  }
  if (chk != syms[count - 1]) return false;

  int codeSet = 0; // 0=A, 1=B, 2=C
  if (syms[0] == 103) codeSet = 0;
  else if (syms[0] == 104) codeSet = 1;
  else if (syms[0] == 105) codeSet = 2;
  else return false;

  int len = 0;
  bool shiftNext = false;

  for (int i = 1; i < count - 1; ++i) {
    int s = syms[i];
    if (s < 0 || s > 102) return false;

    int activeSet = codeSet;
    if (shiftNext) {
      activeSet = (codeSet == 0) ? 1 : 0;
      shiftNext = false;
    }

    if (activeSet == 2) {
      if (s <= 99) {
        if (len + 2 < rawCap) {
          rawOut[len++] = static_cast<char>('0' + (s / 10));
          rawOut[len++] = static_cast<char>('0' + (s % 10));
        }
      } else if (s == 100) {
        codeSet = 1;
      } else if (s == 101) {
        codeSet = 0;
      }
      continue;
    }

    if (s <= 95) {
      char ch = '\0';
      if (activeSet == 0) {
        if (s <= 63) ch = static_cast<char>(32 + s);
      } else {
        if (s <= 94) ch = static_cast<char>(32 + s);
      }
      if (ch != '\0' && len + 1 < rawCap) {
        rawOut[len++] = ch;
      }
    } else if (s == 98) {
      shiftNext = true;
    } else if (s == 99) {
      codeSet = 2;
    } else if (s == 100 && activeSet == 0) {
      codeSet = 1;
    } else if (s == 101 && activeSet == 1) {
      codeSet = 0;
    }
  }

  rawOut[len] = '\0';
  return len >= 3;
}

// ---------------------------------------------------------------------------
// Code 39 Table (44 entries, 9-bit wide/narrow pattern b0 s0 b1 s1 b2 s2 b3 s3 b4)
// ---------------------------------------------------------------------------
struct Code39Entry {
  char ch;
  uint16_t mask;
};

const Code39Entry CODE39_TABLE[44] = {
    {'0', 0x034}, {'1', 0x121}, {'2', 0x061}, {'3', 0x160}, {'4', 0x031},
    {'5', 0x130}, {'6', 0x070}, {'7', 0x025}, {'8', 0x124}, {'9', 0x064},
    {'A', 0x109}, {'B', 0x049}, {'C', 0x148}, {'D', 0x019}, {'E', 0x118},
    {'F', 0x058}, {'G', 0x00D}, {'H', 0x10C}, {'I', 0x04C}, {'J', 0x01C},
    {'K', 0x103}, {'L', 0x043}, {'M', 0x142}, {'N', 0x013}, {'O', 0x112},
    {'P', 0x052}, {'Q', 0x007}, {'R', 0x106}, {'S', 0x046}, {'T', 0x016},
    {'U', 0x181}, {'V', 0x0C1}, {'W', 0x1C0}, {'X', 0x091}, {'Y', 0x190},
    {'Z', 0x0D0}, {'-', 0x085}, {'.', 0x184}, {' ', 0x0C4}, {'$', 0x0A8},
    {'/', 0x0A2}, {'+', 0x08A}, {'%', 0x02A}, {'*', 0x094}};

char decodeCode39Symbol(const float w[9], float *outNarrow, float *outWide) {
  int idx[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
  for (int i = 0; i < 8; ++i) {
    for (int j = i + 1; j < 9; ++j) {
      if (w[idx[j]] > w[idx[i]]) {
        std::swap(idx[i], idx[j]);
      }
    }
  }

  float wideSum = w[idx[0]] + w[idx[1]] + w[idx[2]];
  float narrowSum = w[idx[3]] + w[idx[4]] + w[idx[5]] + w[idx[6]] + w[idx[7]] + w[idx[8]];
  float avgWide = wideSum / 3.0f;
  float avgNarrow = narrowSum / 6.0f;
  if (avgNarrow < 0.7f) return '\0';

  float minWide = w[idx[2]];
  float maxNarrow = w[idx[3]];
  float minNarrow = w[idx[8]];
  float maxWide = w[idx[0]];

  if (minWide <= maxNarrow * 1.42f) return '\0';
  float ratio = avgWide / avgNarrow;
  if (ratio < 1.70f || ratio > 3.90f) return '\0';
  if (minNarrow < avgNarrow * 0.45f || maxWide > avgWide * 1.65f) return '\0';

  uint16_t mask = static_cast<uint16_t>((1U << (8 - idx[0])) |
                                        (1U << (8 - idx[1])) |
                                        (1U << (8 - idx[2])));

  for (const auto &entry : CODE39_TABLE) {
    if (entry.mask == mask) {
      if (outNarrow) *outNarrow = avgNarrow;
      if (outWide) *outWide = avgWide;
      return entry.ch;
    }
  }
  return '\0';
}

// ---------------------------------------------------------------------------
// 1D Sub-Pixel Edge Run Extractor
// ---------------------------------------------------------------------------
int extractScanlineRuns(const uint8_t *line,
                        int len,
                        BarcodeScratch &scratch,
                        int maxRuns) {
  if (!line || len < 32 || len > MAX_BARCODE_SCAN_LEN) return 0;

  constexpr int WIN = 16;
  uint8_t *localMin = scratch.localMin;
  uint8_t *localMax = scratch.localMax;
  float *diff = scratch.diff;
  float *runW = scratch.runW;
  uint8_t *runIsBar = scratch.runIsBar;
  float *runStartPos = scratch.runStart;

  for (int x = 0; x < len; ++x) {
    int x0 = std::max(0, x - WIN);
    int x1 = std::min(len - 1, x + WIN);
    uint8_t lo = 255;
    uint8_t hi = 0;
    for (int k = x0; k <= x1; ++k) {
      uint8_t v = line[k];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    localMin[x] = lo;
    localMax[x] = hi;
  }

  for (int x = 0; x < len; ++x) {
    int contrast = static_cast<int>(localMax[x]) - static_cast<int>(localMin[x]);
    if (contrast < 30) {
      // Quiet or flat region behaves as bright space
      diff[x] = -24.0f;
    } else {
      float thresh = 0.5f * (static_cast<float>(localMin[x]) + static_cast<float>(localMax[x]));
      diff[x] = thresh - static_cast<float>(line[x]); // >0 inside dark bar
    }
  }

  int count = 0;
  bool curBar = (diff[0] > 0.0f);
  float curStart = 0.0f;

  for (int x = 0; x < len - 1; ++x) {
    bool nextBar = (diff[x + 1] > 0.0f);
    if (nextBar != curBar) {
      float denom = diff[x] - diff[x + 1];
      float cross = static_cast<float>(x) + 0.5f;
      if (fabsf(denom) > 1e-4f) {
        float frac = diff[x] / denom;
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        cross = static_cast<float>(x) + frac;
      }

      if (count < maxRuns && (cross - curStart) > 0.35f) {
        runW[count] = cross - curStart;
        runIsBar[count] = curBar ? 1 : 0;
        runStartPos[count] = curStart;
        ++count;
      }
      curStart = cross;
      curBar = nextBar;
    }
  }

  if (count < maxRuns) {
    float endPos = static_cast<float>(len);
    if (endPos - curStart > 0.35f) {
      runW[count] = endPos - curStart;
      runIsBar[count] = curBar ? 1 : 0;
      runStartPos[count] = curStart;
      ++count;
    }
  }

  return count;
}

inline int mapRunIdx(int logicalIdx, int nRuns, bool reverse) {
  return reverse ? (nRuns - 1 - logicalIdx) : logicalIdx;
}

inline void copyRunWindow(const float *runW,
                          int nRuns,
                          int logicalStart,
                          int count,
                          bool reverse,
                          float *dst) {
  for (int i = 0; i < count; ++i) {
    dst[i] = runW[mapRunIdx(logicalStart + i, nRuns, reverse)];
  }
}

bool tryDecodeCode128FromRuns(const BarcodeScratch &scratch,
                              int nRuns,
                              bool reverse,
                              char *rawOut,
                              int rawCap,
                              float *outMinPos,
                              float *outMaxPos) {
  const float *runW = scratch.runW;
  const uint8_t *runIsBar = scratch.runIsBar;
  const float *runStartPos = scratch.runStart;
  int syms[48];
  float win[7];

  for (int s = 1; s + 18 <= nRuns; ++s) {
    if (!runIsBar[mapRunIdx(s, nRuns, reverse)]) continue;

    float mod = 0.0f;
    copyRunWindow(runW, nRuns, s, 6, reverse, win);
    int startSym = matchCode128Symbol(win, &mod);
    if (startSym < 103 || startSym > 105) continue;

    if (runW[mapRunIdx(s - 1, nRuns, reverse)] < 1.5f * mod) continue;

    syms[0] = startSym;
    int symCount = 1;
    int pos = s + 6;
    bool hitStop = false;

    while (pos + 6 <= nRuns && symCount < 48) {
      if (pos + 7 <= nRuns) {
        float stopMod = 0.0f;
        copyRunWindow(runW, nRuns, pos, 7, reverse, win);
        if (isCode128Stop(win, &stopMod)) {
          bool quietAfter = (pos + 7 >= nRuns) ||
                            (runW[mapRunIdx(pos + 7, nRuns, reverse)] >= 1.8f * stopMod);
          if (quietAfter && decodeCode128Sequence(syms, symCount, rawOut, rawCap)) {
            int idxA = mapRunIdx(s, nRuns, reverse);
            int idxB = mapRunIdx(pos + 6, nRuns, reverse);
            int leftIdx = reverse ? idxB : idxA;
            int rightIdx = reverse ? idxA : idxB;
            if (outMinPos) *outMinPos = runStartPos[leftIdx];
            if (outMaxPos) *outMaxPos = runStartPos[rightIdx] + runW[rightIdx];
            hitStop = true;
            break;
          }
        }
      }

      if (pos + 6 > nRuns) break;
      float symMod = 0.0f;
      copyRunWindow(runW, nRuns, pos, 6, reverse, win);
      int nextSym = matchCode128Symbol(win, &symMod);
      if (nextSym < 0 || nextSym > 102) break;
      if (symMod < mod * 0.65f || symMod > mod * 1.45f) break;

      syms[symCount++] = nextSym;
      pos += 6;
    }

    if (hitStop) return true;
  }
  return false;
}

bool tryDecodeCode39FromRuns(const BarcodeScratch &scratch,
                             int nRuns,
                             bool reverse,
                             char *rawOut,
                             int rawCap,
                             float *outMinPos,
                             float *outMaxPos) {
  const float *runW = scratch.runW;
  const uint8_t *runIsBar = scratch.runIsBar;
  const float *runStartPos = scratch.runStart;
  float win[9];

  for (int s = 1; s + 28 <= nRuns; ++s) {
    if (!runIsBar[mapRunIdx(s, nRuns, reverse)]) continue;

    float narrow0 = 0.0f, wide0 = 0.0f;
    copyRunWindow(runW, nRuns, s, 9, reverse, win);
    if (decodeCode39Symbol(win, &narrow0, &wide0) != '*') continue;
    if (runW[mapRunIdx(s - 1, nRuns, reverse)] < 1.5f * narrow0) continue;

    int len = 0;
    int pos = s + 9;
    float refNarrow = narrow0;
    bool closed = false;

    while (pos + 9 < nRuns && len + 1 < rawCap) {
      if (runIsBar[mapRunIdx(pos, nRuns, reverse)]) break;
      float gap = runW[mapRunIdx(pos, nRuns, reverse)];
      if (gap < refNarrow * 0.40f || gap > refNarrow * 2.4f) break;

      int charPos = pos + 1;
      if (charPos + 9 > nRuns) break;

      float curN = 0.0f, curW = 0.0f;
      copyRunWindow(runW, nRuns, charPos, 9, reverse, win);
      char ch = decodeCode39Symbol(win, &curN, &curW);
      if (ch == '\0') break;
      if (curN < refNarrow * 0.60f || curN > refNarrow * 1.60f) break;

      if (ch == '*') {
        bool quietAfter = (charPos + 9 >= nRuns) ||
                          (runW[mapRunIdx(charPos + 9, nRuns, reverse)] >= 1.8f * curN);
        if (quietAfter && len >= 3) {
          rawOut[len] = '\0';
          int idxA = mapRunIdx(s, nRuns, reverse);
          int idxB = mapRunIdx(charPos + 8, nRuns, reverse);
          int leftIdx = reverse ? idxB : idxA;
          int rightIdx = reverse ? idxA : idxB;
          if (outMinPos) *outMinPos = runStartPos[leftIdx];
          if (outMaxPos) *outMaxPos = runStartPos[rightIdx] + runW[rightIdx];
          closed = true;
        }
        break;
      }

      rawOut[len++] = ch;
      refNarrow = 0.5f * (refNarrow + curN);
      pos = charPos + 9;
    }

    if (closed) return true;
  }
  return false;
}

struct BarcodeCandidate {
  uint8_t symbology;
  char text[16];
  char raw[32];
  int minX, minY, maxX, maxY;
  int hits;
};

void analyzeBarcodeLine(const uint8_t *line,
                        int len,
                        int coordFixed,
                        int imgW,
                        int imgH,
                        bool horizontal,
                        BarcodeScratch &scratch,
                        std::vector<BarcodeCandidate> &c39Candidates,
                        PlateCodeResult &out,
                        bool &foundStrong) {
  int nRuns = extractScanlineRuns(line, len, scratch, MAX_BARCODE_RUNS);
  if (nRuns < 18) return;

  // 1) Try Code 128 first (has mandatory Modulo-103 checksum + Start + Stop)
  char rawBuf[48];
  float p0 = 0.0f, p1 = 0.0f;
  bool c128Ok = tryDecodeCode128FromRuns(scratch, nRuns, false, rawBuf, sizeof(rawBuf), &p0, &p1);
  if (!c128Ok) {
    c128Ok = tryDecodeCode128FromRuns(scratch, nRuns, true, rawBuf, sizeof(rawBuf), &p0, &p1);
  }
  if (c128Ok) {
    char normText[16], trimmedRaw[32];
    extractRawAndPlateText(rawBuf, static_cast<int>(strlen(rawBuf)), normText, trimmedRaw);
    if (strlen(normText) >= 3) {
      out.detected = true;
      out.validRead = true;
      out.symbology = PLATE_SYM_CODE128;
      out.confidence = 100;
      snprintf(out.text, sizeof(out.text), "%s", normText);
      snprintf(out.rawPayload, sizeof(out.rawPayload), "%s", trimmedRaw);

      int a = std::max(0, std::min(len - 1, static_cast<int>(lroundf(p0))));
      int b = std::max(0, std::min(len - 1, static_cast<int>(lroundf(p1))));
      int x0 = horizontal ? a : std::max(0, coordFixed - 12);
      int y0 = horizontal ? std::max(0, coordFixed - 12) : a;
      int x1 = horizontal ? b : std::min(imgW - 1, coordFixed + 12);
      int y1 = horizontal ? std::min(imgH - 1, coordFixed + 12) : b;

      out.cornersX[0] = static_cast<int16_t>(x0);
      out.cornersY[0] = static_cast<int16_t>(y0);
      out.cornersX[1] = static_cast<int16_t>(x1);
      out.cornersY[1] = static_cast<int16_t>(y0);
      out.cornersX[2] = static_cast<int16_t>(x1);
      out.cornersY[2] = static_cast<int16_t>(y1);
      out.cornersX[3] = static_cast<int16_t>(x0);
      out.cornersY[3] = static_cast<int16_t>(y1);
      foundStrong = true;
      return;
    }
  }

  // 2) Try Code 39 (accumulate across scanlines; require >= 2 agreeing scanlines)
  bool c39Ok = tryDecodeCode39FromRuns(scratch, nRuns, false, rawBuf, sizeof(rawBuf), &p0, &p1);
  if (!c39Ok) {
    c39Ok = tryDecodeCode39FromRuns(scratch, nRuns, true, rawBuf, sizeof(rawBuf), &p0, &p1);
  }
  if (c39Ok) {
    char normText[16], trimmedRaw[32];
    extractRawAndPlateText(rawBuf, static_cast<int>(strlen(rawBuf)), normText, trimmedRaw);
    if (strlen(normText) >= 3) {
      int a = std::max(0, std::min(len - 1, static_cast<int>(lroundf(p0))));
      int b = std::max(0, std::min(len - 1, static_cast<int>(lroundf(p1))));
      int x0 = horizontal ? a : std::max(0, coordFixed - 8);
      int y0 = horizontal ? std::max(0, coordFixed - 8) : a;
      int x1 = horizontal ? b : std::min(imgW - 1, coordFixed + 8);
      int y1 = horizontal ? std::min(imgH - 1, coordFixed + 8) : b;

      for (auto &cand : c39Candidates) {
        if (strcmp(cand.text, normText) == 0) {
          cand.hits++;
          cand.minX = std::min(cand.minX, x0);
          cand.minY = std::min(cand.minY, y0);
          cand.maxX = std::max(cand.maxX, x1);
          cand.maxY = std::max(cand.maxY, y1);
          return;
        }
      }
      if (c39Candidates.size() < 6) {
        BarcodeCandidate cand{};
        cand.symbology = PLATE_SYM_CODE39;
        snprintf(cand.text, sizeof(cand.text), "%s", normText);
        snprintf(cand.raw, sizeof(cand.raw), "%s", trimmedRaw);
        cand.minX = x0;
        cand.minY = y0;
        cand.maxX = x1;
        cand.maxY = y1;
        cand.hits = 1;
        c39Candidates.push_back(cand);
      }
    }
  }
}

void drawLineGray(uint8_t *gray,
                  int width,
                  int height,
                  int x0,
                  int y0,
                  int x1,
                  int y1,
                  uint8_t color) {
  int dx = abs(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  for (;;) {
    if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) {
      gray[y0 * width + x0] = color;
    }
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void drawThickLineGray(uint8_t *gray,
                       int width,
                       int height,
                       int x0,
                       int y0,
                       int x1,
                       int y1,
                       uint8_t color) {
  drawLineGray(gray, width, height, x0, y0, x1, y1, color);
  drawLineGray(gray, width, height, x0 + 1, y0, x1 + 1, y1, color);
  drawLineGray(gray, width, height, x0, y0 + 1, x1, y1 + 1, color);
}

} // namespace

bool plateCodeInit() {
  if (s_codeReady) return true;

  s_quirc = quirc_new();
  s_qrCode = static_cast<struct quirc_code *>(codeAllocPsram(sizeof(struct quirc_code)));
  s_qrData = static_cast<struct quirc_data *>(codeAllocPsram(sizeof(struct quirc_data)));
  s_barcodeScratch = static_cast<BarcodeScratch *>(codeAllocPsram(sizeof(BarcodeScratch)));

  s_codeReady = (s_quirc != nullptr && s_qrCode != nullptr &&
                 s_qrData != nullptr && s_barcodeScratch != nullptr);
  if (!s_codeReady) {
    if (s_quirc) {
      quirc_destroy(s_quirc);
      s_quirc = nullptr;
    }
    free(s_qrCode);
    s_qrCode = nullptr;
    free(s_qrData);
    s_qrData = nullptr;
    free(s_barcodeScratch);
    s_barcodeScratch = nullptr;
  }
  return s_codeReady;
}

void plateCodeAnalyze(const uint8_t *gray,
                      int width,
                      int height,
                      bool enableQr,
                      bool enableBarcode,
                      PlateCodeResult &out) {
  memset(&out, 0, sizeof(out));
  if (!s_codeReady || !gray || width <= 0 || height <= 0) return;

  // 1) Try QR code decoding via PSRAM-backed quirc
  if (enableQr) {
    if (s_quircW != width || s_quircH != height) {
      if (quirc_resize(s_quirc, width, height) == 0) {
        s_quircW = width;
        s_quircH = height;
      } else {
        s_quircW = 0;
        s_quircH = 0;
      }
    }

    if (s_quircW == width && s_quircH == height) {
      uint8_t *qBuf = quirc_begin(s_quirc, nullptr, nullptr);
      if (qBuf) {
        memcpy(qBuf, gray, static_cast<size_t>(width) * static_cast<size_t>(height));
        quirc_end(s_quirc);

        int qrCount = quirc_count(s_quirc);
        for (int i = 0; i < qrCount; ++i) {
          quirc_extract(s_quirc, i, s_qrCode);

          if (!out.detected) {
            out.detected = true;
            out.symbology = PLATE_SYM_QR;
            for (int c = 0; c < 4; ++c) {
              out.cornersX[c] = static_cast<int16_t>(s_qrCode->corners[c].x);
              out.cornersY[c] = static_cast<int16_t>(s_qrCode->corners[c].y);
            }
          }

          if (quirc_decode(s_qrCode, s_qrData) == QUIRC_SUCCESS &&
              s_qrData->payload_len > 0) {
            char normText[16], trimmedRaw[32];
            extractRawAndPlateText(reinterpret_cast<const char *>(s_qrData->payload),
                                   s_qrData->payload_len,
                                   normText,
                                   trimmedRaw);
            out.detected = true;
            out.symbology = PLATE_SYM_QR;
            for (int c = 0; c < 4; ++c) {
              out.cornersX[c] = static_cast<int16_t>(s_qrCode->corners[c].x);
              out.cornersY[c] = static_cast<int16_t>(s_qrCode->corners[c].y);
            }
            snprintf(out.rawPayload, sizeof(out.rawPayload), "%s", trimmedRaw);
            if (strlen(normText) >= 3) {
              out.validRead = true;
              out.confidence = 100;
              snprintf(out.text, sizeof(out.text), "%s", normText);
              return;
            }
          }
        }
      }
    }
  }

  // 2) Try 1D Barcode decoding (Code 128 + Code 39) across horizontal & vertical scanlines
  if (enableBarcode && width <= MAX_BARCODE_SCAN_LEN && height <= MAX_BARCODE_SCAN_LEN) {
    std::vector<BarcodeCandidate> c39Candidates;
    c39Candidates.reserve(4);
    bool foundStrong = false;

    // Horizontal scanlines
    int stepY = std::max(8, height / 20);
    for (int y = stepY; y < height - stepY && !foundStrong; y += stepY) {
      analyzeBarcodeLine(gray + y * width, width, y, width, height, true,
                         *s_barcodeScratch, c39Candidates, out, foundStrong);
    }
    if (foundStrong) return;

    // Vertical scanlines (for 90-degree rotated barcodes)
    uint8_t *colBuf = s_barcodeScratch->colBuf;
    int stepX = std::max(16, width / 12);
    for (int x = stepX; x < width - stepX && !foundStrong; x += stepX) {
      for (int y = 0; y < height; ++y) {
        colBuf[y] = gray[y * width + x];
      }
      analyzeBarcodeLine(colBuf, height, x, width, height, false,
                         *s_barcodeScratch, c39Candidates, out, foundStrong);
    }
    if (foundStrong) return;

    // Accept Code 39 only if at least 2 scanlines agreed on the exact same payload
    int bestIdx = -1;
    int bestHits = 1;
    for (size_t i = 0; i < c39Candidates.size(); ++i) {
      if (c39Candidates[i].hits > bestHits) {
        bestHits = c39Candidates[i].hits;
        bestIdx = static_cast<int>(i);
      }
    }
    if (bestIdx >= 0) {
      const auto &cand = c39Candidates[bestIdx];
      out.detected = true;
      out.validRead = true;
      out.symbology = PLATE_SYM_CODE39;
      out.confidence = 92;
      snprintf(out.text, sizeof(out.text), "%s", cand.text);
      snprintf(out.rawPayload, sizeof(out.rawPayload), "%s", cand.raw);
      out.cornersX[0] = static_cast<int16_t>(cand.minX);
      out.cornersY[0] = static_cast<int16_t>(cand.minY);
      out.cornersX[1] = static_cast<int16_t>(cand.maxX);
      out.cornersY[1] = static_cast<int16_t>(cand.minY);
      out.cornersX[2] = static_cast<int16_t>(cand.maxX);
      out.cornersY[2] = static_cast<int16_t>(cand.maxY);
      out.cornersX[3] = static_cast<int16_t>(cand.minX);
      out.cornersY[3] = static_cast<int16_t>(cand.maxY);
      return;
    }
  }
}

void plateCodeDrawOverlayGray(uint8_t *gray,
                              int width,
                              int height,
                              const PlateCodeResult &res,
                              bool matched) {
  if (!gray || !res.detected) return;

  uint8_t colOuter = matched ? 255 : 225;
  uint8_t colInner = matched ? 15 : 35;

  for (int i = 0; i < 4; ++i) {
    int j = (i + 1) & 3;
    drawThickLineGray(gray, width, height,
                      res.cornersX[i], res.cornersY[i],
                      res.cornersX[j], res.cornersY[j],
                      colOuter);
    drawLineGray(gray, width, height,
                 res.cornersX[i] - 1, res.cornersY[i] - 1,
                 res.cornersX[j] - 1, res.cornersY[j] - 1,
                 colInner);
  }
}

const char *plateSymbologyName(uint8_t symbology) {
  switch (symbology) {
    case PLATE_SYM_QR:
      return "QR CODE";
    case PLATE_SYM_CODE128:
      return "BARCODE C128";
    case PLATE_SYM_CODE39:
      return "BARCODE C39";
    default:
      return "NONE";
  }
}
