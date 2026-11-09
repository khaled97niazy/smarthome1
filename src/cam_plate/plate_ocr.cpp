#include "plate_ocr.h"

#include <esp_heap_caps.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {

constexpr int BLOCK_SIZE = 4;
constexpr int MAX_ROIS_TO_TRY = 4;
constexpr int MAX_ROI_PIXELS = 320 * 140;

struct BlockCandidate {
  int x0;
  int y0;
  int x1;
  int y1;
  uint32_t score;
};

struct RawBlob {
  int minX;
  int minY;
  int maxX;
  int maxY;
  int area;
};

struct GlyphSpec {
  char ch;
  const char *rows[16];
};

struct PreparedTemplate {
  char ch;
  float vec[256];
};

// 16x16 license-plate character prototypes (centered with 1 px margin, crisp DIN/FE/US-style strokes)
const GlyphSpec RAW_GLYPHS[] = {
  {'0', {
    "................",
    ".....######.....",
    "...###....###...",
    "..###......###..",
    "..##........##..",
    ".###........###.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".###........###.",
    "..##........##..",
    "..###......###..",
    "...###....###...",
    ".....######.....",
    "................",
  }},
  {'0', { // Narrower oval 0
    "................",
    ".....######.....",
    "....##....##....",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "....##....##....",
    ".....######.....",
    "................",
  }},
  {'1', {
    "................",
    ".......###......",
    "......####......",
    ".....#####......",
    "....##..##......",
    "........##......",
    "........##......",
    "........##......",
    "........##......",
    "........##......",
    "........##......",
    "........##......",
    "........##......",
    "........##......",
    ".....########...",
    "................",
  }},
  {'1', { // Sans-serif narrow bar 1
    "................",
    ".......###......",
    "......####......",
    ".....#####......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    "................",
  }},
  {'2', {
    "................",
    "....#######.....",
    "...#########....",
    "..###.....###...",
    "..##.......##...",
    "..........###...",
    ".........###....",
    "........###.....",
    "......####......",
    ".....###........",
    "....###.........",
    "...###..........",
    "..###...........",
    "..###########...",
    "..###########...",
    "................",
  }},
  {'3', {
    "................",
    "...########.....",
    "..##########....",
    "..##......###...",
    "..........###...",
    ".........###....",
    ".....######.....",
    ".....######.....",
    ".........###....",
    "..........###...",
    "..........###...",
    "..##......###...",
    "..###....###....",
    "...########.....",
    "....######......",
    "................",
  }},
  {'4', {
    "................",
    "........####....",
    ".......#####....",
    "......##..##....",
    ".....##...##....",
    "....##....##....",
    "...##.....##....",
    "..##......##....",
    ".##.......##....",
    ".#############..",
    ".#############..",
    "..........##....",
    "..........##....",
    "..........##....",
    "..........##....",
    "................",
  }},
  {'4', { // Open-top 4
    "................",
    "..##......##....",
    "..##......##....",
    "..##......##....",
    "..##......##....",
    "..##......##....",
    "..##......##....",
    "..##......##....",
    "..###########...",
    "..###########...",
    "..........##....",
    "..........##....",
    "..........##....",
    "..........##....",
    "..........##....",
    "................",
  }},
  {'5', {
    "................",
    "..###########...",
    "..###########...",
    "..##............",
    "..##............",
    "..#########.....",
    "..##########....",
    ".........###....",
    "..........###...",
    "..........###...",
    "..........###...",
    "..##.....###....",
    "..###...###.....",
    "...#######......",
    "....#####.......",
    "................",
  }},
  {'6', {
    "................",
    ".....######.....",
    "...###...###....",
    "..##......##....",
    "..##............",
    "..##............",
    "..########......",
    "..##########....",
    "..###....###....",
    "..##......###...",
    "..##.......##...",
    "..###.....###...",
    "...###...###....",
    "....#######.....",
    ".....#####......",
    "................",
  }},
  {'7', {
    "................",
    "..###########...",
    "..###########...",
    ".........###....",
    "........###.....",
    "........##......",
    ".......###......",
    ".......##.......",
    "......###.......",
    "......##........",
    ".....###........",
    ".....##.........",
    "....###.........",
    "....##..........",
    "....##..........",
    "................",
  }},
  {'8', {
    "................",
    "....#######.....",
    "...###...###....",
    "..##......##....",
    "..###....###....",
    "...###..###.....",
    "....######......",
    "....######......",
    "...###..###.....",
    "..###....###....",
    "..##......##....",
    "..##......##....",
    "..###....###....",
    "...########.....",
    "....######......",
    "................",
  }},
  {'9', {
    "................",
    "....#######.....",
    "...###...###....",
    "..###.....##....",
    "..##......###...",
    "..##.......##...",
    "..###.....###...",
    "...##########...",
    ".....########...",
    "..........###...",
    "..........##....",
    "..##.....###....",
    "..###...###.....",
    "...#######......",
    "....#####.......",
    "................",
  }},
  {'A', {
    "................",
    "......####......",
    ".....######.....",
    ".....##..##.....",
    "....##....##....",
    "....##....##....",
    "...##......##...",
    "...##......##...",
    "..###########...",
    "..###########...",
    ".###........###.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    "................",
  }},
  {'B', {
    "................",
    "..#########.....",
    "..##########....",
    "..##......###...",
    "..##.......##...",
    "..##......###...",
    "..#########.....",
    "..##########....",
    "..##......###...",
    "..##.......##...",
    "..##.......##...",
    "..##......###...",
    "..##.....###....",
    "..##########....",
    "..#########.....",
    "................",
  }},
  {'C', {
    "................",
    ".....#######....",
    "...##########...",
    "..###......###..",
    "..##........##..",
    ".##.............",
    ".##.............",
    ".##.............",
    ".##.............",
    ".##.............",
    ".##.............",
    "..##........##..",
    "..###......###..",
    "...##########...",
    ".....#######....",
    "................",
  }},
  {'D', {
    "................",
    "..########......",
    "..##########....",
    "..##......###...",
    "..##.......###..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##.......###..",
    "..##......###...",
    "..##########....",
    "..########......",
    "................",
  }},
  {'E', {
    "................",
    "..###########...",
    "..###########...",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..#########.....",
    "..#########.....",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..###########...",
    "..###########...",
    "................",
  }},
  {'F', {
    "................",
    "..###########...",
    "..###########...",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..#########.....",
    "..#########.....",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "................",
  }},
  {'G', {
    "................",
    ".....#######....",
    "...##########...",
    "..###......###..",
    "..##........##..",
    ".##.............",
    ".##.............",
    ".##.....######..",
    ".##.....######..",
    ".##.........##..",
    ".##.........##..",
    "..##........##..",
    "..###......###..",
    "...##########...",
    ".....#######....",
    "................",
  }},
  {'H', {
    "................",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..############..",
    "..############..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "................",
  }},
  {'I', {
    "................",
    "....########....",
    "....########....",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    "....########....",
    "....########....",
    "................",
  }},
  {'I', { // Vertical bar I (no top/bottom bars)
    "................",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    ".......###......",
    "................",
  }},
  {'J', {
    "................",
    ".........###....",
    ".........###....",
    "..........##....",
    "..........##....",
    "..........##....",
    "..........##....",
    "..........##....",
    "..........##....",
    "..........##....",
    "..##......##....",
    "..##......##....",
    "..###....###....",
    "...########.....",
    "....######......",
    "................",
  }},
  {'K', {
    "................",
    "..##.......##...",
    "..##......###...",
    "..##.....###....",
    "..##....###.....",
    "..##...###......",
    "..##..###.......",
    "..######........",
    "..######........",
    "..##..###.......",
    "..##...###......",
    "..##....###.....",
    "..##.....###....",
    "..##......###...",
    "..##.......##...",
    "................",
  }},
  {'L', {
    "................",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..###########...",
    "..###########...",
    "................",
  }},
  {'M', {
    "................",
    ".##..........##.",
    ".###........###.",
    ".####......####.",
    ".##.##....##.##.",
    ".##..##..##..##.",
    ".##...####...##.",
    ".##....##....##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    "................",
  }},
  {'N', {
    "................",
    "..##........##..",
    "..###.......##..",
    "..####......##..",
    "..##.##.....##..",
    "..##..##....##..",
    "..##...##...##..",
    "..##....##..##..",
    "..##.....##.##..",
    "..##......####..",
    "..##.......###..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "................",
  }},
  {'O', {
    "................",
    ".....######.....",
    "...##########...",
    "..###......###..",
    "..##........##..",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    "..##........##..",
    "..###......###..",
    "...##########...",
    ".....######.....",
    "................",
  }},
  {'P', {
    "................",
    "..#########.....",
    "..##########....",
    "..##......###...",
    "..##.......##...",
    "..##.......##...",
    "..##......###...",
    "..##########....",
    "..#########.....",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "..##............",
    "................",
  }},
  {'Q', {
    "................",
    ".....######.....",
    "...##########...",
    "..###......###..",
    "..##........##..",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##.....##...##.",
    ".##......##..##.",
    "..##......####..",
    "..###......###..",
    "...#########.##.",
    ".....######...#.",
    "................",
  }},
  {'R', {
    "................",
    "..#########.....",
    "..##########....",
    "..##......###...",
    "..##.......##...",
    "..##......###...",
    "..##########....",
    "..#########.....",
    "..##...###......",
    "..##....###.....",
    "..##.....###....",
    "..##......###...",
    "..##.......##...",
    "..##.......###..",
    "..##........##..",
    "................",
  }},
  {'S', {
    "................",
    "....########....",
    "..###########...",
    "..###......###..",
    "..##............",
    "..###...........",
    "...########.....",
    ".....########...",
    "..........###...",
    "...........###..",
    "............##..",
    "..##.......###..",
    "..###.....###...",
    "...#########....",
    ".....######.....",
    "................",
  }},
  {'T', {
    "................",
    ".##############.",
    ".##############.",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    "................",
  }},
  {'U', {
    "................",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "..###......###..",
    "..###......###..",
    "...##########...",
    ".....######.....",
    "................",
  }},
  {'V', {
    "................",
    ".##..........##.",
    ".##..........##.",
    "..##........##..",
    "..##........##..",
    "..##........##..",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "....##....##....",
    "....##....##....",
    ".....##..##.....",
    ".....##..##.....",
    "......####......",
    ".......##.......",
    "................",
  }},
  {'W', {
    "................",
    ".##..........##.",
    ".##..........##.",
    ".##..........##.",
    ".##....##....##.",
    ".##...####...##.",
    "..##..####..##..",
    "..##..####..##..",
    "..##.##..##.##..",
    "..####....####..",
    "...###....###...",
    "...###....###...",
    "...##......##...",
    "...##......##...",
    "...##......##...",
    "................",
  }},
  {'X', {
    "................",
    "..##........##..",
    "..###......###..",
    "...###....###...",
    "....###..###....",
    ".....######.....",
    "......####......",
    "......####......",
    ".....######.....",
    "....###..###....",
    "...###....###...",
    "..###......###..",
    "..##........##..",
    ".###........###.",
    ".##..........##.",
    "................",
  }},
  {'Y', {
    "................",
    "..##........##..",
    "..###......###..",
    "...##......##...",
    "....##....##....",
    "....###..###....",
    ".....######.....",
    "......####......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    ".......##.......",
    "................",
  }},
  {'Z', {
    "................",
    "..###########...",
    "..###########...",
    "..........###...",
    ".........###....",
    "........###.....",
    ".......###......",
    "......###.......",
    ".....###........",
    "....###.........",
    "...###..........",
    "..###...........",
    "..##............",
    "..###########...",
    "..###########...",
    "................",
  }},
};

constexpr size_t NUM_TEMPLATES = sizeof(RAW_GLYPHS) / sizeof(RAW_GLYPHS[0]);
PreparedTemplate s_templates[NUM_TEMPLATES];
bool s_ocrReady = false;

// Reusable scratch buffers in PSRAM/heap to prevent heap fragmentation
uint8_t *s_blockMask = nullptr;
size_t s_blockMaskCap = 0;
uint8_t *s_binGrid = nullptr;
uint8_t *s_visited = nullptr;
int32_t *s_blockQueue = nullptr;
uint8_t *s_binBuf = nullptr;
int32_t *s_queueBuf = nullptr;
float s_patchRaw[256];
float s_patchBlur[256];
std::vector<BlockCandidate> s_rois;
std::vector<RawBlob> s_roiBlobs;
std::vector<RawBlob> s_roiAligned;
std::vector<RawBlob> s_curCluster;
std::vector<RawBlob> s_bestCluster;

void *ocrAllocPsram(size_t bytes) {
  void *ptr = ps_malloc(bytes);
  return ptr ? ptr : malloc(bytes);
}

void blurAndNormalize16x16(const float *src, float *dst) {
  for (int y = 0; y < 16; ++y) {
    int y0 = max(0, y - 1);
    int y2 = min(15, y + 1);
    for (int x = 0; x < 16; ++x) {
      int x0 = max(0, x - 1);
      int x2 = min(15, x + 1);
      float v =
          src[y0 * 16 + x0] * 1.0f + src[y0 * 16 + x] * 2.0f + src[y0 * 16 + x2] * 1.0f +
          src[y * 16 + x0]  * 2.0f + src[y * 16 + x]  * 4.0f + src[y * 16 + x2]  * 2.0f +
          src[y2 * 16 + x0] * 1.0f + src[y2 * 16 + x] * 2.0f + src[y2 * 16 + x2] * 1.0f;
      dst[y * 16 + x] = v / 16.0f;
    }
  }

  float mean = 0.0f;
  for (int i = 0; i < 256; ++i) mean += dst[i];
  mean /= 256.0f;

  float sqSum = 0.0f;
  for (int i = 0; i < 256; ++i) {
    dst[i] -= mean;
    sqSum += dst[i] * dst[i];
  }

  float norm = sqrtf(sqSum);
  if (norm > 1e-5f) {
    float inv = 1.0f / norm;
    for (int i = 0; i < 256; ++i) dst[i] *= inv;
  } else {
    memset(dst, 0, sizeof(float) * 256);
  }
}

void analyzeRoiStats(const uint8_t *gray,
                     int width,
                     int x0,
                     int y0,
                     int rw,
                     int rh,
                     bool &hasContrastOut,
                     uint8_t &otsuOut) {
  uint32_t hist[256] = {0};
  uint32_t total = static_cast<uint32_t>(rw * rh);
  if (total == 0) {
    hasContrastOut = false;
    otsuOut = 128;
    return;
  }

  for (int y = 0; y < rh; ++y) {
    const uint8_t *row = gray + (y0 + y) * width + x0;
    for (int x = 0; x < rw; ++x) {
      hist[row[x]]++;
    }
  }

  uint32_t targetLow = max(1U, total / 20);
  uint32_t targetHigh = total - targetLow;
  uint32_t acc = 0;
  int p5 = 0;
  int p95 = 255;
  bool foundLow = false;
  bool foundHigh = false;
  double sumAll = 0.0;

  for (int i = 0; i < 256; ++i) {
    uint32_t h = hist[i];
    sumAll += static_cast<double>(i) * h;
    acc += h;
    if (!foundLow && acc >= targetLow) {
      p5 = i;
      foundLow = true;
    }
    if (!foundHigh && acc >= targetHigh) {
      p95 = i;
      foundHigh = true;
    }
  }
  // Ensure p95 stays accurate if targetHigh coincided cleanly inside loop
  hasContrastOut = (p95 - p5) >= 42;
  if (!hasContrastOut) {
    otsuOut = 128;
    return;
  }

  double sumBg = 0.0;
  uint32_t wBg = 0;
  double maxBetween = -1.0;
  int firstMaxT = 128;
  int lastMaxT = 128;

  for (int t = 0; t < 256; ++t) {
    wBg += hist[t];
    if (wBg == 0) continue;
    uint32_t wFg = total - wBg;
    if (wFg == 0) break;

    sumBg += static_cast<double>(t) * hist[t];
    double mBg = sumBg / wBg;
    double mFg = (sumAll - sumBg) / wFg;
    double diff = mBg - mFg;
    double between = static_cast<double>(wBg) * static_cast<double>(wFg) * diff * diff;
    if (between > maxBetween + 1e-9) {
      maxBetween = between;
      firstMaxT = t;
      lastMaxT = t;
    } else if (between >= maxBetween - 1e-9) {
      lastMaxT = t;
    }
  }
  otsuOut = static_cast<uint8_t>((firstMaxT + lastMaxT) / 2);
}

void buildBinaryRoi(const uint8_t *gray,
                    int imgWidth,
                    int x0,
                    int y0,
                    int rw,
                    int rh,
                    uint8_t thresh,
                    bool darkOnLight,
                    uint8_t *binOut) {
  for (int y = 0; y < rh; ++y) {
    const uint8_t *row = gray + (y0 + y) * imgWidth + x0;
    uint8_t *dst = binOut + y * rw;
    if (darkOnLight) {
      for (int x = 0; x < rw; ++x) {
        dst[x] = (row[x] <= thresh) ? 1 : 0;
      }
    } else {
      for (int x = 0; x < rw; ++x) {
        dst[x] = (row[x] >= thresh) ? 1 : 0;
      }
    }
  }

  // Strip top/bottom frame lines that cover > 80% of ROI width so characters stay separated
  int rowStripLimit = max(2, rh / 8);
  for (int y = 0; y < rowStripLimit; ++y) {
    int cnt = 0;
    for (int x = 0; x < rw; ++x) cnt += binOut[y * rw + x];
    if (cnt * 100 >= rw * 80) memset(binOut + y * rw, 0, rw);

    int yb = rh - 1 - y;
    cnt = 0;
    for (int x = 0; x < rw; ++x) cnt += binOut[yb * rw + x];
    if (cnt * 100 >= rw * 80) memset(binOut + yb * rw, 0, rw);
  }
}

void extractCandidateBlobs(uint8_t *bin,
                           int rw,
                           int rh,
                           std::vector<RawBlob> &outBlobs) {
  outBlobs.clear();
  int queueCap = rw * rh;

  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  for (int y = 0; y < rh; ++y) {
    for (int x = 0; x < rw; ++x) {
      int startIdx = y * rw + x;
      if ((bin[startIdx] & 0x01) == 0 || (bin[startIdx] & 0x02) != 0) continue;

      int qHead = 0;
      int qTail = 0;
      s_queueBuf[qTail++] = startIdx;
      bin[startIdx] |= 0x02;

      int minX = x, maxX = x, minY = y, maxY = y;
      int area = 0;

      while (qHead < qTail) {
        int cur = s_queueBuf[qHead++];
        int cx = cur % rw;
        int cy = cur / rw;
        area++;
        if (cx < minX) minX = cx;
        if (cx > maxX) maxX = cx;
        if (cy < minY) minY = cy;
        if (cy > maxY) maxY = cy;

        for (int k = 0; k < 4; ++k) {
          int nx = cx + dx[k];
          int ny = cy + dy[k];
          if (nx < 0 || nx >= rw || ny < 0 || ny >= rh) continue;
          int nIdx = ny * rw + nx;
          if ((bin[nIdx] & 0x01) != 0 && (bin[nIdx] & 0x02) == 0 && qTail < queueCap) {
            bin[nIdx] |= 0x02;
            s_queueBuf[qTail++] = nIdx;
          }
        }
      }

      int bw = maxX - minX + 1;
      int bh = maxY - minY + 1;
      if (area < 12) continue;
      if (bh < max(10, (rh * 28) / 100) || bh > (rh * 96) / 100) continue;
      if (bw < 2 || bw > (bh * 24) / 10) continue;

      int boxArea = bw * bh;
      int maxFillPct = (bw * 2 <= bh) ? 100 : 94;
      if (area * 100 < boxArea * 15 || area * 100 > boxArea * maxFillPct) continue;

      // If 2 characters lightly touch side-by-side, split at lowest vertical ink projection near middle
      if (bw * 100 > bh * 125 && bw <= static_cast<int>(bh * 2.3f)) {
        int xStart = minX + bw / 3;
        int xEnd = minX + (bw * 2) / 3;
        int bestSplit = (minX + maxX) / 2;
        int bestColSum = 100000;
        for (int sx = xStart; sx <= xEnd; ++sx) {
          int colSum = 0;
          for (int sy = minY; sy <= maxY; ++sy) {
            colSum += (bin[sy * rw + sx] & 0x01);
          }
          if (colSum < bestColSum) {
            bestColSum = colSum;
            bestSplit = sx;
          }
        }
        if (bestSplit > minX + 2 && bestSplit < maxX - 2) {
          outBlobs.push_back({minX, minY, bestSplit - 1, maxY, area / 2});
          outBlobs.push_back({bestSplit + 1, minY, maxX, maxY, area / 2});
          continue;
        }
      }

      if (bw * 100 <= bh * 125) {
        outBlobs.push_back({minX, minY, maxX, maxY, area});
      }
    }
  }
}

void clusterAlignedCharacters(const std::vector<RawBlob> &blobs,
                              int rw,
                              std::vector<RawBlob> &aligned) {
  aligned.clear();
  if (blobs.size() < 3) return;

  int bestClusterScore = -1;
  s_bestCluster.clear();

  for (size_t i = 0; i < blobs.size(); ++i) {
    int refH = blobs[i].maxY - blobs[i].minY + 1;
    int refCy = (blobs[i].minY + blobs[i].maxY) / 2;
    s_curCluster.clear();
    for (size_t j = 0; j < blobs.size(); ++j) {
      int h = blobs[j].maxY - blobs[j].minY + 1;
      int cy = (blobs[j].minY + blobs[j].maxY) / 2;
      if (h * 100 >= refH * 68 && h * 100 <= refH * 142 &&
          abs(cy - refCy) * 100 <= refH * 38) {
        s_curCluster.push_back(blobs[j]);
      }
    }

    if (s_curCluster.size() >= 3) {
      int score = static_cast<int>(s_curCluster.size()) * 100 + refH;
      if (score > bestClusterScore) {
        bestClusterScore = score;
        s_bestCluster = s_curCluster;
      }
    }
  }

  if (s_bestCluster.size() < 3) return;

  std::sort(s_bestCluster.begin(), s_bestCluster.end(), [](const RawBlob &a, const RawBlob &b) {
    return a.minX < b.minX;
  });

  // Drop outliers whose gap to the main run exceeds 1.8x average char height
  if (s_bestCluster.size() > 10) {
    s_bestCluster.resize(10);
  }

  // Filter horizontally overlapping duplicates (keep taller/larger blob)
  for (const auto &b : s_bestCluster) {
    if (!aligned.empty()) {
      RawBlob &prev = aligned.back();
      int overlap = min(prev.maxX, b.maxX) - max(prev.minX, b.minX) + 1;
      int minW = min(prev.maxX - prev.minX + 1, b.maxX - b.minX + 1);
      if (overlap > minW / 2) {
        if (b.area > prev.area) prev = b;
        continue;
      }
    }
    aligned.push_back(b);
  }

  if (aligned.size() < 3 || aligned.size() > 10) {
    aligned.clear();
    return;
  }

  // Verify reasonable horizontal continuity of the character line
  int sumH = 0;
  for (const auto &b : aligned) sumH += (b.maxY - b.minY + 1);
  float meanH = static_cast<float>(sumH) / aligned.size();
  int maxAllowedGap = max(20, static_cast<int>(meanH * 2.2f));
  int largeGaps = 0;
  for (size_t i = 1; i < aligned.size(); ++i) {
    int gap = aligned[i].minX - aligned[i - 1].maxX;
    if (gap > maxAllowedGap && gap > rw / 3) {
      largeGaps++;
    }
  }
  if (largeGaps > 1) {
    aligned.clear();
  }
}

char classifyCharacterPatch(const uint8_t *gray,
                            int imgWidth,
                            int roiX,
                            int roiY,
                            const RawBlob &blob,
                            uint8_t thresh,
                            bool darkOnLight,
                            uint8_t &confPctOut) {
  int bw = blob.maxX - blob.minX + 1;
  int bh = blob.maxY - blob.minY + 1;
  if (bw <= 0 || bh <= 0) {
    confPctOut = 0;
    return '?';
  }

  // Render cropped grayscale foreground contrast centered with 1 px margin on 16x16 canvas
  float scale = min(14.0f / static_cast<float>(bw), 14.0f / static_cast<float>(bh));
  float drawW = bw * scale;
  float drawH = bh * scale;
  float offX = (16.0f - drawW) * 0.5f;
  float offY = (16.0f - drawH) * 0.5f;

  memset(s_patchRaw, 0, sizeof(s_patchRaw));
  for (int dy = 0; dy < 16; ++dy) {
    float syF = (dy + 0.5f - offY) / scale;
    if (syF < 0.0f || syF >= static_cast<float>(bh)) continue;
    int sy = constrain(static_cast<int>(syF), 0, bh - 1);
    const uint8_t *srcRow = gray + (roiY + blob.minY + sy) * imgWidth + roiX + blob.minX;

    for (int dx = 0; dx < 16; ++dx) {
      float sxF = (dx + 0.5f - offX) / scale;
      if (sxF < 0.0f || sxF >= static_cast<float>(bw)) continue;
      int sx = constrain(static_cast<int>(sxF), 0, bw - 1);

      uint8_t pix = srcRow[sx];
      float fg = 0.0f;
      if (darkOnLight) {
        float diff = static_cast<float>(thresh) + 8.0f - pix;
        if (diff > 0.0f) fg = min(255.0f, diff * 4.5f);
      } else {
        float diff = pix - static_cast<float>(thresh) + 8.0f;
        if (diff > 0.0f) fg = min(255.0f, diff * 4.5f);
      }
      s_patchRaw[dy * 16 + dx] = fg;
    }
  }

  blurAndNormalize16x16(s_patchRaw, s_patchBlur);

  float bestCorr = -1.0f;
  char bestChar = '?';
  for (size_t k = 0; k < NUM_TEMPLATES; ++k) {
    const float *t = s_templates[k].vec;
    float dot = 0.0f;
    for (int i = 0; i < 256; ++i) {
      dot += s_patchBlur[i] * t[i];
    }
    if (dot > bestCorr) {
      bestCorr = dot;
      bestChar = s_templates[k].ch;
    }
  }

  float clamped = max(0.0f, min(1.0f, bestCorr));
  confPctOut = static_cast<uint8_t>(lroundf(clamped * 100.0f));
  return bestChar;
}

void evaluateRoiPolarity(const uint8_t *gray,
                         int width,
                         int rx,
                         int ry,
                         int rw,
                         int rh,
                         uint8_t thresh,
                         bool darkOnLight,
                         int &bestScore,
                         PlateOcrResult &candidateResult) {
  buildBinaryRoi(gray, width, rx, ry, rw, rh, thresh, darkOnLight, s_binBuf);

  extractCandidateBlobs(s_binBuf, rw, rh, s_roiBlobs);

  clusterAlignedCharacters(s_roiBlobs, rw, s_roiAligned);
  if (s_roiAligned.size() < 3) return;

  PlateOcrResult tmp = {};
  tmp.plateLocalized = true;
  tmp.roiX = rx;
  tmp.roiY = ry;
  tmp.roiW = rw;
  tmp.roiH = rh;
  tmp.usedDarkOnLight = darkOnLight ? 1 : 0;
  tmp.charCount = static_cast<uint8_t>(s_roiAligned.size());

  uint32_t confSum = 0;
  for (size_t i = 0; i < s_roiAligned.size(); ++i) {
    uint8_t conf = 0;
    char c = classifyCharacterPatch(gray, width, rx, ry, s_roiAligned[i], thresh, darkOnLight, conf);
    tmp.chars[i].x = static_cast<uint16_t>(rx + s_roiAligned[i].minX);
    tmp.chars[i].y = static_cast<uint16_t>(ry + s_roiAligned[i].minY);
    tmp.chars[i].w = static_cast<uint16_t>(s_roiAligned[i].maxX - s_roiAligned[i].minX + 1);
    tmp.chars[i].h = static_cast<uint16_t>(s_roiAligned[i].maxY - s_roiAligned[i].minY + 1);
    tmp.chars[i].ch = c;
    tmp.chars[i].confPct = conf;
    tmp.text[i] = c;
    confSum += conf;
  }
  tmp.text[s_roiAligned.size()] = '\0';
  tmp.avgConfidence = static_cast<uint8_t>(confSum / s_roiAligned.size());

  int score = static_cast<int>(tmp.avgConfidence) * 4 + static_cast<int>(s_roiAligned.size()) * 12;
  if (score > bestScore) {
    bestScore = score;
    candidateResult = tmp;
  }
}

void localizeCandidateRois(const uint8_t *gray,
                           int width,
                           int height,
                           std::vector<BlockCandidate> &outRois) {
  outRois.clear();
  int bw = width / BLOCK_SIZE;
  int bh = height / BLOCK_SIZE;
  size_t gridCount = static_cast<size_t>(bw * bh);
  if (gridCount > s_blockMaskCap) {
    return;
  }

  uint64_t totalEdge = 0;
  for (int by = 0; by < bh; ++by) {
    for (int bx = 0; bx < bw; ++bx) {
      int sum = 0;
      int baseY = by * BLOCK_SIZE;
      int baseX = bx * BLOCK_SIZE;
      for (int y = 0; y < BLOCK_SIZE; ++y) {
        const uint8_t *row = gray + (baseY + y) * width + baseX;
        for (int x = 0; x < BLOCK_SIZE; ++x) {
          int gx = baseX + x;
          if (gx > 0 && gx + 1 < width) {
            sum += abs(static_cast<int>(row[x + 1]) - static_cast<int>(row[x - 1]));
          }
        }
      }
      int avg = sum / (BLOCK_SIZE * BLOCK_SIZE * 2);
      s_blockMask[by * bw + bx] = static_cast<uint8_t>(min(255, avg));
      totalEdge += avg;
    }
  }

  int meanEdge = static_cast<int>(totalEdge / max<size_t>(1, gridCount));
  int edgeThresh = max(11, (meanEdge * 19) / 10);

  // Build binary active block grid and bridge 2-block horizontal gaps between characters
  memset(s_binGrid, 0, gridCount);
  for (size_t i = 0; i < gridCount; ++i) {
    if (s_blockMask[i] >= edgeThresh) s_binGrid[i] = 1;
  }
  for (int by = 1; by < bh - 1; ++by) {
    for (int bx = 1; bx < bw - 3; ++bx) {
      int idx = by * bw + bx;
      if (s_binGrid[idx]) {
        if (s_binGrid[idx + 2] || s_binGrid[idx + 3]) {
          s_binGrid[idx + 1] = 1;
          if (s_binGrid[idx + 3]) s_binGrid[idx + 2] = 1;
        }
      }
    }
  }

  // BFS on block grid (ignoring top/bottom 2% borders)
  memset(s_visited, 0, gridCount);

  for (int by = 1; by < bh - 1; ++by) {
    for (int bx = 1; bx < bw - 1; ++bx) {
      int sIdx = by * bw + bx;
      if (!s_binGrid[sIdx] || s_visited[sIdx]) continue;

      size_t qHead = 0;
      size_t qTail = 0;
      s_blockQueue[qTail++] = sIdx;
      s_visited[sIdx] = 1;

      int minBx = bx, maxBx = bx, minBy = by, maxBy = by;
      uint32_t edgeEnergy = 0;

      while (qHead < qTail) {
        int cur = s_blockQueue[qHead++];
        int cx = cur % bw;
        int cy = cur / bw;
        edgeEnergy += s_blockMask[cur];
        if (cx < minBx) minBx = cx;
        if (cx > maxBx) maxBx = cx;
        if (cy < minBy) minBy = cy;
        if (cy > maxBy) maxBy = cy;

        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            int nx = cx + dx;
            int ny = cy + dy;
            if (nx < 0 || nx >= bw || ny < 0 || ny >= bh) continue;
            int nIdx = ny * bw + nx;
            if (s_binGrid[nIdx] && !s_visited[nIdx] && qTail < s_blockMaskCap) {
              s_visited[nIdx] = 1;
              s_blockQueue[qTail++] = nIdx;
            }
          }
        }
      }

      int rx0 = max(0, minBx * BLOCK_SIZE - 8);
      int ry0 = max(0, minBy * BLOCK_SIZE - 5);
      int rx1 = min(width - 1, (maxBx + 1) * BLOCK_SIZE + 8);
      int ry1 = min(height - 1, (maxBy + 1) * BLOCK_SIZE + 5);
      int rw = rx1 - rx0 + 1;
      int rh = ry1 - ry0 + 1;
      float aspect = static_cast<float>(rw) / max(1, rh);

      if (rw >= 48 && rw <= width - 8 &&
          rh >= 16 && rh <= min(120, height / 2) &&
          aspect >= 1.5f && aspect <= 8.0f &&
          rw * rh <= MAX_ROI_PIXELS) {
        outRois.push_back({rx0, ry0, rx1, ry1, edgeEnergy});
      }
    }
  }

  std::sort(outRois.begin(), outRois.end(), [](const BlockCandidate &a, const BlockCandidate &b) {
    return a.score > b.score;
  });
  if (outRois.size() > MAX_ROIS_TO_TRY) {
    outRois.resize(MAX_ROIS_TO_TRY);
  }
}

void drawRectGray(uint8_t *gray, int width, int height, int x, int y, int w, int h, uint8_t color, int thickness) {
  for (int t = 0; t < thickness; ++t) {
    int rx0 = constrain(x + t, 0, width - 1);
    int ry0 = constrain(y + t, 0, height - 1);
    int rx1 = constrain(x + w - 1 - t, 0, width - 1);
    int ry1 = constrain(y + h - 1 - t, 0, height - 1);
    if (rx1 <= rx0 || ry1 <= ry0) continue;

    for (int px = rx0; px <= rx1; ++px) {
      gray[ry0 * width + px] = color;
      gray[ry1 * width + px] = color;
    }
    for (int py = ry0; py <= ry1; ++py) {
      gray[py * width + rx0] = color;
      gray[py * width + rx1] = color;
    }
  }
}

bool areConfusableChars(char a, char b) {
  if (a == b) return true;
  auto inGroup = [](char c, const char *grp) {
    return strchr(grp, c) != nullptr;
  };
  const char *groups[] = {"0OQ", "1IL", "2Z", "5S", "6G", "8B"};
  for (const char *g : groups) {
    if (inGroup(a, g) && inGroup(b, g)) return true;
  }
  return false;
}

} // namespace

bool plateOcrInit() {
  if (s_ocrReady) return true;

  for (size_t k = 0; k < NUM_TEMPLATES; ++k) {
    s_templates[k].ch = RAW_GLYPHS[k].ch;
    float raw[256] = {0};
    for (int y = 0; y < 16; ++y) {
      const char *row = RAW_GLYPHS[k].rows[y];
      for (int x = 0; x < 16; ++x) {
        raw[y * 16 + x] = (row[x] == '#') ? 255.0f : 0.0f;
      }
    }
    blurAndNormalize16x16(raw, s_templates[k].vec);
  }

  s_blockMaskCap = 80 * 60;
  s_blockMask = static_cast<uint8_t *>(ocrAllocPsram(s_blockMaskCap));
  s_binGrid = static_cast<uint8_t *>(ocrAllocPsram(s_blockMaskCap));
  s_visited = static_cast<uint8_t *>(ocrAllocPsram(s_blockMaskCap));
  s_blockQueue = static_cast<int32_t *>(ocrAllocPsram(s_blockMaskCap * sizeof(int32_t)));
  s_binBuf = static_cast<uint8_t *>(ocrAllocPsram(MAX_ROI_PIXELS));
  s_queueBuf = static_cast<int32_t *>(ocrAllocPsram(MAX_ROI_PIXELS * sizeof(int32_t)));

  s_rois.reserve(16);
  s_roiBlobs.reserve(32);
  s_roiAligned.reserve(12);
  s_curCluster.reserve(24);
  s_bestCluster.reserve(24);

  s_ocrReady = (s_blockMask != nullptr && s_binGrid != nullptr &&
                s_visited != nullptr && s_blockQueue != nullptr &&
                s_binBuf != nullptr && s_queueBuf != nullptr);
  return s_ocrReady;
}

void plateOcrAnalyze(const uint8_t *gray,
                     int width,
                     int height,
                     PlatePolarity polarity,
                     int minConfidencePct,
                     PlateOcrResult &out) {
  memset(&out, 0, sizeof(out));
  if (!s_ocrReady || !gray || width <= 0 || height <= 0) {
    return;
  }

  localizeCandidateRois(gray, width, height, s_rois);
  if (s_rois.empty()) {
    return;
  }

  // Record best geometric candidate for overlay visualization even if OCR threshold isn't met
  out.plateLocalized = true;
  out.roiX = s_rois[0].x0;
  out.roiY = s_rois[0].y0;
  out.roiW = s_rois[0].x1 - s_rois[0].x0 + 1;
  out.roiH = s_rois[0].y1 - s_rois[0].y0 + 1;

  int bestScore = -1;
  PlateOcrResult bestCandidate = {};

  for (const auto &roi : s_rois) {
    int rw = roi.x1 - roi.x0 + 1;
    int rh = roi.y1 - roi.y0 + 1;
    if (rw * rh > MAX_ROI_PIXELS) continue;

    bool hasContrast = false;
    uint8_t otsu = 128;
    analyzeRoiStats(gray, width, roi.x0, roi.y0, rw, rh, hasContrast, otsu);
    if (!hasContrast) continue;

    if (polarity == PLATE_POLARITY_AUTO || polarity == PLATE_POLARITY_DARK_ON_LIGHT) {
      evaluateRoiPolarity(gray, width, roi.x0, roi.y0, rw, rh, otsu, true, bestScore, bestCandidate);
    }
    if (polarity == PLATE_POLARITY_AUTO || polarity == PLATE_POLARITY_LIGHT_ON_DARK) {
      evaluateRoiPolarity(gray, width, roi.x0, roi.y0, rw, rh, otsu, false, bestScore, bestCandidate);
    }
  }

  if (bestScore > 0 && bestCandidate.charCount >= 3) {
    out = bestCandidate;
    out.validRead = (out.avgConfidence >= minConfidencePct);
  }
}

String plateNormalizeString(const String &raw) {
  String clean;
  clean.reserve(16);
  for (size_t i = 0; i < raw.length() && clean.length() < 12; ++i) {
    char c = raw[i];
    if (isalnum(static_cast<unsigned char>(c))) {
      clean += static_cast<char>(toupper(static_cast<unsigned char>(c)));
    }
  }
  return clean;
}

bool plateMatchesAuthorized(const char *readPlate,
                            const char *authorizedPlate,
                            bool allowConfusable) {
  if (!readPlate || !authorizedPlate) return false;
  size_t lenR = strlen(readPlate);
  size_t lenA = strlen(authorizedPlate);
  if (lenR == 0 || lenR != lenA) return false;

  int confusableDiffs = 0;
  for (size_t i = 0; i < lenR; ++i) {
    char cr = readPlate[i];
    char ca = authorizedPlate[i];
    if (cr == ca) continue;
    if (!allowConfusable || lenR < 4 || !areConfusableChars(cr, ca)) {
      return false;
    }
    confusableDiffs++;
    if (confusableDiffs > 1) return false;
  }
  return true;
}

void plateDrawOverlayGray(uint8_t *gray,
                          int width,
                          int height,
                          const PlateOcrResult &res,
                          bool matched) {
  if (!gray || !res.plateLocalized) return;

  // High contrast white frame (with dark inner border) so boxes stand out on grayscale preview
  uint8_t outerCol = matched ? 255 : 220;
  uint8_t innerCol = matched ? 20 : 40;
  drawRectGray(gray, width, height, res.roiX, res.roiY, res.roiW, res.roiH, outerCol, 2);
  drawRectGray(gray, width, height, res.roiX + 2, res.roiY + 2, res.roiW - 4, res.roiH - 4, innerCol, 1);

  for (uint8_t i = 0; i < res.charCount; ++i) {
    drawRectGray(gray, width, height,
                 res.chars[i].x, res.chars[i].y, res.chars[i].w, res.chars[i].h,
                 255, 1);
  }
}
