/* quirc - QR-code recognition library
 * Copyright (C) 2010-2012 Daniel Beer <dlbeer@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "quirc_internal.h"

/************************************************************************
 * Linear algebra routines
 */

static int line_intersect(const struct quirc_point *p0,
                          const struct quirc_point *p1,
                          const struct quirc_point *q0,
                          const struct quirc_point *q1,
                          struct quirc_point *r)
{
  int a = -(p1->y - p0->y);
  int b = p1->x - p0->x;

  int c = -(q1->y - q0->y);
  int d = q1->x - q0->x;

  int e = a * p1->x + b * p1->y;
  int f = c * q1->x + d * q1->y;

  int det = (a * d) - (b * c);

  if (!det)
    return 0;

  r->x = (d * e - b * f) / det;
  r->y = (-c * e + a * f) / det;

  return 1;
}

static void perspective_setup(float *c,
                              const struct quirc_point *rect,
                              float w, float h)
{
  float x0 = rect[0].x;
  float y0 = rect[0].y;
  float x1 = rect[1].x;
  float y1 = rect[1].y;
  float x2 = rect[2].x;
  float y2 = rect[2].y;
  float x3 = rect[3].x;
  float y3 = rect[3].y;

  float wden = w * (x2 * y3 - x3 * y2 + (x3 - x2) * y1 + x1 * (y2 - y3));
  float hden = h * (x2 * y3 + x1 * (y2 - y3) - x3 * y2 + (x3 - x2) * y1);

  if (fabsf(wden) < 1e-6f || fabsf(hden) < 1e-6f) {
    memset(c, 0, sizeof(float) * QUIRC_PERSPECTIVE_PARAMS);
    return;
  }

  c[0] = (x1 * (x2 * y3 - x3 * y2) + x0 * (-x2 * y3 + x3 * y2 + (x2 - x3) * y1) +
          x1 * (x3 - x2) * y0) / wden;
  c[1] = -(x0 * (x2 * y3 + x1 * (y2 - y3) - x2 * y1) - x1 * x3 * y2 +
           x2 * x3 * y1 + (x1 * x3 - x2 * x3) * y0) / hden;
  c[2] = x0;
  c[3] = (y0 * (x1 * (y3 - y2) - x2 * y3 + x3 * y2) + y1 * (x2 * y3 - x3 * y2) +
          x0 * y1 * (y2 - y3)) / wden;
  c[4] = (x0 * (y1 * y3 - y2 * y3) + x1 * y2 * y3 - x2 * y1 * y3 +
          y0 * (x3 * y2 - x1 * y2 + (x2 - x3) * y1)) / hden;
  c[5] = y0;
  c[6] = (x1 * (y3 - y2) + x0 * (y2 - y3) + (x2 - x3) * y1 + (x3 - x2) * y0) / wden;
  c[7] = (-x2 * y3 + x1 * y3 + x3 * y2 + x0 * (y1 - y2) - x3 * y1 + (x2 - x1) * y0) / hden;
}

static void perspective_map(const float *c,
                            float u, float v, struct quirc_point *ret)
{
  float den = c[6] * u + c[7] * v + 1.0f;
  if (fabsf(den) < 1e-6f) {
    ret->x = -1;
    ret->y = -1;
    return;
  }
  float x = (c[0] * u + c[1] * v + c[2]) / den;
  float y = (c[3] * u + c[4] * v + c[5]) / den;

  ret->x = (int)lroundf(x);
  ret->y = (int)lroundf(y);
}

static void perspective_unmap(const float *c,
                              const struct quirc_point *in,
                              float *u, float *v)
{
  float x = (float)in->x;
  float y = (float)in->y;
  float den = -c[0] * c[7] * y + c[1] * c[6] * y + (c[3] * c[7] - c[4] * c[6]) * x +
              c[0] * c[4] - c[1] * c[3];

  if (fabsf(den) < 1e-6f) {
    *u = 0.0f;
    *v = 0.0f;
    return;
  }

  *u = -(c[1] * (y - c[5]) - c[2] * c[7] * y + (c[5] * c[7] - c[4]) * x + c[2] * c[4]) / den;
  *v = (c[0] * (y - c[5]) - c[2] * c[6] * y + (c[5] * c[6] - c[3]) * x + c[2] * c[3]) / den;
}

/************************************************************************
 * Span-based iterative floodfill routine (PSRAM preallocated stack)
 */

typedef void (*span_func_t)(void *user_data, int y, int left, int right);

static void flood_fill_seed(struct quirc *q, int x, int y, int from, int to,
                            span_func_t func, void *user_data)
{
  int sp = 0;

  if (x < 0 || y < 0 || x >= q->w || y >= q->h || from == to)
    return;

  for (;;) {
    int left = x;
    int right = x;
    int i;
    quirc_pixel_t *row = q->pixels + y * q->w;

    while (left > 0 && row[left - 1] == (quirc_pixel_t)from)
      left--;

    while (right < q->w - 1 && row[right + 1] == (quirc_pixel_t)from)
      right++;

    for (i = left; i <= right; i++)
      row[i] = (quirc_pixel_t)to;

    if (func)
      func(user_data, y, left, right);

    for (;;) {
      if (q->flood_stack && sp < q->flood_stack_cap) {
        if (y > 0) {
          row = q->pixels + (y - 1) * q->w;
          int recurse = 0;
          for (i = left; i <= right; i++) {
            if (row[i] == (quirc_pixel_t)from) {
              q->flood_stack[sp].x = (int16_t)x;
              q->flood_stack[sp].y = (int16_t)y;
              q->flood_stack[sp].l = (int16_t)left;
              q->flood_stack[sp].r = (int16_t)right;
              sp++;
              x = i;
              y = y - 1;
              recurse = 1;
              break;
            }
          }
          if (recurse)
            break;
        }

        if (y < q->h - 1) {
          row = q->pixels + (y + 1) * q->w;
          int recurse = 0;
          for (i = left; i <= right; i++) {
            if (row[i] == (quirc_pixel_t)from) {
              q->flood_stack[sp].x = (int16_t)x;
              q->flood_stack[sp].y = (int16_t)y;
              q->flood_stack[sp].l = (int16_t)left;
              q->flood_stack[sp].r = (int16_t)right;
              sp++;
              x = i;
              y = y + 1;
              recurse = 1;
              break;
            }
          }
          if (recurse)
            break;
        }
      }

      if (sp <= 0)
        return;

      sp--;
      x = q->flood_stack[sp].x;
      y = q->flood_stack[sp].y;
      left = q->flood_stack[sp].l;
      right = q->flood_stack[sp].r;
    }
  }
}

/************************************************************************
 * Adaptive thresholding
 */

#define THRESHOLD_S_MIN 1
#define THRESHOLD_S_DEN 8
#define THRESHOLD_T 5

static void threshold(struct quirc *q)
{
  int x, y;
  int avg_w = 0;
  int avg_u = 0;
  int threshold_s = q->w / THRESHOLD_S_DEN;
  quirc_pixel_t *row = q->pixels;
  int *row_average = q->row_average;

  if (!row_average || q->row_average_cap < q->w)
    return;

  if (threshold_s < THRESHOLD_S_MIN)
    threshold_s = THRESHOLD_S_MIN;

  for (y = 0; y < q->h; y++) {
    memset(row_average, 0, sizeof(int) * (size_t)q->w);

    for (x = 0; x < q->w; x++) {
      int w, u;

      if (y & 1) {
        w = x;
        u = q->w - 1 - x;
      } else {
        w = q->w - 1 - x;
        u = x;
      }

      avg_w = (avg_w * (threshold_s - 1)) / threshold_s + row[w];
      avg_u = (avg_u * (threshold_s - 1)) / threshold_s + row[u];

      row_average[w] += avg_w;
      row_average[u] += avg_u;
    }

    for (x = 0; x < q->w; x++) {
      if ((int)row[x] < row_average[x] * (100 - THRESHOLD_T) / (200 * threshold_s))
        row[x] = QUIRC_PIXEL_BLACK;
      else
        row[x] = QUIRC_PIXEL_WHITE;
    }

    row += q->w;
  }
}

static void area_count(void *user_data, int y, int left, int right)
{
  (void)y;
  ((struct quirc_region *)user_data)->count += right - left + 1;
}

static int region_code(struct quirc *q, int x, int y)
{
  int pixel;
  struct quirc_region *box;
  int region;

  if (x < 0 || y < 0 || x >= q->w || y >= q->h)
    return -1;

  pixel = q->pixels[y * q->w + x];

  if (pixel >= QUIRC_PIXEL_REGION)
    return pixel;

  if (pixel == QUIRC_PIXEL_WHITE)
    return -1;

  if (q->num_regions >= QUIRC_MAX_REGIONS)
    return -1;

  region = q->num_regions;
  box = &q->regions[q->num_regions++];

  memset(box, 0, sizeof(*box));

  box->seed.x = x;
  box->seed.y = y;
  box->capstone = -1;

  flood_fill_seed(q, x, y, pixel, region, area_count, box);

  return region;
}

struct polygon_score_data {
  struct quirc_point ref;
  int scores[4];
  struct quirc_point *corners;
};

static void find_one_corner(void *user_data, int y, int left, int right)
{
  struct polygon_score_data *psd = (struct polygon_score_data *)user_data;
  int xs[2] = {left, right};
  int dy = y - psd->ref.y;
  int i;

  for (i = 0; i < 2; i++) {
    int dx = xs[i] - psd->ref.x;
    int d = dx * dx + dy * dy;

    if (d > psd->scores[0]) {
      psd->scores[0] = d;
      psd->corners[0].x = xs[i];
      psd->corners[0].y = y;
    }
  }
}

static void find_other_corners(void *user_data, int y, int left, int right)
{
  struct polygon_score_data *psd = (struct polygon_score_data *)user_data;
  int xs[2] = {left, right};
  int i;

  for (i = 0; i < 2; i++) {
    int up = xs[i] * psd->ref.x + y * psd->ref.y;
    int rgt = xs[i] * -psd->ref.y + y * psd->ref.x;
    int scores[4] = {up, rgt, -up, -rgt};
    int j;

    for (j = 0; j < 4; j++) {
      if (scores[j] > psd->scores[j]) {
        psd->scores[j] = scores[j];
        psd->corners[j].x = xs[i];
        psd->corners[j].y = y;
      }
    }
  }
}

static void find_region_corners(struct quirc *q,
                                int rcode, const struct quirc_point *ref,
                                struct quirc_point *corners)
{
  struct quirc_region *region = &q->regions[rcode];
  struct polygon_score_data psd;
  int i;

  memset(&psd, 0, sizeof(psd));
  psd.corners = corners;

  memcpy(&psd.ref, ref, sizeof(psd.ref));
  psd.scores[0] = -1;
  flood_fill_seed(q, region->seed.x, region->seed.y,
                  rcode, QUIRC_PIXEL_BLACK,
                  find_one_corner, &psd);

  psd.ref.x = psd.corners[0].x - psd.ref.x;
  psd.ref.y = psd.corners[0].y - psd.ref.y;

  for (i = 0; i < 4; i++)
    memcpy(&psd.corners[i], &region->seed, sizeof(psd.corners[i]));

  i = region->seed.x * psd.ref.x + region->seed.y * psd.ref.y;
  psd.scores[0] = i;
  psd.scores[2] = -i;
  i = region->seed.x * -psd.ref.y + region->seed.y * psd.ref.x;
  psd.scores[1] = i;
  psd.scores[3] = -i;

  flood_fill_seed(q, region->seed.x, region->seed.y,
                  QUIRC_PIXEL_BLACK, rcode,
                  find_other_corners, &psd);
}

static void record_capstone(struct quirc *q, int ring, int stone)
{
  struct quirc_region *stone_reg = &q->regions[stone];
  struct quirc_region *ring_reg = &q->regions[ring];
  struct quirc_capstone *capstone;
  int cs_index;

  if (q->num_capstones >= QUIRC_MAX_CAPSTONES)
    return;

  cs_index = q->num_capstones;
  capstone = &q->capstones[q->num_capstones++];

  memset(capstone, 0, sizeof(*capstone));

  capstone->qr_grid = -1;
  capstone->ring = ring;
  capstone->stone = stone;
  stone_reg->capstone = cs_index;
  ring_reg->capstone = cs_index;

  find_region_corners(q, ring, &stone_reg->seed, capstone->corners);

  perspective_setup(capstone->c, capstone->corners, 7.0f, 7.0f);
  perspective_map(capstone->c, 3.5f, 3.5f, &capstone->center);
}

static void test_capstone(struct quirc *q, int x, int y, int *pb)
{
  int ring_right = region_code(q, x - pb[4], y);
  int stone = region_code(q, x - pb[4] - pb[3] - pb[2], y);
  int ring_left = region_code(q, x - pb[4] - pb[3] - pb[2] - pb[1] - pb[0], y);
  struct quirc_region *stone_reg;
  struct quirc_region *ring_reg;
  int ratio;

  if (ring_left < 0 || ring_right < 0 || stone < 0)
    return;

  if (ring_left != ring_right)
    return;

  if (ring_left == stone)
    return;

  stone_reg = &q->regions[stone];
  ring_reg = &q->regions[ring_left];

  if (stone_reg->capstone >= 0 || ring_reg->capstone >= 0)
    return;

  if (ring_reg->count <= 0)
    return;

  ratio = stone_reg->count * 100 / ring_reg->count;
  if (ratio < 10 || ratio > 70)
    return;

  record_capstone(q, ring_left, stone);
}

static void finder_scan(struct quirc *q, int y)
{
  quirc_pixel_t *row = q->pixels + y * q->w;
  int x;
  int last_color = 0;
  int run_length = 0;
  int run_count = 0;
  int pb[5];

  memset(pb, 0, sizeof(pb));
  for (x = 0; x < q->w; x++) {
    int color = row[x] ? 1 : 0;

    if (x && color != last_color) {
      memmove(pb, pb + 1, sizeof(pb[0]) * 4);
      pb[4] = run_length;
      run_length = 0;
      run_count++;

      if (!color && run_count >= 5) {
        static const int check[5] = {1, 1, 3, 1, 1};
        int avg, err;
        int i;
        int ok = 1;

        avg = (pb[0] + pb[1] + pb[3] + pb[4]) / 4;
        err = avg * 3 / 4;

        for (i = 0; i < 5; i++) {
          if (pb[i] < check[i] * avg - err ||
              pb[i] > check[i] * avg + err)
            ok = 0;
        }

        if (ok)
          test_capstone(q, x, y, pb);
      }
    }

    run_length++;
    last_color = color;
  }
}

static void find_alignment_pattern(struct quirc *q, int index)
{
  struct quirc_grid *qr = &q->grids[index];
  struct quirc_capstone *c0 = &q->capstones[qr->caps[0]];
  struct quirc_capstone *c2 = &q->capstones[qr->caps[2]];
  struct quirc_point a;
  struct quirc_point b;
  struct quirc_point c;
  int size_estimate;
  int step_size = 1;
  int dir = 0;
  float u, v;

  memcpy(&b, &qr->align, sizeof(b));

  perspective_unmap(c0->c, &b, &u, &v);
  perspective_map(c0->c, u, v + 1.0f, &a);
  perspective_unmap(c2->c, &b, &u, &v);
  perspective_map(c2->c, u + 1.0f, v, &c);

  size_estimate = abs((a.x - b.x) * -(c.y - b.y) +
                      (a.y - b.y) * (c.x - b.x));

  while (step_size * step_size < size_estimate * 100 && step_size < q->w) {
    static const int dx_map[] = {1, 0, -1, 0};
    static const int dy_map[] = {0, -1, 0, 1};
    int i;

    for (i = 0; i < step_size; i++) {
      int code = region_code(q, b.x, b.y);

      if (code >= 0) {
        struct quirc_region *reg = &q->regions[code];

        if (reg->count >= size_estimate / 2 &&
            reg->count <= size_estimate * 2) {
          qr->align_region = code;
          return;
        }
      }

      b.x += dx_map[dir];
      b.y += dy_map[dir];
    }

    dir = (dir + 1) % 4;
    if (!(dir & 1))
      step_size++;
  }
}

static void find_leftmost_to_line(void *user_data, int y, int left, int right)
{
  struct polygon_score_data *psd = (struct polygon_score_data *)user_data;
  int xs[2] = {left, right};
  int i;

  for (i = 0; i < 2; i++) {
    int d = -psd->ref.y * xs[i] + psd->ref.x * y;

    if (d < psd->scores[0]) {
      psd->scores[0] = d;
      psd->corners[0].x = xs[i];
      psd->corners[0].y = y;
    }
  }
}

static int timing_scan(const struct quirc *q,
                       const struct quirc_point *p0,
                       const struct quirc_point *p1)
{
  int n = p1->x - p0->x;
  int d = p1->y - p0->y;
  int x = p0->x;
  int y = p0->y;
  int *dom, *nondom;
  int dom_step;
  int nondom_step;
  int a = 0;
  int i;
  int run_length = 0;
  int count = 0;

  if (p0->x < 0 || p0->y < 0 || p0->x >= q->w || p0->y >= q->h)
    return -1;
  if (p1->x < 0 || p1->y < 0 || p1->x >= q->w || p1->y >= q->h)
    return -1;

  if (abs(n) > abs(d)) {
    int swap = n;
    n = d;
    d = swap;
    dom = &x;
    nondom = &y;
  } else {
    dom = &y;
    nondom = &x;
  }

  if (n < 0) {
    n = -n;
    nondom_step = -1;
  } else {
    nondom_step = 1;
  }

  if (d < 0) {
    d = -d;
    dom_step = -1;
  } else {
    dom_step = 1;
  }

  x = p0->x;
  y = p0->y;
  for (i = 0; i <= d; i++) {
    int pixel;

    if (y < 0 || y >= q->h || x < 0 || x >= q->w)
      break;

    pixel = q->pixels[y * q->w + x];

    if (pixel) {
      if (run_length >= 2)
        count++;
      run_length = 0;
    } else {
      run_length++;
    }

    a += n;
    *dom += dom_step;
    if (a >= d) {
      *nondom += nondom_step;
      a -= d;
    }
  }

  return count;
}

static int measure_timing_pattern(struct quirc *q, int index)
{
  struct quirc_grid *qr = &q->grids[index];
  int i;
  int scan;
  int ver;
  int size;

  for (i = 0; i < 3; i++) {
    static const float us[] = {6.5f, 6.5f, 0.5f};
    static const float vs[] = {0.5f, 6.5f, 6.5f};
    struct quirc_capstone *cap = &q->capstones[qr->caps[i]];

    perspective_map(cap->c, us[i], vs[i], &qr->tpep[i]);
  }

  qr->hscan = timing_scan(q, &qr->tpep[1], &qr->tpep[2]);
  qr->vscan = timing_scan(q, &qr->tpep[1], &qr->tpep[0]);

  scan = qr->hscan;
  if (qr->vscan > scan)
    scan = qr->vscan;

  if (scan < 0)
    return -1;

  size = scan * 2 + 13;
  ver = (size - 15) / 4;
  qr->grid_size = ver * 4 + 17;

  return 0;
}

static int read_cell(const struct quirc *q, int index, int x, int y)
{
  const struct quirc_grid *qr = &q->grids[index];
  struct quirc_point p;

  perspective_map(qr->c, x + 0.5f, y + 0.5f, &p);
  if (p.y < 0 || p.y >= q->h || p.x < 0 || p.x >= q->w)
    return 0;

  return q->pixels[p.y * q->w + p.x] ? 1 : -1;
}

static int fitness_cell(const struct quirc *q, int index, int x, int y)
{
  const struct quirc_grid *qr = &q->grids[index];
  int score = 0;
  int u, v;

  for (v = 0; v < 3; v++) {
    for (u = 0; u < 3; u++) {
      static const float offsets[] = {0.3f, 0.5f, 0.7f};
      struct quirc_point p;

      perspective_map(qr->c, x + offsets[u], y + offsets[v], &p);
      if (p.y < 0 || p.y >= q->h || p.x < 0 || p.x >= q->w)
        continue;

      if (q->pixels[p.y * q->w + p.x])
        score++;
      else
        score--;
    }
  }

  return score;
}

static int fitness_ring(const struct quirc *q, int index, int cx, int cy,
                        int radius)
{
  int i;
  int score = 0;

  for (i = 0; i < radius * 2; i++) {
    score += fitness_cell(q, index, cx - radius + i, cy - radius);
    score += fitness_cell(q, index, cx - radius, cy + radius - i);
    score += fitness_cell(q, index, cx + radius, cy - radius + i);
    score += fitness_cell(q, index, cx + radius - i, cy + radius);
  }

  return score;
}

static int fitness_apat(const struct quirc *q, int index, int cx, int cy)
{
  return fitness_cell(q, index, cx, cy) -
         fitness_ring(q, index, cx, cy, 1) +
         fitness_ring(q, index, cx, cy, 2);
}

static int fitness_capstone(const struct quirc *q, int index, int x, int y)
{
  x += 3;
  y += 3;

  return fitness_cell(q, index, x, y) +
         fitness_ring(q, index, x, y, 1) -
         fitness_ring(q, index, x, y, 2) +
         fitness_ring(q, index, x, y, 3);
}

static int fitness_all(const struct quirc *q, int index)
{
  const struct quirc_grid *qr = &q->grids[index];
  int version = (qr->grid_size - 17) / 4;
  const struct quirc_version_info *info;
  int score = 0;
  int i, j;
  int ap_count;

  if (version < 0 || version > QUIRC_MAX_VERSION)
    return 0;

  info = &quirc_version_db[version];

  for (i = 0; i < qr->grid_size - 14; i++) {
    int expect = (i & 1) ? 1 : -1;

    score += fitness_cell(q, index, i + 7, 6) * expect;
    score += fitness_cell(q, index, 6, i + 7) * expect;
  }

  score += fitness_capstone(q, index, 0, 0);
  score += fitness_capstone(q, index, qr->grid_size - 7, 0);
  score += fitness_capstone(q, index, 0, qr->grid_size - 7);

  ap_count = 0;
  while ((ap_count < QUIRC_MAX_ALIGNMENT) && info->apat[ap_count])
    ap_count++;

  for (i = 1; i + 1 < ap_count; i++) {
    score += fitness_apat(q, index, 6, info->apat[i]);
    score += fitness_apat(q, index, info->apat[i], 6);
  }

  for (i = 1; i < ap_count; i++) {
    for (j = 1; j < ap_count; j++) {
      score += fitness_apat(q, index, info->apat[i], info->apat[j]);
    }
  }

  return score;
}

static void jiggle_perspective(struct quirc *q, int index)
{
  struct quirc_grid *qr = &q->grids[index];
  int best = fitness_all(q, index);
  int pass;
  float adjustments[8];
  int i;

  for (i = 0; i < 8; i++)
    adjustments[i] = qr->c[i] * 0.02f;

  for (pass = 0; pass < 5; pass++) {
    for (i = 0; i < 16; i++) {
      int j = i >> 1;
      int test;
      float old = qr->c[j];
      float step = adjustments[j];
      float test_val = (i & 1) ? (old + step) : (old - step);

      qr->c[j] = test_val;
      test = fitness_all(q, index);

      if (test > best)
        best = test;
      else
        qr->c[j] = old;
    }

    for (i = 0; i < 8; i++)
      adjustments[i] *= 0.5f;
  }
}

static void setup_qr_perspective(struct quirc *q, int index)
{
  struct quirc_grid *qr = &q->grids[index];
  struct quirc_point rect[4];

  memcpy(&rect[0], &q->capstones[qr->caps[1]].corners[0], sizeof(rect[0]));
  memcpy(&rect[1], &q->capstones[qr->caps[2]].corners[0], sizeof(rect[0]));
  memcpy(&rect[2], &qr->align, sizeof(rect[0]));
  memcpy(&rect[3], &q->capstones[qr->caps[0]].corners[0], sizeof(rect[0]));
  perspective_setup(qr->c, rect, (float)(qr->grid_size - 7), (float)(qr->grid_size - 7));

  jiggle_perspective(q, index);
}

static void rotate_capstone(struct quirc_capstone *cap,
                            const struct quirc_point *h0,
                            const struct quirc_point *hd)
{
  struct quirc_point copy[4];
  int j;
  int best = 0;
  int best_score = 0;

  for (j = 0; j < 4; j++) {
    struct quirc_point *p = &cap->corners[j];
    int score = (p->x - h0->x) * -hd->y + (p->y - h0->y) * hd->x;

    if (!j || score < best_score) {
      best = j;
      best_score = score;
    }
  }

  for (j = 0; j < 4; j++)
    memcpy(&copy[j], &cap->corners[(j + best) % 4], sizeof(copy[j]));
  memcpy(cap->corners, copy, sizeof(cap->corners));
  perspective_setup(cap->c, cap->corners, 7.0f, 7.0f);
}

static void record_qr_grid(struct quirc *q, int a, int b, int c)
{
  struct quirc_point h0, hd;
  int i;
  int qr_index;
  struct quirc_grid *qr;

  if (q->num_grids >= QUIRC_MAX_GRIDS)
    return;

  memcpy(&h0, &q->capstones[a].center, sizeof(h0));
  hd.x = q->capstones[c].center.x - q->capstones[a].center.x;
  hd.y = q->capstones[c].center.y - q->capstones[a].center.y;

  if ((q->capstones[b].center.x - h0.x) * -hd.y +
      (q->capstones[b].center.y - h0.y) * hd.x > 0) {
    int swap = a;
    a = c;
    c = swap;
    hd.x = -hd.x;
    hd.y = -hd.y;
  }

  qr_index = q->num_grids;
  qr = &q->grids[q->num_grids++];

  memset(qr, 0, sizeof(*qr));
  qr->caps[0] = a;
  qr->caps[1] = b;
  qr->caps[2] = c;
  qr->align_region = -1;

  for (i = 0; i < 3; i++) {
    struct quirc_capstone *cap = &q->capstones[qr->caps[i]];
    rotate_capstone(cap, &h0, &hd);
    cap->qr_grid = qr_index;
  }

  if (measure_timing_pattern(q, qr_index) < 0)
    goto fail;

  if (!line_intersect(&q->capstones[a].corners[0],
                      &q->capstones[a].corners[1],
                      &q->capstones[c].corners[0],
                      &q->capstones[c].corners[3],
                      &qr->align))
    goto fail;

  if (qr->grid_size > 21) {
    find_alignment_pattern(q, qr_index);

    if (qr->align_region >= 0) {
      struct polygon_score_data psd;
      struct quirc_region *reg = &q->regions[qr->align_region];

      memcpy(&qr->align, &reg->seed, sizeof(qr->align));
      memcpy(&psd.ref, &hd, sizeof(psd.ref));
      psd.corners = &qr->align;
      psd.scores[0] = -hd.y * qr->align.x + hd.x * qr->align.y;

      flood_fill_seed(q, reg->seed.x, reg->seed.y,
                      qr->align_region, QUIRC_PIXEL_BLACK,
                      NULL, NULL);
      flood_fill_seed(q, reg->seed.x, reg->seed.y,
                      QUIRC_PIXEL_BLACK, qr->align_region,
                      find_leftmost_to_line, &psd);
    }
  }

  setup_qr_perspective(q, qr_index);
  return;

fail:
  for (i = 0; i < 3; i++)
    q->capstones[qr->caps[i]].qr_grid = -1;
  q->num_grids--;
}

struct neighbour {
  int index;
  float distance;
};

struct neighbour_list {
  struct neighbour n[QUIRC_MAX_CAPSTONES];
  int count;
};

static void test_neighbours(struct quirc *q, int i,
                            const struct neighbour_list *hlist,
                            const struct neighbour_list *vlist)
{
  int j, k;
  float best_score = 0.0f;
  int best_h = -1, best_v = -1;

  for (j = 0; j < hlist->count; j++) {
    for (k = 0; k < vlist->count; k++) {
      const struct neighbour *hn = &hlist->n[j];
      const struct neighbour *vn = &vlist->n[k];
      float score;

      if (vn->distance < 1e-3f)
        continue;
      score = fabsf(1.0f - hn->distance / vn->distance);

      if (score > 2.5f)
        continue;

      if (best_h < 0 || score < best_score) {
        best_h = hn->index;
        best_v = vn->index;
        best_score = score;
      }
    }
  }

  if (best_h < 0 || best_v < 0)
    return;

  record_qr_grid(q, best_h, i, best_v);
}

static void test_grouping(struct quirc *q, int i)
{
  struct quirc_capstone *c1 = &q->capstones[i];
  int j;
  struct neighbour_list hlist;
  struct neighbour_list vlist;

  if (c1->qr_grid >= 0)
    return;

  hlist.count = 0;
  vlist.count = 0;

  for (j = 0; j < q->num_capstones; j++) {
    struct quirc_capstone *c2 = &q->capstones[j];
    float u, v;

    if (i == j || c2->qr_grid >= 0)
      continue;

    perspective_unmap(c1->c, &c2->center, &u, &v);

    u = fabsf(u - 3.5f);
    v = fabsf(v - 3.5f);

    if (u < 0.2f * v && hlist.count < QUIRC_MAX_CAPSTONES) {
      struct neighbour *n = &hlist.n[hlist.count++];
      n->index = j;
      n->distance = v;
    }

    if (v < 0.2f * u && vlist.count < QUIRC_MAX_CAPSTONES) {
      struct neighbour *n = &vlist.n[vlist.count++];
      n->index = j;
      n->distance = u;
    }
  }

  if (!(hlist.count && vlist.count))
    return;

  test_neighbours(q, i, &hlist, &vlist);
}

static void pixels_setup(struct quirc *q)
{
  if (sizeof(*q->image) == sizeof(*q->pixels)) {
    q->pixels = (quirc_pixel_t *)q->image;
  } else {
    int count = q->w * q->h;
    int i;
    for (i = 0; i < count; i++) {
      q->pixels[i] = q->image[i];
    }
  }
}

uint8_t *quirc_begin(struct quirc *q, int *w, int *h)
{
  if (!q)
    return NULL;

  q->num_regions = QUIRC_PIXEL_REGION;
  q->num_capstones = 0;
  q->num_grids = 0;

  if (w)
    *w = q->w;
  if (h)
    *h = q->h;

  return q->image;
}

void quirc_end(struct quirc *q)
{
  int i;

  if (!q || !q->image || !q->pixels)
    return;

  pixels_setup(q);
  threshold(q);

  for (i = 0; i < q->h; i++) {
    finder_scan(q, i);
  }

  for (i = 0; i < q->num_capstones; i++) {
    test_grouping(q, i);
  }
}

void quirc_extract(const struct quirc *q, int index,
                   struct quirc_code *code)
{
  const struct quirc_grid *qr;
  int y;
  int i = 0;

  if (!q || !code || index < 0 || index >= q->num_grids)
    return;

  qr = &q->grids[index];
  memset(code, 0, sizeof(*code));

  perspective_map(qr->c, 0.0f, 0.0f, &code->corners[0]);
  perspective_map(qr->c, (float)qr->grid_size, 0.0f, &code->corners[1]);
  perspective_map(qr->c, (float)qr->grid_size, (float)qr->grid_size, &code->corners[2]);
  perspective_map(qr->c, 0.0f, (float)qr->grid_size, &code->corners[3]);

  code->size = qr->grid_size;

  for (y = 0; y < qr->grid_size; y++) {
    int x;
    for (x = 0; x < qr->grid_size; x++) {
      if ((i >> 3) < QUIRC_MAX_BITMAP && read_cell(q, index, x, y) > 0)
        code->cell_bitmap[i >> 3] |= (uint8_t)(1 << (i & 7));
      i++;
    }
  }
}
