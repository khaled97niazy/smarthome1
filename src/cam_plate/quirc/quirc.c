/* quirc -- QR-code recognition library
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

#include <stdlib.h>
#include <string.h>
#include "quirc_internal.h"

#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

void *quirc_zalloc_ext(size_t size)
{
#ifdef ESP_PLATFORM
  void *ptr = heap_caps_calloc(1, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ptr)
    return ptr;
#endif
  return calloc(1, size);
}

const char *quirc_version(void)
{
  return "1.0-psram";
}

struct quirc *quirc_new(void)
{
  return (struct quirc *)quirc_zalloc_ext(sizeof(struct quirc));
}

void quirc_destroy(struct quirc *q)
{
  if (!q)
    return;

  if (q->image)
    free(q->image);
  if (q->pixels && (void *)q->pixels != (void *)q->image)
    free(q->pixels);
  if (q->row_average)
    free(q->row_average);
  if (q->flood_stack)
    free(q->flood_stack);

  free(q);
}

int quirc_resize(struct quirc *q, int w, int h)
{
  if (!q || w <= 0 || h <= 0)
    return -1;

  if (q->image) {
    if (q->pixels && (void *)q->pixels != (void *)q->image)
      free(q->pixels);
    free(q->image);
  } else if (q->pixels) {
    free(q->pixels);
  }
  q->image = NULL;
  q->pixels = NULL;

  if (q->row_average) {
    free(q->row_average);
    q->row_average = NULL;
  }
  q->row_average_cap = 0;

  if (q->flood_stack) {
    free(q->flood_stack);
    q->flood_stack = NULL;
  }
  q->flood_stack_cap = 0;
  q->w = 0;
  q->h = 0;

  uint8_t *new_image = (uint8_t *)quirc_zalloc_ext((size_t)w * (size_t)h);
  if (!new_image)
    return -1;

  quirc_pixel_t *new_pixels = NULL;
  if (sizeof(*q->image) != sizeof(*q->pixels)) {
    new_pixels = (quirc_pixel_t *)quirc_zalloc_ext((size_t)w * (size_t)h * sizeof(quirc_pixel_t));
    if (!new_pixels) {
      free(new_image);
      return -1;
    }
  } else {
    new_pixels = (quirc_pixel_t *)new_image;
  }

  int *new_row_avg = (int *)calloc((size_t)w, sizeof(int));
  if (!new_row_avg) {
    new_row_avg = (int *)quirc_zalloc_ext((size_t)w * sizeof(int));
  }
  if (!new_row_avg) {
    if ((void *)new_pixels != (void *)new_image)
      free(new_pixels);
    free(new_image);
    return -1;
  }

  struct quirc_flood_span *new_stack = (struct quirc_flood_span *)quirc_zalloc_ext(
      QUIRC_FLOOD_STACK_CAP * sizeof(struct quirc_flood_span));
  if (!new_stack) {
    free(new_row_avg);
    if ((void *)new_pixels != (void *)new_image)
      free(new_pixels);
    free(new_image);
    return -1;
  }

  q->image = new_image;
  q->pixels = new_pixels;
  q->row_average = new_row_avg;
  q->row_average_cap = w;
  q->flood_stack = new_stack;
  q->flood_stack_cap = QUIRC_FLOOD_STACK_CAP;
  q->w = w;
  q->h = h;
  return 0;
}

int quirc_count(const struct quirc *q)
{
  return q ? q->num_grids : 0;
}

static const char *const error_table[] = {
    [QUIRC_SUCCESS] = "Success",
    [QUIRC_ERROR_INVALID_GRID_SIZE] = "Invalid grid size",
    [QUIRC_ERROR_INVALID_VERSION] = "Invalid version",
    [QUIRC_ERROR_FORMAT_ECC] = "Format data ECC failure",
    [QUIRC_ERROR_DATA_ECC] = "ECC failure",
    [QUIRC_ERROR_UNKNOWN_DATA_TYPE] = "Unknown data type",
    [QUIRC_ERROR_DATA_OVERFLOW] = "Data overflow",
    [QUIRC_ERROR_DATA_UNDERFLOW] = "Data underflow"};

const char *quirc_strerror(quirc_decode_error_t err)
{
  if (err >= 0 && (size_t)err < sizeof(error_table) / sizeof(error_table[0]))
    return error_table[err];

  return "Unknown error";
}
