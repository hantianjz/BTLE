#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct brf_shift_queue {
  size_t write_idx;
  size_t read_idx;

  size_t capacity;

  uint8_t *buff;
} brf_shift_queue_t;

void brf_shift_queue_init(brf_shift_queue_t *q, uint8_t *buff,
                          size_t buff_size);

size_t brf_shift_queue_size(brf_shift_queue_t *q);

size_t brf_shift_queue_remain(brf_shift_queue_t *q);

size_t brf_shift_queue_push(brf_shift_queue_t *q, const uint8_t *src,
                            size_t size);

bool brf_shift_queue_compare(brf_shift_queue_t *q, const uint8_t *src,
                             size_t size);

#ifdef __cplusplus
}
#endif
