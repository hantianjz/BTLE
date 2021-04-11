#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t brf_crc24_update(const uint8_t *data, size_t data_len, uint32_t crc);

uint32_t brf_crc24(const uint8_t *data, size_t data_len);

#ifdef __cplusplus
}
#endif
