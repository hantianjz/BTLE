/*
 * Link layer PDU
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  BRF_LL_PDU_CRC_SIZE = 3,
};

typedef struct brf_ll_pdu {
  uint8_t crc[BRF_LL_PDU_CRC_SIZE];
} brf_ll_pdu_t;

uint32_t brf_ll_pdu_get_crc(brf_ll_pdu_t *pdu);

#ifdef __cplusplus
}
#endif
