#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  BRF_ADV_PDU_PAYLOAD_ADD_SIZE = 6,
  BRF_ADV_PDU_PAYLOAD_ADV_DATA_MAX_SIZE = 31,
  BRF_ADV_PDU_PAYLOAD_COMMON_EXT_ADV_PAYLOAD_HEADER_MAX_SIZE = 63,
  BRF_ADV_PDU_PAYLOAD_COMMON_EXT_ADV_PAYLOAD_DATA_MAX_SIZE = 254,
};

typedef struct brf_adv_pdu_payload_adv {
  uint8_t *adv_a; // Fixed 6 bytes

  uint8_t *adv_data;
  size_t adv_data_len;
} brf_adv_pdu_payload_adv_t;

typedef struct brf_adv_pdu_payload_adv_direct {
  uint8_t *adv_a;    // Fixed 6 bytes
  uint8_t *target_a; // Fixed 6 bytes
} brf_adv_pdu_payload_adv_direct_t;

typedef struct brf_adv_pdu_payload_adv_nonconn {
  uint8_t *adv_a; // Fixed 6 bytes

  uint8_t *adv_data;
  size_t adv_data_len;
} brf_adv_pdu_payload_adv_nonconn_t;

typedef struct brf_adv_pdu_payload_adv_scan {
  uint8_t *adv_a; // Fixed 6 bytes

  uint8_t *adv_data;
  size_t adv_data_len;
} brf_adv_pdu_payload_adv_scan_t;

#ifdef __cplusplus
}
#endif
