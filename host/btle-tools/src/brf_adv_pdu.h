/*
 * Advertising physical channel PDU
 */
#pragma once

#include "brf_adv_pdu_payload.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  BRF_ADV_PDU_HDR_BIT_LEN = 16,

  BRF_ADV_PDU_HEADER_SIZE = 2,
  BRF_ADV_PDU_PAYLOAD_MAX_SIZE = 255,

  BRF_ADV_PDU_ADV_A_SIZE = 6,
  BRF_ADV_PDU_SCAN_A_SIZE = 6,
};

typedef enum {
  BRF_ADV_PDU_IND = 0x0,
  BRF_ADV_PDU_DIRECT_IND = 0x1,
  BRF_ADV_PDU_DIRECT_NONCONN_IND = 0x2,
  BRF_ADV_PDU_SCAN_REQ = 0x3,
  BRF_ADV_PDU_SCAN_RSP = 0x4,
  BRF_ADV_PDU_CONNECT_IND = 0x5,
  BRF_ADV_PDU_ADV_SCAN_IND = 0x6,
  BRF_ADV_PDU_EXT_IND = 0x7,
  BRF_ADV_PDU_CONNECT_RSP = 0x8,
} brf_adv_pdu_type_t;

typedef struct brf_adv_pdu_hdr {
  /*
   * Type : 4 bit
   * RFU  : 1 bit
   * ChSel: 1 bit
   * TxAdd: 1 bit
   * RxAdd: 1 bit
   */
  uint8_t header;
  uint8_t length;
} brf_adv_pdu_hdr_t;

typedef struct brf_adv_pdu_payload {
  union {
    brf_adv_pdu_payload_adv_t adv;
    brf_adv_pdu_payload_adv_direct_t adv_direct;
    brf_adv_pdu_payload_adv_nonconn_t adv_nonconn;
    brf_adv_pdu_payload_adv_scan_t adv_scan;
  } p;
  size_t buff_size;
  uint8_t buff[BRF_ADV_PDU_PAYLOAD_MAX_SIZE];
} brf_adv_pdu_payload_t;

typedef struct brf_adv_pdu {
  brf_adv_pdu_hdr_t hdr;
  brf_adv_pdu_payload_t payload;
} brf_adv_pdu_t;

const char *brf_adv_pdu_get_type_str(brf_adv_pdu_t *pdu);

brf_adv_pdu_type_t brf_adv_pdu_get_type(brf_adv_pdu_t *pdu);

bool brf_adv_pdu_get_rfu(brf_adv_pdu_t *pdu);

bool brf_adv_pdu_get_chsel(brf_adv_pdu_t *pdu);

bool brf_adv_pdu_get_tx_add(brf_adv_pdu_t *pdu);

bool brf_adv_pdu_get_rx_add(brf_adv_pdu_t *pdu);

uint8_t brf_adv_pdu_get_len(brf_adv_pdu_t *pdu);

uint32_t brf_adv_pdu_get_crc(brf_adv_pdu_t *pdu);

bool brf_adv_pdu_get_adv_a(brf_adv_pdu_t *pdu, uint8_t *byte_in,
                           size_t byte_in_size);

#ifdef __cplusplus
}
#endif
