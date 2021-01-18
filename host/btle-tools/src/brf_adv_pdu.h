#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  BRF_ADV_PDU_HDR_BIT_LEN = 16,
  BRF_ADV_PDU_PAYLOAD_MAX_SIZE = 37,
  BRF_ADV_PDU_CRC_SIZE = 3,

  BRF_ADV_PDU_ADV_A_SIZE = 6,
  BRF_ADV_PDU_SCAN_A_SIZE = 6,
};

typedef enum {
  BRF_ADV_PDU_IND,
  BRF_ADV_PDU_DIRECT_IND,
  BRF_ADV_PDU_DIRECT_NONCONN_IND,
  BRF_ADV_PDU_SCAN_REQ,
  BRF_ADV_PDU_SCAN_RSP,
  BRF_ADV_PDU_CONNECT_IND,
  BRF_ADV_PDU_ADV_SCAN_IND,
  BRF_ADV_PDU_EXT_IND,
  BRF_ADV_PDU_CONNECT_RSP,
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

typedef struct brf_adv_pdu {
  brf_adv_pdu_hdr_t hdr;
  uint8_t payload[BRF_ADV_PDU_PAYLOAD_MAX_SIZE];
  uint8_t crc[BRF_ADV_PDU_CRC_SIZE];
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
