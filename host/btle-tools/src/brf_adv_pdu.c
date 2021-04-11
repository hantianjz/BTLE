#include "brf_adv_pdu.h"

#include <assert.h>
#include <string.h>

static const char *s_adv_pdu_type_str[] = {
    "ADV_IND",     /* 0b0000 - ADV_IND        (Primary Advertising)     (LE 1M)
                    */
    "DIRECT_IND",  /* 0b0001 - ADV_DIRECT_IND (Primary Advertising)  (LE 1M)
                    */
    "NONCONN_IND", /* 0b0010 - ADV_NONCONN_IND(Primary Advertising) (LE 1M)
                    */
    "SCAN_REQ",    /* 0b0011 - SCAN_REQ       (Primary Advertising)    (LE 1M)
                    *        - AUX_SCAN_REQ   (Secondary advertising)    (LE
                    * 1M/2M/Coded)
                    */
    "SCAN_RSP",    /* 0b0100 - SCAN_RSP       (Primary Advertising)    (LE 1M)
                    */
    "CONNECT_IND", /* 0b0101 - CONNECT_IND    (Primary Advertising) (LE 1M)
                    *        - AUX_CONNECT_REQ(Secondary advertising) (LE
                    * 1M/2M/Coded)
                    */
    "SCAN_IND",    /* 0b0110 - ADV_SCAN_IND   (Primary Advertising)    (LE 1M)
                    */
    "EXT_IND",     /* 0b0111 - ADV_EXT_IND    (Primary Advertising)     (LE 1M)
                    *        - AUX_ADV_IND    (Secondary advertising)     (LE
                    * 1M/2M/Coded)
                    *        - AUX_SCAN_RSP   (Secondary advertising)     (LE
                    * 1M/2M/Coded)
                    *        - AUX_SYNC_IND   (Periodic)     (LE 1M/2M/Coded)
                    *        - AUX_CHAIN_IND  (Secondary advertising and
                    * Periodic)(LE 1M/2M/Coded)
                    */
    "CONNECT_RSP", /* 0b1000 - AUX_CONNECT_RSP(Secondary advertising) (LE
                    * 1M/2M/Coded)
                    */
    // TODO: Update for LE 2M or LE Coded
    "RESERVED0", "RESERVED1", "RESERVED2", "RESERVED3", "RESERVED4",
    "RESERVED5", "RESERVED6", "RESERVED7", "RESERVED8"};

const char *brf_adv_pdu_get_type_str(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return s_adv_pdu_type_str[brf_adv_pdu_get_type(pdu)];
}

brf_adv_pdu_type_t brf_adv_pdu_get_type(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return (brf_adv_pdu_type_t)(pdu->hdr.header & 0x0f);
}

bool brf_adv_pdu_get_chsel(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return (pdu->hdr.header & 0x20) > 0;
}

bool brf_adv_pdu_get_tx_add(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return (pdu->hdr.header & 0x40) > 0;
}

bool brf_adv_pdu_get_rx_add(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return (pdu->hdr.header & 0x80) > 0;
}

uint8_t brf_adv_pdu_get_len(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return pdu->hdr.length;
}

bool brf_adv_pdu_get_adv_a(brf_adv_pdu_t *pdu, uint8_t *byte_in,
                           size_t byte_in_size) {
  assert(pdu);
  assert(pdu->payload.buff_size);
  assert(byte_in_size >= BRF_ADV_PDU_ADV_A_SIZE);

  switch (brf_adv_pdu_get_type(pdu)) {
  case BRF_ADV_PDU_IND:
  case BRF_ADV_PDU_DIRECT_IND:
  case BRF_ADV_PDU_DIRECT_NONCONN_IND:
  case BRF_ADV_PDU_ADV_SCAN_IND:
  case BRF_ADV_PDU_SCAN_RSP:
    memcpy(byte_in, pdu->payload.buff, BRF_ADV_PDU_ADV_A_SIZE);
    break;
  case BRF_ADV_PDU_SCAN_REQ:
  case BRF_ADV_PDU_CONNECT_IND:
    memcpy(byte_in, &pdu->payload.buff[BRF_ADV_PDU_SCAN_A_SIZE],
           BRF_ADV_PDU_ADV_A_SIZE);
    break;
  case BRF_ADV_PDU_EXT_IND:
  case BRF_ADV_PDU_CONNECT_RSP:
  default:
    return false;
    break;
  }
  return true;
}
