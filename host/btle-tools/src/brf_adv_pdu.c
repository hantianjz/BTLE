#include "brf_adv_pdu.h"

#include <assert.h>
#include <string.h>

static const char *s_adv_pdu_type_str[] = {
    "ADV_IND",   "ADV_DIRECT_IND", "ADV_NONCONN_IND", "SCAN_REQ",
    "SCAN_RSP",  "CONNECT_REQ",    "ADV_SCAN_IND",    "RESERVED0",
    "RESERVED1", "RESERVED2",      "RESERVED3",       "RESERVED4",
    "RESERVED5", "RESERVED6",      "RESERVED7",       "RESERVED8"};

const char *brf_adv_pdu_get_type_str(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return s_adv_pdu_type_str[brf_adv_pdu_get_type(pdu)];
}

brf_adv_pdu_type_t brf_adv_pdu_get_type(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return (brf_adv_pdu_type_t)(pdu->hdr.header & 0x0f);
}

bool brf_adv_pdu_get_rfu(brf_adv_pdu_t *pdu) {
  assert(pdu);
  return (pdu->hdr.header & 0x10) > 0;
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
  return pdu->hdr.length & 0x3F;
}

uint32_t brf_adv_pdu_get_crc(brf_adv_pdu_t *pdu) {
  assert(pdu);
  uint32_t crc24;

  crc24 = 0;
  crc24 = ((crc24 << 8) | pdu->crc[2]);
  crc24 = ((crc24 << 8) | pdu->crc[1]);
  crc24 = ((crc24 << 8) | pdu->crc[0]);

  return crc24;
}

bool brf_adv_pdu_get_adv_a(brf_adv_pdu_t *pdu, uint8_t *byte_in,
                           size_t byte_in_size) {
  assert(pdu);
  assert(byte_in_size >= BRF_ADV_PDU_ADV_A_SIZE);

  switch (brf_adv_pdu_get_type(pdu)) {
    case BRF_ADV_PDU_IND:
    case BRF_ADV_PDU_DIRECT_IND:
    case BRF_ADV_PDU_DIRECT_NONCONN_IND:
    case BRF_ADV_PDU_ADV_SCAN_IND:
    case BRF_ADV_PDU_SCAN_RSP:
      memcpy(byte_in, pdu->payload, BRF_ADV_PDU_ADV_A_SIZE);
      break;
    case BRF_ADV_PDU_SCAN_REQ:
      memcpy(byte_in, &pdu->payload[BRF_ADV_PDU_SCAN_A_SIZE],
             BRF_ADV_PDU_ADV_A_SIZE);
      break;
    case BRF_ADV_PDU_EXT_IND:
    case BRF_ADV_PDU_CONNECT_IND:
    case BRF_ADV_PDU_CONNECT_RSP:
    default:
      return false;
      break;
  }
  return true;
}
