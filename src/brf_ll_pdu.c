#include "brf_ll_pdu.h"

#include <assert.h>

uint32_t brf_ll_pdu_get_crc(brf_ll_pdu_t *pdu) {
  assert(pdu);
  uint32_t crc24;

  crc24 = 0;
  crc24 = ((crc24 << 8) | pdu->crc[2]);
  crc24 = ((crc24 << 8) | pdu->crc[1]);
  crc24 = ((crc24 << 8) | pdu->crc[0]);

  return crc24;
}
