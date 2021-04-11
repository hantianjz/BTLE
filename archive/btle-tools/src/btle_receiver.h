#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct btle_receiver {
  int pkt_avaliable;
  int hop;
  int new_chm_flag;
  int interval;
  uint32_t access_addr;
  uint32_t crc_init;
  uint8_t chm[5];
  bool crc_ok;
} btle_receiver_t;

#define LEN_DEMOD_BUF_PREAMBLE_ACCESS \
  ((NUM_PREAMBLE_ACCESS_BYTE * 8) - 8)  // to get 2^x integer
/* #define LEN_DEMOD_BUF_PREAMBLE_ACCESS 32 */
/* #define LEN_DEMOD_BUF_PREAMBLE_ACCESS (NUM_PREAMBLE_ACCESS_BYTE * 8) */
#define LEN_DEMOD_BUF_ACCESS (NUM_ACCESS_ADDR_BYTE * 8)  // 32 = 2^5

#ifdef __cplusplus
}
#endif
