#pragma once

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------BTLE SPEC related----------------------*/
#include "scramble_table.h"
enum {
  DEFAULT_CHANNEL = 37,
  PREAMBLE_1M = (0xAA),
  DEFAULT_ACCESS_ADDR = (0x8E89BED6),
  DEFAULT_ACCESS_MASK = (0xFFFFFFFF),
  DEFAULT_CRC_INIT = (0x555555),
  MAX_CHANNEL_NUMBER = 39,
  MAX_NUM_INFO_BYTE = (43),
  MAX_NUM_PHY_BYTE = (47),
  IQ_PER_SYMBOL = 2,
  MAX_NUM_PHY_SAMPLE = (MAX_NUM_PHY_BYTE * 8 * IQ_PER_SYMBOL),
  LEN_BUF_MAX_NUM_PHY_SAMPLE = (2 * MAX_NUM_PHY_SAMPLE),

  NUM_DATA_PER_SYMBOL = 2,  // IQ Pair each uint16_t
  NUM_DATA_PER_SAMPLE = NUM_DATA_PER_SYMBOL * IQ_PER_SYMBOL,
  NUM_BIT_PER_BYTE = 8,
  NUM_PREAMBLE_BYTE = (1),
  NUM_ACCESS_ADDR_BYTE = (4),
  NUM_PREAMBLE_ACCESS_BYTE = (NUM_PREAMBLE_BYTE + NUM_ACCESS_ADDR_BYTE),
};

/*----------------------------BTLE SPEC related---------------------------*/

typedef int16_t IQ_TYPE;
typedef int32_t IQ_SAMPLE;

typedef struct iq_sample {
  IQ_TYPE I;
  IQ_TYPE Q;
} iq_sample_t;

/*----------------------Basic async stream buffer setup----------------------*/

enum {
  PER_BLOCK_SAMPLE_SIZE = 0x1000,
  SAMPLES_PER_BLOCK = (IQ_PER_SYMBOL * PER_BLOCK_SAMPLE_SIZE),
  PER_BLOCK_BUF_SIZE = (SAMPLES_PER_BLOCK * sizeof(IQ_SAMPLE)),
  NUM_OF_BLOCKS = 8,
};

#define MEGAHZ 1000000ull
/*----------------------some basic signal definition----------------------*/

/*----------------------BladeRF struct definition----------------------*/
// Forard declaration
struct bladerf_metadata;
struct bladerf;
struct bladerf_stream;

typedef struct brf_async_task {
  pthread_t rx_task;
  pthread_t demod_task;

  pthread_cond_t rx_data_cond;
  pthread_mutex_t rx_data_lock;

  pthread_mutex_t stderr_lock;
  volatile bool do_exit;
} brf_async_task_t;

typedef struct brf_data {
  unsigned char rx_block_buf[PER_BLOCK_BUF_SIZE];
  pthread_mutex_t lock;
  bool new_sample;
} brf_data_t;

typedef struct brf {
  struct bladerf_stream *stream;

  /* Transmit buffers are allocated by bladerf_init_stream().
   * It is defined as an array of pointers of len NUM_OF_BLOCKS
   * where each pointer in the array point to a block of memory of size
   * PER_BLOCK_BUF_SIZE. Hence it is helpful it keep track of a idx value
   * to keep track which one we currently using.
   * */
  void **stream_buffers;
  int stream_buffers_idx;

  brf_async_task_t async_tasks;
  brf_data_t rx_data[NUM_OF_BLOCKS];
  struct bladerf *rf_dev;

  int channel_number;

  bool debug;

  uint8_t preamble_bit_array[NUM_BIT_PER_BYTE];                        // 8
  uint8_t access_addr_bit_array[sizeof(uint32_t) * NUM_BIT_PER_BYTE];  // 32
  uint8_t access_mask_bit_array[sizeof(uint32_t) * NUM_BIT_PER_BYTE];  // 32
} brf_t;
/*----------------------BladeRF struct definition----------------------*/

uint64_t btle_get_freq_by_channel_number(int channel_number);

int btle_setup_board(brf_t *brf, uint64_t freq_hz, int gain);

void btle_teardown_board(brf_t *brf);

int btle_setup_stream(brf_t *brf);

void btle_teardown_stream(brf_t *brf);

int btle_setup_demod(brf_t *brf);

void scramble_byte(uint8_t *bytes, int num_byte,
                   const uint8_t *scramble_table_byte);

void btle_sigint_callback_handler(int signum);

void *btle_stream_callback(struct bladerf *dev, struct bladerf_stream *stream,
                           struct bladerf_metadata *metadata, void *samples,
                           size_t num_samples, void *user_data);

void brf_bit_array_to_bytes(uint8_t *bit_array, size_t bit_array_size,
                            uint8_t *bytes, size_t bytes_size);

void brf_uint8_to_bit_array(uint8_t uint8_in, uint8_t *bit_array,
                            size_t bit_array_size);

void brf_uint32_to_bit_array(uint32_t uint32_in, uint8_t *bit_array,
                             size_t bit_array_size);

void btle_print_xxd(uint8_t *data, size_t len);

void btle_print_mac_addr(uint8_t *data, size_t len);

uint8_t *brf_search_bit_array_pattern(uint8_t *data, size_t data_len,
                                      uint8_t *pattern, size_t pattern_len);

int brf_search_unique_bits(const iq_sample_t *iq_buff, size_t iq_count,
                           const uint8_t *unique_bits,
                           const uint8_t *unique_bits_mask,
                           size_t unique_bits_size);

void *btle_rx_task_run(void *ctx);

void *btle_demod_task_run(void *ctx);

uint8_t btle_demod_bit(const iq_sample_t *iq, size_t num_samples);

size_t btle_demod_bits(iq_sample_t *iq, size_t iq_size, uint8_t *out_bits,
                       size_t out_bit_size);

size_t btle_demod_bytes(iq_sample_t *iq, size_t iq_size, uint8_t *out_bytes,
                        size_t out_byte_size);

void btle_demod_process(brf_t *brf, brf_data_t *data);

#ifdef __cplusplus
}
#endif
