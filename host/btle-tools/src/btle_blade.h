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
  DEFAULT_ACCESS_ADDR = (0x8E89BED6),
  DEFAULT_CRC_INIT = (0x555555),
  MAX_CHANNEL_NUMBER = 39,
  MAX_NUM_INFO_BYTE = (43),
  MAX_NUM_PHY_BYTE = (47),
  SAMPLE_PER_SYMBOL = 4,  // 4M sampling rate
  /* MAX_NUM_PHY_SAMPLE = ((MAX_NUM_PHY_BYTE * 8 * SAMPLE_PER_SYMBOL) + */
  /*                       (LEN_GAUSS_FILTER * SAMPLE_PER_SYMBOL)), */
  MAX_NUM_PHY_SAMPLE = (MAX_NUM_PHY_BYTE * 8 * SAMPLE_PER_SYMBOL),
  LEN_BUF_MAX_NUM_PHY_SAMPLE = (2 * MAX_NUM_PHY_SAMPLE),

  NUM_PREAMBLE_BYTE = (1),
  NUM_ACCESS_ADDR_BYTE = (4),
  NUM_PREAMBLE_ACCESS_BYTE = (NUM_PREAMBLE_BYTE + NUM_ACCESS_ADDR_BYTE),
};
/*----------------------------BTLE SPEC related---------------------------*/

typedef int16_t IQ_TYPE;
typedef int32_t IQ_SAMPLE;

/*----------------------Basic async stream buffer setup----------------------*/

#define SAMPLES_PER_BLOCK 0x1000
// 4096 samples = ~1ms for 4Msps; ATTENTION each rx callback get
// hackrf.c:lib_device->buffer_size samples!!!

#define PER_BLOCK_BUF_SIZE SAMPLES_PER_BLOCK * sizeof(IQ_SAMPLE)

#define NUM_OF_BLOCKS 4

#define MEGAHZ 1000000ull
/*----------------------some basic signal definition----------------------*/

/*----------------------BladeRF struct definition----------------------*/
// Forard declaration
struct bladerf_metadata;
struct bladerf;
struct bladerf_stream;

typedef struct brf_async_task {
  pthread_t rx_task;
  pthread_mutex_t stderr_lock;
  volatile bool do_exit;
} brf_async_task_t;

typedef struct brf_data {
  unsigned char rx_block_buf[PER_BLOCK_BUF_SIZE];
  bool new_sample;
} brf_data_t;

typedef struct brf {
  struct bladerf_stream *stream;
  void **stream_buffers; /* Transmit buffers */

  brf_async_task_t async_task;
  brf_data_t rx_data;
  struct bladerf *rf_dev;

  bool debug;
} brf_t;
/*----------------------BladeRF struct definition----------------------*/

uint64_t btle_get_freq_by_channel_number(int channel_number);

int btle_setup_board(brf_t *brf, uint64_t freq_hz, int gain);

void btle_teardown_board(brf_t *brf);

int btle_setup_stream(brf_t *brf);

void btle_teardown_stream(brf_t *brf);

void btle_sigint_callback_handler(int signum);

void *btle_stream_callback(struct bladerf *dev, struct bladerf_stream *stream,
                           struct bladerf_metadata *metadata, void *samples,
                           size_t num_samples, void *user_data);

void *btle_rx_task_run(void *ctx);

void btle_demod_byte(IQ_TYPE *rxp, int num_byte, uint8_t *out_byte);

#ifdef __cplusplus
}
#endif
