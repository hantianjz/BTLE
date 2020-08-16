#pragma once

#include <pthread.h>
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

/*----------------------some basic signal definition----------------------*/

#define LEN_BUF_IN_SAMPLE \
  (4 * 4096)  // 4096 samples = ~1ms for 4Msps; ATTENTION each rx callback get
              // hackrf.c:lib_device->buffer_size samples!!!
#define LEN_BUF (LEN_BUF_IN_SAMPLE * 2)
#define LEN_BUF_IN_SYMBOL (LEN_BUF_IN_SAMPLE / SAMPLE_PER_SYMBOL)
#define MEGAHZ 1000000ull
/*----------------------some basic signal definition----------------------*/

typedef int8_t IQ_TYPE;

/*----------------------BladeRF struct definition----------------------*/
struct bladerf_metadata;
typedef struct bladerf_stream bladerf_stream_t;
typedef struct bladerf bladerf_t;

typedef struct bladerf_async_task {
  pthread_t rx_task;
  pthread_mutex_t stderr_lock;
} bladerf_async_task_t;

typedef struct bladerf_data {
  void **buffers;            /* Transmit buffers */
  size_t num_buffers;        /* Number of buffers */
  size_t samples_per_buffer; /* Number of samples per buffer */
  unsigned int idx;          /* The next one that needs to go out */
  volatile IQ_TYPE rx_buf[LEN_BUF + LEN_BUF_MAX_NUM_PHY_SAMPLE];
  //    bladerf_module      module;         /* Direction */
  //    FILE                *fout;          /* Output file (RX only) */
  //    ssize_t             samples_left;   /* Number of samples left */
} bladerf_data_t;

typedef struct bladerf {
  bladerf_stream_t *stream;
  bladerf_async_task_t async_task;
  bladerf_data_t rx_data;
  bladerf_t *rf_dev;
} bladerf_t;
/*----------------------BladeRF struct definition----------------------*/

uint64_t btle_get_freq_by_channel_number(int channel_number);

int btle_setup_board(bladerf_t *blade, uint64_t freq_hz, int gain);

void btle_teardown_board(bladerf_t *blade);

void btle_sigint_callback_handler(int signum);

void *btle_stream_callback(struct bladerf *dev, struct bladerf_stream *stream,
                           struct bladerf_metadata *metadata, void *samples,
                           size_t num_samples, void *user_data);

void *btle_rx_task_run(void *ctx);

#ifdef __cplusplus
}
#endif
