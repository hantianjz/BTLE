#include "btle_blade.h"

#include <libbladeRF.h>

// System includes
#include <byteswap.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
/* #include <ctype.h> */
/* #include <errno.h> */
/* #include <fcntl.h> */
/* #include <math.h> */
/* #include <netinet/in.h> */
/* #include <stdlib.h> */
/* #include <string.h> */
/* #include <sys/stat.h> */
/* #include <sys/types.h> */

/* Thread-safe wrapper around fprintf(stderr, ...) */
#define brf_print(repeater_, ...)                    \
  do {                                               \
    pthread_mutex_lock(&(repeater_)->stderr_lock);   \
    fprintf(stdout, __VA_ARGS__);                    \
    fflush(stdout);                                  \
    pthread_mutex_unlock(&(repeater_)->stderr_lock); \
  } while (0)

#define MAX_GAIN 60
#define DEFAULT_GAIN 45

volatile int rx_buf_offset;  // remember to initialize it!

static brf_t s_brf;

void btle_sigint_callback_handler(int signum) {
  fprintf(stderr, "\nCaught signal %d\n", signum);
  s_brf.async_tasks.do_exit = true;
}

uint64_t btle_get_freq_by_channel_number(int channel_number) {
  uint64_t freq_hz;
  if (channel_number == 37) {
    freq_hz = 2402 * MEGAHZ;
  } else if (channel_number == 38) {
    freq_hz = 2426 * MEGAHZ;
  } else if (channel_number == 39) {
    freq_hz = 2480 * MEGAHZ;
  } else if ((channel_number >= 0) && (channel_number <= 10)) {
    freq_hz = (2404 * MEGAHZ) + (channel_number * (2 * MEGAHZ));
  } else if ((channel_number >= 11) && (channel_number <= 36)) {
    freq_hz = (2428 * MEGAHZ) + ((channel_number - 11) * (2 * MEGAHZ));
  } else {
    freq_hz = 0xffffffffffffffff;
  }
  return (freq_hz);
}

void *btle_stream_callback(struct bladerf *dev, struct bladerf_stream *stream,
                           struct bladerf_metadata *metadata, void *samples,
                           size_t num_samples, void *user_data) {
  (void)dev;
  (void)stream;
  (void)metadata;
  (void)num_samples;
  brf_t *brf = (brf_t *)user_data;

  /* brf_print(&brf->async_tasks, "%s: samples(%p) count expected(%x)
   * got(%lx)\n", */
  /*           __func__, samples, SAMPLES_PER_BLOCK, num_samples); */

  brf_data_t *rx_data = NULL;
  for (unsigned i = 0; i < sizeof(brf->rx_data) / sizeof(brf->rx_data[0]);
       i++) {
    if (!brf->rx_data[i].new_sample) {
      rx_data = &brf->rx_data[i];
      break;
    }
  }

  if (rx_data) {
    pthread_mutex_lock(&rx_data->lock);
    memcpy(rx_data->rx_block_buf, samples, PER_BLOCK_BUF_SIZE);
    rx_data->new_sample = true;
    pthread_mutex_unlock(&rx_data->lock);

    pthread_cond_signal(&brf->async_tasks.rx_data_cond);
  } else {
    brf_print(&brf->async_tasks,
              "btle_stream_callback: skipping samples due to low buffer.\n");
  }

  if (brf->debug) {
    brf->async_tasks.do_exit = true;
  }

  if (brf->async_tasks.do_exit) {
    return BLADERF_STREAM_SHUTDOWN;
  }

  brf->stream_buffers_idx = (brf->stream_buffers_idx + 1) % NUM_OF_BLOCKS;
  return brf->stream_buffers[brf->stream_buffers_idx];
}

void brf_uint32_to_bit_array(uint32_t uint32_in, uint8_t *bit) {
  for (size_t i = 0; i < sizeof(uint32_in) * NUM_BIT_PER_BYTE; i++) {
    bit[i] = 0x01 & uint32_in;
    uint32_in = (uint32_in >> 1);
  }
}

void btle_print_xxd(uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    printf("%02x", data[i]);
    if (i % 4 == 3) {
      printf(" ");
    }
    if (i % 16 == 15) {
      printf("\n");
    }
  }
  printf("\n");
}

uint8_t *brf_search_bit_array_pattern(uint8_t *data, size_t data_len,
                                      uint8_t *pattern, size_t pattern_len) {
  for (size_t i = 0; i < (data_len - pattern_len); i++) {
    if (!memcmp(&data[i], pattern, pattern_len)) {
      return &data[i];
    }
  }
  return NULL;
}

inline int search_unique_bits(IQ_TYPE *rxp, unsigned search_len,
                              uint8_t *unique_bits, uint8_t *unique_bits_mask,
                              const int num_bits) {
  int sp, i0, q0, i1, q1, k, p, phase_idx;
  bool unequal_flag;
  const int demod_buf_len = num_bits;
  int demod_buf_offset = 0;

  uint8_t demod_buf_access[SAMPLE_PER_SYMBOL][LEN_DEMOD_BUF_ACCESS];

  // demod_buf_preamble_access[SAMPLE_PER_SYMBOL][LEN_DEMOD_BUF_PREAMBLE_ACCESS]
  // memset(demod_buf_preamble_access, 0,
  // SAMPLE_PER_SYMBOL*LEN_DEMOD_BUF_PREAMBLE_ACCESS);
  memset(demod_buf_access, 0, SAMPLE_PER_SYMBOL * LEN_DEMOD_BUF_ACCESS);
  for (unsigned i = 0; i < search_len * SAMPLE_PER_SYMBOL * 2;
       i = i + (SAMPLE_PER_SYMBOL * 2)) {
    sp = ((demod_buf_offset - demod_buf_len + 1) & (demod_buf_len - 1));
    // sp = (demod_buf_offset-demod_buf_len+1);
    // if (sp>=demod_buf_len)
    //  sp = sp - demod_buf_len;

    for (unsigned j = 0; j < (SAMPLE_PER_SYMBOL * 2); j = j + 2) {
      i0 = rxp[i + j];
      q0 = rxp[i + j + 1];
      i1 = rxp[i + j + 2];
      q1 = rxp[i + j + 3];

      phase_idx = j / 2;
      // demod_buf_preamble_access[phase_idx][demod_buf_offset] = (i0*q1 -
      // i1*q0) > 0? 1: 0;
      demod_buf_access[phase_idx][demod_buf_offset] =
          (i0 * q1 - i1 * q0) > 0 ? 1 : 0;

      k = sp;
      unequal_flag = false;
      for (p = 0; p < demod_buf_len; p++) {
        // if (demod_buf_preamble_access[phase_idx][k] != unique_bits[p]) {
        if (demod_buf_access[phase_idx][k] != unique_bits[p] &&
            unique_bits_mask[p]) {
          unequal_flag = true;
          break;
        }
        k = ((k + 1) & (demod_buf_len - 1));
        // k = (k + 1);
        // if (k>=demod_buf_len)
        //  k = k - demod_buf_len;
      }

      if (unequal_flag == false) {
        return (i + j - (demod_buf_len - 1) * SAMPLE_PER_SYMBOL * 2);
      }
    }

    demod_buf_offset = ((demod_buf_offset + 1) & (demod_buf_len - 1));
    // demod_buf_offset  = (demod_buf_offset+1);
    // if (demod_buf_offset>=demod_buf_len)
    //  demod_buf_offset = demod_buf_offset - demod_buf_len;
  }

  return (-1);
}

size_t btle_demod_bits(IQ_TYPE *iq, size_t num_samples, uint8_t *out_bits,
                       size_t out_bit_size) {
  size_t out_bit_idx = 0;
  size_t sample_idx = 0;
  while (((sample_idx + 1) < num_samples) && (out_bit_idx < out_bit_size)) {
    IQ_TYPE I0 = iq[sample_idx];
    IQ_TYPE Q0 = iq[sample_idx + 1];
    IQ_TYPE I1 = iq[sample_idx + 2];
    IQ_TYPE Q1 = iq[sample_idx + 3];

    uint8_t bit_decision = (I0 * Q1 - I1 * Q0) > 0 ? 1 : 0;
    /* printf("I0(%d); Q0(%d); I1(%d); Q1(%d) == %d\n", I0, Q0, I1, Q1, */
    /*        bit_decision); */
    out_bits[out_bit_idx] = bit_decision;
    out_bit_idx++;
    sample_idx += 4;
  }

  return out_bit_idx;
}

void *btle_rx_task_run(void *ctx) {
  int status;
  brf_t *brf = (brf_t *)ctx;

  brf_print(&brf->async_tasks, "Start RX bladerf stream.\n");

  /* Start stream and stay there until we kill the stream */
  status = bladerf_stream(brf->stream, BLADERF_MODULE_RX);
  if (status < 0) {
    brf_print(&brf->async_tasks, "RX stream failure: %s\r\n",
              bladerf_strerror(status));
  }

  brf_print(&brf->async_tasks, "Exiting RX bladerf stream: %d.\n", status);
  return NULL;
}

void *btle_demod_task_run(void *ctx) {
  brf_t *brf = (brf_t *)ctx;

  while (!s_brf.async_tasks.do_exit) {
    brf_data_t *rx_data = NULL;
    for (unsigned i = 0; i < sizeof(brf->rx_data) / sizeof(brf->rx_data[0]);
         i++) {
      if (brf->rx_data[i].new_sample) {
        rx_data = &brf->rx_data[i];
        break;
      }
    }

    if (!rx_data) {
      pthread_cond_wait(&brf->async_tasks.rx_data_cond,
                        &brf->async_tasks.rx_data_lock);
    } else {
      pthread_mutex_lock(&rx_data->lock);
      btle_demod_process(brf, rx_data);
      rx_data->new_sample = false;
      pthread_mutex_unlock(&rx_data->lock);
    }
  }

  brf_print(&brf->async_tasks, "Exiting RX demod task\n");
  return NULL;
}

int btle_setup_board(brf_t *brf, uint64_t freq_hz, int gain) {
  int status;
  unsigned int actual;

  if (signal(SIGINT, btle_sigint_callback_handler) == SIG_ERR ||
      signal(SIGTERM, btle_sigint_callback_handler) == SIG_ERR) {
    fprintf(stderr, "Failed to set up signal handler\n");
    goto exit_failure;
  }

  status = bladerf_open(&brf->rf_dev, NULL);
  if (status < 0) {
    fprintf(stderr, "Failed to open device: %s\n", bladerf_strerror(status));
    goto exit_failure;
  } else {
    fprintf(stdout, "open device: %s\n", bladerf_strerror(status));
  }

  status = bladerf_is_fpga_configured(brf->rf_dev);
  if (status < 0) {
    fprintf(stderr, "Failed to determine FPGA state: %s\n",
            bladerf_strerror(status));
    goto exit_close_blade;
  } else if (status == 0) {
    fprintf(stderr, "Error: FPGA is not loaded.\n");
    goto exit_close_blade;
  } else {
    fprintf(stdout, "FPGA is loaded.\n");
  }

  status = bladerf_set_frequency(brf->rf_dev, BLADERF_MODULE_RX, freq_hz);
  if (status != 0) {
    fprintf(stderr, "Failed to set frequency: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "set frequency: %luHz %s\n", freq_hz,
            bladerf_strerror(status));
  }

  status = bladerf_set_sample_rate(brf->rf_dev, BLADERF_MODULE_RX,
                                   SAMPLE_PER_SYMBOL * 1000000ul, &actual);
  if (status != 0) {
    fprintf(stderr, "Failed to set sample rate: %s\n",
            bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "set sample rate: %dHz %s\n", actual,
            bladerf_strerror(status));
  }

  status = bladerf_set_bandwidth(brf->rf_dev, BLADERF_MODULE_RX,
                                 SAMPLE_PER_SYMBOL * 1000000ul / 2, &actual);
  if (status != 0) {
    fprintf(stderr, "Failed to set bandwidth: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "bladerf_set_bandwidth: %d %s\n", actual,
            bladerf_strerror(status));
  }

  status = bladerf_set_gain(brf->rf_dev, BLADERF_MODULE_RX, gain);
  if (status != 0) {
    fprintf(stderr, "Failed to set gain: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "bladerf_set_gain: %d %s\n", gain,
            bladerf_strerror(status));
  }

  status = bladerf_get_gain(brf->rf_dev, BLADERF_MODULE_RX, &gain);
  if (status != 0) {
    fprintf(stderr, "Failed to get gain: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "bladerf_get_gain: %d %s\n", gain,
            bladerf_strerror(status));
  }
  return EXIT_SUCCESS;

exit_close_blade:
  if (brf->rf_dev) {
    bladerf_close(brf->rf_dev);
  }

exit_failure:
  return EXIT_FAILURE;
}

void btle_teardown_board(brf_t *brf) {
  int status;

  status = bladerf_enable_module(brf->rf_dev, BLADERF_MODULE_RX, false);
  if (status < 0) {
    fprintf(stderr, "Failed to enable module: %s\n", bladerf_strerror(status));
  } else {
    fprintf(stdout, "enable module false: %s\n", bladerf_strerror(status));
  }

  bladerf_close(brf->rf_dev);
  printf("bladerf_close.\n");
}

int btle_setup_stream(brf_t *brf) {
  int status;

  /* Initialize the stream */
  status = bladerf_init_stream(
      &brf->stream, brf->rf_dev, btle_stream_callback, &brf->stream_buffers,
      NUM_OF_BLOCKS, BLADERF_FORMAT_SC16_Q11, SAMPLES_PER_BLOCK, 1, brf);
  brf->stream_buffers_idx = 0;

  if (status != 0) {
    fprintf(stderr, "Failed to init stream: %s\n", bladerf_strerror(status));
    goto exit_failure;
  } else {
    fprintf(stdout, "init stream: %s\n", bladerf_strerror(status));
  }

  bladerf_set_stream_timeout(brf->rf_dev, BLADERF_RX, 100);

  status = bladerf_enable_module(brf->rf_dev, BLADERF_MODULE_RX, true);
  if (status < 0) {
    fprintf(stderr, "Failed to enable module: %s\n", bladerf_strerror(status));
    bladerf_deinit_stream(brf->stream);
    goto exit_failure;
  } else {
    fprintf(stdout, "enable module true: %s\n", bladerf_strerror(status));
  }

  status =
      pthread_create(&(brf->async_tasks.rx_task), NULL, btle_rx_task_run, brf);
  if (status < 0) {
    goto exit_failure;
  }

  return EXIT_SUCCESS;

exit_failure:
  return EXIT_FAILURE;
}

void btle_teardown_stream(brf_t *brf) {
  pthread_join(brf->async_tasks.demod_task, NULL);
  pthread_join(brf->async_tasks.rx_task, NULL);

  bladerf_deinit_stream(brf->stream);
  printf("bladerf_deinit_stream.\n");

  // pthread_cancel(async_tasks.rx_task);
  printf("bladeRF rx thread quit.\n");
}

int btle_setup_demod(brf_t *brf) {
  pthread_cond_init(&brf->async_tasks.rx_data_cond, NULL);

  int status = pthread_create(&(brf->async_tasks.demod_task), NULL,
                              btle_demod_task_run, brf);
  if (status < 0) {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

void btle_demod_process(brf_t *brf, brf_data_t *rx_data) {
  int num_symbol_left =
      sizeof(rx_data->rx_block_buf) / (SAMPLE_PER_SYMBOL * 2);  // 2 for IQ
  int ret =
      search_unique_bits((IQ_TYPE *)rx_data->rx_block_buf, num_symbol_left,
                         brf->access_addr_bit_array, brf->access_mask_bit_array,
                         LEN_DEMOD_BUF_ACCESS);
  if (ret > 0) {
    brf_print(&brf->async_tasks, "+");
  } else {
    brf_print(&brf->async_tasks, ".");
  }

  /*
  uint8_t demod_bits[SAMPLES_PER_BLOCK / SAMPLE_PER_SYMBOL];

  brf_print(&brf->async_tasks, ".");
  size_t demod_bits_count =
      btle_demod_bits((IQ_TYPE *)rx_data->rx_block_buf, SAMPLES_PER_BLOCK,
                      demod_bits, sizeof(demod_bits));

  uint8_t *aligned_bits = brf_search_bit_array_pattern(
      demod_bits, demod_bits_count, brf->access_addr_bit_array,
      sizeof(brf->access_addr_bit_array));
  if (aligned_bits) {
    brf_print(&brf->async_tasks, "Found bits at (%p) bytes offset (%lu)\n",
              aligned_bits, aligned_bits - demod_bits);
  }
  */
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  uint64_t freq_hz;
  int gain, chan, phase, verbose_flag, raw_flag, hop_flag;
  uint32_t access_addr, access_addr_mask, crc_init;
  (void)phase;
  (void)hop_flag;
  (void)access_addr_mask;

  char *filename_pcap = NULL;

  // Default values
  chan = DEFAULT_CHANNEL;

  gain = DEFAULT_GAIN;

  access_addr = DEFAULT_ACCESS_ADDR;

  crc_init = 0x555555;

  verbose_flag = 0;

  raw_flag = 0;

  freq_hz = 123;

  access_addr_mask = 0xFFFFFFFF;

  hop_flag = 0;

  filename_pcap = NULL;

  phase = 0;

  if (freq_hz == 123) {
    freq_hz = btle_get_freq_by_channel_number(chan);
  }

  printf(
      "Cmd line input: chan %d, freq %lldMHz, access addr %08x, crc init %06x "
      "raw %d verbose %d rx %ddB file=%s\n",
      chan, freq_hz / MEGAHZ, access_addr, crc_init, raw_flag, verbose_flag,
      gain, filename_pcap);

  // run cyclic recv in background
  s_brf.async_tasks.do_exit = false;
  if (btle_setup_board(&s_brf, freq_hz, gain) != EXIT_SUCCESS) {
    if (s_brf.rf_dev != NULL) {
      goto program_quit;
    } else {
      return (1);
    }
  }

  s_brf.debug = false;

  brf_uint32_to_bit_array(DEFAULT_ACCESS_ADDR, s_brf.access_addr_bit_array);
  btle_print_xxd(s_brf.access_addr_bit_array,
                 sizeof(s_brf.access_addr_bit_array));

  brf_uint32_to_bit_array(DEFAULT_ACCESS_MASK, s_brf.access_mask_bit_array);
  btle_print_xxd(s_brf.access_mask_bit_array,
                 sizeof(s_brf.access_mask_bit_array));

  if (btle_setup_stream(&s_brf) != EXIT_SUCCESS) {
    goto program_quit;
  }

  if (btle_setup_demod(&s_brf) != EXIT_SUCCESS) {
    goto program_quit;
  }

  // scan
  s_brf.async_tasks.do_exit = false;
  rx_buf_offset = 0;
  while (s_brf.async_tasks.do_exit == false) {
    sleep(1);
  }

program_quit:
  printf("Exit main loop ...\n");
  btle_teardown_stream(&s_brf);
  btle_teardown_board(&s_brf);

  /* if (fh_pcap_store) fclose(fh_pcap_store); */
  return (0);
}
