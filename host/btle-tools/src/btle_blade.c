#include "btle_blade.h"

#include <libbladeRF.h>

// System includes
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
    fprintf(stderr, __VA_ARGS__);                    \
    pthread_mutex_unlock(&(repeater_)->stderr_lock); \
  } while (0)

#define MAX_GAIN 60
#define DEFAULT_GAIN 45

volatile int rx_buf_offset;  // remember to initialize it!

static brf_t s_brf;

void btle_sigint_callback_handler(int signum) {
  fprintf(stdout, "Caught signal %d\n", signum);
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

void btle_demod_byte(IQ_TYPE *rxp, int num_byte, uint8_t *out_byte) {
  int i, j;
  int I0, Q0, I1, Q1;
  uint8_t bit_decision;
  int sample_idx = 0;

  for (i = 0; i < num_byte; i++) {
    out_byte[i] = 0;
    for (j = 0; j < 8; j++) {
      I0 = rxp[sample_idx];
      Q0 = rxp[sample_idx + 1];
      I1 = rxp[sample_idx + 2];
      Q1 = rxp[sample_idx + 3];
      bit_decision = (I0 * Q1 - I1 * Q0) > 0 ? 1 : 0;
      out_byte[i] = out_byte[i] | (bit_decision << j);

      sample_idx = sample_idx + SAMPLE_PER_SYMBOL * 2;
    }
  }
}

void *btle_stream_callback(struct bladerf *dev, struct bladerf_stream *stream,
                           struct bladerf_metadata *metadata, void *samples,
                           size_t num_samples, void *user_data) {
  (void)dev;
  (void)stream;
  (void)metadata;
  (void)samples;
  (void)num_samples;
  (void)user_data;
  brf_t *brf = (brf_t *)user_data;

  brf_print(&brf->async_task,
            "btle_stream_callback: sample count expected(%x) got(%lx)\n",
            SAMPLES_PER_BLOCK, num_samples);

  brf_data_t *rx_data = &brf->rx_data;
  memcpy(rx_data->rx_block_buf, samples, PER_BLOCK_BUF_SIZE);
  rx_data->new_sample = true;

  if (brf->debug) {
    return BLADERF_STREAM_SHUTDOWN;
  }

  if (brf->async_task.do_exit) {
    return BLADERF_STREAM_SHUTDOWN;
  }

  return samples;
}

void btle_demod_byte(IQ_TYPE *rxp, int num_byte, uint8_t *out_byte) {
  int *sample = (int32_t *)samples;
  for (size_t i = 0; i < num_samples; i++) {
    int16_t Q = (((*sample) & 0xffff0000) >> 16);
    int16_t I = (((*sample) & 0x0000ffff));
    /* rx_buf[rx_buf_offset] = (((*sample) >> 4) & 0xFF); */
    /* rx_buf[rx_buf_offset + 1] = (((*(sample + 1)) >> 4) & 0xFF); */
    /* rx_buf_offset = (rx_buf_offset + 2) & (LEN_BUF - 1);  // cyclic buffer */
    /* sample++; */
  }
}

void *btle_rx_task_run(void *ctx) {
  int status;
  brf_t *brf = (brf_t *)ctx;

  brf_print(&brf->async_task, "Start RX bladerf stream.\n");

  /* Start stream and stay there until we kill the stream */
  status = bladerf_stream(brf->stream, BLADERF_MODULE_RX);
  if (status < 0) {
    brf_print(&brf->async_task, "RX stream failure: %s\r\n",
              bladerf_strerror(status));
  }

  brf_print(&brf->async_task, "Existing RX bladerf stream.\n");
  s_brf.async_task.do_exit = true;
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
      pthread_create(&(brf->async_task.rx_task), NULL, btle_rx_task_run, brf);
  if (status < 0) {
    goto exit_failure;
  }

  return EXIT_SUCCESS;

exit_failure:
  return EXIT_FAILURE;
}

void btle_teardown_stream(brf_t *brf) {
  bladerf_deinit_stream(brf->stream);
  printf("bladerf_deinit_stream.\n");

  pthread_join(brf->async_task.rx_task, NULL);
  // pthread_cancel(async_task.rx_task);
  printf("bladeRF rx thread quit.\n");
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
  s_brf.async_task.do_exit = false;
  if (btle_setup_board(&s_brf, freq_hz, gain) != EXIT_SUCCESS) {
    if (s_brf.rf_dev != NULL) {
      goto program_quit;
    } else {
      return (1);
    }
  }

  s_brf.debug = true;

  if (btle_setup_stream(&s_brf) != EXIT_SUCCESS) {
    goto program_quit;
  }

  // scan
  s_brf.async_task.do_exit = false;
  rx_buf_offset = 0;
  while (s_brf.async_task.do_exit == false) {
    sleep(1);
  }

  printf("data read %d...\n", s_brf.rx_data.new_sample);

  goto program_quit;
program_quit:
  printf("Exit main loop ...\n");
  btle_teardown_stream(&s_brf);
  btle_teardown_board(&s_brf);

  /* if (fh_pcap_store) fclose(fh_pcap_store); */
  return (0);
}
