#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <libbladeRF.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "btle_rx.h"

volatile bool do_exit = false;

#define MAX_GAIN 60
#define DEFAULT_GAIN 45

volatile int rx_buf_offset;  // remember to initialize it!

static bladerf_t s_blade;

void btle_sigint_callback_handler(int signum) {
  fprintf(stdout, "Caught signal %d\n", signum);
  do_exit = true;
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

void *btle_stream_callback(bladerf_t *dev, bladerf_stream_t *stream,
                           struct bladerf_metadata *metadata, void *samples,
                           size_t num_samples, void *user_data) {
  (void)dev;
  (void)stream;
  (void)metadata;
  (void)samples;
  (void)num_samples;
  bladerf_t *blade = (bladerf_t *)user_data;
  (void)blade;

  return NULL;
}

void *btle_rx_task_run(void *ctx) {
  bladerf_t *blade = (bladerf_t *)ctx;
  (void)blade;

  return NULL;
}

int btle_setup_board(bladerf_t *blade, uint64_t freq_hz, int gain) {
  int status;
  unsigned int actual;

  blade->rx_data.idx = 0;
  blade->rx_data.num_buffers = 2;
  blade->rx_data.samples_per_buffer = (LEN_BUF / 2);

  if (signal(SIGINT, btle_sigint_callback_handler) == SIG_ERR ||
      signal(SIGTERM, btle_sigint_callback_handler) == SIG_ERR) {
    fprintf(stderr, "Failed to set up signal handler\n");
    goto exit_failure;
  }

  status = bladerf_open(&blade->rf_dev, NULL);
  if (status < 0) {
    fprintf(stderr, "Failed to open device: %s\n", bladerf_strerror(status));
    goto exit_failure;
  } else {
    fprintf(stdout, "open device: %s\n", bladerf_strerror(status));
  }

  status = bladerf_is_fpga_configured(blade->rf_dev);
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

  status = bladerf_set_frequency(blade->rf_dev, BLADERF_MODULE_RX, freq_hz);
  if (status != 0) {
    fprintf(stderr, "Failed to set frequency: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "set frequency: %luHz %s\n", freq_hz,
            bladerf_strerror(status));
  }

  status = bladerf_set_sample_rate(blade->rf_dev, BLADERF_MODULE_RX,
                                   SAMPLE_PER_SYMBOL * 1000000ul, &actual);
  if (status != 0) {
    fprintf(stderr, "Failed to set sample rate: %s\n",
            bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "set sample rate: %dHz %s\n", actual,
            bladerf_strerror(status));
  }

  status = bladerf_set_bandwidth(blade->rf_dev, BLADERF_MODULE_RX,
                                 SAMPLE_PER_SYMBOL * 1000000ul / 2, &actual);
  if (status != 0) {
    fprintf(stderr, "Failed to set bandwidth: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "bladerf_set_bandwidth: %d %s\n", actual,
            bladerf_strerror(status));
  }

  status = bladerf_set_gain(blade->rf_dev, BLADERF_MODULE_RX, gain);
  if (status != 0) {
    fprintf(stderr, "Failed to set gain: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "bladerf_set_gain: %d %s\n", gain,
            bladerf_strerror(status));
  }

  status = bladerf_get_gain(blade->rf_dev, BLADERF_MODULE_RX, &gain);
  if (status != 0) {
    fprintf(stderr, "Failed to get gain: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "bladerf_get_gain: %d %s\n", gain,
            bladerf_strerror(status));
  }

  /* Initialize the stream */
  status = bladerf_init_stream(
      &blade->stream, blade->rf_dev, btle_stream_callback,
      &blade->rx_data.buffers, blade->rx_data.num_buffers,
      BLADERF_FORMAT_SC16_Q11, blade->rx_data.samples_per_buffer,
      blade->rx_data.num_buffers, blade);

  if (status != 0) {
    fprintf(stderr, "Failed to init stream: %s\n", bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "init stream: %s\n", bladerf_strerror(status));
  }

  bladerf_set_stream_timeout(blade->rf_dev, BLADERF_RX, 100);

  status = bladerf_enable_module(blade->rf_dev, BLADERF_MODULE_RX, true);
  if (status < 0) {
    fprintf(stderr, "Failed to enable module: %s\n", bladerf_strerror(status));
    bladerf_deinit_stream(blade->stream);
    goto exit_close_blade;
  } else {
    fprintf(stdout, "enable module true: %s\n", bladerf_strerror(status));
  }

  status = pthread_create(&(blade->async_task.rx_task), NULL, btle_rx_task_run,
                          NULL);
  if (status < 0) {
    goto exit_close_blade;
  }

  return EXIT_SUCCESS;

exit_close_blade:
  if (blade->rf_dev) {
    bladerf_close(blade->rf_dev);
  }

exit_failure:
  return EXIT_FAILURE;
}

void btle_teardown_board(bladerf_t *blade) {
  int status;

  bladerf_deinit_stream(blade->stream);
  printf("bladerf_deinit_stream.\n");

  status = bladerf_enable_module(blade->rf_dev, BLADERF_MODULE_RX, false);
  if (status < 0) {
    fprintf(stderr, "Failed to enable module: %s\n", bladerf_strerror(status));
  } else {
    fprintf(stdout, "enable module false: %s\n", bladerf_strerror(status));
  }

  bladerf_close(blade->rf_dev);
  printf("bladerf_close.\n");

  pthread_join(blade->async_task.rx_task, NULL);
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
  do_exit = false;
  if (btle_setup_board(&s_blade, freq_hz, gain) != 0) {
    if (s_blade.rf_dev != NULL) {
      goto program_quit;
    } else {
      return (1);
    }
  }

  // scan
  do_exit = false;
  rx_buf_offset = 0;
  while (do_exit == false) {
  }

  goto program_quit;
program_quit:
  printf("Exit main loop ...\n");
  btle_teardown_board(&s_blade);

  /* if (fh_pcap_store) fclose(fh_pcap_store); */
  return (0);
}
