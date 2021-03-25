#include "btle_blade.h"

#include <libbladeRF.h>

#include "brf_adv_pdu.h"
#include "brf_crc.h"
#include "brf_shift_queue.h"

// System includes
#include <assert.h>
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

void brf_bit_array_to_bytes(uint8_t *bit_array, size_t bit_array_size,
                            uint8_t *bytes, size_t bytes_size) {
  assert(bit_array);
  assert(bit_array_size >= (bytes_size * NUM_BIT_PER_BYTE));

  memset(bytes, 0, bytes_size);

  // Data is in Little-endian, least significant bit transmitted first
  for (unsigned i = 0; i < bytes_size * NUM_BIT_PER_BYTE; i++) {
    bytes[i / NUM_BIT_PER_BYTE] |= bit_array[i] << (i % NUM_BIT_PER_BYTE);
  }
}

void brf_uint8_to_bit_array(uint8_t uint8_in, uint8_t *bit_array,
                            size_t bit_array_size) {
  assert(bit_array);
  assert(bit_array_size >= (sizeof(uint8_in) * NUM_BIT_PER_BYTE));

  // Data is in Little-endian, least significant bit transmitted first
  for (size_t i = 0; i < sizeof(uint8_in) * NUM_BIT_PER_BYTE; i++) {
    bit_array[i] = 0x01 & uint8_in;
    uint8_in = (uint8_in >> 1);
  }
}

void brf_uint32_to_bit_array(uint32_t uint32_in, uint8_t *bit_array,
                             size_t bit_array_size) {
  assert(bit_array);
  assert(bit_array_size >= (sizeof(uint32_in) * NUM_BIT_PER_BYTE));

  // Data is in Little-endian, least significant bit transmitted first
  for (size_t i = 0; i < sizeof(uint32_in) * NUM_BIT_PER_BYTE; i++) {
    bit_array[i] = 0x01 & uint32_in;
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

void btle_print_mac_addr(uint8_t *data, size_t len) {
  assert(len >= 6);
  printf("%02x:%02x:%02x:%02x:%02x:%02x", data[5], data[4], data[3], data[2],
         data[1], data[0]);
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

int brf_search_unique_bits(const iq_sample_t *iq_samples, size_t iq_count,
                           const uint8_t *unique_bits,
                           const uint8_t *unique_bits_mask,
                           size_t unique_bits_size) {
  assert(iq_samples);

  // Ensure unique_bits_size is order of 2 value
  assert(!((unique_bits_size - 1) & unique_bits_size));

  brf_shift_queue_t demod_queue[IQ_PER_SYMBOL];
  uint8_t *demod_queue_buf =
      (uint8_t *)malloc(IQ_PER_SYMBOL * unique_bits_size);

  for (int i = 0; i < (int)IQ_PER_SYMBOL; i++) {
    brf_shift_queue_init(&demod_queue[i],
                         &demod_queue_buf[i * unique_bits_size],
                         unique_bits_size);
  }

  int ret_sample_idx = -1;

  // Iterate over iq_samples one sample at a time attempting to
  // find the unique_bits.
  // 4 samplees demod to 1 bit, but iq samples does not provide framing we need
  // to find start of the frame outself, hence the unique access addr.
  //
  // We maintain 4 sliding window short of bit buffs with size of
  // |unique_bits_size|. Each sliding window is offset by 1 samples, since there
  // are 4 samples per bit. Each new sample allow one new bit to be demodulated
  // in 1 of 4 sliding windows.
  for (unsigned i = 0; i < iq_count - IQ_PER_SYMBOL; i++) {
    int phase_idx = i % IQ_PER_SYMBOL;

    // Ensure don't access buffer beyond given
    assert((i + IQ_PER_SYMBOL) < iq_count);
    uint8_t demod_bit = btle_demod_bit(&iq_samples[i], IQ_PER_SYMBOL);

    brf_shift_queue_push(&demod_queue[phase_idx], &demod_bit,
                         sizeof(demod_bit));

    if (brf_shift_queue_size(&demod_queue[phase_idx]) >= unique_bits_size) {
      (void)unique_bits_mask;
      // TODO: Support match with mask
      bool is_match = brf_shift_queue_compare(&demod_queue[phase_idx],
                                              unique_bits, unique_bits_size);
      if (is_match) {
        ret_sample_idx = i + IQ_PER_SYMBOL;
        break;
      }
    }
  }

  free(demod_queue_buf);

  return ret_sample_idx;
}

void scramble_byte(uint8_t *bytes, int num_byte,
                   const uint8_t *scramble_table_byte) {
  int i;
  for (i = 0; i < num_byte; i++) {
    bytes[i] = bytes[i] ^ scramble_table_byte[i];
  }
}

uint8_t btle_demod_bit(const iq_sample_t *iq, size_t num_samples) {
  assert(num_samples >= IQ_PER_SYMBOL);
  (void)num_samples;

  // Frequency-Shift Keying
  // Phase delta > 0 == 1
  // Phase delta < 0 == 0
  // Phase delta == Q1/I1 - Q0/I0
  // Which roughly translate to below

  return ((iq[0].I * iq[1].Q) - (iq[1].I * iq[0].Q)) > 0 ? 1 : 0;
}

size_t btle_demod_bits(iq_sample_t *iq, size_t iq_size, uint8_t *out_bits,
                       size_t out_bit_size) {
  assert(iq && out_bits);
  assert((out_bit_size * IQ_PER_SYMBOL) < iq_size);

  for (size_t i = 0; i < out_bit_size; i++) {
    out_bits[i] = btle_demod_bit(&iq[i * IQ_PER_SYMBOL], IQ_PER_SYMBOL);
  }

  return (out_bit_size * IQ_PER_SYMBOL);
}

size_t btle_demod_bytes(iq_sample_t *iq, size_t iq_size, uint8_t *out_bytes,
                        size_t out_byte_size) {
  assert(iq && out_bytes);
  assert((out_byte_size * NUM_BIT_PER_BYTE * IQ_PER_SYMBOL) < iq_size);

  iq_sample_t *curr_iq = iq;
  size_t iq_size_left = iq_size;
  for (size_t i = 0; i < out_byte_size; i++) {
    assert(iq_size_left > 0);

    uint8_t bits_out[NUM_BIT_PER_BYTE];
    size_t advanced_count =
        btle_demod_bits(curr_iq, iq_size_left, bits_out, sizeof(bits_out));
    iq_size_left -= advanced_count;
    curr_iq += advanced_count;

    brf_bit_array_to_bytes(bits_out, sizeof(bits_out), &out_bytes[i], 1);
  }

  return iq_size - iq_size_left;
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

  brf_data_t *rx_data = NULL;
  while (!s_brf.async_tasks.do_exit) {
    if (!rx_data) {
      pthread_cond_wait(&brf->async_tasks.rx_data_cond,
                        &brf->async_tasks.rx_data_lock);
    }

    rx_data = NULL;
    for (unsigned i = 0; i < sizeof(brf->rx_data) / sizeof(brf->rx_data[0]);
         i++) {
      if (brf->rx_data[i].new_sample) {
        rx_data = &brf->rx_data[i];
        break;
      }
    }

    if (rx_data) {
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
                                   IQ_PER_SYMBOL * 1000000ul, &actual);
  if (status != 0) {
    fprintf(stderr, "Failed to set sample rate: %s\n",
            bladerf_strerror(status));
    goto exit_close_blade;
  } else {
    fprintf(stdout, "set sample rate: %dHz %s\n", actual,
            bladerf_strerror(status));
  }

  status = bladerf_set_bandwidth(brf->rf_dev, BLADERF_MODULE_RX,
                                 IQ_PER_SYMBOL * 1000000ul / 2, &actual);
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
  assert(brf);
  assert(rx_data);

  assert(sizeof(brf->access_addr_bit_array) ==
         sizeof(brf->access_mask_bit_array));

  iq_sample_t *iq_samples = (iq_sample_t *)rx_data;
  size_t iq_samples_count = sizeof(rx_data->rx_block_buf) / sizeof(iq_sample_t);

  while (iq_samples_count > 0) {
    /***** Search for the preamble *****/
    if (iq_samples_count < (sizeof(brf->preamble_bit_array) * IQ_PER_SYMBOL)) {
      break;
    }
    int advanced_samples = brf_search_unique_bits(
        iq_samples, iq_samples_count, brf->preamble_bit_array,
        brf->access_mask_bit_array, sizeof(brf->preamble_bit_array));
    if (advanced_samples < 0) {
      break;
    }
    iq_samples_count -= advanced_samples;
    iq_samples += advanced_samples;

    // TODO: Do both togather
    /***** Search for access address after the preamble *****/
    if (iq_samples_count <
        (sizeof(brf->access_addr_bit_array) * IQ_PER_SYMBOL)) {
      break;
    }

    advanced_samples = brf_search_unique_bits(
        iq_samples, iq_samples_count, brf->access_addr_bit_array,
        brf->access_mask_bit_array, sizeof(brf->access_addr_bit_array));
    if (advanced_samples < 0) {
      break;
    }

    iq_samples_count -= advanced_samples;
    iq_samples += advanced_samples;

    /***** Demod the main packet header *****/
    brf_adv_pdu_t pdu;
    memset(&pdu, 0, sizeof(pdu));

    // Ensure there is at least 2 bytes worth of iq samples left
    // TODO: Handle warp around to next rx data block
    if (iq_samples_count <
        (sizeof(pdu.hdr) * NUM_BIT_PER_BYTE * IQ_PER_SYMBOL)) {
      break;
    }

    // Demodulate header byte + length byte immediately after the access addr
    advanced_samples = btle_demod_bytes(iq_samples, iq_samples_count,
                                        (uint8_t *)&pdu.hdr, sizeof(pdu.hdr));
    iq_samples_count -= advanced_samples;
    iq_samples += advanced_samples;

    // Descramble the header 2 bytes
    scramble_byte((uint8_t *)&pdu.hdr, sizeof(pdu.hdr),
                  scramble_table[brf->channel_number]);

    size_t payload_len = brf_adv_pdu_get_len(&pdu);

    if (payload_len < 6 || payload_len > 37) {
      brf_print(&brf->async_tasks,
                "Error: ADV(%s) payload length(%lu) should be 6~37!\n",
                brf_adv_pdu_get_type_str(&pdu), payload_len);
      continue;
    }

    // Ensure there is at least payload len worth of iq samples left
    // TODO: Handle warp around to next rx data block
    if (iq_samples_count < ((payload_len + BRF_ADV_PDU_CRC_SIZE) *
                            NUM_BIT_PER_BYTE * IQ_PER_SYMBOL)) {
      break;
    }

    // Demodulate main payload
    advanced_samples = btle_demod_bytes(iq_samples, iq_samples_count,
                                        pdu.payload, payload_len);
    iq_samples_count -= advanced_samples;
    iq_samples += advanced_samples;

    // Demodulate crc bytes
    advanced_samples = btle_demod_bytes(iq_samples, iq_samples_count, pdu.crc,
                                        BRF_ADV_PDU_CRC_SIZE);
    iq_samples_count -= advanced_samples;
    iq_samples += advanced_samples;

    // Descramble the payload bytes
    scramble_byte(pdu.payload, payload_len,
                  &scramble_table[brf->channel_number][sizeof(pdu.hdr)]);

    // Descramble the crc bytes
    scramble_byte(
        pdu.crc, sizeof(pdu.crc),
        &scramble_table[brf->channel_number][sizeof(pdu.hdr) + payload_len]);

    uint32_t calc_crc = brf_crc24((uint8_t *)&pdu.hdr, sizeof(pdu.hdr));
    calc_crc = brf_crc24_update(pdu.payload, payload_len, calc_crc);
    bool crc_flag = brf_adv_pdu_get_crc(&pdu) == calc_crc;

    if (crc_flag) {
      uint8_t adv_a[BRF_ADV_PDU_ADV_A_SIZE];
      bool ret = brf_adv_pdu_get_adv_a(&pdu, adv_a, sizeof(adv_a));
      if (ret) {
        btle_print_mac_addr(adv_a, sizeof(adv_a));
      } else {
        brf_print(&brf->async_tasks, "\t");
      }
      brf_print(&brf->async_tasks, " %s  \tT:%d tR:%d \tCRC%d\n",
                brf_adv_pdu_get_type_str(&pdu), brf_adv_pdu_get_tx_add(&pdu),
                brf_adv_pdu_get_rx_add(&pdu), crc_flag);
    }
  }
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
    s_brf.channel_number = chan;
  }

  printf(
      "Cmd line input: chan %d, freq %lldMHz, access addr %08x, crc init "
      "%06x "
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

  brf_uint32_to_bit_array(DEFAULT_ACCESS_ADDR, s_brf.access_addr_bit_array,
                          sizeof(s_brf.access_addr_bit_array));
  btle_print_xxd(s_brf.access_addr_bit_array,
                 sizeof(s_brf.access_addr_bit_array));

  brf_uint32_to_bit_array(DEFAULT_ACCESS_MASK, s_brf.access_mask_bit_array,
                          sizeof(s_brf.access_mask_bit_array));
  btle_print_xxd(s_brf.access_mask_bit_array,
                 sizeof(s_brf.access_mask_bit_array));

  brf_uint8_to_bit_array(PREAMBLE_1M, s_brf.preamble_bit_array,
                         sizeof(s_brf.preamble_bit_array));
  btle_print_xxd(s_brf.preamble_bit_array, sizeof(s_brf.preamble_bit_array));

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
