#include "brf_shift_queue.h"

#include <assert.h>
#include <string.h>
#include <sys/param.h>

void brf_shift_queue_init(brf_shift_queue_t *q, uint8_t *buff,
                          size_t buff_size) {
  assert(q);
  assert(!((buff_size - 1) & buff_size));

  memset(q, 0, sizeof(*q));
  memset(buff, 0, buff_size);

  q->buff = buff;
  q->capacity = buff_size;
}

size_t brf_shift_queue_size(brf_shift_queue_t *q) {
  assert(q);

  return (q->write_idx - q->read_idx);
}

size_t brf_shift_queue_remain(brf_shift_queue_t *q) {
  assert(q);

  return q->capacity - (brf_shift_queue_size(q) & (q->capacity - 1));
}

size_t brf_shift_queue_push(brf_shift_queue_t *q, const uint8_t *src,
                            size_t size) {
  assert(q);
  assert(src);

  unsigned char const *buf_ptr = (unsigned char const *)src;

  // Get write offset for accessing queue buffer
  const size_t write_offset = q->write_idx & (q->capacity - 1);
  // Find the number of bytes can write before wrap around
  const size_t pre_wrap_copy_len = MIN(size, q->capacity - write_offset);
  memcpy(&q->buff[write_offset], buf_ptr, pre_wrap_copy_len);
  buf_ptr += pre_wrap_copy_len;

  // Check if we need to finish enqueue data at head of queue
  if (pre_wrap_copy_len < size) {
    memcpy(&q->buff[0], buf_ptr, size - pre_wrap_copy_len);
  }
  q->write_idx += size;

  // Determine if read_idx needs to be bumpped due to over writting
  const size_t overwrite_len =
      MAX((q->write_idx - q->read_idx), q->capacity) - q->capacity;
  q->read_idx += overwrite_len;

  return overwrite_len;
}

bool brf_shift_queue_compare(brf_shift_queue_t *q, const uint8_t *src,
                             size_t size) {
  assert(q);
  assert(src);

  // Get read offset for accessing queue buffer
  const size_t read_offset = q->read_idx & (q->capacity - 1);
  // Find the number of bytes can read before wrap around
  const size_t pre_wrap_read_len = MIN(size, q->capacity - read_offset);

  bool ret = !memcmp(&q->buff[read_offset], src, pre_wrap_read_len);

  if (pre_wrap_read_len < size) {
    ret = ret && !memcmp(&q->buff[0], &src[pre_wrap_read_len],
                         size - pre_wrap_read_len);
  }
  return ret;
}
