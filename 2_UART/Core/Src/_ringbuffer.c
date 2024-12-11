#include "_ringbuffer.h"

void RingBuffer_Init(ring_buffer_t *buffer) {
  buffer->head_idx = 0;
  buffer->tail_idx = 0;
}

void RingBuffer_Queue(ring_buffer_t *buffer, char data) {
  if (RingBuffer_IsFull(buffer)) {
    buffer->tail_idx = (buffer->tail_idx + 1) & RINGBUFFER_MASK;
  }

  buffer->buffer[buffer->head_idx] = data;
  buffer->head_idx = (buffer->head_idx + 1) & RINGBUFFER_MASK;
}

void RingBuffer_Queue_Arr(ring_buffer_t *buffer, const char *data,
                          ring_buffer_size_t len) {
  ring_buffer_size_t i;

  for (i = 0; i < len; i++) {
    RingBuffer_Queue(buffer, data[i]);
  }
}

uint8_t RingBuffer_Dequeue(ring_buffer_t *buffer, char *data) {
  if (RingBuffer_IsEmpty(buffer)) {
    return 0;
  }

  *data = buffer->buffer[buffer->tail_idx];
  buffer->tail_idx = (buffer->tail_idx + 1) & RINGBUFFER_MASK;
  return 1;
}

uint8_t RingBuffer_Dequeue_Arr(ring_buffer_t *buffer, char *data,
                               ring_buffer_size_t len) {
  if (RingBuffer_IsEmpty(buffer)) {
    return 0;
  }

  char *data_ptr = data;
  uint8_t cnt = 0;

  while ((cnt < len) && RingBuffer_Dequeue(buffer, data_ptr)) {
    cnt++;
    data_ptr++;
  }

  return cnt;
}

uint8_t RingBuffer_Peek(ring_buffer_t *buffer, char *data,
                        ring_buffer_size_t index) {
  if (index >= RingBuffer_Num_Items(buffer)) {
    return 0;
  }

  ring_buffer_size_t data_index = (buffer->tail_idx + index) & RINGBUFFER_MASK;
  *data = buffer->buffer[data_index];

  return 1;
}

uint8_t RingBuffer_IsEmpty(ring_buffer_t *buffer) {
  return (buffer->head_idx == buffer->tail_idx);
}

uint8_t RingBuffer_IsFull(ring_buffer_t *buffer) {
  //= ex) head = 2, tail = 3 -> -1 = 4bit(1111) -> IsFull
  //= ex) head = 2, tail = 4 -> -2 = 4bit(1110) -> Is not Full
  return ((buffer->head_idx - buffer->tail_idx) & RINGBUFFER_MASK) ==
         RINGBUFFER_MASK;
}

uint8_t RingBuffer_Num_Items(ring_buffer_t *buffer) {
  //= The remaining storage space in the buffer
  return ((buffer->head_idx - buffer->tail_idx) & RINGBUFFER_MASK);
}