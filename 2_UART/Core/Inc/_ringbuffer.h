#ifndef _RINGBUFFER_H
#define _RINGBUFFER_H

#include <inttypes.h>

#define RINGBUFFER_SIZE 256
//= ??? & RINGBUFFER_MASK = between 0~255 num
#define RINGBUFFER_MASK (RINGBUFFER_SIZE - 1)

#if (RINGBUFFER_SIZE & (RINGBUFFER_MASK) != 0)
#error "Ring buffer size must be a power of two"
#endif

typedef uint8_t ring_buffer_size_t;

typedef struct ring_buffer_t {
  char buffer[RINGBUFFER_SIZE];
  ring_buffer_size_t head_idx;
  ring_buffer_size_t tail_idx;
} ring_buffer_t;

void RingBuffer_Init(ring_buffer_t *buffer);
void RingBuffer_Queue(ring_buffer_t *buffer, char data);
void RingBuffer_Queue_Arr(ring_buffer_t *buffer, const char *data,
                          ring_buffer_size_t len);
uint8_t RingBuffer_Dequeue(ring_buffer_t *buffer, char *data);
uint8_t RingBuffer_Dequeue_Arr(ring_buffer_t *buffer, char *data,
                               ring_buffer_size_t len);
uint8_t RingBuffer_Peek(ring_buffer_t *buffer, char *data,
                        ring_buffer_size_t index);
uint8_t RingBuffer_IsEmpty(ring_buffer_t *buffer);
uint8_t RingBuffer_IsFull(ring_buffer_t *buffer);
uint8_t RingBuffer_Num_Items(ring_buffer_t *buffer);

#endif