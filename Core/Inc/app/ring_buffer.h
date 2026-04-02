/**
 * @file ring_buffer.h
 * @brief Ring buffer API and global error_stats (overwrite counter).
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "app_types.h"
#include <stdbool.h>

extern Ring ringbuffer;      /**< Single global sample queue. */
extern ErrorStats error_stats; /**< Fault counters (I2C, ring overflow). */

void rb_init(Ring *rb); /**< Zero entire ring structure. */

void sample_init(Sample *sample); /**< Zero one sample (e.g. after CLI reset). */

void rb_push_overwrite(Ring *rb, const Sample *sample); /**< Push; drop oldest if full. */

bool rb_pop(Ring *rb, Sample *out); /**< Pop oldest; false if empty. */

bool rb_peek_latest(Ring *rb, Sample *out); /**< Read newest without removing. */

#endif /* RING_BUFFER_H */
