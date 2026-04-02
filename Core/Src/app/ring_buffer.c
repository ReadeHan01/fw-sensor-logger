/**
 * @file ring_buffer.c
 * @brief Lock-free-safe ring buffer with PRIMASK critical sections (Cortex-M).
 */
#include "app/ring_buffer.h"

#include <string.h>

Ring ringbuffer;       /* Global queue instance used by acquisition + CLI. */
ErrorStats error_stats; /* Zero-init BSS; counts I2C fails (elsewhere) + overwrites here. */

/* Save IRQ state and disable interrupts so ISR producer vs main consumer cannot race. */
static uint32_t rb_enter_critical(void)
{
  uint32_t primask; /* Stores previous PRIMASK so we can restore exactly. */
  __asm volatile ("MRS %0, PRIMASK" : "=r" (primask)); /* Read current mask. */
  __asm volatile ("CPSID i"); /* Disable IRQs (set PRIMASK). */
  return primask; /* Return old value for rb_exit_critical. */
}

/* Restore IRQ state saved by rb_enter_critical. */
static void rb_exit_critical(uint32_t primask)
{
  __asm volatile ("MSR PRIMASK, %0" :: "r" (primask)); /* Write back previous mask. */
}

void rb_init(Ring *rb)
{
  memset(rb, 0, sizeof(*rb)); /* Clear indices, count, and sample payload area. */
}

void sample_init(Sample *sample)
{
  memset(sample, 0, sizeof(*sample)); /* CLI "reset" uses this to clear last sample view. */
}

void rb_push_overwrite(Ring *rb, const Sample *sample)
{
  uint32_t key = rb_enter_critical(); /* Begin atomic update of head/tail/count. */

  rb->buf[rb->head] = *sample; /* Store new sample at current write index. */
  rb->head = (rb->head + 1u) & RB_MASK; /* Advance head with wrap (power-of-two size). */

  if (rb->count < RB_CAPACITY) {
    rb->count++; /* Buffer not full yet: just grow occupancy. */
  } else {
    rb->tail = (rb->tail + 1u) & RB_MASK; /* Full: drop oldest by advancing tail. */
    error_stats.ring_overwrite++; /* Count lost samples for debugging. */
  }
  rb_exit_critical(key); /* End critical section; IRQs restored. */
}

bool rb_pop(Ring *rb, Sample *out)
{
  uint32_t key = rb_enter_critical(); /* Serialize vs producer ISR. */

  if (rb->count == 0u) {
    rb_exit_critical(key); /* Nothing to read; release lock. */
    return false; /* Empty. */
  }

  *out = rb->buf[rb->tail]; /* Copy oldest sample to caller. */
  rb->tail = (rb->tail + 1u) & RB_MASK; /* Consume one slot. */
  rb->count--; /* Decrement occupancy. */

  rb_exit_critical(key); /* Release lock. */
  return true; /* Success. */
}

bool rb_peek_latest(Ring *rb, Sample *out)
{
  uint32_t key = rb_enter_critical(); /* Protect head index read vs concurrent push. */

  if (rb->count == 0u) {
    rb_exit_critical(key); /* Empty buffer. */
    return false; /* No latest sample. */
  }

  uint16_t latest_index = (rb->head + RB_CAPACITY - 1u) & RB_MASK; /* Slot before head is newest. */
  *out = rb->buf[latest_index]; /* Copy without dequeue. */

  rb_exit_critical(key); /* Release lock. */
  return true; /* Success. */
}
