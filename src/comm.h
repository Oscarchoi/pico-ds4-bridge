#ifndef COMM_H_
#define COMM_H_

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include <hardware/sync.h>

#include "dualshock4.h"

typedef struct {
  uint32_t timestamp;
  uni_gamepad_t gamepad;
  uint8_t battery;
} ds4_frame_t;

/*
 * Wait-free single-writer / single-reader "latest value" mailbox
 * (triple buffer).
 *
 * The three slots are always partitioned between the writer (write_idx),
 * the reader (read_idx) and the mailbox itself (ready). Publishing and
 * consuming are a single atomic exchange of slot ownership, so:
 *   - the Bluetooth core never waits, retries or disables IRQs, and
 *   - the USB core always reads a complete frame (no torn reads, ever),
 * unlike a seqlock where the reader must retry while the writer is active.
 *
 * DS4_MAILBOX_FRESH marks that the mailbox slot holds a frame the reader
 * has not consumed yet. Only the writer sets it, only the reader clears it.
 */
#define DS4_MAILBOX_FRESH 0x80000000u
#define DS4_MAILBOX_IDX_MASK 0x3u

typedef struct {
  ds4_frame_t slot[3];
  _Atomic uint32_t ready;  // mailbox-owned slot index | DS4_MAILBOX_FRESH
  uint32_t write_idx;      // owned by the writer (Bluetooth core)
  uint32_t read_idx;       // owned by the reader (USB core)
} ds4_mailbox_t;

extern ds4_mailbox_t g_ds4_mailbox;

// Writer side (Bluetooth core). Fill the returned slot, then publish.
static inline ds4_frame_t* ds4_mailbox_write_slot(ds4_mailbox_t* mb) {
  return &mb->slot[mb->write_idx];
}

static inline void ds4_mailbox_publish(ds4_mailbox_t* mb) {
  uint32_t prev = atomic_exchange_explicit(
      &mb->ready, mb->write_idx | DS4_MAILBOX_FRESH, memory_order_release);
  mb->write_idx = prev & DS4_MAILBOX_IDX_MASK;
  __sev();  // wake the USB core out of WFE immediately
}

// Reader side (USB core). Returns true and fills *out only when a frame
// published after the previous successful read is available.
static inline bool ds4_mailbox_read(ds4_mailbox_t* mb, ds4_frame_t* out) {
  if (!(atomic_load_explicit(&mb->ready, memory_order_relaxed) &
        DS4_MAILBOX_FRESH)) {
    return false;
  }
  uint32_t prev = atomic_exchange_explicit(&mb->ready, mb->read_idx,
                                           memory_order_acquire);
  mb->read_idx = prev & DS4_MAILBOX_IDX_MASK;
  *out = mb->slot[mb->read_idx];
  return true;
}

#endif  // COMM_H_
