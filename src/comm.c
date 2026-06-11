#include "comm.h"

// Slot ownership starts partitioned: writer=0, reader=1, mailbox=2.
ds4_mailbox_t g_ds4_mailbox = {
    .ready = 2,
    .write_idx = 0,
    .read_idx = 1,
};
