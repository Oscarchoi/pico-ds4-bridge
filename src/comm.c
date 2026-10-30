#include "comm.h"

// Slot ownership starts partitioned: writer=0, reader=1, mailbox=2.
ds4_mailbox_t g_ds4_mailbox = {
    .ready = 2,
    .write_idx = 0,
    .read_idx = 1,
};

_Atomic uint32_t g_rumble_request = 0;
_Atomic bool g_usb_link_up = false;
_Atomic uint32_t g_bt_heartbeat = 0;
