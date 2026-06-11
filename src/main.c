#include <hardware/structs/bus_ctrl.h>
#include <pico/cyw43_arch.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <tusb.h>

#include "comm.h"
#include "debug.h"
#include "pico_bluetooth.h"
#include "sdkconfig.h"
#include "tusb_config.h"
#include "usb_descriptors.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

#define BT_UPDATE_TIMEOUT_US 40000  // 40ms timeout for bluetooth packet updates
#define BT_UPDATE_PER_SEC 250       // 250 times per second

// Upper bound for one USB loop iteration while idle. New Bluetooth frames
// wake the loop immediately via SEV, so this only caps how late the
// disconnect-timeout / suspend checks can run.
#define USB_LOOP_IDLE_TIMEOUT_US 1000

// btstack + bluepad32 callbacks nest deeper than the 2KB default core1
// stack; give the Bluetooth core a dedicated 8KB stack.
#define BT_CORE_STACK_SIZE 0x2000
static uint32_t bt_core_stack[BT_CORE_STACK_SIZE / sizeof(uint32_t)];

void bluetooth_thread_run() {
  // initialize CYW43 driver architecture
  if (cyw43_arch_init()) {
    PICO_ERROR("failed to initialise cyw43_arch\n");
    return;
  }

  bluetooth_init();
  bluetooth_run();
}

void usb_thread_run() {
  const ds4_report_t zero_report = default_ds4_report();

  tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE,
                                 .speed = TUSB_SPEED_AUTO};
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  while (!is_usb_mounted) {
    tud_task();
    sleep_ms(10);
  }

  // Wait for USB HID Device stack to be initialized
  do {
    tud_task();
    sleep_ms(10);
  } while (!tud_hid_report(0x01, &zero_report, sizeof(ds4_report_t)));

  // Wait for 1 second to ensure the device is ready
  sleep_ms(1000);

  // Communication variables
  bool is_connected = false;
  bool frame_pending = false;
  ds4_frame_t frame;
  ds4_report_t report;
  absolute_time_t last_reported = get_absolute_time();

  // Stats based on time interval (디버그 모드에서만)
#if IS_PICO_DEBUG
  absolute_time_t stat_start_time = get_absolute_time();
  absolute_time_t last_stat_time = get_absolute_time();
  uint32_t ds4_update_count = 0;
  uint32_t ds4_missed_count = 0;
#endif

  while (true) {
    tud_task();

    if (tud_hid_ready() && !report_in_flight) {
      // A frame kept from a failed send takes priority; otherwise pull the
      // latest complete frame published by the Bluetooth core.
      if (frame_pending || ds4_mailbox_read(&g_ds4_mailbox, &frame)) {
        convert_uni_to_ds4(&frame.gamepad, frame.battery, &report);
        if (tud_hid_report(0x01, &report, sizeof(ds4_report_t))) {
          frame_pending = false;
          report_in_flight = true;
          last_reported = get_absolute_time();
          is_connected = true;
#if IS_PICO_DEBUG
          ds4_update_count++;
#endif
        } else {
          frame_pending = true;
        }
      } else if (is_connected) {
        // Reset the host to a neutral state if bluetooth updates stop.
        absolute_time_t now = get_absolute_time();
        int64_t elapsed_us = absolute_time_diff_us(last_reported, now);
        if (elapsed_us > BT_UPDATE_TIMEOUT_US &&
            tud_hid_report(0x01, &zero_report, sizeof(ds4_report_t))) {
          report_in_flight = true;
          last_reported = get_absolute_time();
          is_connected = false;
#if IS_PICO_DEBUG
          ds4_missed_count++;
          PICO_DEBUG("[USB] USB report missed for %lld us.\n", elapsed_us);
#endif
        }
      }
    }

#if IS_PICO_DEBUG
    absolute_time_t now = get_absolute_time();
    int64_t stat_elapsed_us = absolute_time_diff_us(last_stat_time, now);
    if (stat_elapsed_us >= 1000000) {
      double elapsed_sec =
          absolute_time_diff_us(stat_start_time, now) / 1000000.0;
      last_stat_time = now;
      PICO_DEBUG("[USB] USB Elapsed: %f, Updates: %u, Misses: %u\n",
                 elapsed_sec, ds4_update_count, ds4_missed_count);
      ds4_update_count = 0;
      ds4_missed_count = 0;
    }
#endif

    if (tud_suspended()) {
      tud_remote_wakeup();
    }

    // Sleep until a USB IRQ, the SEV from the Bluetooth core (new frame),
    // or the idle timeout - whichever comes first.
    best_effort_wfe_or_timeout(
        make_timeout_time_us(USB_LOOP_IDLE_TIMEOUT_US));
  }
}

int main() {
  stdio_init_all();

  // initialize GPIO for UART
  uart_init(uart0, 115200);
  gpio_set_function(0, GPIO_FUNC_UART);  // GP0: TX
  gpio_set_function(1, GPIO_FUNC_UART);  // GP1: RX
  sleep_ms(250);

  PICO_INFO("RPI PICO 2W started.\n");

  // Prioritize the Bluetooth core on the bus fabric so HCI traffic is never
  // stalled by the USB core under contention.
  bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;

  sleep_ms(250);

  // Run Bluetooth on core 1 with a dedicated stack
  multicore_launch_core1_with_stack(bluetooth_thread_run, bt_core_stack,
                                    sizeof(bt_core_stack));

  // Initialize the USB thread
  usb_thread_run();

  return 0;
}
