#include <stdlib.h>

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
#define BT_UPDATE_PER_SEC 250  // 250 times per second

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

  tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  // Initialize USB HID Device stack
  while (!is_ds4_initialized) {
    tud_task();
    sleep_ms(2);
  }
  do {
    tud_task();
    sleep_ms(2);
  } while (!tud_hid_report(0, &zero_report, sizeof(ds4_report_t)));

  // Communication variables
  uint32_t last_updated = 0;
  uint32_t timestamp = 0;
  bool is_updated = false;
  bool is_connected = false;
  absolute_time_t last_reported = get_absolute_time();

  // Blink LED every 250 reports
  volatile uint32_t blink_on = 0;
  volatile uint32_t counter = 0;

  // Stats based on time interval (디버그 모드에서만)
#if IS_PICO_DEBUG
  absolute_time_t stat_start_time = get_absolute_time();
  absolute_time_t last_stat_time = get_absolute_time();
  uint32_t ds4_update_count = 0;
  uint32_t ds4_missed_count = 0;
#endif

  // Allocate memory for the local report
  ds4_report_t* local_report_ptr = (ds4_report_t*)malloc(sizeof(ds4_report_t));
  if (local_report_ptr == NULL) {
    PICO_ERROR("failed to allocate memory for local report\n");
    return;
  }
  memset(local_report_ptr, 0, sizeof(ds4_report_t));

  while (true) {
    tud_task();

    ds4_frame_t data;
    ds4_report_t report;

    if (tud_hid_ready()) {
      is_updated = false;

      SEQLOCK_TRY_READ(&data, g_ds4_shared);
      int32_t time_diff = data.timestamp - last_updated;
      if (time_diff > 0) {
        last_updated = data.timestamp;
        convert_uni_to_ds4(data.gamepad, data.battery, &report);

        is_updated = true;
        is_connected = true;
      }

      // report when dualshock4 is updated or send default report if update is
      // not received for 5ms
      if (is_updated) {
        // report when dualshock4 is updated
        tud_hid_report(0, &report, sizeof(ds4_report_t));
        last_reported = get_absolute_time();

        // blink the LED every 250 reports
        if (counter++ % (BT_UPDATE_PER_SEC / 2) == 0) {
          cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, blink_on ^= 1);
          sleep_us(100);
        }

#if IS_PICO_DEBUG
        ds4_update_count++;
#endif
      } else if (is_connected) {
        absolute_time_t now = get_absolute_time();
        int64_t elapsed_us = absolute_time_diff_us(last_reported, now);
        if (elapsed_us > BT_UPDATE_TIMEOUT_US) {
          tud_hid_report(0, &zero_report, sizeof(ds4_report_t));
          last_reported = get_absolute_time();
          is_connected = false;
#if IS_PICO_DEBUG
          ds4_missed_count++;
          PICO_DEBUG("[USB] USB report missed for %lld us.\n", elapsed_us);
#endif
        }
      }

#if IS_PICO_DEBUG
      absolute_time_t now = get_absolute_time();
      int64_t stat_elapsed_us = absolute_time_diff_us(last_stat_time, now);
      if (stat_elapsed_us >= 1000000) {
        double elapsed_sec = absolute_time_diff_us(stat_start_time, now) / 1000000.0;
        last_stat_time = now;
        PICO_DEBUG("[USB] USB Elapsed: %f, Updates: %u, Misses: %u\n", elapsed_sec, ds4_update_count, ds4_missed_count);
        ds4_update_count = 0;
        ds4_missed_count = 0;
      }
#endif
    }
    sleep_ms(1);
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

  // initialize dualshock4 shared data
  g_ds4_shared.data.timestamp = to_ms_since_boot(get_absolute_time());
  sleep_ms(250);

  // Initialize the CYW43 driver
  multicore_launch_core1(bluetooth_thread_run);

  // Initialize the USB thread
  usb_thread_run();

  return 0;
}
