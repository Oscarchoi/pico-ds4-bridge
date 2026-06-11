#include "pico_bluetooth.h"

#include <stddef.h>
#include <string.h>

#include <bt/uni_bt.h>
#include <btstack.h>
#include <controller/uni_gamepad.h>
#include <pico/cyw43_arch.h>
#include <pico.h>
#include <pico/time.h>
#include <uni.h>
#include <uni_hid_device.h>

#include "comm.h"
#include "debug.h"
#include "dualshock4.h"
#include "sdkconfig.h"

#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t* d);

// LED status indication, all driven from this core since the CYW43 LED
// shares the Bluetooth SPI bus:
//   - scanning / waiting for a controller: fast blink (timer below)
//   - controller connected and streaming:  ~1Hz heartbeat from
//     pico_bluetooth_on_controller_data()
#define LED_SCAN_BLINK_INTERVAL_MS 150

static btstack_timer_source_t led_scan_timer;
static bool led_device_connected = false;

static void led_set(bool on) {
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
}

static void led_scan_timer_handler(btstack_timer_source_t* ts) {
  // A device connected since the last tick; data heartbeat takes over.
  if (led_device_connected)
    return;

  static bool led_on = false;
  led_on = !led_on;
  led_set(led_on);

  btstack_run_loop_set_timer(ts, LED_SCAN_BLINK_INTERVAL_MS);
  btstack_run_loop_add_timer(ts);
}

// Must be called from the btstack run loop context (core 1).
static void led_scan_blink_start(void) {
  led_device_connected = false;
  btstack_run_loop_remove_timer(&led_scan_timer);
  btstack_run_loop_set_timer_handler(&led_scan_timer, led_scan_timer_handler);
  btstack_run_loop_set_timer(&led_scan_timer, LED_SCAN_BLINK_INTERVAL_MS);
  btstack_run_loop_add_timer(&led_scan_timer);
}

static void led_scan_blink_stop(void) {
  led_device_connected = true;
  btstack_run_loop_remove_timer(&led_scan_timer);
  led_set(false);
}

// Platform Overrides
static void pico_bluetooth_init(int argc, const char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
}

static void pico_bluetooth_on_init_complete(void) {
  // Safe to call "unsafe" functions since they are called
  PICO_INFO("Bluetooth initialization complete.\n");

#if PICO_DS4_DELETE_BT_KEYS_ON_BOOT
  // Delete stored BT keys for fresh pairing (helpful for initial connection).
  // Keeping the keys (default) lets an already-paired controller reconnect
  // right after a power cycle without re-pairing.
  uni_bt_del_keys_unsafe();
#endif

  // Start scanning and autoconnect to supported controllers.
  uni_bt_start_scanning_and_autoconnect_safe();
  led_scan_blink_start();
  PICO_INFO("Started Bluetooth scanning for new devices.\n");

  uni_property_dump_all();
}

static uni_error_t pico_bluetooth_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
  // You can filter discovered devices here. Return any value different from UNI_ERROR_SUCCESS;
  // @param addr: the Bluetooth address
  // @param name: could be NULL, could be zero-length, or might contain the name.
  // @param cod: Class of Device. See "uni_bt_defines.h" for possible values.
  // @param rssi: Received Signal Strength Indicator (RSSI) measured in dBms. The higher (255) the better.

  const char* device_name = (name && strlen(name) > 0) ? name : "Unknown";
  PICO_DEBUG("[BT] Found device: %s (%02X:%02X:%02X:%02X:%02X:%02X) CoD=0x%04X RSSI=%ddBm\n", device_name, addr[0],
             addr[1], addr[2], addr[3], addr[4], addr[5], cod, rssi);

  // Check if it's a DualShock4 controller
  if (name && (strstr(name, "Wireless Controller") || strstr(name, "DUALSHOCK") || strstr(name, "DualShock"))) {
    PICO_INFO("DualShock4 controller detected! Attempting connection...\n");
  }

  // As an example, if you want to filter out keyboards, do:
  if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
    PICO_DEBUG("[BT] Ignoring keyboard device\n");
    return UNI_ERROR_IGNORE_DEVICE;
  }

  return UNI_ERROR_SUCCESS;
}

static void pico_bluetooth_on_device_connected(uni_hid_device_t* d) {
  PICO_INFO("Device connected: %s (%02X:%02X:%02X:%02X:%02X:%02X)\n", d->name, d->conn.btaddr[0], d->conn.btaddr[1],
            d->conn.btaddr[2], d->conn.btaddr[3], d->conn.btaddr[4], d->conn.btaddr[5]);

  // Disable scanning when a device is connected to save power
  uni_bt_stop_scanning_safe();
  led_scan_blink_stop();
  PICO_DEBUG("[BT] Stopped scanning (device connected)\n");
}

static void pico_bluetooth_on_device_disconnected(uni_hid_device_t* d) {
  PICO_INFO("Device disconnected: %s (%02X:%02X:%02X:%02X:%02X:%02X)\n", d->name, d->conn.btaddr[0], d->conn.btaddr[1],
            d->conn.btaddr[2], d->conn.btaddr[3], d->conn.btaddr[4], d->conn.btaddr[5]);

  // Re-enable scanning when a device is disconnected
  uni_bt_start_scanning_and_autoconnect_safe();
  led_scan_blink_start();
  PICO_DEBUG("[BT] Restarted scanning (device disconnected)\n");
}

static uni_error_t pico_bluetooth_on_device_ready(uni_hid_device_t* d) {
  // You can reject the connection by returning an error.
  return UNI_ERROR_SUCCESS;
}

static const uni_property_t* pico_bluetooth_get_property(uni_property_idx_t idx) {
  ARG_UNUSED(idx);
  return NULL;
}

// Hot path: runs once per Bluetooth packet (250Hz). Kept in RAM so its
// timing never depends on the XIP flash cache shared with the USB core.
static void __not_in_flash_func(pico_bluetooth_on_controller_data)(uni_hid_device_t* d, uni_controller_t* ctl) {
  ARG_UNUSED(d);

  if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD)
    return;

  ds4_frame_t* frame = ds4_mailbox_write_slot(&g_ds4_mailbox);
  frame->gamepad = ctl->gamepad;
  frame->battery = ctl->battery;
  frame->timestamp = time_us_32();
  ds4_mailbox_publish(&g_ds4_mailbox);

  // LED heartbeat. The CYW43 LED shares the Bluetooth SPI bus, so it must be
  // driven from this core only; toggling twice a second is negligible here,
  // while doing it from the USB core would contend with HCI traffic.
  static uint32_t led_counter = 0;
  static bool led_on = false;
  if (++led_counter >= 125) {
    led_counter = 0;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
  }

#if IS_PICO_DEBUG
  static int count = 0;
  static absolute_time_t last_updated = 0;

  count++;

  absolute_time_t now = get_absolute_time();
  int64_t elapsed_us = absolute_time_diff_us(last_updated, now);
  if (elapsed_us >= 1000000) {
    PICO_DEBUG("[BT] Bluetooth data received: %u\n", count);
    last_updated = now;
    count = 0;
  }
#endif
}

static void pico_bluetooth_on_oob_event(uni_platform_oob_event_t event, void* data) {
  switch (event) {
    case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON:
      // Optional: do something when "system" button gets pressed.
      trigger_event_on_gamepad((uni_hid_device_t*)data);
      break;

    case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
      // When the "bt scanning" is on / off. Could be triggered by different events
      // Useful to notify the user
      logi("my_platform_on_oob_event: Bluetooth enabled: %d\n", (bool)(data));
      break;

    default:
      logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
  }
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
  if (d->report_parser.set_lightbar_color != NULL) {
    static uint8_t red = 0x10;
    static uint8_t green = 0x20;
    static uint8_t blue = 0x40;

    red += 0x10;
    green -= 0x20;
    blue += 0x40;
    d->report_parser.set_lightbar_color(d, red, green, blue);
  }
}

struct uni_platform* get_my_platform(void) {
  static struct uni_platform plat = {
      .name = "Pico2 W",
      .init = pico_bluetooth_init,
      .on_init_complete = pico_bluetooth_on_init_complete,
      .on_device_discovered = pico_bluetooth_on_device_discovered,
      .on_device_connected = pico_bluetooth_on_device_connected,
      .on_device_disconnected = pico_bluetooth_on_device_disconnected,
      .on_device_ready = pico_bluetooth_on_device_ready,
      .on_oob_event = pico_bluetooth_on_oob_event,
      .on_controller_data = pico_bluetooth_on_controller_data,
      .get_property = pico_bluetooth_get_property,
  };

  return &plat;
}

void bluetooth_init(void) {
  PICO_DEBUG("[INIT] Starting Bluetooth initialization...\n");

  // Keep Wi-Fi off but don't fully disable to avoid interfering with Bluetooth
  // cyw43_arch_disable_sta_mode();
  cyw43_arch_disable_ap_mode();

  // Must be called before uni_init()
  uni_platform_set_custom(get_my_platform());
  PICO_DEBUG("[INIT] Custom platform registered\n");

  // Initialize BP32
  uni_init(0, NULL);
  PICO_INFO("Bluepad32 initialized\n");
}

void bluetooth_run(void) {
  btstack_run_loop_execute();
}