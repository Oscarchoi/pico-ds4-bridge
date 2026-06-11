#ifndef CONFIG_SDKCONFIG_H_
#define CONFIG_SDKCONFIG_H_

#define CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#define CONFIG_TARGET_PICO_W

// Bluepad32 log levels: 0=off, 1=error, 2=info, 3=debug.
// Logs go to the blocking 115200 UART from the Bluetooth core, so anything
// chatty directly eats into the 4ms packet budget. Keep errors only in
// release builds.
#if defined(PICO_DEBUG_MODE) && PICO_DEBUG_MODE
#define CONFIG_BLUEPAD32_LOG_LEVEL 2  // Info
#else
#define CONFIG_BLUEPAD32_LOG_LEVEL 1  // Error
#endif

#define CONFIG_BLUEPAD32_MAX_DEVICES 1
#define CONFIG_BLUEPAD32_MAX_ALLOWLIST 1
#define CONFIG_BLUEPAD32_GAP_SECURITY 1

// DualShock 4 / DualSense both connect over BR/EDR (Classic). Leaving BLE
// scanning enabled makes the radio interleave LE scan windows with the
// Classic link and hurts its stability, so keep it off.
// NOTE: bluepad32 checks this with #ifdef - to enable BLE, define it
// (any value); to disable, leave it undefined.
// #define CONFIG_BLUEPAD32_ENABLE_BLE_BY_DEFAULT 1

#endif  // CONFIG_SDKCONFIG_H_
