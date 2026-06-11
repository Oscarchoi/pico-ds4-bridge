#include "dualshock4.h"

#include <stdbool.h>
#include <string.h>

#include <pico.h>

// Neutral report template. convert_uni_to_ds4() starts every report from
// this copy, so all the constant/zero fields are set in one struct copy
// instead of field-by-field stores and memsets.
static const ds4_report_t k_default_report = {
    // Report ID is removed in DS4 report format. It's injected by the
    // tud_hid_report() function.
    // .report_id = 0x01,
    .left_stick_x = DS4_JOYSTICK_MID,
    .left_stick_y = DS4_JOYSTICK_MID,
    .right_stick_x = DS4_JOYSTICK_MID,
    .right_stick_y = DS4_JOYSTICK_MID,
    .dpad = 0x0F,
    .button_touchpad = 0,  // 터치패드 버튼 비활성화
    .battery_level = 0x5,
};

ds4_report_t default_ds4_report() {
  return k_default_report;
}

// Hat values indexed by the bluepad32 dpad bitmask
// (bit0=up, bit1=down, bit2=right, bit3=left); invalid combos -> 0x0F.
static const uint8_t k_dpad_to_hat[16] = {
    0x0F, 0x00, 0x04, 0x0F, 0x02, 0x01, 0x03, 0x0F,
    0x06, 0x07, 0x05, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
};

uint8_t dpad_mask_to_hat(uint8_t mask) {
  return k_dpad_to_hat[mask & 0x0F];
}

// Hot path: runs once per report (250Hz) on the USB core. Kept in RAM to
// avoid XIP flash cache contention with the Bluetooth core.
void __not_in_flash_func(convert_uni_to_ds4)(const uni_gamepad_t* gamepad, uint8_t battery, ds4_report_t* ds4) {
  if (ds4 == NULL || gamepad == NULL) {
    return;
  }

  *ds4 = k_default_report;

  ds4->left_stick_x = (uint8_t)(gamepad->axis_x / 4 + 127);
  ds4->left_stick_y = (uint8_t)(gamepad->axis_y / 4 + 127);
  ds4->right_stick_x = (uint8_t)(gamepad->axis_rx / 4 + 127);
  ds4->right_stick_y = (uint8_t)(gamepad->axis_ry / 4 + 127);
  ds4->dpad = (uint32_t)(dpad_mask_to_hat(gamepad->dpad & 0x0F));

  uint16_t buttons = gamepad->buttons;
  uint16_t misc_buttons = gamepad->misc_buttons;

  // buttons
  ds4->button_south = (buttons >> 0) & 0x01;
  ds4->button_east = (buttons >> 1) & 0x01;
  ds4->button_west = (buttons >> 2) & 0x01;
  ds4->button_north = (buttons >> 3) & 0x01;
  ds4->button_l1 = (buttons >> 4) & 0x01;
  ds4->button_r1 = (buttons >> 5) & 0x01;
  ds4->button_l2 = (buttons >> 6) & 0x01;
  ds4->button_r2 = (buttons >> 7) & 0x01;
  ds4->button_l3 = (buttons >> 8) & 0x01;
  ds4->button_r3 = (buttons >> 9) & 0x01;

  // misc_buttons
  ds4->button_home = (misc_buttons >> 0) & 0x01;
  ds4->button_select = (misc_buttons >> 1) & 0x01;
  ds4->button_start = (misc_buttons >> 2) & 0x01;

  static uint8_t report_counter = 0;
  ds4->report_counter = report_counter++ & 0x3F;

  ds4->left_trigger = (uint8_t)(gamepad->brake / 4);
  ds4->right_trigger = (uint8_t)(gamepad->throttle / 4);

  // sensor data
  ds4->gyro_x = (uint16_t)(gamepad->gyro[0]);
  ds4->gyro_y = (uint16_t)(gamepad->gyro[1]);
  ds4->gyro_z = (uint16_t)(gamepad->gyro[2]);
  ds4->accel_x = (uint16_t)(gamepad->accel[0]);
  ds4->accel_y = (uint16_t)(gamepad->accel[1]);
  ds4->accel_z = (uint16_t)(gamepad->accel[2]);

  ds4->battery_level = (battery / 25) & 0x0F;
}
