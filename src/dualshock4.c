#include "dualshock4.h"

#include <stdbool.h>
#include <string.h>

ds4_report_t default_ds4_report() {
  ds4_report_t report = {
      // .report_id = 0x01,
      .left_stick_x = DS4_JOYSTICK_MID,
      .left_stick_y = DS4_JOYSTICK_MID,
      .right_stick_x = DS4_JOYSTICK_MID,
      .right_stick_y = DS4_JOYSTICK_MID,
      .dpad = 0x0F,
      .button_west = 0,
      .button_south = 0,
      .button_east = 0,
      .button_north = 0,
      .button_l1 = 0,
      .button_r1 = 0,
      .button_l2 = 0,
      .button_r2 = 0,
      .button_l3 = 0,
      .button_r3 = 0,
      .button_select = 0,
      .button_start = 0,
      .button_home = 0,
      .button_touchpad = 0,  // 터치패드 버튼 비활성화
      .report_counter = 0,
      .left_trigger = 0,
      .right_trigger = 0,
      .axis_timing = 0,
      .temperature = 0,
      .gyro_x = 0,
      .gyro_y = 0,
      .gyro_z = 0,
      .accel_x = 0,
      .accel_y = 0,
      .accel_z = 0,
      .unknown = {0, 0, 0, 0, 0},
      .battery_level = 0x5,
      .usb = 0,
      .microphone = 0,
      .headphone = 0,
      .extension = 0,
      .unknown2 = {0, 0},
      .touch_event = 0,
      .unknown3 = 0,
      .tpad_counter = 0,
      .tpad_touch1 = {0, 0, 0, 0},
      .tpad_touch2 = {0, 0, 0, 0},
      .unknown4 = 0,
      .tpad_prev_touch1 = {0, 0, 0, 0},
      .tpad_prev_touch2 = {0, 0, 0, 0},
      .unknown5 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  };

  return report;
}

uint8_t dpad_mask_to_hat(uint8_t mask) {
  switch (mask) {
    case 0x00:
      return 0x0F;  // none
    case DS4_BP_UP:
      return 0x00;  // up
    case DS4_BP_UP | DS4_BP_RIGHT:
      return 0x01;  // up‑right
    case DS4_BP_RIGHT:
      return 0x02;  // right
    case DS4_BP_DOWN | DS4_BP_RIGHT:
      return 0x03;  // right‑down
    case DS4_BP_DOWN:
      return 0x04;  // down
    case DS4_BP_DOWN | DS4_BP_LEFT:
      return 0x05;  // down‑left
    case DS4_BP_LEFT:
      return 0x06;  // left
    case DS4_BP_UP | DS4_BP_LEFT:
      return 0x07;  // up-left
    default:
      return 0x0F;  // default
  }
}

void convert_uni_to_ds4(const uni_gamepad_t gamepad, const uint8_t battery, ds4_report_t* ds4) {
  if (ds4 == NULL) {
    return;
  }

  // ds4->report_id = 0x01;
  ds4->left_stick_x = (uint8_t)(gamepad.axis_x / 4 + 127);
  ds4->left_stick_y = (uint8_t)(gamepad.axis_y / 4 + 127);
  ds4->right_stick_x = (uint8_t)(gamepad.axis_rx / 4 + 127);
  ds4->right_stick_y = (uint8_t)(gamepad.axis_ry / 4 + 127);
  ds4->dpad = (uint32_t)(dpad_mask_to_hat(gamepad.dpad & 0x0F));

  uint16_t buttons = gamepad.buttons;
  uint16_t misc_buttons = gamepad.misc_buttons;

  // buttons
  // printf("buttons: %04X\n", buttons);
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
  ds4->button_touchpad = 0;  // 터치패드 버튼 비활성화

  // misc_buttons
  // printf("buttons: %04X\n", misc_buttons);
  ds4->button_home = (misc_buttons >> 0) & 0x01;
  ds4->button_select = (misc_buttons >> 1) & 0x01;
  ds4->button_start = (misc_buttons >> 2) & 0x01;

  static uint8_t report_counter = 0;
  ds4->report_counter = report_counter++ & 0x3F;

  ds4->left_trigger = (uint8_t)(gamepad.brake / 4);
  ds4->right_trigger = (uint8_t)(gamepad.throttle / 4);
  ds4->axis_timing = 0;

  // sensor data
  ds4->temperature = 0;

  ds4->gyro_x = (uint16_t)(gamepad.gyro[0]);
  ds4->gyro_y = (uint16_t)(gamepad.gyro[1]);
  ds4->gyro_z = (uint16_t)(gamepad.gyro[2]);
  ds4->accel_x = (uint16_t)(gamepad.accel[0]);
  ds4->accel_y = (uint16_t)(gamepad.accel[1]);
  ds4->accel_z = (uint16_t)(gamepad.accel[2]);

  memset(ds4->unknown, 0, sizeof(ds4->unknown));


  ds4->battery_level = (battery / 25) & 0x0F;
  ds4->usb = 0x0;
  ds4->microphone = 0x0;
  ds4->headphone = 0x0;
  ds4->extension = 0x0;
  memset(ds4->unknown2, 0, sizeof(ds4->unknown2));

  ds4->touch_event = 0;
  ds4->unknown3 = 0;
  ds4->tpad_counter = 0;
  memset(ds4->tpad_touch1, 0, sizeof(ds4->tpad_touch1));
  memset(ds4->tpad_touch2, 0, sizeof(ds4->tpad_touch2));
  ds4->unknown4 = 0;
  memset(ds4->tpad_prev_touch1, 0, sizeof(ds4->tpad_prev_touch1));
  memset(ds4->tpad_prev_touch2, 0, sizeof(ds4->tpad_prev_touch2));
  memset(ds4->unknown5, 0, sizeof(ds4->unknown5));
}
