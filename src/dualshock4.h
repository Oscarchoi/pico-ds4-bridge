#ifndef DUALSHOCK4_H_
#define DUALSHOCK4_H_

#include <stdint.h>
#include <stddef.h>

#include <controller/uni_controller.h>

#define DS4_JOYSTICK_MIN 0x00
#define DS4_JOYSTICK_MID 0x7F
#define DS4_JOYSTICK_MAX 0xFF

#define DS4_BP_UP (1 << 0)
#define DS4_BP_DOWN (1 << 1)
#define DS4_BP_RIGHT (1 << 2)
#define DS4_BP_LEFT (1 << 3)

typedef struct __attribute__((packed)) {
  uint8_t reportID;  // 0

  // 1
  uint8_t enableUpdateRumble : 1;
  uint8_t enableUpdateLED : 1;
  uint8_t enableUpdateLEDBlink : 1;
  uint8_t enableUpdateExtData : 1;
  uint8_t enableUpdateVolLeft : 1;
  uint8_t enableUpdateVolRight : 1;
  uint8_t enableUpdateVolMic : 1;
  uint8_t enableUpdateVolSpeaker : 1;

  // 2
  uint8_t : 8;

  // 3
  uint8_t unknown0;

  // 4
  uint8_t rumbleRight;

  // 5
  uint8_t rumbleLeft;

  // 6
  uint8_t ledRed;

  // 7
  uint8_t ledGreen;

  // 8
  uint8_t ledBlue;

  // 9
  uint8_t ledBlinkOn;

  // 10
  uint8_t ledBlinkOff;

  // 11
  uint8_t extData[8];

  // 19
  uint8_t volumeLeft;  // 0x00-0x4F

  // 20
  uint8_t volumeRight;  // 0x00-0x4F

  // 21
  uint8_t volumeMic;  // 0x01-0x4F, 0x00 is special state

  // 22
  uint8_t volumeSpeaker;  // 0x00-0x4F

  // 23
  uint8_t unknownAudio;

  // 24
  uint8_t padding[8];
} ds4_feature_output_report_t;

// See for more details: https://www.psdevwiki.com/ps4/DS4-USB#Data_Format
typedef struct __attribute__((packed)) {
  // NOTE(oscarchoi): if report does not contain report_id, it is required to
  // invoke tud_hid_report() with report_id which is not 0x00.
  // uint8_t report_id;      // 0 (not used)
  uint8_t left_stick_x;   // 1
  uint8_t left_stick_y;   // 2
  uint8_t right_stick_x;  // 3
  uint8_t right_stick_y;  // 4

  // 4 bits for the d-pad.
  uint32_t dpad : 4;

  // 14 bits for buttons.
  uint32_t button_west : 1;
  uint32_t button_south : 1;
  uint32_t button_east : 1;
  uint32_t button_north : 1;
  uint32_t button_l1 : 1;
  uint32_t button_r1 : 1;
  uint32_t button_l2 : 1;
  uint32_t button_r2 : 1;
  uint32_t button_select : 1;
  uint32_t button_start : 1;
  uint32_t button_l3 : 1;
  uint32_t button_r3 : 1;
  uint32_t button_home : 1;
  uint32_t button_touchpad : 1;
  uint32_t report_counter : 6;
  uint32_t left_trigger : 8;   // 8
  uint32_t right_trigger : 8;  // 9
  uint16_t axis_timing;        // timing counter: 10-11

  uint8_t temperature;  // 12

  uint16_t gyro_x;  // 13-14
  uint16_t gyro_y;  // 15-16
  uint16_t gyro_z;  // 17-18

  uint16_t accel_x;  // 19-20
  uint16_t accel_y;  // 21-22
  uint16_t accel_z;  // 23-24

  uint8_t unknown[5];  // 25-29

  uint8_t battery_level : 4;  // 30
  uint8_t usb : 1;  // 30
  uint8_t microphone : 1;  // 30
  uint8_t headphone : 1;  // 30
  uint8_t extension : 1;  // 30
  uint8_t unknown2[2];  // 31-32

  uint8_t touch_event : 4;  // 33
  uint8_t unknown3 : 4;  // 33

  uint8_t tpad_counter;  // 34
  uint8_t tpad_touch1[4];  // 35-38
  uint8_t tpad_touch2[4];  // 39-42

  uint8_t unknown4;  // 43
  uint8_t tpad_prev_touch1[4];  // 44-47
  uint8_t tpad_prev_touch2[4];  // 48-51

  uint8_t unknown5[12];  // 52-63
} ds4_report_t;

_Static_assert(sizeof(ds4_report_t) == 63, "DS4 report size mismatch");
_Static_assert(offsetof(ds4_report_t, temperature) == 11, "temperature byte must be at 12");
_Static_assert(offsetof(ds4_report_t, tpad_counter) == 33,
               "touch block must start at byte offset 34");
_Static_assert(offsetof(ds4_report_t, unknown5) == 51, "unknown5 byte must be at 52");

ds4_report_t default_ds4_report();

uint8_t dpad_mask_to_hat(uint8_t mask);

void convert_uni_to_ds4(const uni_gamepad_t gamepad, const uint8_t battery, ds4_report_t* ds4);

#endif  // DUALSHOCK4_H_