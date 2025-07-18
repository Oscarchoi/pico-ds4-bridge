#ifndef DUALSHOCK4_H_
#define DUALSHOCK4_H_

#include <stdint.h>

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

typedef struct {
  uint8_t counter : 7;
  uint8_t unpressed : 1;
  uint8_t data[3];  // 12 bit X, followed by 12 bit Y
} ds4_touchpad_t;

typedef struct {
  ds4_touchpad_t p1;
  ds4_touchpad_t p2;
} ds4_touchpad_data_t;

typedef struct __attribute__((packed)) {
  int16_t x;
  int16_t y;
  int16_t z;
} ds4_sensor_t;

typedef struct __attribute__((packed)) {
  ds4_sensor_t gyro;   // 6 bytes
  ds4_sensor_t accel;  // 6 bytes
  uint8_t misc[5];     // 5 bytes
  uint8_t batteryLevel : 4;
  uint8_t usb : 1;
  uint8_t microphone : 1;
  uint8_t headphone : 1;
  uint8_t extension : 1;  // 1 byte
  uint8_t extData0 : 1;
  uint8_t extData1 : 1;
  uint8_t notConnected : 1;
  uint8_t extData3 : 5;  // 1 byte
  uint8_t misc2;         // 1 byte
} ds4_sensor_data_t;     // 19 bytes

typedef struct __attribute__((packed)) {
  uint8_t report_id;      // 0
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

  uint8_t battery;  // 12
  ds4_sensor_data_t sensor_data;

  uint8_t unknown[7];
  uint8_t num_trackpad_packets;
  uint8_t touchpad_active : 2;
  uint8_t padding : 6;
  uint8_t tpad_increment;
  ds4_touchpad_data_t touchpad_data;
  uint8_t mystery_2[13];
} ds4_report_t;

ds4_report_t default_ds4_report();

uint8_t dpad_mask_to_hat(uint8_t mask);

void convert_uni_to_ds4(const uni_gamepad_t gamepad, const uint8_t battery, ds4_report_t* ds4);

#endif  // DUALSHOCK4_H_