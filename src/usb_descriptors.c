#include <class/hid/hid.h>
#include <pico/cyw43_arch.h>
#include <tusb.h>

#include "debug.h"
#include "dualshock4.h"

#define min(x, y) (x) < (y) ? (x) : (y)
#define max(x, y) (x) > (y) ? (x) : (y)

#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)

#define GAMEPAD_INTERFACE 0
#define GAMEPAD_ENDPOINT 1
#define GAMEPAD_SIZE 64

#define DS4_GET_CALIBRATION 0x02        // PS4 Controller Calibration
#define DS4_DEFINITION 0x03             // PS4 Controller Definition
#define DS4_SET_FEATURE_STATE 0x05      // PS4 Controller Features
#define DS4_GET_MAC_ADDRESS 0x12        // PS4 Controller MAC
#define DS4_GET_MAC_ADDRESS_SHORT 0x81  // PS4 Controller MAC
#define DS4_SET_HOST_MAC 0x13           // Set Host MAC
#define DS4_SET_USB_BT_CONTROL 0x14     // Set USB/BT Control Mode
#define DS4_GET_VERSION_DATE 0xA3       // PS4 Controller Version & Date
#define DS4_SET_AUTH_PAYLOAD 0xF0       // Set Auth Payload
#define DS4_GET_SIGNATURE_NONCE 0xF1    // Get Signature Nonce
#define DS4_GET_SIGNING_STATE 0xF2      // Get Signing State
#define DS4_RESET_AUTH 0xF3             // Unknown (PS4 Report 0xF3)

bool is_ds4_initialized = false;
bool is_usb_mounted = false;
bool report_in_flight = false;

// Device Descriptor
tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x054C,   // Sony Corporation
    .idProduct = 0x09CC,  // DualShock 4
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,  // String Descriptor index 1
    .iProduct = 0x02,       // String Descriptor index 2
    .iSerialNumber = 0x00,
    .bNumConfigurations = 0x01,
};

// HID Report Descriptor for DualShock 4 Interface
uint8_t const ds4_hid_report_desc[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop Controls)
    0x09, 0x05,  // Usage (Game Pad)
    0xA1, 0x01,  // Collection (Application)

    0x85, 0x01,        // Report ID (1)
    0x09, 0x30,        // Usage (X)
    0x09, 0x31,        // Usage (Y)
    0x09, 0x32,        //	Usage (Z)
    0x09, 0x35,        // Usage (Rz)
    0x15, 0x00,        // Logical Minimum (0)
    0x26, 0xFF, 0x00,  // Logical Maximum (255)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x04,        // Report Count (4)
    0x81, 0x02,        // Input (...)

    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x42,        //   Input (...)
    0x65, 0x00,        //   Unit (None)

    0x05, 0x09,  //   Usage Page (Button)
    0x19, 0x01,  //   Usage Minimum (0x01)
    0x29, 0x0E,  //   Usage Maximum (0x0E)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x0E,  //   Report Count (14)
    0x81, 0x02,  //   Input (...)

    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x20,        //   Usage (0x20)
    0x75, 0x06,        //   Report Size (6)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (...)

    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input ()

    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x36,        //   Report Count (54)
    0x81, 0x02,        //   Input (...)`

    0x85, 0x05,  //   Report ID (5)
    0x09, 0x22,  //   Usage (0x22)
    0x95, 0x1F,  //   Report Count (31)
    0x91, 0x02,  //   Output (...)

    0x85, 0x03,        //   Report ID (3)
    0x0A, 0x21, 0x27,  //   Usage (0x2721)
    0x95, 0x2F,        //   Report Count (47)
    0xB1, 0x02,        //   Feature (...)

    0x85, 0x02,        //   Report ID (2)
    0x09, 0x24,        //   Usage (0x24)
    0x95, 0x24,        //   Report Count (36)
    0xB1, 0x02,        //
    0x85, 0x08,        //   Report ID (8)
    0x09, 0x25,        //   Usage (0x25)
    0x95, 0x03,        //   Report Count (3)
    0xB1, 0x02,        //
    0x85, 0x10,        //   Report ID (16)
    0x09, 0x26,        //   Usage (0x26)
    0x95, 0x04,        //   Report Count (4)
    0xB1, 0x02,        //
    0x85, 0x11,        //   Report ID (17)
    0x09, 0x27,        //   Usage (0x27)
    0x95, 0x02,        //   Report Count (2)
    0xB1, 0x02,        //
    0x85, 0x12,        //   Report ID (18)
    0x06, 0x02, 0xFF,  //   Usage Page (Vendor Defined 0xFF02)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x0F,        //   Report Count (15)
    0xB1, 0x02,        //
    0x85, 0x13,        //   Report ID (19)
    0x09, 0x22,        //   Usage (0x22)
    0x95, 0x16,        //   Report Count (22)
    0xB1, 0x02,        //
    0x85, 0x14,        //   Report ID (20)
    0x06, 0x05, 0xFF,  //   Usage Page (Vendor Defined 0xFF05)
    0x09, 0x20,        //   Usage (0x20)
    0x95, 0x10,        //   Report Count (16)
    0xB1, 0x02,        //
    0x85, 0x15,        //   Report ID (21)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x2C,        //   Report Count (44)
    0xB1, 0x02,        //
    0x06, 0x80, 0xFF,  //   Usage Page (Vendor Defined 0xFF80)
    0x85, 0x80,        //   Report ID (128)
    0x09, 0x20,        //   Usage (0x20)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //
    0x85, 0x81,        //   Report ID (129)
    0x09, 0x21,        //   Usage (0x21)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //
    0x85, 0x82,        //   Report ID (130)
    0x09, 0x22,        //   Usage (0x22)
    0x95, 0x05,        //   Report Count (5)
    0xB1, 0x02,        //
    0x85, 0x83,        //   Report ID (131)
    0x09, 0x23,        //   Usage (0x23)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //
    0x85, 0x84,        //   Report ID (132)
    0x09, 0x24,        //   Usage (0x24)
    0x95, 0x04,        //   Report Count (4)
    0xB1, 0x02,        //
    0x85, 0x85,        //   Report ID (133)
    0x09, 0x25,        //   Usage (0x25)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //
    0x85, 0x86,        //   Report ID (134)
    0x09, 0x26,        //   Usage (0x26)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //
    0x85, 0x87,        //   Report ID (135)
    0x09, 0x27,        //   Usage (0x27)
    0x95, 0x23,        //   Report Count (35)
    0xB1, 0x02,        //
    0x85, 0x88,        //   Report ID (136)
    0x09, 0x28,        //   Usage (0x28)
    0x95, 0x22,        //   Report Count (34)
    0xB1, 0x02,        //
    0x85, 0x89,        //   Report ID (137)
    0x09, 0x29,        //   Usage (0x29)
    0x95, 0x02,        //   Report Count (2)
    0xB1, 0x02,        //
    0x85, 0x90,        //   Report ID (144)
    0x09, 0x30,        //   Usage (0x30)
    0x95, 0x05,        //   Report Count (5)
    0xB1, 0x02,        //
    0x85, 0x91,        //   Report ID (145)
    0x09, 0x31,        //   Usage (0x31)
    0x95, 0x03,        //   Report Count (3)
    0xB1, 0x02,        //
    0x85, 0x92,        //   Report ID (146)
    0x09, 0x32,        //   Usage (0x32)
    0x95, 0x03,        //   Report Count (3)
    0xB1, 0x02,        //
    0x85, 0x93,        //   Report ID (147)
    0x09, 0x33,        //   Usage (0x33)
    0x95, 0x0C,        //   Report Count (12)
    0xB1, 0x02,        //
    0x85, 0xA0,        //   Report ID (160)
    0x09, 0x40,        //   Usage (0x40)
    0x95, 0x06,        //   Report Count (6)
    0xB1, 0x02,        //
    0x85, 0xA1,        //   Report ID (161)
    0x09, 0x41,        //   Usage (0x41)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //
    0x85, 0xA2,        //   Report ID (162)
    0x09, 0x42,        //   Usage (0x42)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //
    0x85, 0xA3,        //   Report ID (163)
    0x09, 0x43,        //   Usage (0x43)
    0x95, 0x30,        //   Report Count (48)
    0xB1, 0x02,        //
    0x85, 0xA4,        //   Report ID (164)
    0x09, 0x44,        //   Usage (0x44)
    0x95, 0x0D,        //   Report Count (13)
    0xB1, 0x02,        //
    0x85, 0xA5,        //   Report ID (165)
    0x09, 0x45,        //   Usage (0x45)
    0x95, 0x15,        //   Report Count (21)
    0xB1, 0x02,        //
    0x85, 0xA6,        //   Report ID (166)
    0x09, 0x46,        //   Usage (0x46)
    0x95, 0x15,        //   Report Count (21)
    0xB1, 0x02,        //
    0x85, 0xA7,        //   Report ID (247)
    0x09, 0x4A,        //   Usage (0x4A)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //
    0x85, 0xA8,        //   Report ID (250)
    0x09, 0x4B,        //   Usage (0x4B)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //
    0x85, 0xA9,        //   Report ID (251)
    0x09, 0x4C,        //   Usage (0x4C)
    0x95, 0x08,        //   Report Count (8)
    0xB1, 0x02,        //
    0x85, 0xAA,        //   Report ID (252)
    0x09, 0x4E,        //   Usage (0x4E)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //
    0x85, 0xAB,        //   Report ID (253)
    0x09, 0x4F,        //   Usage (0x4F)
    0x95, 0x39,        //   Report Count (57)
    0xB1, 0x02,        //
    0x85, 0xAC,        //   Report ID (254)
    0x09, 0x50,        //   Usage (0x50)
    0x95, 0x39,        //   Report Count (57)
    0xB1, 0x02,        //
    0x85, 0xAD,        //   Report ID (255)
    0x09, 0x51,        //   Usage (0x51)
    0x95, 0x0B,        //   Report Count (11)
    0xB1, 0x02,        //
    0x85, 0xAE,        //   Report ID (256)
    0x09, 0x52,        //   Usage (0x52)
    0x95, 0x01,        //   Report Count (1)
    0xB1, 0x02,        //
    0x85, 0xAF,        //   Report ID (175)
    0x09, 0x53,        //   Usage (0x53)
    0x95, 0x02,        //   Report Count (2)
    0xB1, 0x02,        //
    0x85, 0xB0,        //   Report ID (176)
    0x09, 0x54,        //   Usage (0x54)
    0x95, 0x3F,        //   Report Count (63)
    0xB1, 0x02,        //
    0xC0,              // End Collection

    0x06, 0xF0, 0xFF,  // Usage Page (Vendor Defined 0xFFF0)
    0x09, 0x40,        // Usage (0x40)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0xF0,        //   Report ID (-16) AUTH F0
    0x09, 0x47,        //   Usage (0x47)
    0x95, 0x3F,        //   Report Count (63)
    0xB1, 0x02,        //
    0x85, 0xF1,        //   Report ID (-15) AUTH F1
    0x09, 0x48,        //   Usage (0x48)
    0x95, 0x3F,        //   Report Count (63)
    0xB1, 0x02,        //
    0x85, 0xF2,        //   Report ID (-14) AUTH F2
    0x09, 0x49,        //   Usage (0x49)
    0x95, 0x0F,        //   Report Count (15)
    0xB1, 0x02,        //
    0x85, 0xF3,        //   Report ID (-13) Auth F3 (Reset)
    0x0A, 0x01, 0x47,  //   Usage (0x4701)
    0x95, 0x07,        //   Report Count (7)
    0xB1, 0x02,        //
    0xC0,              // End Collection
};

#define DS4_CONFIG1_DESC_SIZE (9 + 9 + 9 + 7 + 7)
static const uint8_t ds4_configuration_descriptor[] = {
    // configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
    9,                           // bLength;
    2,                           // bDescriptorType;
    LSB(DS4_CONFIG1_DESC_SIZE),  // wTotalLength
    MSB(DS4_CONFIG1_DESC_SIZE),
    1,     // bNumInterfaces
    1,     // bConfigurationValue
    0,     // iConfiguration
    0x80,  // bmAttributes
    50,    // bMaxPower
           // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
    9,     // bLength
    4,     // bDescriptorType
    0,  // bInterfaceNumber
    0,                  // bAlternateSetting
    2,                  // bNumEndpoints
    0x03,               // bInterfaceClass (0x03 = HID)
    0x00,               // bInterfaceSubClass (0x00 = No Boot)
    0x00,               // bInterfaceProtocol (0x00 = No Protocol)
    0,                  // iInterface
    // HID interface descriptor, HID 1.11 spec, section 6.2.1
    9,                                 // bLength
    0x21,                              // bDescriptorType
    0x11, 0x01,                        // bcdHID
    0,                                 // bCountryCode
    1,                                 // bNumDescriptors
    0x22,                              // bDescriptorType
    LSB(sizeof(ds4_hid_report_desc)),  // wDescriptorLength
    MSB(sizeof(ds4_hid_report_desc)),
    // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
    7,                        // bLength
    5,                        // bDescriptorType
    GAMEPAD_ENDPOINT | 0x80,  // bEndpointAddress
    0x03,                     // bmAttributes (0x03=intr)
    GAMEPAD_SIZE, 0,          // wMaxPacketSize
    4,                        // bInterval (1 ms)
    0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x01};

// --- String Descriptors ---
char const* string_desc_arr[] = {
    (const char[]){0x09, 0x04},        // 0: Language ID (United States)
    "Sony Interactive Entertainment",  // 1: Manufacturer
    "Wireless Controller",             // 2: Product
    "123456789abc",                    // 3: Serial (DUMMY)
};

static uint16_t _desc_str[32];

// Callback: Device Descriptor
uint8_t const* tud_descriptor_device_cb(void) {
  return (uint8_t const*)&desc_device;
}

// Callback: Configuration Descriptor
uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;  // Only one configuration supported
  return ds4_configuration_descriptor;
}

// Callback: HID Report Descriptor
uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
  (void)instance;
  return ds4_hid_report_desc;
}

// Callback: String Descriptor
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;

  uint8_t chr_count;
  if (index == 0) {
    // LangID
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else {
    if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
      return NULL;
    const char* str = string_desc_arr[index];
    chr_count = strlen(str);
    if (chr_count > 31)
      chr_count = 31;
    for (uint8_t i = 0; i < chr_count; i++) {
      _desc_str[i + 1] = str[i];
    }
  }
  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
  return _desc_str;
}

const char* board_usb_get_serial(void) {
  return "000000000001";
}

//---------------------------------------------------------------------
// tud_hid_get_report_cb
//---------------------------------------------------------------------
uint16_t tud_hid_get_report_cb(uint8_t instance,
                               uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t* buffer,
                               uint16_t reqlen) {
  (void)instance;
  if (report_type != HID_REPORT_TYPE_FEATURE) {
    PICO_DEBUG("tud_hid_get_report_cb: not feature report\n");
    return 0;
  }
  printf("############# tud_hid_get_report_cb: id=%d, type=%d, size=%d\n", report_id,
         report_type, reqlen);

  PICO_INFO("Got hid report: id=%d, type=%d, size=%d\n", report_id, report_type,
            reqlen);

  uint16_t responseLen = 0;
  switch (report_id) {
    case DS4_GET_CALIBRATION: {
      static uint8_t calibration[] = {
          0xfe, 0xff, 0x0e, 0x00, 0x04, 0x00, 0xd4, 0x22, 0x2a,
          0xdd, 0xbb, 0x22, 0x5e, 0xdd, 0x81, 0x22, 0x84, 0xdd,
          0x1c, 0x02, 0x1c, 0x02, 0x85, 0x1f, 0xb0, 0xe0, 0xc6,
          0x20, 0xb5, 0xe0, 0xb1, 0x20, 0x83, 0xdf, 0x0c, 0x00};
      responseLen = min(reqlen, sizeof(calibration));
      memcpy(buffer, calibration, responseLen);
      return responseLen;
    }
    case DS4_DEFINITION: {
      static uint8_t controller_desc[] = {
          0x21, 0x27, 0x04, 0xcf, 0x00, 0x2c, 0x56, 0x08, 0x00, 0x3d,
          0x00, 0xe8, 0x03, 0x04, 0x00, 0xff, 0x7f, 0x0d, 0x0d, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      responseLen = min(reqlen, sizeof(controller_desc));
      memcpy(buffer, controller_desc, responseLen);
      buffer[4] = 0x00;  // controller type: 0x00 = DS4
      return responseLen;
    }
    case DS4_GET_MAC_ADDRESS: {
      static uint8_t mac_address[] = {
          0x01, 0x02, 0x03, 0x04, 0x05, 0x06,  // device MAC address
          0x08, 0x25, 0x00,                    // BT device class
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00   // host MAC address
      };
      responseLen = min(reqlen, sizeof(mac_address));
      memcpy(buffer, mac_address, responseLen);
      return responseLen;
    }
    case DS4_GET_MAC_ADDRESS_SHORT: {
      static uint8_t mac_address_short[] = {
          0x01, 0x02, 0x03, 0x04, 0x05, 0x06,  // device MAC address
      };
      responseLen = min(reqlen, sizeof(mac_address_short));
      memcpy(buffer, mac_address_short, responseLen);
      return responseLen;
    }
    case DS4_GET_VERSION_DATE: {
      static uint8_t firmware_version_date[] = {
          0x4a, 0x75, 0x6e, 0x20, 0x20, 0x39, 0x20, 0x32, 0x30, 0x31,
          0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31, 0x32, 0x3a, 0x33,
          0x36, 0x3a, 0x34, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x01, 0x08, 0xb4, 0x01, 0x00, 0x00, 0x00,
          0x07, 0xa0, 0x10, 0x20, 0x00, 0xa0, 0x02, 0x00};
      responseLen = min(reqlen, sizeof(firmware_version_date));
      memcpy(buffer, firmware_version_date, responseLen);
      return responseLen;
    }
    case DS4_GET_SIGNATURE_NONCE: {
      PICO_DEBUG("tud_hid_get_report_cb: DS4_GET_SIGNATURE_NONCE\n");
      return 63;
    }
    case DS4_GET_SIGNING_STATE: {
      PICO_DEBUG("tud_hid_get_report_cb: DS4_GET_SIGNING_STATE\n");
      return 15;
    }
    case DS4_RESET_AUTH: {
      PICO_DEBUG("tud_hid_get_report_cb: DS4_RESET_AUTH\n");
      static uint8_t reset_auth[] = {0x0, 0x38, 0x38, 0, 0, 0, 0};
      responseLen = min(reqlen, sizeof(reset_auth));
      memcpy(buffer, reset_auth, responseLen);
      return responseLen;
    }
    default:
      PICO_ERROR("Unknown report ID %d\n", report_id);
      break;
  }

  return 0;
}

//---------------------------------------------------------------------
// tud_hid_set_report_cb
//---------------------------------------------------------------------
void tud_hid_set_report_cb(uint8_t instance,
                           uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const* buffer,
                           uint16_t bufsize) {
  (void)instance;
  PICO_DEBUG(
      "################### tud_hid_set_report_cb: ID=%d, Type=%d, Size=%d\n",
      report_id, report_type, bufsize);

#if IS_PICO_DEBUG
  static int report_count = 0;
  static absolute_time_t last_updated = 0;

  absolute_time_t now = get_absolute_time();
  int64_t elapsed_us = absolute_time_diff_us(last_updated, now);

  report_count++;
  if (elapsed_us > 1000000) {  // 1 sec
    PICO_DEBUG("[USB] USB Report received: %d count\n", report_count);
    last_updated = now;
    report_count = 0;
  }
#endif

  // printf("tud_hid_set_report_cb: ID=%d, Type=%d, Size=%d\n", report_id,
  // report_type, bufsize);
  ds4_feature_output_report_t feature;
  if (report_type == HID_REPORT_TYPE_OUTPUT) {
    if (report_id == 0 && !is_ds4_initialized) {
      memcpy(&feature, buffer, bufsize);
    }
    // printf("Feature Report:\n");
    // printf("Report ID: %d\n", feature.reportID);
    // printf("Enable Update Rumble: %d\n", feature.enableUpdateRumble);
    // printf("Enable Update LED: %d\n", feature.enableUpdateLED);
    // printf("Enable Update LED Blink: %d\n", feature.enableUpdateLEDBlink);
    // printf("Enable Update Ext Data: %d\n", feature.enableUpdateExtData);
    // printf("Enable Update Volume Left: %d\n", feature.enableUpdateVolLeft);
    // printf("Enable Update Volume Right: %d\n", feature.enableUpdateVolRight);
    // printf("Enable Update Volume Mic: %d\n", feature.enableUpdateVolMic);
    // printf("Enable Update Volume Speaker: %d\n",
    // feature.enableUpdateVolSpeaker); printf("Rumble: %d, %d\n",
    // feature.rumbleLeft, feature.rumbleRight); printf("LED: %d, %d, %d\n",
    // feature.ledRed, feature.ledGreen, feature.ledBlue); printf("LED Blink:
    // %d, %d\n", feature.ledBlinkOn, feature.ledBlinkOff); printf("Volume: %d,
    // %d, %d, %d\n", feature.volumeLeft, feature.volumeRight,
    // feature.volumeMic,
    //        feature.volumeSpeaker);
    // printf("Ext Data: ");
    // for (int i = 0; i < sizeof(feature.extData); i++) {
    //   printf("%02X ", feature.extData[i]);
    // }
    // printf("\n");
    // printf("Unknown: %d\n", feature.unknown0);
    // printf("Unknown Audio: %d\n", feature.unknownAudio);
    // printf("Padding: ");
    // for (int i = 0; i < sizeof(feature.padding); i++) {
    //   printf("%02X ", feature.padding[i]);
    // }
    // printf("\n");
    // printf("[INFO] Received feature report with ID %d, size %d\n", report_id,
    // bufsize);

    is_ds4_initialized = true;
  }
}

void tud_mount_cb(void) {
  PICO_INFO("USB mounted\n");
  is_usb_mounted = true;
}

void tud_umount_cb(void) {
  PICO_INFO("USB unmounted\n");
}

void tud_hid_report_complete_cb(uint8_t instance,
                                uint8_t const* report,
                                uint16_t len) {
  report_in_flight = false;
  // PICO_DEBUG("################### tud_hid_report_complete_cb: ID=%d, Size=%d\n",
  //            report[0], len);
}

void tud_hid_report_failed_cb(uint8_t instance,
                              hid_report_type_t report_type,
                              uint8_t const* report,
                              uint16_t xferred_bytes) {
  PICO_DEBUG(
      "################### tud_hid_report_failed_cb: ID=%d, Type=%d, Size=%d\n",
      report[0], report_type, xferred_bytes);
}

void tud_suspend_cb(bool remote_wakeup_en) {
  PICO_DEBUG("[USB] SUSPEND (remote_wakeup_en=%d)\n", remote_wakeup_en);
}

void tud_resume_cb(void) {
  PICO_DEBUG("[USB] RESUME\n");
}

void tud_hid_report_received_cb(uint8_t itf, uint8_t const* rpt, uint16_t len) {
  PICO_DEBUG("################### tud_hid_report_received_cb: itf=%d, size=%d\n", itf, len);
  // if (len && rpt[0] == 0x05) { /* LED/rumble */ }
}