# Pico DS4 Bridge

A standalone project that converts DualShock 4 Bluetooth input to USB HID gamepad output using Raspberry Pi Pico 2W, based on the [Bluepad32](https://github.com/ricardoquesada/bluepad32) library.

## Purpose

This project enables your Raspberry Pi Pico 2 W to act as a bridge between DualShock 4 controllers and devices that only support USB HID gamepads. The Pico 2 W receives DualShock 4 Bluetooth input and presents it as a standard USB HID gamepad, letting it serve as a drop-in replacement for the DualShock 4 USB wireless dongle.

## Features

- **DualShock 4 Support**: Seamless connection to PS4 DualShock 4 controllers
- **USB HID Output**: Presents as a standard USB gamepad to host devices
- **Dual-Core Operation**:
  - Core 1: Bluetooth communication and DS4 handling
  - Core 0: USB HID processing and reporting
- **Status LED**: Visual feedback for DS4 connection status
- **Low Latency**: Optimized for gaming with minimal input delay (250Hz update rate)
- **Battery Status**: Forwards DS4 battery level information
- **Debug Support**: Optional debug output via UART

## Planned Features

- **DualSense Support** (TODO): PS5 DualSense controller support with haptic feedback and adaptive triggers

## Libraries Used

- **[Bluepad32](https://github.com/ricardoquesada/bluepad32)**: Bluetooth gamepad controller library
- **[Pico SDK](https://github.com/raspberrypi/pico-sdk)**: Official Raspberry Pi Pico development SDK
- **[TinyUSB](https://github.com/hathach/tinyusb)**: USB stack for embedded systems
- **[BTStack](https://github.com/bluekitchen/btstack)**: Bluetooth protocol stack
- **CYW43 Driver**: WiFi/Bluetooth chip driver for Pico W/2W

## Hardware Requirements

- Raspberry Pi Pico 2W
- USB cable for programming and power
- Sony DualShock 4 controller
- Sony DualSense controller (planned support)

## Build Instructions

### Prerequisites

1. Install the Pico SDK following the [official guide](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
2. Set the `PICO_SDK_PATH` environment variable
3. Install CMake and build tools

### Building

```bash
mkdir build
cd build
cmake ..
make -j4
```

The build will generate `pico_ds4_bridge.uf2` file.

### Flashing

1. Hold the BOOTSEL button on your Pico 2W while connecting it to your computer
2. Copy the generated `pico_ds4_bridge.uf2` file to the mounted RPI-RP2 drive
3. The Pico 2W will automatically reboot and run the firmware

## Usage

1. Power on your Pico 2W with the flashed firmware
2. Put your DualShock 4 into pairing mode (hold Share + PS buttons until light bar flashes)
3. The Pico 2W will automatically discover and connect to the DS4
4. Connect the Pico 2W to your target device via USB
5. Your DS4 input will be forwarded as a standard USB HID gamepad
6. The onboard LED will blink to indicate active connection and data transfer

## Configuration

Key configuration parameters in `src/main.c`:

- `BT_UPDATE_TIMEOUT_US`: Timeout for Bluetooth packet updates (40ms)
- `BT_UPDATE_PER_SEC`: Expected update rate (250 Hz)

CMake options (pass with `-D<option>=ON` when configuring):

- `DS4_DEBUG_LOGGING` (default `OFF`): Compile in debug logs. Logs are
  printed over a blocking 115200 UART from both cores, so leave this off in
  production - a single log line on the Bluetooth core costs milliseconds
  out of the 4ms packet budget.
- `DS4_DELETE_BT_KEYS_ON_BOOT` (default `OFF`): Delete stored Bluetooth
  pairing keys on every boot, forcing a fresh pairing each power cycle.
  Keeping them off lets an already-paired controller reconnect
  automatically.

## Debug Output

Build with `-DDS4_DEBUG_LOGGING=ON` to enable debug logs.

Debug information is available via UART on GPIO pins:
- GPIO 0 (Pin 1): UART TX
- GPIO 1 (Pin 2): UART RX
- Baud rate: 115200

### Accessing Debug Output

Connect a USB-to-Serial adapter to the UART pins and use:

```bash
sudo minicom -b 115200 -o -D /dev/ttyUSB0
```

Replace `/dev/ttyUSB0` with your actual serial device path.


## References

- [Raspberry Pi Pico 2W Datasheet](https://datasheets.raspberrypi.com/pico/pico-2-datasheet.pdf)
- [Getting Started with Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf)
- [Pico W Bluetooth Examples](https://github.com/raspberrypi/pico-examples/tree/master/pico_w)
- [Bluepad32 Documentation](https://github.com/ricardoquesada/bluepad32)

## License

This project uses libraries with various licenses. Please check individual library licenses for compliance.