cmake_minimum_required(VERSION 3.13)

if(DEFINED PICO_BOARD)
    message(DEBUG, "Using PICO_BOARD ${PICO_BOARD}")
else()
    message(VERBOSE, "PICO_BOARD not defined. Using 'pico2_w'")
    set(PICO_BOARD "pico2_w")
endif()

if(DEFINED PICO_PLATFORM)
    message(DEBUG, "Using PICO_PLATFORM ${PICO_PLATFORM}")
else()
    message(VERBOSE, "PICO_PLATFORM not defined. Using 'rp2350'")
    set(PICO_PLATFORM "rp2350")
endif()

# Initialize the SDK based on PICO_SDK_PATH
# NOTE: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_ds4_bridge C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

###############################################################################

# Debug logging uses blocking UART printf on both cores; on the Bluetooth
# core a single log line can cost milliseconds, so it is opt-in.
option(DS4_DEBUG_LOGGING "Enable debug logging (blocking UART printf)" OFF)

# Deleting stored pairing keys forces a fresh pairing on every boot.
# Keeping them (default) lets a paired controller auto-reconnect.
option(DS4_DELETE_BT_KEYS_ON_BOOT "Delete stored BT pairing keys on boot" OFF)

if(DS4_DEBUG_LOGGING)
    add_compile_definitions(PICO_DEBUG_MODE=1)
else()
    add_compile_definitions(PICO_DEBUG_MODE=0)
endif()

if(DS4_DELETE_BT_KEYS_ON_BOOT)
    add_compile_definitions(PICO_DS4_DELETE_BT_KEYS_ON_BOOT=1)
else()
    add_compile_definitions(PICO_DS4_DELETE_BT_KEYS_ON_BOOT=0)
endif()

add_subdirectory(src)

include_directories(config src)

set(BLUEPAD32_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/lib/bluepad32)
add_subdirectory(${BLUEPAD32_ROOT}/src/components/bluepad32 libbluepad32)