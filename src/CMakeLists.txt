include_directories(../config)

add_library(dualshock4 STATIC
    comm.c
    dualshock4.c
    pico_bluetooth.c
)

target_include_directories(dualshock4 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${BLUEPAD32_ROOT}/src/components/bluepad32/include
    ${PICO_SDK_PATH}/lib/btstack/src
)

# cyw43 driver work runs in a low-priority IRQ on the core that calls
# cyw43_arch_init() (core 1), keeping it isolated from the USB core.
target_link_libraries(dualshock4 PUBLIC
    pico_stdlib
    pico_cyw43_arch_threadsafe_background # No Wi-Fi/lwIP stack
    pico_btstack_classic
    pico_btstack_ble
    pico_btstack_cyw43
    bluepad32
)

add_executable(${PROJECT_NAME}
    main.c
    usb_descriptors.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    config
    ${BLUEPAD32_ROOT}/src/components/bluepad32/include
    ${PICO_SDK_PATH}/lib/btstack/src
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    pico_stdlib
    pico_cyw43_arch_threadsafe_background # No Wi-Fi/lwIP stack
    pico_multicore
    dualshock4
    tinyusb_device
)

# Useful for debugging
target_compile_definitions(${PROJECT_NAME} PRIVATE
    CFG_TUD_CDC=0
    # PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=3000
    # WANT_HCI_DUMP=1 # This enables btstack debug
    # ENABLE_SEGGER_RTT=1
)

pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${PROJECT_NAME})
