#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdio.h>

// Debug mode detection. Normally set by CMake (DS4_DEBUG_LOGGING option);
// default to release since UART printf blocks the calling core.
#ifndef PICO_DEBUG_MODE
#define PICO_DEBUG_MODE 0
#endif

#define IS_PICO_DEBUG (PICO_DEBUG_MODE == 1)

// LOG MACROS - 프로젝트 전용 매크로 이름 사용
#if IS_PICO_DEBUG
    #define PICO_DEBUG(fmt, ...) printf("[DEBUG] " fmt, ##__VA_ARGS__)
    #define PICO_INFO(fmt, ...) printf("[INFO] " fmt, ##__VA_ARGS__)
    #define PICO_ERROR(fmt, ...) printf("[ERROR] " fmt, ##__VA_ARGS__)
#else
    #define PICO_DEBUG(fmt, ...) ((void)0)
    #define PICO_INFO(fmt, ...) ((void)0)
    #define PICO_ERROR(fmt, ...) printf("[ERROR] " fmt, ##__VA_ARGS__)
#endif

#endif  // DEBUG_H_