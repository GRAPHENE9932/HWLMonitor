#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdint.h>

#define DEBUG_LOG_DEPTH 256

#ifndef NDEBUG

extern const char* debug_log_buf[DEBUG_LOG_DEPTH];
extern uint32_t debug_log_idx;

inline static void debug_log(const char* str) {
    debug_log_buf[debug_log_idx] = str;
    debug_log_idx = (debug_log_idx + 1) % DEBUG_LOG_DEPTH;
}

#else

#define debug_log(str)

#endif // NDEBUG

#endif // DEBUG_LOG_H
