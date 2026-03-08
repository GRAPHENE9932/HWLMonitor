#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

// This macro is distinct from FreeRTOS's pdMS_TO_TICKS by rounding the result
// up instead of down.
#define MS_TO_TICKS(ms) ((TickType_t)(                                         \
    ((uint64_t)(ms) * (uint64_t)configTICK_RATE_HZ + 999ull)                   \
    / 1000ull                                                                  \
))

// Same as MS_TO_TICKS, but outputs at least 2 ticks. May be useful for FreeRTOS
// functions with timeouts since timeout of 1 tick can actually trigger in less
// than 1 tick equivalent time.
#define MS_TO_TICKS_ATL2(ms) (MS_TO_TICKS(ms) > 2 ? MS_TO_TICKS(ms) : 2)

// Returns amount of output characters.
uint32_t i32_to_str(char* out, uint32_t buf_size, int32_t n);

#endif // UTILS_H
