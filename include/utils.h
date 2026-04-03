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

static inline uint32_t min_u32(uint32_t a, uint32_t b) {
    return a < b ? a : b;
}

static inline int32_t max_i32(int32_t a, int32_t b) {
    return a > b ? a : b;
}

static inline int32_t min_i32(int32_t a, int32_t b) {
    return a < b ? a : b;
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Returns amount of output characters.
uint32_t i32_to_str(char* out, uint32_t buf_size, int32_t n);

/**
 * @brief Converts a 32-bit unsigned integer into a text decimal representation
 * with preceding zeroes (if the buffer is large enough).
 *
 * @example
 * u32_to_str_padded(buf, 10u, 12345u) will write "0000012345" into buf.
 * 
 * @param size Amount of digits that WILL be written.
 * @param n The number itself.
 * @return uint32_t Amount of chars written out. Equal to size.
 */
uint32_t u32_to_str_padded(char* out, uint32_t size, uint32_t n);

/**
 * @brief Calculates log10(n) rounded down.
 */
uint8_t u32_log10(uint32_t n);

/**
 * @brief Calculates 10^n
 */
uint32_t u32_pow10(uint8_t n);

#endif // UTILS_H
