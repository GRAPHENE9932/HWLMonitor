#ifndef FIX32_H
#define FIX32_H

#include <stdint.h>

// 16.16 signed fixed-point number.
typedef int32_t fix32_t;

#define FIX32_PRV_LOG10(x) (                                                   \
    ((uint64_t)(x)) == 0U ? 0 :                                                \
    ((uint64_t)(x)) < 10U ? 1 :                                                \
    ((uint64_t)(x)) < 100U ? 2 :                                               \
    ((uint64_t)(x)) < 1000U ? 3 :                                              \
    ((uint64_t)(x)) < 10000U ? 4 :                                             \
    ((uint64_t)(x)) < 100000U ? 5 :                                            \
    ((uint64_t)(x)) < 1000000U ? 6 :                                           \
    ((uint64_t)(x)) < 10000000U ? 7 :                                          \
    ((uint64_t)(x)) < 100000000U ? 8 :                                         \
    ((uint64_t)(x)) < 1000000000U ? 9 :                                        \
    ((uint64_t)(x)) < 10000000000U ? 10 :                                      \
    ((uint64_t)(x)) < 100000000000U ? 11 :                                     \
    ((uint64_t)(x)) < 1000000000000U ? 12 :                                    \
    ((uint64_t)(x)) < 10000000000000U ? 13 :                                   \
    ((uint64_t)(x)) < 100000000000000U ? 14 :                                  \
    ((uint64_t)(x)) < 1000000000000000U ? 15 :                                 \
    ((uint64_t)(x)) < 10000000000000000U ? 16 :                                \
    ((uint64_t)(x)) < 100000000000000000U ? 17 :                               \
    ((uint64_t)(x)) < 1000000000000000000U ? 18 :                              \
    ((uint64_t)(x)) < 10000000000000000000U ? 19 : -1                          \
)

#define FIX32_PRV_POW(x, p) (                                                  \
    ((p) == 0) ? 1 :                                                           \
    ((p) == 1) ? ((uint64_t)(x)) :                                             \
    ((p) == 2) ? ((uint64_t)(x)) * ((uint64_t)(x)) :                           \
    ((p) == 3) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) :         \
    ((p) == 4) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) :                                             \
    ((p) == 5) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) * ((uint64_t)(x)) :                           \
    ((p) == 6) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) :         \
    ((p) == 7) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) :                                             \
    ((p) == 8) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) * ((uint64_t)(x)) :                           \
    ((p) == 9) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *         \
                 ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) :         \
    ((p) == 10) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) :                                            \
    ((p) == 11) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) :                          \
    ((p) == 12) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) :        \
    ((p) == 13) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) :                                            \
    ((p) == 14) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) :                          \
    ((p) == 15) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) :        \
    ((p) == 16) ? ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) * ((uint64_t)(x)) * ((uint64_t)(x)) *        \
                  ((uint64_t)(x)) : 0xFFFFFFFFFFFFFFFF                         \
)

#define FIX32_PRV_POS_CONST(i, f, z) (                                         \
    (((uint32_t)(i)) << 16) |                                                  \
    (                                                                          \
        (((uint64_t)(f)) << (16U - (FIX32_PRV_LOG10(f) + z))) /                \
        FIX32_PRV_POW(5, (FIX32_PRV_LOG10(f) + z))                             \
    )                                                                          \
)

/**
 * @brief Creates a fix32_t constant from the human-readable (almost)
 * representation in compile-time.
 *
 * If the integer OR fraction part is negative, the constant will be negative.
 * If the described number cannot be represented with fix32_t exactly, the
 * result is rounded down.
 * Examples:
 * - FIX32_CONST(31, 125, 0) // 31.125
 * - FIX32_CONST(-31, 125, 0) // -31.125
 * - FIX32_CONST(0, 1373291015625, 3) // 0.0001373291015625
 * - FIX32_CONST(0, -261688232421875, 1) // -0.0261688232421875
 * - FIX32_CONST(0, 9999000000000000, 0) // 0.9998931884765625
 * 
 * @param i The integer part of the number.
 * @param f The fractional part of the number (without preceding zeroes).
 * @param z The number of zeroes preceding the fraction.
 * part.
 */
#define FIX32_CONST(i, f, z) ((fix32_t)(                                       \
    ((i) < 0) ? -FIX32_PRV_POS_CONST(-(i), f, z) : ((f) < 0) ?                 \
        -FIX32_PRV_POS_CONST(i, -(f), z) : FIX32_PRV_POS_CONST(i, f, z)))

#define FIX32_ZERO ((fix32_t)0x00000000)
#define FIX32_ONE ((fix32_t)0x00010000)
#define FIX32_MINUS_ONE ((fix32_t)0xFFFEFFFF)
#define FIX32_MAX ((fix32_t)0x7FFFFFFF)
#define FIX32_MIN ((fix32_t)0x80000000)

fix32_t fix32_mul(fix32_t a, fix32_t b);
fix32_t fix32_div(fix32_t a, fix32_t b);

fix32_t fix32_sqrt(fix32_t num);
fix32_t fix32_inv_sqrt(fix32_t num);

static inline fix32_t fix32_abs(fix32_t num) {
    return num < FIX32_ZERO ? -num : num;
}

static inline uint32_t fix32_round_to_u32(fix32_t num) {
    return (num + FIX32_ONE / 2) >> 16;
}

#endif // FIX32_H
