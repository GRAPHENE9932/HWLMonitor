#include "utils.h"

// Only 1 division per digit!
uint32_t i32_to_str(char* out, uint32_t buf_size, int32_t n) {
    if (buf_size == 0) {
        return 0;
    }

    if (n == 0 && buf_size > 0) {
        out[0] = '0';
        return 1;
    }

    uint32_t off = 0;
    uint32_t written = 0;
    if (n < 0 && buf_size > 0) {
        out[0] = '-';
        ++off;
        ++written;
        n = -n;
    }

    uint32_t len = 0;
    uint32_t pow = 1;
    // 1 billion is the maximum power of ten that fits into int32_t.
    while (n >= pow && pow < 1000000000) {
        pow *= 10;
        ++len;
    }

    off += len;
    while (n != 0) {
        const int32_t next_n = n / 10;
        // Same as n % 10, but apparently compiler (or me, who configured it)
        // is stupid and even with -O3 would not reuse the division above.
        const uint8_t digit = n - next_n * 10;
        n = next_n;

        --off;
        if (off < buf_size) {
            out[off] = '0' + digit;
            ++written;
        }
    }

    return written;
}

uint32_t u32_to_str_padded(char* out, uint32_t size, uint32_t n) {
    if (size > 9u) {
        for (uint32_t i = 0u; i < size - 9u; ++i) {
            out[i] = '0';
        }
    }

    int32_t off = size - 1;
    while (off >= 0) {
        const uint32_t next_n = n / 10u;
        const uint8_t digit = n - next_n * 10u;
        n = next_n;
        out[off--] = '0' + digit;
    }

    return size;
}

uint8_t u32_log10(uint32_t n) {
    // Special case. If we don't handle it, the further logic will overflow.
    if (n >= 1'000'000'000u) {
        return 9u;
    }

    if (n == 0u) {
        return 0u;
    }

    uint8_t res = 0u;
    uint32_t pow = 1u;

    while (n >= pow) {
        ++res;
        pow *= 10u;
    }

    return res - 1u;
}

uint32_t u32_pow10(uint8_t n) {
    uint32_t result = 1u;

    for (uint32_t i = 0u; i < n; ++i) {
        result *= 10u;
    }

    return result;
}
