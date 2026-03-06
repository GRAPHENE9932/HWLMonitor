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
        // is stupid and even with -O3 would not reuse the devision above.
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
