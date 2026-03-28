#include "opt4001.h"
#include "i2c.h"

// TODO: error handling.

static void write_reg(uint8_t addr, uint16_t data) {
    const uint8_t to_send[3u] = { addr, data >> 8u, data };
    i2c1_tx(OPT4001_I2C_ADDR, to_send, sizeof(to_send));
}

static uint16_t read_reg(uint8_t addr) {
    uint8_t res[2u];
    i2c1_tx(OPT4001_I2C_ADDR, &addr, 1u);
    i2c1_rx(OPT4001_I2C_ADDR, res, sizeof(res));
    return (res[0] << 8u) | res[1];
}

void opt4001_init(void) {
    i2c1_init();
}

void opt4001_start_cont(void) {
    // Disable quick wake-up, enable auto-range, 200 ms conversion time,
    // continuous conversion mode. Rest is at the reset value.
    write_reg(0x0Au, (12u << 10u) | (9u << 6u) | (3u << 4u) | (1u << 3u));
}

static uint32_t calc_mlx(uint8_t exponent, uint32_t mantissa) {
    const uint32_t adc_codes = mantissa << exponent;

    // adc_codes is a 28-bit number. To get the Lux value datasheet tells us to
    // multiply adc_codes by 437.5E-6. As we want to output the result in mlx,
    // we need to multiply it by 0.4375 = 7/16.
    return (adc_codes * 7u) >> 4u;
}

static uint8_t calc_crc(uint16_t reg_1, uint16_t reg_2) {
    const uint32_t regs = ((uint32_t)reg_1 << 16u) | reg_2;

    constexpr uint32_t MASK_0 = 0xFFFFFFF0u;
    constexpr uint32_t MASK_1 = 0xAAAAAAA0u;
    constexpr uint32_t MASK_2 = 0x88888880u;
    constexpr uint32_t MASK_3 = 0x08080800u;

    // TODO: use a portable way to XOR the whole uint32_t.
    return
        __builtin_parity(regs & MASK_0) |
        (__builtin_parity(regs & MASK_1) << 1u) |
        (__builtin_parity(regs & MASK_2) << 2u) |
        (__builtin_parity(regs & MASK_3) << 3u);
}

uint32_t opt4001_read_mlx(void) {
    while (true) {
        const uint16_t reg_1 = read_reg(0x00u);
        const uint16_t reg_2 = read_reg(0x01u);
        const uint8_t crc = reg_2 & 0x000Fu;

        if (calc_crc(reg_1, reg_2) == crc) {
            const uint8_t exponent = reg_1 >> 12u;
            const uint32_t mantissa =
                ((uint32_t)(reg_1 & 0x0FFFu) << 8u) | (reg_2 >> 8u);
            return calc_mlx(exponent, mantissa);
        }
    }
}

void opt4001_standby(void) {
    // Writes the reset value, which has OPERATING_MODE=Power-down.
    write_reg(0x0Au, 0x3208u);
}
