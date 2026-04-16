#include "tcs3400.h"
#include "i2c.h"
#include "utils.h"
#include "gpio.h"
#include <string.h>

// TODO: error handling.

// These coefficients are derived from the area under the spectral response
// curves in the TCS3400 datasheet and are using R as a reference point.
#define NORM_COEF_POW 15u
#define G_NORM_COEF 40499u // 1.235923 * 32768
#define B_NORM_COEF 45194u // 1.3792 * 32768
#define IR_NORM_COEF 20403u // 0.622652 * 32768

#define CT_COEF 3610u
#define CT_OFFSET 1805u

#define DEFAULT_SENS TCS3400_SENS_1
#define SENS_INCREASE_THRESHOLD 12288u // Full range is 65535.
#define GAIN_MAX GAIN_64
#define GAIN_MIN GAIN_1

enum gain : uint8_t {
    GAIN_1 = 0b00u,
    GAIN_4 = 0b01u,
    GAIN_16 = 0b10u,
    GAIN_64 = 0b11u,
};

static enum tcs3400_sensivity cur_sens;
static struct tcs3400_raw_data last_results;

static void write_reg_8(uint8_t addr, uint8_t data) {
    const uint8_t to_send[2u] = { addr, data };
    i2c2_tx(TCS3400_I2C_ADDR, to_send, sizeof(to_send));
}

static void read_regs(uint8_t addr, uint8_t* out, uint8_t len) {
    i2c2_tx(TCS3400_I2C_ADDR, &addr, 1u);
    i2c2_rx(TCS3400_I2C_ADDR, out, len);
}

static void set_gain(enum gain gain) {
    write_reg_8(0x8Fu, gain & 0b11u);
}

static void set_integration_time(uint16_t cycles) {
    write_reg_8(0x81u, 256u - cycles);
}

static void apply_sensivity(enum tcs3400_sensivity sens) {
    enum gain gain = 0u;
    uint16_t cycles = 0u;

    switch (sens) {
    case TCS3400_SENS_1:
    case TCS3400_SENS_4:
    case TCS3400_SENS_16:
    case TCS3400_SENS_64:
        cycles = 64u;
        break;
    case TCS3400_SENS_256:
        cycles = 256u;
        break;
    }

    switch (sens) {
    case TCS3400_SENS_1:
        gain = GAIN_1;
        break;
    case TCS3400_SENS_4:
        gain = GAIN_4;
        break;
    case TCS3400_SENS_16:
        gain = GAIN_16;
        break;
    case TCS3400_SENS_64:
    case TCS3400_SENS_256:
        gain = GAIN_64;
        break;
    }

    set_gain(gain);
    set_integration_time(cycles);
}

void tcs3400_init() {
    i2c2_init();
    gpio_init(TCS3400_INT_GPIO, TCS3400_INT_PIN, GPIO_INPUT | GPIO_PULL_UP, 0u);

    // Power ON, ADC Enable and ALS Interrupt Enable at the Enable Register.
    write_reg_8(0x80u, 0b00010011u);

    cur_sens = DEFAULT_SENS;
    apply_sensivity(cur_sens);

    // Enable IR Sensor access at the IR Register.
    write_reg_8(0xC0u, 0b10000000u);
}

static void clear_saturation_flag(void) {
    write_reg_8(0xE6u, 0xFFu);
}

static uint16_t max_channel(const struct tcs3400_raw_data* data) {
    uint16_t res = max_u16(data->ir, data->r);
    res = max_u16(res, data->g);
    res = max_u16(res, data->b);
    return res;
}

static void adjust_sensivity(const struct tcs3400_raw_data* data) {
    const uint16_t max_ch = max_channel(data);

    if (data->satur_flag || max_ch == 0xFFFFu) {
        if (data->sens > TCS3400_MIN_SENSIVITY) {
            cur_sens = data->sens - 1u;
            apply_sensivity(cur_sens);
        }
        
        return;
    }

    if (cur_sens < TCS3400_MAX_SENSIVITY && max_ch < SENS_INCREASE_THRESHOLD) {
        cur_sens = data->sens + 1u;
        apply_sensivity(cur_sens);
    }
}

static bool results_ready(void) {
    return !gpio_read(TCS3400_INT_GPIO, TCS3400_INT_PIN);
}

static void clear_interrupts(void) {
    write_reg_8(0xE7u, 0xFFu);
}

void tcs3400_get_last(struct tcs3400_raw_data* data, bool adjust_sens) {
    if (results_ready()) {
        clear_interrupts();
        
        // Read several registers at once:
        // STATUS (0x93) - contains saturation flag.
        // CDATAL (0x94)
        // CDATAH (0x95)
        // RDATAL (0x96)
        // RDATAH (0x97)
        // GDATAL (0x98)
        // GDATAH (0x99)
        // BDATAL (0x9A)
        // BDATAH (0x9B)
        uint8_t data_regs[9u] = { 0u };
        read_regs(0x93u, (uint8_t*)data_regs, sizeof(data_regs));

        last_results.ir = data_regs[1] | ((uint16_t)data_regs[2] << 8u);
        last_results.r = data_regs[3] | ((uint16_t)data_regs[4] << 8u);
        last_results.g = data_regs[5] | ((uint16_t)data_regs[6] << 8u);
        last_results.b = data_regs[7] | ((uint16_t)data_regs[8] << 8u);
        last_results.satur_flag = data_regs[0] & 0x80u;
        last_results.sens = cur_sens;

        if (last_results.satur_flag) {
            clear_saturation_flag();
        }

        if (adjust_sens) {
            adjust_sensivity(&last_results);
        }
    }

    memcpy(data, &last_results, sizeof(last_results));
}

uint16_t tcs3400_compute_ct(const struct tcs3400_raw_data* data) {
    return (uint32_t)data->b * CT_COEF / data->r + CT_OFFSET; 
}

void tcs3400_normalize_data(
    const struct tcs3400_raw_data* raw,
    struct tcs3400_norm_data* out
) {
    const uint16_t ir = min_u32(
        ((uint32_t)raw->ir * IR_NORM_COEF) >> NORM_COEF_POW,
        0xFFFFu
    );
    const uint16_t r = raw->r;
    const uint16_t g = min_u32(
        ((uint32_t)raw->g * G_NORM_COEF) >> NORM_COEF_POW,
        0xFFFFu
    );
    const uint16_t b = min_u32(
        ((uint32_t)raw->b * B_NORM_COEF) >> NORM_COEF_POW,
        0xFFFFu
    );

    const uint16_t max_ch = max_u16(max_u16(ir, r), max_u16(g, b));

    // Here we are effectively applying this equation:
    // result = ch / max * 65535 = ch * 65535 / max.
    out->ir = ((uint32_t)ir * 65535u) / max_ch;
    out->r = ((uint32_t)r * 65535u) / max_ch;
    out->g = ((uint32_t)g * 65535u) / max_ch;
    out->b = ((uint32_t)b * 65535u) / max_ch;
}

void tcs3400_standby(void) {
    // Write the Enable Register back to its reset value.
    write_reg_8(0x80u, 0x00u);
}
