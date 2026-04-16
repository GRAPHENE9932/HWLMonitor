#ifndef TCS3400_H
#define TCS3400_H

#include <stdint.h>

/**
 * This module takes over the same peripherals as I2C2 module and this module
 * itself.
 *
 * Accesses to the device are not protected by a mutex, so the accesses must be
 * done from the same task.
 */

#define TCS3400_I2C_ADDR 0x39u
#define TCS3400_INT_GPIO GPIOB
#define TCS3400_INT_PIN 2u
#define TCS3400_MIN_SENSIVITY TCS3400_SENS_1
#define TCS3400_MAX_SENSIVITY TCS3400_SENS_256

enum tcs3400_sensivity : uint8_t {
    TCS3400_SENS_1 = 0u,
    TCS3400_SENS_4,
    TCS3400_SENS_16,
    TCS3400_SENS_64,
    TCS3400_SENS_256,
};

struct tcs3400_raw_data {
    uint16_t ir;
    uint16_t r;
    uint16_t g;
    uint16_t b;
    bool satur_flag;
    enum tcs3400_sensivity sens;
};

struct tcs3400_norm_data {
    uint16_t ir;
    uint16_t r;
    uint16_t g;
    uint16_t b;
};

/**
 * @brief Initializes the chip with 64 cycle integration time and IR sensor
 * enabled.
 *
 * Beware, that the clear channel is inaccessible as it is mapped by the IR
 * channel.
 */
void tcs3400_init(void);

/**
 * @brief Reads out the last measured raw count value from the device.
 *
 * @param sens If set, adjusts sensivity for the next conversion.
 */
void tcs3400_get_last(struct tcs3400_raw_data* data, bool adjust_sens);

/**
 * @brief Calculates color temperature in Kelvins based on the previous color
 * measurement.
 *
 * Results are approximated by a linear equation and have decent accuracy
 * (~50 K max variation compared to Planck's law) in range between 2000 K and
 * 6000 K.
 */
uint16_t tcs3400_compute_ct(const struct tcs3400_raw_data* data);

/**
 * @brief Normalizes the data to keep one of the channels at max uint16, while
 * keeping the W/m^2 ratio for all other channels.
 */
void tcs3400_normalize_data(
    const struct tcs3400_raw_data* raw,
    struct tcs3400_norm_data* out
);

void tcs3400_standby(void);

#endif // TCS3400_H
