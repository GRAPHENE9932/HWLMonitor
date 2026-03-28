#ifndef I2C_H
#define I2C_H

#include <stm32f072xb.h>
#include <stdint.h>

/**
 * This module completely takes over I2C1, DMA1 CH6 and CH7 if using I2C1;
 * I2C2, DMA1 CH4, CH5 if using I2C2;
 * implements dma_ch4_5_6_7_dma2_ch3_4_5_handler ISR either way.
 *
 * Peripheral accesses are NOT protected by a mutex or other synchronization
 * methods, so accesses to the particular I2C's must be done from the same task.
 */

#define I2C1_SCL_GPIO GPIOB
#define I2C1_SCL_PIN 6u
#define I2C1_SCL_AF 1u
#define I2C1_SDA_GPIO GPIOB
#define I2C1_SDA_PIN 7u
#define I2C1_SDA_AF 1u
#define I2C1_RX_DMA DMA1_Channel7
#define I2C1_TX_DMA DMA1_Channel6

#define I2C2_SCL_GPIO GPIOB
#define I2C2_SCL_PIN 10u
#define I2C2_SCL_AF 1u
#define I2C2_SDA_GPIO GPIOB
#define I2C2_SDA_PIN 11u
#define I2C2_SDA_AF 1u
#define I2C2_RX_DMA DMA1_Channel5
#define I2C2_TX_DMA DMA1_Channel4

// Initializes I2C1 in 100 kHz standard mode.
void i2c1_init(void);
// Initializes I2C2 in 100 kHz standard mode.
void i2c2_init(void);

/**
 * @brief Transmits len bytes to the specified address.
 * 
 * Blocks (yields) until the transmission is finished.
 *
 * @param addr 8-bit peripheral address with LSB signifying R/W.
 */
void i2c1_tx(uint8_t addr, const uint8_t* data, uint8_t len);

/**
 * @brief Transmits len bytes to the specified address.
 * 
 * Blocks (yields) until the transmission is finished.
 *
 * @param addr 8-bit peripheral address with LSB signifying R/W.
 */
void i2c2_tx(uint8_t addr, const uint8_t* data, uint8_t len);

/**
 * @brief Reads out at most len bytes from the slave.
 *
 * Blocks (yields) until the reception is complete.
 * 
 * @param addr 8-bit peripheral address with LSB signifying R/W.
 */
void i2c1_rx(uint8_t addr, uint8_t* data, uint8_t len);

/**
 * @brief Reads out at most len bytes from the slave.
 *
 * Blocks (yields) until the reception is complete.
 * 
 * @param addr 8-bit peripheral address with LSB signifying R/W.
 */
void i2c2_rx(uint8_t addr, uint8_t* data, uint8_t len);

#endif // I2C_H
