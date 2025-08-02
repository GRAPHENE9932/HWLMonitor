#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

// This function will be called from interrupt context when the i2c_dma_send
// succeeds.
extern void (*i2c_transfer_over_handler)(void);

void i2c_dma_initialize(void);
void i2c_dma_send(uint8_t address, const uint8_t* data_ptr, uint16_t size);
void i2c_dma_wait_for_transfer_to_complete(void);

#endif // I2C_H
