#ifndef SPI_H
#define SPI_H

/**
 * This module completely takes over SPI1.
 * Accesses to SPI are NOT protected with a mutex.
 */

#include <stdint.h>

// Initializes SPI1 with PB0 being CS, PA5 - SCK and PA7 - MOSI.
void spi1_init(void);

void spi1_tx_byte(uint8_t data);

// Returns either zero or an error that happened in the past. This call
// clears the error. SPI1 should be reset with spi1_init() if an error took
// place.
int32_t spi1_get_error(void);

#endif // SPI_H
