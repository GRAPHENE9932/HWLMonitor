#ifndef SPI_H
#define SPI_H

/**
 * This module completely takes over SPI1, DMA1 CH3 and dma_ch2_3_dma2_ch1_2
 * ISR.
 * Accesses to SPI are NOT protected with a mutex and must be done from the same
 * task.
 */

#include <stdint.h>

#define SPI1_CS_GPIO GPIOB
#define SPI1_CS_PIN 0

void spi1_init(void);

// Returns either 0 or a negative error.
int32_t spi1_tx_byte_sync(uint8_t data);
// Transfers data with DMA. Blocks until the transfer is complete, but makes the
// current task yield for the transmission time.
// Returns either 0 or a negative error.
int32_t spi1_tx(const uint8_t* data, uint32_t len);
// Transfers two bytes of <data> with DMA <repeats> times continuously. Blocks
// until the transfer is complete, but makes the current task yield for the
// transmission time.
// Returns either 0 or a negative error.
int32_t spi1_tx_repeating(uint16_t data, uint32_t repeats);

#endif // SPI_H
