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

void spi1_tx_byte_sync(uint8_t data);
void spi1_tx_hword_sync(uint16_t data);

// Transfers half-words with DMA. If a previous asynchronous transfer is still
// ongoing, blocks (yields) until it finishes.
void spi1_tx_async(const uint16_t* data, uint32_t len);

// Transfers two bytes of <data> with DMA <repeats> times continuously. Blocks
// until the transfer is complete, but makes the current task yield for the
// transmission time.
void spi1_tx_repeating_hword_async(uint16_t data, uint32_t repeats);

// Wait until the last asynchronous transmission finishes.
void spi1_wait(void);

// Returns and clears the last error that happened in the functions above.
// Returns 0 if no errors took place.
int32_t spi1_get_error(void);

#endif // SPI_H
