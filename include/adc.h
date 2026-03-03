#ifndef ADC_H
#define ADC_H

/**
 * This module completely takes over ADC1, TIM15 and DMA CH1.
 * Accesses to ADC are protected with a mutex.
 */

#include <stdint.h>

int32_t adc_init(void);

// Returns the real sample period (sp) in nanoseconds that will be used during
// data acquisition. Can return a negative value, indicating an error.
int32_t adc_dma_acquire(
    uint16_t* data, uint16_t len, uint8_t channel, uint32_t desired_sp
);

// Must be called to free the ADC after adc_dma_acquire. Blocks until the
// acquisition finishes.
int32_t adc_dma_wait(void);

// Returns either a positive ADC value or a negative error. Blocks until the
// conversion finishes.
int32_t adc_single_convert(uint8_t channel);

#endif // ADC_H
