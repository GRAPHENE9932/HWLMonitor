/**
 * This module completely takes over ADC1, TIM15, DMA CH1 and dma_ch1_handler
 * ISR.
 * Accesses to ADC are protected with a mutex.
 */

#ifndef ADC_H
#define ADC_H

#include "fix32.h"
#include <stdint.h>

#define ADC_FULL_SCALE 4095

void adc_init(void);

// Returns the real sample period (sp) in nanoseconds that will be used during
// data acquisition. Can return zero, indicating an error. The error still needs
// to be cleared with adc_get_error. Every call to adc_dma_acquire must be
// concluded with adc_dma_wait.
uint32_t adc_dma_acquire(
    uint16_t* data, uint16_t len, uint8_t channel, uint32_t desired_sp);

// Must be called to free the ADC after adc_dma_acquire. Blocks until the
// acquisition finishes.
void adc_dma_wait(void);

// Blocks until the conversion finishes. Returns 0 in case of an error. The
// error still needs to be cleared with adc_get_error.
uint16_t adc_one_shot(uint8_t channel);
// Does a a single conversion of VREFINT and channel <ch> and returns the
// calculated value in volts. Returns 0 in case of an error.
fix32_t adc_one_shot_volts(uint8_t channel);

// Estimates VDDA (ADC's range top point) by doing a single conversion of
// VREFINT. Retuns voltage in volts.
fix32_t adc_measure_vdda(void);

int32_t adc_get_error(void);

#endif // ADC_H
