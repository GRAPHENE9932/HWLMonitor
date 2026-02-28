#include "spi.h"
#include "err.h"
#include "gpio.h"
#include <stm32f072xb.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdint.h>

static volatile int32_t cur_error = 0;

void spi1_init(void) {
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

    gpio_init(GPIOA, 5, GPIO_AF | GPIO_SPEED_HIGH, 0);
    gpio_init(GPIOA, 7, GPIO_AF | GPIO_SPEED_HIGH, 0);
    gpio_init(SPI1_CS_GPIO, SPI1_CS_PIN, GPIO_OUTPUT | GPIO_SPEED_HIGH, 0);

    // Set baud rate to f_PCLK/4 (12 MHz) and enable master mode.
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0 | SPI_CR1_MSTR;
    // Set 8-bit data size, enable TX buffer empty interrupt and error
    // interrupt.
    SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_ERRIE;
    
    NVIC_EnableIRQ(SPI1_IRQn);

    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI.
}

void spi1_tx_byte(uint8_t data) {
    while ((SPI1->SR & SPI_SR_BSY) != 0) {}
    while ((SPI1->SR & SPI_SR_TXE) == 0) {}
    *(uint8_t*)&(SPI1->DR) = data;
}

int32_t spi1_get_error(void) {
    int32_t tmp = cur_error;
    cur_error = 0;
    return tmp;
}

void __attribute__((interrupt("IRQ"))) spi1_handler(void) {
    if ((SPI1->SR & SPI_SR_TXE) != 0) {
        
    }

    if ((SPI1->SR & SPI_SR_OVR) != 0) {
        // Must be impossible since we are not receiving any data.
        SPI1->DR;
        SPI1->SR;
        cur_error = EOVR;
    }

    if ((SPI1->SR & SPI_SR_MODF) != 0) {
        SPI1->CR1 |= SPI_CR1_MSTR;
        cur_error = EBUS;
    }

    if ((SPI1->SR & SPI_SR_CRCERR) != 0) {
        // Must be impossible since we are not using CRC.
        cur_error = EBUS;
    }

    if ((SPI1->SR & SPI_SR_FRE) != 0) {
        // Must be impossible since we are not using the TI frame format.
        cur_error = EBUS;
    }
}
