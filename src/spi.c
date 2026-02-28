#include "spi.h"
#include "utils.h"
#include "err.h"
#include <stm32f072xb.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdint.h>

static volatile int32_t cur_error = 0;

static void spi1_init_cs(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Set output push-pull mode.
    SET_REG(
        GPIOB->OTYPER,
        GPIO_OTYPER_OT_0,
        0
    );
    // Disable pull-ups and pull-downs.
    SET_REG(
        GPIOB->PUPDR,
        GPIO_PUPDR_PUPDR0_Msk,
        0
    );
    // Set output speed to high.
    SET_REG(
        GPIOB->OSPEEDR,
        GPIO_OSPEEDR_OSPEEDR0_Msk,
        GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1
    );
    // Set the GPIO output mode.
    SET_REG(
        GPIOB->MODER,
        GPIO_MODER_MODER0_Msk,
        GPIO_MODER_MODER0_0
    );
}

void spi1_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

    // Set alternate function to AF0 for PA5 (SCK) and PA7 (MOSI).
    SET_REG(
        GPIOA->AFR[0],
        GPIO_AFRL_AFSEL5_Msk | GPIO_AFRL_AFSEL7_Msk,
        0
    );
    // Set output push-pull mode.
    SET_REG(
        GPIOA->OTYPER,
        GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6,
        0
    );
    // Disable pull-ups and pull-downs.
    SET_REG(
        GPIOA->PUPDR,
        GPIO_PUPDR_PUPDR5_Msk | GPIO_PUPDR_PUPDR7_Msk,
        0
    );
    // Set output speed to high.
    SET_REG(
        GPIOA->OSPEEDR,
        GPIO_OSPEEDR_OSPEEDR5_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk,
        GPIO_OSPEEDR_OSPEEDR5_0 | GPIO_OSPEEDR_OSPEEDR5_1 |
        GPIO_OSPEEDR_OSPEEDR7_0 | GPIO_OSPEEDR_OSPEEDR7_1
    );
    // Set alternate function mode for PA5 (SCK) and PA7 (MOSI).
    SET_REG(
        GPIOA->MODER,
        GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER6_Msk,
        GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_0
    );

    spi1_init_cs();

    // Set baud rate to f_PCLK/4 (12 MHz) and enable master mode.
    SPI1->CR1 = SPI_CR1_BR_0 | SPI_CR1_MSTR;
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
        
    } else if ((SPI1->SR & SPI_SR_OVR)) {
        // Must be impossible since we are not receiving any data.
        SPI1->DR;
        SPI1->SR;
        cur_error = EOVR;
    } else if ((SPI1->SR & SPI_SR_MODF)) {
        SPI1->CR1 |= SPI_CR1_MSTR;
        cur_error = EBUS;
    } else if ((SPI1->SR & SPI_SR_CRCERR)) {
        // Must be impossible since we are not using CRC.
        cur_error = EBUS;
    } else if ((SPI1->SR & SPI_SR_FRE)) {
        // Must be impossible since we are not using the TI frame format.
        cur_error = EBUS;
    }
}
