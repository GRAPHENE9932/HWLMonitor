#include "spi1.h"
#include "utils.h"

#include <stm32f042x6.h>

#include <stdint.h>

static uint32_t cur_data_idx = 0;
static const uint16_t DATA_TO_SEND[] = {
    0x00AE, // Set display OFF.
    0x00D5,
    0x0090,
    0x00A8,
    0x003F,
    0x00D3,
    0x0000,
    0x0040,
    0x00A1,
    0x00C8,
    0x00DA,
    0x0012,
    0x0081,
    0x00B0,
    0x00D9,
    0x0022,
    0x00DB,
    0x0030,
    0x00A4,
    0x00A6,
    0x008D,
    0x0014,
    0x00AF,
};

void spi1_initialize(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Enable the SPI1 clock.

    // Set alternate function to AF0 for PA5 (SCK) and PA7 (MOSI).
    SET_REG(
        GPIOA->AFR[0],
        GPIO_AFRL_AFSEL5_Msk | GPIO_AFRL_AFSEL7_Msk,
        0
    );
    // Set output push-pull mode to PA5 and PA7.
    SET_REG(
        GPIOA->OTYPER,
        GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6,
        0
    );
    // Enable pull-up for PA5 and PA7.
    SET_REG(
        GPIOA->PUPDR,
        GPIO_PUPDR_PUPDR5_Msk | GPIO_PUPDR_PUPDR7_Msk,
        1ul << GPIO_PUPDR_PUPDR5_Pos | 1ul << GPIO_PUPDR_PUPDR7_Pos
    );
    // Set output speed for PA5 and PA7 to high.
    SET_REG(
        GPIOA->OSPEEDR,
        GPIO_OSPEEDR_OSPEEDR5_Msk | GPIO_OSPEEDR_OSPEEDR7_Msk,
        2ul << GPIO_OSPEEDR_OSPEEDR5_Pos | 2ul << GPIO_OSPEEDR_OSPEEDR7_Pos
    );
    // Set alternate function mode for PA5 (SCK) and PA7 (MOSI).
    SET_REG(
        GPIOA->MODER,
        GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER6_Msk,
        GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_0
    );

    // Set baud rate to f_PCLK/256, clock polarity to 1 when idle, clock phase
    // to "The second clock transition is the first data capture edge",
    // enable the simplex transfer-only mode, set the MSB first frame format,
    // disable CRC, disable software slave management and select the master
    // configuration.
    SPI1->CR1 = 7ul << SPI_CR1_BR_Pos | 1ul << SPI_CR1_CPOL_Pos |
        1ul << SPI_CR1_CPHA_Pos | 1ul << SPI_CR1_BIDIMODE_Pos |
        1ul << SPI_CR1_BIDIOE_Pos | 1ul << SPI_CR1_MSTR_Pos | SPI_CR1_SSM | SPI_CR1_SSI;
    
    // Set data size to 9 bits, disable slave select output, don't use the
    // TI protocol, set FIFO reception threshold to 16 bits and umask the TXE
    // interrupt.
    SPI1->CR2 = 0b1000 << SPI_CR2_DS_Pos | 1ul << SPI_CR2_TXEIE_Pos;

    NVIC_EnableIRQ(SPI1_IRQn);

    for (volatile int i = 0; i < 10000; i++) {}
    GPIOA->ODR |= GPIO_ODR_6;

    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI.

    *(volatile uint16_t*)&(SPI1->DR) = DATA_TO_SEND[cur_data_idx++];
}

void __attribute__((interrupt("IRQ"))) spi1_handler(void) {
    if ((SPI1->SR & SPI_SR_TXE) == SPI_SR_TXE) {
        if (cur_data_idx == sizeof(DATA_TO_SEND) / sizeof(uint16_t) - 1) {

        }
        else {
            *(uint16_t*)&(SPI1->DR) = DATA_TO_SEND[cur_data_idx++];
        }
    }
}
