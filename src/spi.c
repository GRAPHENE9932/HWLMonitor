#include "spi.h"
#include "err.h"
#include "gpio.h"
#include <stm32f072xb.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdint.h>

#define DMA_TIMEOUT_MS 100

static volatile int32_t cur_error = 0;
static TaskHandle_t task_to_notify = NULL;

void spi1_init(void) {
    RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

    gpio_init(GPIOA, 5, GPIO_AF | GPIO_SPEED_HIGH, 0);
    gpio_init(GPIOA, 7, GPIO_AF | GPIO_SPEED_HIGH, 0);
    gpio_init(SPI1_CS_GPIO, SPI1_CS_PIN, GPIO_OUTPUT | GPIO_SPEED_HIGH, 0);

    // Set baud rate to f_PCLK/4 (12 MHz) and enable master mode.
    SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0 | SPI_CR1_MSTR;
    // Set 8-bit data size, enable error interrupt.
    SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_ERRIE;
    
    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_SetPriority(SPI1_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
    NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3);

    SPI1_CS_GPIO->ODR |= 1 << SPI1_CS_PIN;
}

int32_t spi1_tx_byte_sync(uint8_t data) {
    cur_error = 0;
    SPI1->CR2 = (SPI1->CR2 & ~SPI_CR2_DS_Msk) | // Force 8-bit mode.
        SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
    SPI1->CR1 |= SPI_CR1_SPE;
    SPI1_CS_GPIO->ODR &= ~(1 << SPI1_CS_PIN);
    *(uint8_t*)&(SPI1->DR) = data;
    while (SPI1->SR & SPI_SR_BSY) {}
    SPI1_CS_GPIO->ODR |= 1 << SPI1_CS_PIN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    return cur_error;
}

int32_t spi1_tx(const uint8_t* data, uint32_t len) {
    cur_error = 0;
    task_to_notify = xTaskGetCurrentTaskHandle();

    while (SPI1->SR & SPI_SR_BSY) {}

    SPI1->CR2 = (SPI1->CR2 & ~SPI_CR2_DS_Msk) | // Force 8-bit mode.
        SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;

    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CPAR = (uint32_t)(&(SPI1->DR));
    DMA1_Channel3->CMAR = (uint32_t)data;
    DMA1_Channel3->CNDTR = len;
    // Set low priority (0b00), 8 bit memory and peripheral size, enable memory
    // increment mode, enable transfer error interrupt, enable transfer complete
    // interrupt.
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TEIE |
        DMA_CCR_TCIE;
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    SPI1_CS_GPIO->ODR &= ~(1 << SPI1_CS_PIN);

    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR1 |= SPI_CR1_SPE;

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DMA_TIMEOUT_MS)) != 1) {
        return ETIMEOUT;
    }

    while (SPI1->SR & SPI_SR_FTLVL_Msk) {}
    while (SPI1->SR & SPI_SR_BSY) {}

    SPI1_CS_GPIO->ODR |= 1 << SPI1_CS_PIN;

    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR2 &= ~SPI_CR2_TXDMAEN;

    return cur_error;
}

int32_t spi1_tx_repeating(uint16_t data, uint32_t repeats) {
    cur_error = 0;
    task_to_notify = xTaskGetCurrentTaskHandle();

    while (SPI1->SR & SPI_SR_BSY) {}

    SPI1->CR2 = (SPI1->CR2 & ~SPI_CR2_DS_Msk) | // Force 16-bit mode.
        SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;

    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CPAR = (uint32_t)(&(SPI1->DR));
    DMA1_Channel3->CMAR = (uint32_t)(&data);
    DMA1_Channel3->CNDTR = repeats;
    // Set low priority (0b00), 16 bit memory and peripheral size, disable
    // memory increment mode, enable transfer error interrupt, enable transfer
    // complete interrupt.
    DMA1_Channel3->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_DIR |
        DMA_CCR_TEIE | DMA_CCR_TCIE;
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    SPI1_CS_GPIO->ODR &= ~(1 << SPI1_CS_PIN);

    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR1 |= SPI_CR1_SPE;

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DMA_TIMEOUT_MS)) != 1) {
        return ETIMEOUT;
    }

    while (SPI1->SR & SPI_SR_FTLVL_Msk) {}
    while (SPI1->SR & SPI_SR_BSY) {}

    SPI1_CS_GPIO->ODR |= 1 << SPI1_CS_PIN;

    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR2 &= ~SPI_CR2_TXDMAEN;

    return cur_error;
}

void spi1_handler(void) {
    if (SPI1->SR & SPI_SR_OVR) {
        // Almost always will be generated because we are using transmit-only
        // DMA.
        SPI1->DR;
        SPI1->SR;
    }

    if (SPI1->SR & SPI_SR_MODF) {
        SPI1->CR1 |= SPI_CR1_MSTR;
        cur_error = EBUS;
    }

    if (SPI1->SR & SPI_SR_CRCERR) {
        // Must be impossible since we are not using CRC.
        cur_error = EBUS;
    }

    if (SPI1->SR & SPI_SR_FRE) {
        // Must be impossible since we are not using the TI frame format.
        cur_error = EBUS;
    }
}

void dma_ch2_3_dma2_ch1_2_handler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF3) {
        DMA1->IFCR |= DMA_IFCR_CTCIF3;
    } else if (DMA1->ISR & DMA_ISR_TEIF3) {
        DMA1->IFCR |= DMA_IFCR_CTEIF3;
        cur_error = EDMA;
    }

    configASSERT(task_to_notify != NULL);
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task_to_notify, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
