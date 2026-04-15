#include "i2c.h"
#include "gpio.h"
#include "utils.h"
#include "err.h"
#include <FreeRTOS.h>
#include <task.h>

// TODO: add get_error().

#define DMA_TIMEOUT_MS 100u

static TaskHandle_t i2c1_waiting_task = NULL;
static TaskHandle_t i2c2_waiting_task = NULL;
static int32_t i2c1_cur_error = 0;
static int32_t i2c2_cur_error = 0;

static void generic_init(I2C_TypeDef* i2c) {
    i2c->TIMINGR = 0x10805D88u; // 100 kHz, 100 ns rise/fall @ 48 MHz.
    i2c->CR1 = I2C_CR1_ERRIE;
    i2c->CR2 = I2C_CR2_AUTOEND;
    i2c->CR1 |= I2C_CR1_PE;
}

void i2c1_init(void) {
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    __DSB();
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    RCC->CFGR3 |= RCC_CFGR3_I2C1SW; // Set the SYSCLK clock source.
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_I2C1_DMA_RMP;

    NVIC_EnableIRQ(I2C1_IRQn);
    NVIC_SetPriority(I2C1_IRQn, 3u);
    NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
    NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 3u);

    gpio_init(
        I2C1_SCL_GPIO,
        I2C1_SCL_PIN,
        GPIO_AF | GPIO_OPEN_DRAIN | GPIO_SPEED_HIGH | GPIO_PULL_UP,
        I2C1_SCL_AF
    );
    gpio_init(
        I2C1_SDA_GPIO,
        I2C1_SDA_PIN,
        GPIO_AF | GPIO_OPEN_DRAIN | GPIO_SPEED_HIGH | GPIO_PULL_UP,
        I2C1_SDA_AF
    );

    generic_init(I2C1);
}

void i2c2_init(void) {
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
    __DSB();
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    NVIC_EnableIRQ(I2C2_IRQn);
    NVIC_SetPriority(I2C2_IRQn, 3u);
    NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
    NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 3u);

    gpio_init(
        I2C2_SCL_GPIO,
        I2C2_SCL_PIN,
        GPIO_AF | GPIO_OPEN_DRAIN | GPIO_SPEED_HIGH | GPIO_PULL_UP,
        I2C2_SCL_AF
    );
    gpio_init(
        I2C2_SDA_GPIO,
        I2C2_SDA_PIN,
        GPIO_AF | GPIO_OPEN_DRAIN | GPIO_SPEED_HIGH | GPIO_PULL_UP,
        I2C2_SDA_AF
    );

    generic_init(I2C2);
}

static void i2c_tx_generic(
    I2C_TypeDef* i2c, DMA_Channel_TypeDef* dma, uint8_t addr,
    const uint8_t* data, uint8_t len) {
    dma->CCR &= ~DMA_CCR_EN;
    i2c->CR1 = I2C_CR1_ERRIE | I2C_CR1_PE;
    i2c->CR2 = I2C_CR2_AUTOEND | (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_START |
        (addr << (I2C_CR2_SADD_Pos + 1u));
    dma->CPAR = (uint32_t)(&(i2c->TXDR));
    dma->CMAR = (uint32_t)data;
    dma->CNDTR = len;
    dma->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TEIE | DMA_CCR_TCIE;
    __DSB();
    dma->CCR |= DMA_CCR_EN;
    __DSB();
    i2c->CR1 |= I2C_CR1_TXDMAEN;
}

void i2c1_tx(uint8_t addr, const uint8_t* data, uint8_t len) {
    if (i2c1_cur_error != 0) {
        return;
    }

    configASSERT(i2c1_waiting_task == NULL);
    i2c1_waiting_task = xTaskGetCurrentTaskHandle();
    i2c_tx_generic(I2C1, I2C1_TX_DMA, addr, data, len);

    if (ulTaskNotifyTake(pdTRUE, MS_TO_TICKS_ATL2(DMA_TIMEOUT_MS)) != pdPASS) {
        i2c1_cur_error = ETIMEOUT;
        return;
    }
}

void i2c2_tx(uint8_t addr, const uint8_t* data, uint8_t len) {
    if (i2c2_cur_error != 0) {
        return;
    }

    configASSERT(i2c2_waiting_task == NULL);
    i2c2_waiting_task = xTaskGetCurrentTaskHandle();
    i2c_tx_generic(I2C2, I2C2_TX_DMA, addr, data, len);

    if (ulTaskNotifyTake(pdTRUE, MS_TO_TICKS_ATL2(DMA_TIMEOUT_MS)) != pdPASS) {
        i2c2_cur_error = ETIMEOUT;
        return;
    }
}

static void i2c_rx_generic(
    I2C_TypeDef* i2c, DMA_Channel_TypeDef* dma, uint8_t addr, uint8_t* data,
    uint8_t len) {
    i2c->CR1 = I2C_CR1_RXDMAEN | I2C_CR1_ERRIE | I2C_CR1_PE;
    i2c->CR2 = (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN |
        (addr << (I2C_CR2_SADD_Pos + 1u));
    dma->CCR &= ~DMA_CCR_EN;
    dma->CPAR = (uint32_t)(&(i2c->RXDR));
    dma->CMAR = (uint32_t)data;
    dma->CNDTR = len;
    dma->CCR = DMA_CCR_MINC | DMA_CCR_TEIE | DMA_CCR_TCIE;
    __DSB();
    dma->CCR |= DMA_CCR_EN;

    __DSB();
    i2c->CR2 |= I2C_CR2_START;
}

void i2c1_rx(uint8_t addr, uint8_t* data, uint8_t len) {
    if (i2c1_cur_error != 0) {
        return;
    }

    configASSERT(i2c1_waiting_task == NULL);
    i2c1_waiting_task = xTaskGetCurrentTaskHandle();
    i2c_rx_generic(I2C1, I2C1_RX_DMA, addr, data, len);

    if (ulTaskNotifyTake(pdTRUE, MS_TO_TICKS_ATL2(DMA_TIMEOUT_MS)) != pdPASS) {
        i2c1_cur_error = ETIMEOUT;
        return;
    }
    __DSB();
}

void i2c2_rx(uint8_t addr, uint8_t* data, uint8_t len) {
    if (i2c2_cur_error != 0) {
        return;
    }

    configASSERT(i2c2_waiting_task == NULL);
    i2c2_waiting_task = xTaskGetCurrentTaskHandle();
    i2c_rx_generic(I2C2, I2C2_RX_DMA, addr, data, len);

    if (ulTaskNotifyTake(pdTRUE, MS_TO_TICKS_ATL2(DMA_TIMEOUT_MS)) != pdPASS) {
        i2c2_cur_error = ETIMEOUT;
        return;
    }
    __DSB();
}

void dma_ch4_5_6_7_dma2_ch3_4_5_handler(void) {
    bool wake_i2c1 = false;
    bool wake_i2c2 = false;

    // TODO: use macros instead of hardcoded DMA channel values.
    if (DMA1->ISR & DMA_ISR_TCIF6) {
        DMA1->IFCR |= DMA_IFCR_CTCIF6;
        wake_i2c1 = true;
    }
    if (DMA1->ISR & DMA_ISR_TCIF7) {
        DMA1->IFCR |= DMA_IFCR_CTCIF7;
        wake_i2c1 = true;
    }
    if (DMA1->ISR & DMA_ISR_TCIF4) {
        DMA1->IFCR |= DMA_IFCR_CTCIF4;
        wake_i2c2 = true;
    }
    if (DMA1->ISR & DMA_ISR_TCIF5) {
        DMA1->IFCR |= DMA_IFCR_CTCIF5;
        wake_i2c2 = true;
    }

    if (DMA1->ISR & DMA_ISR_TEIF6) {
        DMA1->IFCR |= DMA_IFCR_CTEIF6;
        i2c1_cur_error = EDMA;
        wake_i2c1 = true;
    }
    if (DMA1->ISR & DMA_ISR_TEIF7) {
        DMA1->IFCR |= DMA_IFCR_CTEIF7;
        i2c1_cur_error = EDMA;
        wake_i2c1 = true;
    }
    if (DMA1->ISR & DMA_ISR_TEIF4) {
        DMA1->IFCR |= DMA_IFCR_CTEIF4;
        i2c2_cur_error = EDMA;
        wake_i2c2 = true;
    }
    if (DMA1->ISR & DMA_ISR_TEIF5) {
        DMA1->IFCR |= DMA_IFCR_CTEIF5;
        i2c2_cur_error = EDMA;
        wake_i2c2 = true;
    }

    if (wake_i2c1 && i2c1_waiting_task != NULL) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(i2c1_waiting_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        i2c1_waiting_task = NULL;
    }

    if (wake_i2c2 && i2c2_waiting_task != NULL) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(i2c2_waiting_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
        i2c2_waiting_task = NULL;
    }
}

void i2c1_handler(void) {
    if (I2C1->ISR & I2C_ISR_BERR) {
        I2C1->ICR |= I2C_ICR_BERRCF;
        i2c1_cur_error = EBUS;
    }
    if (I2C1->ISR & I2C_ISR_ARLO) {
        I2C1->ICR |= I2C_ICR_ARLOCF;
        i2c1_cur_error = EBUS;
    }
    if (I2C1->ISR & I2C_ISR_OVR) {
        I2C1->ICR |= I2C_ICR_OVRCF;
        i2c1_cur_error = EOVR;
    }
    if (I2C1->ISR & I2C_ISR_PECERR) {
        I2C1->ICR |= I2C_ICR_PECCF;
        i2c1_cur_error = EBUS;
    }
    if (I2C1->ISR & I2C_ISR_TIMEOUT) {
        I2C1->ICR |= I2C_ICR_TIMOUTCF;
        i2c1_cur_error = EBUS;
    }
    if (I2C1->ISR & I2C_ISR_ALERT) {
        I2C1->ICR |= I2C_ICR_ALERTCF;
        i2c1_cur_error = EBUS;
    }
}

void i2c2_handler(void) {
    if (I2C2->ISR & I2C_ISR_BERR) {
        I2C2->ICR |= I2C_ICR_BERRCF;
        i2c2_cur_error = EBUS;
    }
    if (I2C2->ISR & I2C_ISR_ARLO) {
        I2C2->ICR |= I2C_ICR_ARLOCF;
        i2c2_cur_error = EBUS;
    }
    if (I2C2->ISR & I2C_ISR_OVR) {
        I2C2->ICR |= I2C_ICR_OVRCF;
        i2c2_cur_error = EOVR;
    }
    if (I2C2->ISR & I2C_ISR_PECERR) {
        I2C2->ICR |= I2C_ICR_PECCF;
        i2c2_cur_error = EBUS;
    }
    if (I2C2->ISR & I2C_ISR_TIMEOUT) {
        I2C2->ICR |= I2C_ICR_TIMOUTCF;
        i2c2_cur_error = EBUS;
    }
    if (I2C2->ISR & I2C_ISR_ALERT) {
        I2C2->ICR |= I2C_ICR_ALERTCF;
        i2c2_cur_error = EBUS;
    }
}
