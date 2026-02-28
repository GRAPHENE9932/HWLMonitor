#include "adc.h"
#include "err.h"
#include <stm32f072xb.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <assert.h>

// Maximum amount of times the busy-wait loop can iterate. At 48MHz it should
// result in a timeout of ~341.3us per instruction in the busy-wait loop.
#define ADC_CONFIG_TIMEOUT 16384
#define DMA_TIMEOUT_MS 30000
#define ADC_SINGLE_TIMEOUT_MS 5

// Boundary, at which adc_dma_acquire switches from ADC continuous mode to
// a timer-triggered mode (in nanoseconds).
#define MIN_TIMER_MODE_PERIOD 19000

#define SMPR_MAX 0b111
// Sampling period of the ADC in continuous mode depending on the ADC_SMPR
// register value in nanoseconds.
static const uint32_t CONT_SAMPLING_PERIOD[SMPR_MAX + 1] = {
    1000,
    1429,
    1857,
    2929,
    3857,
    4857,
    6000,
    18000
};

static StaticSemaphore_t mutex_mem;
static SemaphoreHandle_t mutex = NULL;
enum DmaStatus : uint8_t {
    DMA_FREE = 0,
    DMA_BUSY = 1,
    DMA_ERROR = 2,
};
static volatile uint8_t dma_status = DMA_FREE;
static TaskHandle_t task_to_notify = NULL;

static int32_t adc_calibrate(void) {
    if ((ADC1->CR & ADC_CR_ADEN) != 0) {
        ADC1->CR |= ADC_CR_ADDIS;
    }

    uint32_t wait_time = 0;
    while ((ADC1->CR & ADC_CR_ADEN) != 0) {
        if (++wait_time >= ADC_CONFIG_TIMEOUT) {
            return ETIMEOUT;
        }
    }

    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
    ADC1->CR |= ADC_CR_ADCAL;

    wait_time = 0;
    while ((ADC1->CR & ADC_CR_ADCAL) != 0) {
        if (++wait_time >= ADC_CONFIG_TIMEOUT) {
            return ETIMEOUT;
        }
    }

    return 0;
}

static int32_t adc_enable(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    RCC->CR2 |= RCC_CR2_HSI14ON;
    
    uint32_t wait_time = 0;
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) {
        if (++wait_time >= ADC_CONFIG_TIMEOUT) {
            return ETIMEOUT;
        }
    }

    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) {
        ADC1->ISR |= ADC_ISR_ADRDY;
    }

    ADC1->CR |= ADC_CR_ADEN;

    wait_time = 0;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
        if (++wait_time >= ADC_CONFIG_TIMEOUT) {
            return ETIMEOUT;
        }
    }

    return 0;
}

static void dma_init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Set high priority (0b10), 16 bit memory and peripheral size, enable
    // memory increment mode, enable transfer error interrupt, enable transfer
    // complete interrupt.
    DMA1_Channel1->CCR = DMA_CCR_PL_1 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
        DMA_CCR_MINC | DMA_CCR_TEIE | DMA_CCR_TCIE;
}

static void tim15_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

    TIM15->CR2 = TIM_CR2_MMS_1; // TRGO on update event.
}

int32_t adc_init(void) {
    mutex = xSemaphoreCreateMutexStatic(&mutex_mem);
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ERTOS;
    }

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(ADC1_COMP_IRQn);

    uint32_t ecode = 0;
    if ((ecode = adc_calibrate())) {
        return ecode;
    }
    if ((ecode = adc_enable())) {
        return ecode;
    }

    dma_init();
    tim15_init();

    if (xSemaphoreGive(mutex) != pdTRUE) {
        return ERTOS;
    }

    return 0;
}

void adc_init_port(GPIO_TypeDef* gpio, uint8_t port) {
    gpio->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0) << (port * 2);
}

static void acquire_cont(void) {
    ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_DMAEN;
    ADC1->CR |= ADC_CR_ADSTART;
}

static uint32_t acquire_with_timer(uint32_t desired_sp) {
    uint32_t arr = (desired_sp + 500) / 1000;

    // Select the hardware trigger TRG4 (TIM15_TRGO).
    ADC1->CFGR1 = ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTSEL_2 | ADC_CFGR1_DMAEN;

    TIM15->PSC = configCPU_CLOCK_HZ / 1000000; // Count every microsecond.
    TIM15->ARR = arr;
    TIM15->CNT = 0;

    ADC1->CR |= ADC_CR_ADSTART;
    TIM15->CR1 |= TIM_CR1_CEN;

    return arr * 1000;
}

static uint32_t get_smpr(uint32_t desired_sp, uint32_t* real_sp_out) {
    for (int32_t cur_smpr = SMPR_MAX; cur_smpr >= 0; --cur_smpr) {
        if (CONT_SAMPLING_PERIOD[cur_smpr] <= desired_sp) {
            *real_sp_out = CONT_SAMPLING_PERIOD[cur_smpr];
            return cur_smpr;
        }
    }

    *real_sp_out = CONT_SAMPLING_PERIOD[0];
    return 0;
}

int32_t adc_dma_acquire(
    uint16_t* data, uint16_t len, uint8_t channel, uint32_t desired_sp
) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ERTOS;
    }

    task_to_notify = xTaskGetCurrentTaskHandle();

    uint32_t real_sp = 0;
    ADC1->SMPR = get_smpr(desired_sp, &real_sp);
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 << channel;
    ADC1->IER = 0;
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
    DMA1_Channel1->CMAR = (uint32_t)(data);
    DMA1_Channel1->CNDTR = len;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    if (desired_sp < MIN_TIMER_MODE_PERIOD) {
        acquire_cont();
    } else {
        real_sp = acquire_with_timer(desired_sp);
    }
    
    return real_sp;
}

void __attribute__((interrupt("IRQ"))) dma_ch1_handler(void) {
    ADC1->CR |= ADC_CR_ADSTP;
    TIM15->CR1 &= ~(TIM_CR1_CEN);

    if (DMA1->ISR & DMA_ISR_TCIF1) { // Transfer complete.
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        dma_status = DMA_FREE;
    } else if (DMA1->ISR & DMA_ISR_TEIF1) { // Transfer error.
        DMA1->IFCR |= DMA_IFCR_CTEIF1;
        dma_status = DMA_ERROR;
    }

    configASSERT(task_to_notify != NULL);
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task_to_notify, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

int32_t adc_dma_wait(void) {
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DMA_TIMEOUT_MS)) != 1) {
        return ETIMEOUT;
    }

    uint32_t wait_time = 0;
    while ((ADC1->CR & ADC_CR_ADSTP) != 0) {
        if (++wait_time >= ADC_CONFIG_TIMEOUT) {
            return ETIMEOUT;
        }
    }

    if (xSemaphoreGive(mutex) != pdTRUE) {
        return ERTOS;
    }

    if (dma_status == DMA_FREE) {
        return 0;
    } else if (dma_status == DMA_ERROR) {
        return EDMA;
    } else {
        return EDMA;
    }
}

int32_t adc_single_convert(uint8_t channel) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ERTOS;
    }

    task_to_notify = xTaskGetCurrentTaskHandle();

    ADC1->SMPR = SMPR_MAX;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 << channel;
    ADC1->CFGR1 = 0;
    ADC1->IER = ADC_IER_EOCIE;

    ADC1->CR |= ADC_CR_ADSTART;

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ADC_SINGLE_TIMEOUT_MS)) != 1) {
        return ETIMEOUT;
    }

    const int32_t result = ADC1->DR;
    if (xSemaphoreGive(mutex) != pdTRUE) {
        return ERTOS;
    }
    return result;
}

void __attribute__((interrupt("IRQ"))) adc_comp_handler(void) {
    ADC1->ISR |= ADC_ISR_EOC;

    configASSERT(task_to_notify != NULL);
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task_to_notify, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
