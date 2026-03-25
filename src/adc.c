#include "adc.h"
#include "err.h"
#include "utils.h"
#include <stm32f072xb.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdbool.h>
#include <assert.h>

#define VREF_CH 17
#define VREFINT_CAL (*(const volatile uint16_t*)(0x1FFFF7BA))
#define VDDA_CHARAC FIX32_CONST(3, 3, 0)

// Maximum amount of times the busy-wait loop can iterate. At 48MHz it should
// result in a timeout of ~341.3us per instruction in the busy-wait loop.
#define ADC_CONFIG_TIMEOUT 16384
#define DMA_TIMEOUT_MS 2000
#define ADC_SINGLE_TIMEOUT_MS 5

// Boundary, at which adc_dma_acquire switches from ADC continuous mode to
// a timer-triggered mode (in nanoseconds).
#define MIN_TIMER_MODE_PERIOD 22000

#define SMPR_MAX 0b111
// Sampling period of the ADC in continuous mode depending on the ADC_SMPR
// register value in nanoseconds. Values are calculated for f_PCLK = 48 MHz,
// f_ADC = f_PCLK/4, succeessive approximation = 12.5 1/f_ADC.
static const uint32_t CONT_SAMPLING_PERIOD[SMPR_MAX + 1] = {
    1167,
    1667,
    2167,
    3417,
    4500,
    5667,
    7000,
    21000
};

static StaticSemaphore_t mutex_mem;
static SemaphoreHandle_t mutex = NULL;
static volatile TaskHandle_t task_to_notify = NULL;
static volatile int32_t dma_isr_error = 0;
static int32_t cur_error = 0;

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
    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) {
        ADC1->ISR |= ADC_ISR_ADRDY;
    }

    ADC->CCR = ADC_CCR_VBATEN | ADC_CCR_TSEN | ADC_CCR_VREFEN;
    ADC1->CR |= ADC_CR_ADEN;

    uint32_t wait_time = 0;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
        if (++wait_time >= ADC_CONFIG_TIMEOUT) {
            return ETIMEOUT;
        }
    }

    return 0;
}

static void dma_init(void) {
    // Set high priority (0b10), 16 bit memory and peripheral size, enable
    // memory increment mode, enable transfer error interrupt, enable transfer
    // complete interrupt.
    DMA1_Channel1->CCR = DMA_CCR_PL_1 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
        DMA_CCR_MINC | DMA_CCR_TEIE | DMA_CCR_TCIE;
}

static void tim15_init(void) {
    TIM15->CR2 = TIM_CR2_MMS_1; // TRGO on update event.
}

void adc_init(void) {
    mutex = xSemaphoreCreateMutexStatic(&mutex_mem);
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        cur_error = ERTOS;
        return;
    }

    DMA1_Channel1->CCR = 0;
    DMA1->IFCR |= DMA_IFCR_CGIF1; // TODO: read errata on that.
    RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST | RCC_APB2RSTR_TIM15RST;
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADC1RST | RCC_APB2RSTR_TIM15RST);
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN | RCC_APB2ENR_TIM15EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) {} // TODO: timeout.

    NVIC_EnableIRQ(ADC1_COMP_IRQn);
    NVIC_SetPriority(ADC1_COMP_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 3);

    int32_t ecode = 0;
    if ((ecode = adc_calibrate())) {
        cur_error = ecode;
        xSemaphoreGive(mutex);
        return;
    }
    if ((ecode = adc_enable())) {
        cur_error = ecode;
        xSemaphoreGive(mutex);
        return;
    }

    dma_init();
    tim15_init();

    if (xSemaphoreGive(mutex) != pdTRUE) {
        cur_error = ERTOS;
        return;
    }
}

// Finds SMPR value such that the sample period is maximum while being smaller
// or equal than <desired_sp> (except when <desired_sp> is smaller than the
// smallest possible sample period. In that case, it is set to the smallest
// possible sample period).
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

static uint32_t acquire_cont(
    uint16_t* data, uint16_t len, uint8_t channel, uint32_t desired_sp) {
    dma_isr_error = 0;
    task_to_notify = xTaskGetCurrentTaskHandle();

    uint32_t real_sp = 0;
    ADC1->SMPR = get_smpr(desired_sp, &real_sp);
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 << channel;
    ADC1->IER = 0;
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
    DMA1_Channel1->CMAR = (uint32_t)(data);
    DMA1_Channel1->CNDTR = len;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_DMAEN;
    ADC1->CR |= ADC_CR_ADSTART;

    return real_sp;
}

static uint32_t acquire_with_timer(
    uint16_t* data, uint16_t len, uint8_t channel, uint32_t desired_sp) {
    dma_isr_error = 0;
    task_to_notify = xTaskGetCurrentTaskHandle();

    uint32_t real_sp = 0;
    ADC1->SMPR = get_smpr(desired_sp, &real_sp);
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 << channel;
    // Select the hardware trigger TRG4 (TIM15_TRGO).
    ADC1->CFGR1 = ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTSEL_2 | ADC_CFGR1_DMAEN;
    ADC1->IER = 0;
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
    DMA1_Channel1->CMAR = (uint32_t)(data);
    DMA1_Channel1->CNDTR = len;
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    const uint32_t arr = (desired_sp + 500) / 1000 - 1;
    TIM15->PSC = configCPU_CLOCK_HZ / 1000000 - 1; // Count every microsecond.
    TIM15->ARR = arr;
    TIM15->CNT = 0;

    ADC1->CR |= ADC_CR_ADSTART;
    TIM15->CR1 |= TIM_CR1_CEN;

    return arr * 1000;
}

uint32_t adc_dma_acquire(uint16_t* data, uint16_t len, uint8_t channel,
    uint32_t desired_sp) {
    configASSERT(channel <= 18);
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        cur_error = ERTOS;
        return 0;
    }
    if (cur_error != 0) {
        return 0;
    }

    if (desired_sp < MIN_TIMER_MODE_PERIOD) {
        return acquire_cont(data, len, channel, desired_sp);
    } else {
        return acquire_with_timer(data, len, channel, desired_sp);
    }
}

void dma_ch1_handler(void) {
    ADC1->CR |= ADC_CR_ADSTP;
    TIM15->CR1 &= ~(TIM_CR1_CEN);

    if (DMA1->ISR & DMA_ISR_TCIF1) { // Transfer complete.
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
    } else if (DMA1->ISR & DMA_ISR_TEIF1) { // Transfer error.
        DMA1->IFCR |= DMA_IFCR_CTEIF1;
        dma_isr_error = EDMA;
    }

    configASSERT(task_to_notify != NULL);
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task_to_notify, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void adc_dma_wait(void) {
    if (cur_error != 0) {
        xSemaphoreGive(mutex);
        return;
    }

    if (ulTaskNotifyTake(pdTRUE, MS_TO_TICKS_ATL2(DMA_TIMEOUT_MS)) != 1) {
        xSemaphoreGive(mutex);
        cur_error = ETIMEOUT;
        return;
    }

    uint32_t wait_time = 0;
    while ((ADC1->CR & ADC_CR_ADSTP) != 0) {
        if (++wait_time >= ADC_CONFIG_TIMEOUT) {
            cur_error = ETIMEOUT;
            xSemaphoreGive(mutex);
            return;
        }
    }

    if (dma_isr_error) {
        cur_error = dma_isr_error;
        return;
    }

    if (xSemaphoreGive(mutex) != pdTRUE) {
        cur_error = ERTOS;
        return;
    }
}

uint16_t adc_one_shot(uint8_t channel) {
    configASSERT(channel <= 18);
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        cur_error = ERTOS;
        return 0; 
    }
    if (cur_error != 0) {
        xSemaphoreGive(mutex);
        return 0;
    }

    task_to_notify = xTaskGetCurrentTaskHandle();

    ADC1->SMPR = SMPR_MAX;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0 << channel;
    ADC1->CFGR1 = 0;
    ADC1->IER = ADC_IER_EOCIE;

    ADC1->CR |= ADC_CR_ADSTART;

    if (ulTaskNotifyTake(pdTRUE, MS_TO_TICKS_ATL2(ADC_SINGLE_TIMEOUT_MS))
        != 1) {
        xSemaphoreGive(mutex);
        cur_error = ETIMEOUT;
        return 0;
    }

    const int32_t result = ADC1->DR;
    if (xSemaphoreGive(mutex) != pdTRUE) {
        cur_error = ERTOS;
        return 0;
    }

    return result;
}

fix32_t adc_one_shot_volts(uint8_t channel) {
    const int32_t vrefint_data = adc_one_shot(VREF_CH);
    const int32_t adc_data = adc_one_shot(channel);

    if (vrefint_data == 0 || adc_data == 0) {
        return 0;
    }

    return VDDA_CHARAC * VREFINT_CAL * (uint64_t)adc_data /
        (vrefint_data * ADC_FULL_SCALE);
}

fix32_t adc_measure_vdda(void) {
    uint32_t wait_time = 0;
    while ((PWR->CSR & PWR_CSR_VREFINTRDYF) == 0) {
        if (++wait_time >= ADC_CONFIG_TIMEOUT) {
            cur_error = ETIMEOUT;
            return 0;
        }
    }
    const uint16_t vrefint_data = adc_one_shot(VREF_CH);

    if (vrefint_data == 0) {
        return 0;
    }

    return VDDA_CHARAC * VREFINT_CAL / vrefint_data;
}

int32_t adc_get_error(void) {
    const int32_t tmp = cur_error;
    cur_error = 0;
    return tmp;
}

void adc_comp_handler(void) {
    ADC1->ISR |= ADC_ISR_EOC;

    configASSERT(task_to_notify != NULL);
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task_to_notify, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}
