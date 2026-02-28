#include "usb_hid.h"
#include "adc.h"
#include <FreeRTOS.h>
#include <adc.h>
#include <task.h>
#include <stm32f072xb.h>
#include <string.h>

static StaticTask_t idle_task_tcb;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

void initialize_gpio(void) {
    GPIOB->MODER &= ~(GPIO_MODER_MODER5);
    GPIOB->MODER |= GPIO_MODER_MODER5_0;
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_5);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
    GPIOB->BSRR = GPIO_BSRR_BR_5;

    GPIOA->MODER &= ~(GPIO_MODER_MODER15);
    GPIOA->MODER |= GPIO_MODER_MODER15_0;
    GPIOA->ODR &= ~(GPIO_ODR_15);
}

void __attribute__((interrupt("IRQ"))) hard_fault_handler(void) {
    while (true) {
        GPIOB->ODR ^= GPIO_ODR_5;
        for (volatile int i = 0; i < 100000; i++);
    }
}

void blinking_task(void*) {
    while (true) {
        GPIOB->ODR ^= GPIO_ODR_5;
        for (volatile int i = 0; i < 1000000; i++);
    }
    vTaskDelete(NULL);
}

static uint16_t data[512];
static uint16_t single_data;
static bool done = false;

void daq_task(void*) {
    adc_init_port(GPIOA, 0);
    adc_init_port(GPIOA, 1);

    int32_t err = 0;
    if ((err = adc_init()) < 0) {
        done = false;
    }

    adc_dma_acquire(data, 512, 0, 100000);
    if ((err = adc_dma_wait()) < 0) {
        done = false;
    }

    single_data = adc_single_convert(1);

    while (true) {
        done = true;
    }
}

// Use HSI48 for everything, also enable CRS.
// Enable GPIOA and GPIOB clock.
// Enable DMA clock.
void init_clock_tree(void) {
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
    RCC->CR2 |= RCC_CR2_HSI48ON; // Enable HSI48.
    while (!(RCC->CR2 & RCC_CR2_HSI48RDY)); // Wait for HSI48 to be ready.
    RCC->CFGR |= RCC_CFGR_SW_HSI48; // Switch SYSCLK to HSI48.
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48); // Wait for switch to complete.

    CRS->CR |= CRS_CR_AUTOTRIMEN; // Enable autotrimming for CRS for HSI48.

    RCC->CFGR3 &= ~RCC_CFGR3_USBSW; // Switch USB peripheral to HSI48.
    RCC->APB1ENR |= RCC_APB1ENR_USBEN; // Enable USB interface clock.

    // Enable GPIOA and GPIOB.
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
}

int main(void) {
    init_clock_tree();

    initialize_gpio();

    usb_init();

    xTaskCreate(
        blinking_task, "bli", 64,
        NULL, 0, NULL
    );

    xTaskCreate(
        daq_task, "daq", 64,
        NULL, 1, NULL
    );

    /*ret = xTaskCreate(
        user_input_task, "uin", 64,
        NULL, 2, NULL
    );

    TaskHandle_t drawing_task_handle;
    ret = xTaskCreate(
        drawing_task, "dra", 64,
        NULL, 1, &drawing_task_handle
    );

    sh1106_initialize(drawing_task_handle);

    if (ret != pdPASS) {
        
    }*/

    vTaskStartScheduler();
    //spi1_initialize();

    while (true) {}
}

void vApplicationMallocFailedHook(void) {
    
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    
}

void vAssertCalled(const char* file, int line) {
    
}

void vApplicationGetIdleTaskMemory(
    StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
    uint32_t *pulIdleTaskStackSize
) {
    *ppxIdleTaskTCBBuffer = &idle_task_tcb;
    *ppxIdleTaskStackBuffer = idle_task_stack;
    *pulIdleTaskStackSize = sizeof(idle_task_stack) / sizeof(StackType_t);
}
