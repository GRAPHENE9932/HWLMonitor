#include "usb_hid.h"
#include "drawing_task.h"
#include "sh1106.h"

#include <FreeRTOS.h>
#include <task.h>
#include <stm32f042x6.h>

void initialize_gpio(void) {
    GPIOB->MODER &= ~(GPIO_MODER_MODER3);
    GPIOB->MODER |= GPIO_MODER_MODER3_0;
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_3);
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
    GPIOB->BSRR = GPIO_BSRR_BR_3;
}

void blinking_task(void*) {
    while (true) {
        GPIOB->ODR ^= GPIO_ODR_3;
        for (volatile int i = 0; i < 100000; i++);
    }
    vTaskDelete(NULL);
}

// Use HSI48 for everything, also enable CRS.
// Enable the GPIOB clock.
void init_clock_tree(void) {
    FLASH->ACR |= 0b001; // Set LATENCY to One wait state.
    RCC->CR2 |= RCC_CR2_HSI48ON; // Enable HSI48.
    while (!(RCC->CR2 & RCC_CR2_HSI48RDY)); // Wait for HSI48 to be ready.
    RCC->CFGR |= RCC_CFGR_SW_HSI48; // Switch SYSCLK to HSI48.
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48); // Wait for switch to complete.

    CRS->CR |= CRS_CR_AUTOTRIMEN; // Enable autotrimming for CRS for HSI48.

    RCC->CFGR3 &= ~RCC_CFGR3_USBSW; // Switch USB peripheral to HSI48.
    RCC->APB1ENR |= RCC_APB1ENR_USBEN; // Enable USB interface clock.

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB.
}

int main(void) {
    init_clock_tree();
    initialize_gpio();
    usb_init();

    BaseType_t ret = xTaskCreate(
        blinking_task, "bli", 64,
        NULL, 0, NULL
    );

    TaskHandle_t drawing_task_handle;
    ret = xTaskCreate(
        drawing_task, "dra", 64,
        NULL, 1, &drawing_task_handle
    );

    sh1106_initialize(drawing_task_handle);

    if (ret != pdPASS) {
        
    }

    vTaskStartScheduler();

    while (true) {}
}

void vApplicationMallocFailedHook(void) {
    
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    
}

void vAssertCalled(const char* file, int line) {
    
}
