#include <FreeRTOS.h>
#include <task.h>
#include <stm32f042x6.h>

void initialize_gpio(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
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

int main(void) {
    initialize_gpio();

    BaseType_t ret = xTaskCreate(
        blinking_task, "bli", 64,
        NULL, 0, NULL
    );

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
