#include "rcc.h"
#include "main_task.h"
#include "pwr_ctrl.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stm32f072xb.h>

static StaticTask_t idle_task_mem;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t main_task_mem;
static StackType_t main_task_stack[MAIN_TASK_STACK_SIZE];

void hard_fault_handler(void) {
    while (true) {
        GPIOB->ODR ^= GPIO_ODR_5;
        for (volatile uint32_t i = 0u; i < 100000u; i++);
    }
}

int main(void) {
    pwr_ctrl_start_iwdg();
    rcc_switch_to_hsi48();

    const TaskHandle_t handle = xTaskCreateStatic(
        main_task,
        "mai",
        MAIN_TASK_STACK_SIZE,
        NULL,
        0u,
        main_task_stack,
        &main_task_mem
    );
    configASSERT(handle != NULL);

    vTaskStartScheduler();

    while (true) {}
}

void vApplicationMallocFailedHook(void) {
    
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    while (true) { // Blinks with 90% duty cycle.
        GPIOB->ODR |= GPIO_ODR_5;
        for (volatile uint32_t i = 0u; i < 1000000u; i++);
        GPIOB->ODR &= ~GPIO_ODR_5;
        for (volatile uint32_t i = 0u; i < 100000u; i++);
    }
}

void vAssertCalled(const char* file, int line) {
    while (true) { // Blinks with 10% duty cycle.
        GPIOB->ODR |= GPIO_ODR_5;
        for (volatile uint32_t i = 0u; i < 100000u; i++);
        GPIOB->ODR &= ~GPIO_ODR_5;
        for (volatile uint32_t i = 0u; i < 1000000u; i++);
    }
}

void vApplicationGetIdleTaskMemory(
    StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
    uint32_t *pulIdleTaskStackSize
) {
    *ppxIdleTaskTCBBuffer = &idle_task_mem;
    *ppxIdleTaskStackBuffer = idle_task_stack;
    *pulIdleTaskStackSize = sizeof(idle_task_stack) / sizeof(StackType_t);
}
