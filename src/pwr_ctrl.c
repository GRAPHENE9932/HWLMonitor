#include "pwr_ctrl.h"
#include "gpio.h"
#include "adc.h"
#include "utils.h"
#include "delay.h"
#include "ui.h"
#include "rcc.h"
#include "rtc.h"
#include <FreeRTOS.h>
#include <task.h>
#include <stm32f072xb.h>

#define TASK_PERIOD_MS 1000u
#define TASK_STACK_DEPTH 128u

#define BATV_AIN_GPIO GPIOA
#define BATV_AIN_PIN 1u
#define BATV_AIN_CH 1u
#define BATV_UNDER_THRESHOLD_MAX_COUNT 4u
#define BATV_THRESHOLD FIX32_CONST(3, 0, 0)
#define BATV_VOLTAGE_DIVIDER_RATIO 2u

#define BTN_PWR_EXTI_N 12u
#define BTN_PWR_GPIO GPIOB
#define BTN_PWR_PIN 12u
#define BTN_PWR_PRESSED_MS 50u
#define BTN_PWR_RELEASED_MS 150u

static StaticTask_t task_mem;
static StackType_t task_stack[TASK_STACK_DEPTH];
static TaskHandle_t task = NULL;
static TickType_t task_last_wake = 0u;

static fix32_t last_batv = FIX32_CONST(3, 7, 0);

static void check_batv(void) {
    static uint32_t batv_under_threshold_count = 0u;

    last_batv = adc_one_shot_volts(BATV_AIN_CH) * BATV_VOLTAGE_DIVIDER_RATIO;
    if (last_batv < BATV_THRESHOLD) {
        if (++batv_under_threshold_count > BATV_UNDER_THRESHOLD_MAX_COUNT) {
            pwr_ctrl_power_off();
        }
    } else {
        batv_under_threshold_count = 0u;
    }
}

static void iwdg_start_3s(void) {
    taskENTER_CRITICAL();

    IWDG->KR = 0x0000CCCCu;
    IWDG->KR = 0x00005555u;
    IWDG->PR = 0b011u; // /32 divider (1.25 kHz).
    IWDG->RLR = 0xFFFu; // Watchdog will reset after 3.276 s of inactivity.
    while ((IWDG->SR & IWDG_SR_PVU) != 0u) {} // TODO: timeout.
    IWDG->KR = 0x0000AAAAu;

    taskEXIT_CRITICAL();
}

static void iwdg_set_26s(void) {
    taskENTER_CRITICAL();

    IWDG->KR = 0x0000AAAAu;
    IWDG->KR = 0x00005555u;
    IWDG->PR = 0b110; // /256 divider ()
    IWDG->RLR = 0xFFFu; // Watchdog will reset after 26.208 s of inactivity.
    while ((IWDG->SR & IWDG_SR_PVU) != 0u) {} // TODO: timeout.
    IWDG->KR = 0x0000AAAAu;

    taskENTER_CRITICAL();
}

inline static void iwdg_refresh(void) {
    IWDG->KR = 0x0000AAAAu;
}

static void pwr_ctrl_task(void*) {
    task_last_wake = xTaskGetTickCount();
    while (true) {
        iwdg_refresh();
        check_batv();
        xTaskDelayUntil(&task_last_wake, MS_TO_TICKS(TASK_PERIOD_MS));
    }
}

void pwr_ctrl_init(void) {
    configASSERT(UI_BTN_GPIO[BTN_PWR] == BTN_PWR_GPIO);
    configASSERT(UI_BTN_PIN[BTN_PWR] == BTN_PWR_PIN);

    iwdg_start_3s();
    gpio_init(BATV_AIN_GPIO, BATV_AIN_PIN, GPIO_ANALOG, 0u);
    ui_set_callback(BTN_PWR, pwr_ctrl_power_off);

    task = xTaskCreateStatic(pwr_ctrl_task, "pwr", TASK_STACK_DEPTH, NULL, 3u,
        task_stack, &task_mem); // Highest priority.
    configASSERT(task);
}

fix32_t pwr_ctrl_bat_voltage(void) {
    return last_batv;
}

noreturn void pwr_ctrl_power_off(void) {
    iwdg_set_26s();

    vTaskSuspendAll();
    portDISABLE_INTERRUPTS();

    rcc_put_all_under_reset();
    rcc_disable_all();

    // We need SYSCFG for EXTI.
    RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;
    __DSB();
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SYSCFGRST;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    // We need PWR for RTC.
    RCC->APB1RSTR |= RCC_APB1RSTR_PWRRST;
    __DSB();
    RCC->APB1RSTR &= ~RCC_APB1RSTR_PWRRST;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    gpio_reset_en(BTN_PWR_GPIO);
    gpio_init(BTN_PWR_GPIO, BTN_PWR_PIN, GPIO_INPUT | GPIO_PULL_UP, 0u);
    exti_init(BTN_PWR_GPIO, BTN_PWR_EXTI_N, EXTI_EVENT | EXTI_FALLING);

    __SEV();
    __WFE();

    rtc_periodic_wakeup(20u);

    rcc_switch_to_hsi();

    // Enter STOP mode only after the button was released for
    // ~BTN_PWR_RELEASED_MS.
    uint32_t released_ms = 0u;
    while (released_ms < BTN_PWR_RELEASED_MS) {
        if ((BTN_PWR_GPIO->IDR & (1u << BTN_PWR_PIN)) != 0) {
            ++released_ms;
        }
        delay_us_at_8mhz(1000u);
    }

#ifndef NDEBUG
    DBGMCU->CR |= DBGMCU_CR_DBG_STOP;
#endif
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    PWR->CR &= ~PWR_CR_PDDS;
    PWR->CR |= PWR_CR_LPDS;

    __SEV();
    __WFE();

    // Only two wake up sources expected: RTC auto wake up and the power button.
    // So after waking up we are refreshing the watchdog and wait for the user
    // to hold the power button for ~BTN_PWR_PRESSED_MS (debouncing). Only after
    // that we reset the system.
    uint32_t pressed_ms = 0u;
    while (true) {
        iwdg_refresh();

        if ((BTN_PWR_GPIO->IDR & (1u << BTN_PWR_PIN)) == 0) {
            if (++pressed_ms > BTN_PWR_PRESSED_MS) {
                rtc_domain_reset();
                rcc_sys_reset();
            }
        } else {
            pressed_ms = 0u;
            __DSB();
            __WFE();
            rtc_clear_event();
            exti_clear(BTN_PWR_EXTI_N);
        }

        delay_us_at_8mhz(1000u);
    }
}
