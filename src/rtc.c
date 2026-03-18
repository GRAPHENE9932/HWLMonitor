#include "rtc.h"
#include "rcc.h"
#include "gpio.h"
#include <stm32f072xb.h>
#include <stddef.h>

#define EXTI_LINE 20u

static void pwr_interface_en(void) {
    RCC->APB1RSTR |= RCC_APB1RSTR_PWRRST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_PWRRST;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
}

static void rtc_reset(void) {
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;
}

static void rtc_dis_write_prot(void) {
    PWR->CR |= PWR_CR_DBP;
    RTC->WPR = 0xCAu;
    RTC->WPR = 0x53u;
}

static void rtc_en_write_prot(void) {
    RTC->WPR = 0xFEu;
    RTC->WPR = 0x64u;
    PWR->CR &= ~PWR_CR_DBP;
}

void rtc_periodic_wakeup(uint32_t period_s) {
    rtc_reset();
    rcc_start_lsi();
    pwr_interface_en();

    rtc_dis_write_prot();

    // Select LSI as RTC clock.
    RCC->BDCR &= ~RCC_BDCR_RTCSEL_Msk;
    RCC->BDCR |= (0b10u << RCC_BDCR_RTCSEL_Pos) | RCC_BDCR_RTCEN; 

    RTC->ISR |= RTC_ISR_INIT; // Enter init mode.
    while ((RTC->ISR & RTC_ISR_INITF) == 0u) {} // TODO: timeout.

    RTC->CR &= ~RTC_CR_WUTE;
    while ((RTC->ISR & RTC_ISR_WUTWF) == 0u) {} // TODO: timeout.

    // ck_spre (usually 1 Hz) clock for wake-up.
    RTC->CR &= ~RTC_CR_WUCKSEL_Msk;
    RTC->CR |= 0b100u << RTC_CR_WUCKSEL_Pos;

    RTC->PRER = // ~1.002 Hz @ 40 kHz LSI, 0.751-1.252 Hz @ 30-50 kHz LSI.
        0x7Fu << RTC_PRER_PREDIV_A_Pos | 0x0137u << RTC_PRER_PREDIV_S_Pos;

    RTC->WUTR = period_s - 1u;

    RTC->ISR &= ~RTC_ISR_INIT; // Exit init mode.

    RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;

    rtc_clear_event();

    rtc_en_write_prot();

    exti_init(NULL, EXTI_LINE, EXTI_EVENT | EXTI_RISING);
}

void rtc_clear_event(void) {
    rtc_dis_write_prot();
    RTC->ISR &= ~RTC_ISR_WUTF;
    rtc_en_write_prot();
    exti_clear(EXTI_LINE);
}

void rtc_domain_reset(void) {
    RCC->BDCR |= RCC_BDCR_BDRST;
    __DSB();
    RCC->BDCR &= ~RCC_BDCR_BDRST;
}
