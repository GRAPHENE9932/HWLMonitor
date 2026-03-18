#include "rcc.h"
#include <stdint.h>

void rcc_switch_to_hsi(void) {
    RCC->CR |= RCC_CR_HSION;
    uint32_t wait_time = 0u;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0u) {
        if (++wait_time > 128u) { // HSI must stabilize after 2 us max.
            NVIC_SystemReset(); // Fatal error.
        }
    }
    RCC->CFGR &= ~RCC_CFGR_SW_Msk;
}

void rcc_start_lsi(void) {
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0u) {} // TODO: timeout.
}

void rcc_put_all_under_reset(void) {
    const uint32_t APB2RSTR_ALL =
        RCC_APB2RSTR_DBGMCURST |
        RCC_APB2RSTR_TIM17RST |
        RCC_APB2RSTR_TIM16RST |
        RCC_APB2RSTR_TIM15RST |
        RCC_APB2RSTR_USART1RST |
        RCC_APB2RSTR_SPI1RST |
        RCC_APB2RSTR_TIM1RST |
        RCC_APB2RSTR_ADCRST;
        RCC_APB2RSTR_SYSCFGRST;
    
    const uint32_t APB1RSTR_ALL =
        RCC_APB1RSTR_CECRST |
        RCC_APB1RSTR_DACRST |
        RCC_APB1RSTR_PWRRST |
        RCC_APB1RSTR_CRSRST |
        RCC_APB1RSTR_CANRST |
        RCC_APB1RSTR_USBRST |
        RCC_APB1RSTR_I2C2RST |
        RCC_APB1RSTR_I2C1RST |
        RCC_APB1RSTR_USART4RST |
        RCC_APB1RSTR_USART3RST |
        RCC_APB1RSTR_USART2RST |
        RCC_APB1RSTR_SPI2RST |
        RCC_APB1RSTR_WWDGRST |
        RCC_APB1RSTR_TIM14RST |
        RCC_APB1RSTR_TIM7RST |
        RCC_APB1RSTR_TIM6RST |
        RCC_APB1RSTR_TIM3RST |
        RCC_APB1RSTR_TIM2RST;
    
    const uint32_t AHBRSTR_ALL =
        RCC_AHBRSTR_TSCRST |
        RCC_AHBRSTR_GPIOFRST |
        RCC_AHBRSTR_GPIOERST |
        RCC_AHBRSTR_GPIODRST |
        RCC_AHBRSTR_GPIOCRST |
        RCC_AHBRSTR_GPIOBRST |
        RCC_AHBRSTR_GPIOARST;

    RCC->APB2RSTR |= APB2RSTR_ALL;
    RCC->APB1RSTR |= APB1RSTR_ALL;
    RCC->AHBRSTR |= AHBRSTR_ALL;
}
