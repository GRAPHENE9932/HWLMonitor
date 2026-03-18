#ifndef RCC_H
#define RCC_H

#include <stm32f072xb.h>
#include <stdnoreturn.h>

void rcc_switch_to_hsi(void);
void rcc_start_lsi(void);

static inline noreturn void rcc_sys_reset(void) {
    NVIC_SystemReset();
}
void rcc_put_all_under_reset(void);
static inline void rcc_disable_all(void) {
    RCC->AHBENR = 0x00000000u;
    RCC->APB2ENR = 0x00000000u;
    RCC->APB1ENR = 0x00000000u;
}

#endif // RCC_H
