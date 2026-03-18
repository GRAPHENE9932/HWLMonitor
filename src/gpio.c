#include "gpio.h"
#include <stddef.h>

#define GPIO_INDEX(gpiox) (                                                    \
    (gpiox) == GPIOA ? 0u :                                                    \
    (gpiox) == GPIOB ? 1u :                                                    \
    (gpiox) == GPIOC ? 2u :                                                    \
    (gpiox) == GPIOD ? 3u :                                                    \
    (gpiox) == GPIOE ? 4u :                                                    \
    (gpiox) == GPIOF ? 5u : 0u                                                 \
)

void gpio_init(GPIO_TypeDef* gpio, uint8_t pin, uint32_t attrs, uint32_t af) {
    gpio->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
    gpio->OTYPER |= ((attrs & 0x0000FF00) >> 8) << pin;

    gpio->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk << (pin * 2));
    gpio->OSPEEDR |= ((attrs & 0x00FF0000) >> 16) << (pin * 2);

    gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk << (pin * 2));
    gpio->PUPDR |= ((attrs & 0xFF000000) >> 24) << (pin * 2);

    gpio->AFR[af / 8] &= ~(GPIO_AFRL_AFSEL0_Msk << (pin % 8 * 4));
    gpio->AFR[af / 8] |= af << (pin % 8 * 4);

    gpio->MODER &= ~(GPIO_MODER_MODER0_Msk << (pin * 2));
    gpio->MODER |= (attrs & 0x000000FF) << (pin * 2);
}

void exti_init(GPIO_TypeDef* gpio, uint8_t exti_line, enum exti_attrs attrs) {
    if (attrs & EXTI_EVENT) {
        EXTI->EMR |= 1u << exti_line;
    } else {
        EXTI->EMR &= ~(1u << exti_line);
    }

    if (attrs & EXTI_INT) {
        EXTI->IMR |= 1u << exti_line;
    } else {
        EXTI->IMR &= ~(1u << exti_line);
    }
    
    if (attrs & EXTI_RISING) {
        EXTI->RTSR |= 1u << exti_line;
    } else {
        EXTI->RTSR &= ~(1u << exti_line);
    }

    if (attrs & EXTI_FALLING) {
        EXTI->FTSR |= 1u << exti_line;
    } else {
        EXTI->FTSR &= ~(1u << exti_line);
    }
    
    if (gpio != NULL) {
        SYSCFG->EXTICR[exti_line / 4u] &=
            ~(0b1111u << (exti_line % 4u) * 4u);
        SYSCFG->EXTICR[exti_line / 4u] |=
            (GPIO_INDEX(gpio)) << (exti_line % 4u) * 4u;
    }
}
