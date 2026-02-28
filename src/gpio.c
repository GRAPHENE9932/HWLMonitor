#include "gpio.h"

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
