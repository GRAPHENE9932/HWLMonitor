#ifndef GPIO_H
#define GPIO_H

#include <stm32f072xb.h>

enum gpio_mode : uint8_t {
    GPIO_INPUT = 0b00,
    GPIO_OUTPUT = 0b01,
    GPIO_AF = 0b10,
    GPIO_ANALOG = 0b11
};

enum gpio_output_type : uint32_t {
    GPIO_PUSH_PULL = 0,
    GPIO_OPEN_DRAIN = 1 << 8
};

enum gpio_speed : uint32_t {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM = 0b01 << 16,
    GPIO_SPEED_HIGH = 0b11 << 16
};

enum gpio_pull_up_down : uint32_t {
    GPIO_NO_PULL = 0,
    GPIO_PULL_UP = 0b01 << 24,
    GPIO_PULL_DOWN = 0b10 << 24
};

// The attrs argument parameters must contain ORred gpio_mode, gpio_output_type,
// gpio_speed and gpio_pull_up_down.
void gpio_init(GPIO_TypeDef* gpio, uint8_t pin, uint32_t attrs, uint32_t af);

#endif // GPIO_H
