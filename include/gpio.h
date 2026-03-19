#ifndef GPIO_H
#define GPIO_H

#include <stm32f072xb.h>

#define GPIO_AHBRSTR_BIT(gpiox) (                                              \
    (gpiox) == GPIOA ? RCC_AHBRSTR_GPIOARST :                                  \
    (gpiox) == GPIOB ? RCC_AHBRSTR_GPIOBRST :                                  \
    (gpiox) == GPIOC ? RCC_AHBRSTR_GPIOCRST :                                  \
    (gpiox) == GPIOD ? RCC_AHBRSTR_GPIODRST :                                  \
    (gpiox) == GPIOE ? RCC_AHBRSTR_GPIOERST :                                  \
    (gpiox) == GPIOF ? RCC_AHBRSTR_GPIOFRST : 0u                               \
)

#define GPIO_AHBENR_BIT(gpiox) (                                               \
    (gpiox) == GPIOA ? RCC_AHBENR_GPIOAEN :                                    \
    (gpiox) == GPIOB ? RCC_AHBENR_GPIOBEN :                                    \
    (gpiox) == GPIOC ? RCC_AHBENR_GPIOCEN :                                    \
    (gpiox) == GPIOD ? RCC_AHBENR_GPIODEN :                                    \
    (gpiox) == GPIOE ? RCC_AHBENR_GPIOEEN :                                    \
    (gpiox) == GPIOF ? RCC_AHBENR_GPIOFEN : 0u                                 \
)

static inline void gpio_reset_en(GPIO_TypeDef* gpio) {
    const uint32_t reset_bit = GPIO_AHBRSTR_BIT(gpio);
    const uint32_t en_bit = GPIO_AHBENR_BIT(gpio);

    RCC->AHBRSTR |= reset_bit;
    __DSB();
    RCC->AHBRSTR &= ~reset_bit;
    RCC->AHBENR |= en_bit;
}

enum gpio_mode : uint32_t {
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

inline static void gpio_write_hi(GPIO_TypeDef* gpio, uint8_t pin) {
    gpio->ODR |= 1u << pin;
}

inline static void gpio_write_lo(GPIO_TypeDef* gpio, uint8_t pin) {
    gpio->ODR &= ~(1u << pin);
}

inline static bool gpio_read(GPIO_TypeDef* gpio, uint8_t pin) {
    return gpio->IDR & (1u << pin);
}

enum exti_attrs : uint32_t {
    EXTI_EVENT = 0x01u,
    EXTI_INT = 0x02u,
    EXTI_RISING = 0x04u,
    EXTI_FALLING = 0x08u,
};

// The gpio argument can be NULL.
void exti_init(GPIO_TypeDef* gpio, uint8_t exti_line, enum exti_attrs attrs);
inline static void exti_clear(uint8_t exti_line) {
    EXTI->PR = 1u << exti_line;
}

#endif // GPIO_H
