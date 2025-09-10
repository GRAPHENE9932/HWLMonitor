#include "user_input.h"

#include <stm32f042x6.h>
#include <FreeRTOS.h>
#include <task.h>

#include <stdbool.h>

#define GPIO_BTN GPIOA
static const uint32_t IDR_BITS[USER_INPUT_BUTTONS_AMOUNT] = {
    GPIO_IDR_3,
    GPIO_IDR_0,
    GPIO_IDR_5,
    GPIO_IDR_1,
    GPIO_IDR_4,
};

#define TICKS_TO_TRIGGER 3

struct button_state {
    uint8_t counter;
    enum : uint8_t {
        WAITING_FOR_PRESS,
        WAITING_FOR_UNPRESS,
        TRIGGERED,
    } state;
};

static struct button_state states[USER_INPUT_BUTTONS_AMOUNT];

static void user_input_init(void) {
    // Enable input mode with pull-up for all of them.
    GPIO_BTN->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
        GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIO_BTN->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 |
        GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);
    GPIO_BTN->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 |
        GPIO_PUPDR_PUPDR3_0 | GPIO_PUPDR_PUPDR4_0 | GPIO_PUPDR_PUPDR5_0);
}

// Increment avoiding overflow.
static uint8_t safe_inc(uint8_t* n) {
    if (*n == 0xFF) {
        return *n;
    }
    
    return ++*n;
}

static void button_state_handle_tick(struct button_state* state, bool pressed) {
    switch (state->state) {
    case WAITING_FOR_PRESS:
        if (pressed && safe_inc(&state->counter) >= TICKS_TO_TRIGGER) {
            state->state = WAITING_FOR_UNPRESS;
        }
        else if (!pressed) {
            state->counter = 0;
        }
        break;
    case WAITING_FOR_UNPRESS:
        if (!pressed && safe_inc(&state->counter) >= TICKS_TO_TRIGGER) {
            state->state = TRIGGERED;
        }
        else if (pressed) {
            state->counter = 0;
        }
        break;
    case TRIGGERED:
        break;
    default:
        configASSERT(false);
    }
}

static void button_state_reset(struct button_state* state) {
    state->counter = 0;
    state->state = WAITING_FOR_PRESS;
}

static void handle_input(void) {
    uint32_t idr = GPIO_BTN->IDR;
    for (uint8_t i = 0; i < USER_INPUT_BUTTONS_AMOUNT; ++i) {
        button_state_handle_tick(&states[i], !(idr & IDR_BITS[i]));
    }
}

void user_input_task(void*) {
    user_input_init();

    while (true) {
        handle_input();

        vTaskDelay(1);
    }
}

enum user_input_action user_input_take(void) {
    enum user_input_action result = 0;

    for (uint8_t i = 0; i < USER_INPUT_BUTTONS_AMOUNT; ++i) {
        if (states[i].state == TRIGGERED) {
            result |= 1 << i;
            button_state_reset(&states[i]);
        }
    }

    return result;
}
