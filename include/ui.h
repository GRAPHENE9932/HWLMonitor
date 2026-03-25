#ifndef UI_H // User Input.
#define UI_H

#include <stdint.h>
#include <stdbool.h>
#include <stm32f072xb.h>
#include <FreeRTOS.h>

#define UI_BTN_AMOUNT 6u

enum ui_btn : uint32_t {
    BTN_PWR = 0u,
    BTN_UP,
    BTN_DOWN,
    BTN_LEFT,
    BTN_RIGHT,
    BTN_OK,
};

static GPIO_TypeDef* const UI_BTN_GPIO[UI_BTN_AMOUNT] = {
    GPIOB,
    GPIOB,
    GPIOA,
    GPIOA,
    GPIOB,
    GPIOB,
};

static const uint8_t UI_BTN_PIN[UI_BTN_AMOUNT] = {
    12u,
    13u,
    8u,
    9u,
    14u,
    15u,
};

void ui_init(void);

// Returns true when the selected button was CLICKED, that is its state changed
// from unpressed to pressed. Returns true once for every click. If multiple
// clicks happened between the calls, they won't be recompensated.
bool ui_get_click(enum ui_btn btn);

TickType_t ui_last_poll(void);

// Consumes the clicks, that is, ui_get_click won't receive them.
void ui_set_callback(enum ui_btn btn, void (*func)(void));

// Waits for a click of a button and returns when any button is clicked. This
// function must not be called from several tasks concurrently, otherwise, the
// second task will hang here. Consumes a click, that is, ui_get_click won't
// receive it.
enum ui_btn ui_wait_click(void);

#endif // UI_H
