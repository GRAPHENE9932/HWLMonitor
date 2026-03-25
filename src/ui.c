#include "ui.h"
#include "gpio.h"
#include "utils.h"
#include <FreeRTOS.h>
#include <task.h>

#define POLL_TASK_STACK_DEPTH 64u
#define POLL_TASK_PERIOD_MS 5u
#define BTN_TRIG_PRESS 8u
#define BTN_TRIG_UNPRESS 4u
#define BTN_MAX_COUNT 12u

struct btn_state {
    uint16_t count;
    bool pressed; // Mirrors current state of the button after debouncing.
    bool clicked; // Set by polling, cleared externally.
    void (*callback)(void);
};

static StackType_t poll_task_stack[POLL_TASK_STACK_DEPTH];
static StaticTask_t poll_task_mem;
static TickType_t last_poll;

static TaskHandle_t waiting_task = NULL;
static struct btn_state states[UI_BTN_AMOUNT] = { 0 };

static void handle_click(enum ui_btn btn) {
    bool consumed = false;

    if (states[btn].callback != NULL) {
        consumed = true;
        states[btn].callback();
    }

    if (waiting_task != NULL) {
        consumed = true;
        [[maybe_unused]] uint32_t ret =
            xTaskNotify(waiting_task, btn, eSetValueWithOverwrite);
        configASSERT(ret == pdPASS);
        waiting_task = NULL;
    }

    states[btn].clicked = !consumed;
}

static void poll_btn(enum ui_btn btn) {
    if ((UI_BTN_GPIO[btn]->IDR & (1u << UI_BTN_PIN[btn])) == 0u) {
        if (states[btn].count < BTN_MAX_COUNT) {
            states[btn].count += 1u;
        }
        
        if (states[btn].count == BTN_TRIG_PRESS) {
            handle_click(btn);
            states[btn].pressed = true;
        }
    } else {
        if (states[btn].count > 0u) {
            states[btn].count -= 1u;
        }

        if (states[btn].count == BTN_TRIG_UNPRESS) {
            states[btn].pressed = false;
        }
    }
}

static void poll_task(void*) {
    last_poll = xTaskGetTickCount();

    while (true) {
        for (uint32_t i = 0u; i < UI_BTN_AMOUNT; ++i) {
            poll_btn(i);
        }

        xTaskDelayUntil(
            &last_poll, MS_TO_TICKS(POLL_TASK_PERIOD_MS));
    }
}

void ui_init(void) {
    for (uint32_t i = 0u; i < UI_BTN_AMOUNT; ++i) {
        gpio_init(UI_BTN_GPIO[i], UI_BTN_PIN[i], GPIO_INPUT | GPIO_PULL_UP, 0u);
        // We start from the MAX_COUNT, so the buttons trigger only after they
        // have been released. This is especially useful for the power button,
        // which may still be pressed when the whole system has already been
        // reset.
        states[i].count = BTN_MAX_COUNT;
    }

    [[maybe_unused]] const TaskHandle_t task = xTaskCreateStatic(
        poll_task,
        "pol",
        POLL_TASK_STACK_DEPTH,
        NULL,
        3,
        poll_task_stack,
        &poll_task_mem
    );
    configASSERT(task != NULL);
}

bool ui_get_click(enum ui_btn btn) {
    taskENTER_CRITICAL();
    const bool result = states[btn].clicked;
    states[btn].clicked = false;
    taskEXIT_CRITICAL();
    return result;
}

TickType_t ui_last_poll(void) {
    return last_poll;
}

void ui_set_callback(enum ui_btn btn, void (*func)(void)) {
    configASSERT(func != NULL);
    states[btn].callback = func;
}

enum ui_btn ui_wait_click(void) {
    // If there is a button clicked, return right away.
    for (uint32_t i = 0u; i < UI_BTN_AMOUNT; ++i) {
        taskENTER_CRITICAL();
        if (states[i].clicked) {
            states[i].clicked = false;
            taskEXIT_CRITICAL();
            return i;
        }
        taskEXIT_CRITICAL();
    }

    taskENTER_CRITICAL();
    if (waiting_task != NULL) {
        taskEXIT_CRITICAL();
        configASSERT(false);
        while (true) {}
    }
    waiting_task = xTaskGetCurrentTaskHandle();
    taskEXIT_CRITICAL();
    
    uint32_t btn = 0u;
    [[maybe_unused]] const uint32_t ret =
        xTaskNotifyWait(0u, 0xFFFFFFFFu, &btn, portMAX_DELAY);
    configASSERT(ret == pdTRUE);

    return btn;
}
