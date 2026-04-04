#include "gui/status_bar.h"
#include "gui/gui.h"
#include "gui/text.h"
#include "utils.h"
#include "st7735.h"
#include "images.h"
#include "fix32.h"
#include "pwr_ctrl.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <string.h>

#define TASK_STACK_DEPTH 128

#define BG_COLOR ST7735_COLOR(0, 0, 0)
#define TEXT_POS_X 0u
#define TEXT_POS_Y 2
#define TEXT_COLOR ST7735_COLOR(25, 50, 25)
#define BAT_POS_X (GUI_SCR_WIDTH - 16u)
#define BAT_POS_Y 0

#define BAT_THR_0 FIX32_CONST(3, 2, 0)
#define BAT_THR_1 FIX32_CONST(3, 6, 0)
#define BAT_THR_2 FIX32_CONST(3, 8, 0)

static StackType_t task_stack[TASK_STACK_DEPTH] = { 0 };
static StaticTask_t task_mem;
static TaskHandle_t task_handle = NULL;
static TickType_t task_last_wake_time = 0;

static StaticQueue_t mutex_mem;
static QueueHandle_t mutex;

static struct gui_text text;

static void redraw_text(void) {
    [[maybe_unused]] int32_t err = xSemaphoreTake(mutex, portMAX_DELAY);
    configASSERT(err == pdPASS);

    gui_text_draw(&text);

    err = xSemaphoreGive(mutex);
    configASSERT(err == pdPASS);
}

static void redraw_connection(void) {

}

static const color_t* bat_img_for(fix32_t vbat) {
    if (vbat < BAT_THR_0) {
        return BATTERY_0_IMG;
    } else if (vbat < BAT_THR_1) {
        return BATTERY_33_IMG;
    } else if (vbat < BAT_THR_2) {
        return BATTERY_67_IMG;
    } else {
        return BATTERY_100_IMG;
    }
}

static void redraw_battery(void) {
    const fix32_t vbat = pwr_ctrl_bat_voltage();
    st7735_output_image(bat_img_for(vbat), BAT_POS_X, BAT_POS_Y);
}

static void update_battery(void) {
    static const color_t* bat_img = BATTERY_100_IMG;

    const fix32_t vbat = pwr_ctrl_bat_voltage();
    const color_t* const new_bat_img = bat_img_for(vbat);
    if (new_bat_img != bat_img) {
        bat_img = new_bat_img;
        st7735_output_image(bat_img, BAT_POS_X, BAT_POS_Y);
    }
}

void status_bar_full_redraw(void) {
    const struct st7735_rect rect = {
        .x = 0, .y = 0, .w = GUI_SCR_WIDTH, .h = STATUS_BAR_HEIGHT
    };

    st7735_output_rect(rect, BG_COLOR);

    redraw_text();
    redraw_battery();
    redraw_connection();
}

static void updater_task(void*) {
    status_bar_full_redraw();
    
    while (true) {
        update_battery();

        xTaskDelayUntil(&task_last_wake_time,
            MS_TO_TICKS(STATUS_BAR_UPDATE_PERIOD_MS));
    }
}

void status_bar_init(void) {
    task_last_wake_time = xTaskGetTickCount();

    gui_text_init(&text);
    gui_text_set_fg(&text, TEXT_COLOR);
    gui_text_set_bg(&text, BG_COLOR);
    gui_text_set_pos(&text, TEXT_POS_X, TEXT_POS_Y);
    gui_text_set_height_cutoff(&text, STATUS_BAR_HEIGHT - TEXT_POS_Y);

    mutex = xSemaphoreCreateMutexStatic(&mutex_mem);
    configASSERT(mutex != NULL);

    task_handle = xTaskCreateStatic(updater_task, "bar", TASK_STACK_DEPTH, NULL,
        1u, task_stack, &task_mem);
    configASSERT(task_handle != NULL);
}

void status_bar_set_text(const char* new_text) {
    [[maybe_unused]] int32_t err = xSemaphoreTake(mutex, portMAX_DELAY);
    configASSERT(err == pdPASS);

    gui_text_set_text(&text, new_text, strlen(new_text));

    err = xSemaphoreGive(mutex);
    configASSERT(err == pdPASS);

    redraw_text();
}
