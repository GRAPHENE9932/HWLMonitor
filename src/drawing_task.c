#include "drawing_task.h"
#include "sh1106.h"
#include "gui/mode_menu.h"

static enum : uint8_t {
    SCR_MODE_MENU,
} screen = SCR_MODE_MENU;

static void draw_interface(void) {
    switch (screen) {
    case SCR_MODE_MENU:
        mode_menu_take_user_input();
        mode_menu_draw();
        break;
    default:
        configASSERT(false);
        break;
    }
}

void drawing_task(void*) {
    while (true) {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        draw_interface();

        sh1106_begin_sending();
    }
}
