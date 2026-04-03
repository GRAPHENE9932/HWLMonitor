#include "main_task.h"
#include "st7735.h"
#include "ui.h"
#include "pwr_ctrl.h"
#include "adc.h"
#include "gui/status_bar.h"
#include "gui/menu.h"

void main_task(void*) {
    ui_init();
    adc_init();
    pwr_ctrl_start_task();
    gui_init();
    status_bar_init();

    while (true) {
        menu_start(MENU_STATISTICS);
    }
}
