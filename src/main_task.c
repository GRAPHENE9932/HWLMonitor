#include "main_task.h"
#include "gui/gui.h"
#include "ui.h"
#include "pwr_ctrl.h"
#include "adc.h"
#include "gui/status_bar.h"
#include "gui/menu.h"
#include "gui/mode_statistics.h"

void main_task(void*) {
    ui_init();
    adc_init();
    pwr_ctrl_start_task();
    gui_init();
    status_bar_init();

    while (true) {
        const enum menu_mode mode = menu_start(MENU_STATISTICS);
        switch (mode) {
        case MENU_STATISTICS:
            mode_statistics_start();
            break;
        default:
            break;
        }
    }
}
