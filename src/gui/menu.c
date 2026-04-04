#include "gui/menu.h"
#include "gui/status_bar.h"
#include "gui/gui.h"
#include "gui/text.h"
#include "images.h"
#include "st7735.h"
#include "ui.h"
#include "FreeRTOSConfig.h"
#include <stdbool.h>
#include <string.h>

#define IMG_POS_X 33u
#define IMG_POS_Y 24u
#define NAME_POS_X IMG_POS_X
#define NAME_POS_Y (IMG_POS_Y + 64u)
#define ARROW_LEFT_POS_X 1u
#define ARROW_LEFT_POS_Y NAME_POS_Y
#define ARROW_RIGHT_POS_X (GUI_SCR_WIDTH - 7u)
#define ARROW_RIGHT_POS_Y NAME_POS_Y

#define NAME_COLOR ST7735_COLOR(29u, 59u, 29u)

static const color_t* const MODE_IMAGES[MENU_MODES] = {
    STATISTICS_IMG,
    WAVEFORM_IMG,
    DELAY_MEAS_IMG,
    SETTINGS_IMG,
    SYS_INFO_IMG,
};

static const char* const MODE_NAMES[MENU_MODES] = {
    "Statistics",
    "Waveform",
    "Delay meas.",
    "Settings",
    "System info",
};

enum menu_mode menu_start(enum menu_mode cur) {
    configASSERT(cur < MENU_MODES);

    gui_clear_bg();
    status_bar_set_text("Menu");
    st7735_output_image(ARROW_LEFT_IMG, ARROW_LEFT_POS_X, ARROW_LEFT_POS_Y);
    st7735_output_image(ARROW_RIGHT_IMG, ARROW_RIGHT_POS_X, ARROW_RIGHT_POS_Y);

    struct gui_text name_text;
    gui_text_init(&name_text);
    gui_text_set_text(&name_text, MODE_NAMES[cur], strlen(MODE_NAMES[cur]));
    gui_text_set_fg(&name_text, NAME_COLOR);
    gui_text_set_pos(&name_text, NAME_POS_X, NAME_POS_Y);

    while (true) {
        st7735_output_image(MODE_IMAGES[cur], IMG_POS_X, IMG_POS_Y);
        gui_text_set_text(&name_text, MODE_NAMES[cur], strlen(MODE_NAMES[cur]));
        gui_text_draw(&name_text);

        switch(ui_wait_click()) {
        case BTN_LEFT:
            cur = cur > 0u ? cur - 1u : 0u;
            break;
        case BTN_RIGHT:
            cur = cur < MENU_MODES - 1u ? cur + 1u : MENU_MODES - 1u;
            break;
        case BTN_OK:
            return cur;
        default:
            break;
        }
    }
}
