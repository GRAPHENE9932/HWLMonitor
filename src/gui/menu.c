#include "gui/menu.h"
#include "gui/status_bar.h"
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
#define BG_COLOR ST7735_COLOR(6u, 12u, 6u)
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

    status_bar_set_text("Menu");

    static const struct st7735_rect BG_RECT = {
        .x = 0u, .y = STATUS_BAR_HEIGHT,
        .w = ST7735_WIDTH, .h = ST7735_WIDTH - STATUS_BAR_HEIGHT
    };

    st7735_output_rect(BG_RECT, BG_COLOR);

    struct st7735_text name_text = {
        .text = MODE_NAMES[cur],
        .len = strlen(MODE_NAMES[cur]),
        .bg = BG_COLOR,
        .fg = NAME_COLOR,
        .x = NAME_POS_X,
        .y = NAME_POS_Y,
        .prev_len = 0u
    };

    while (true) {
        st7735_output_image(MODE_IMAGES[cur], IMG_POS_X, IMG_POS_Y);
        name_text.text = MODE_NAMES[cur];
        name_text.len = strlen(MODE_NAMES[cur]);
        st7735_output_text(&name_text);

        switch(ui_wait_click()) {
        case BTN_LEFT:
            cur = cur > 0u ? cur - 1u : 0u;
            break;
        case BTN_RIGHT:
            cur = cur < MENU_MODES - 1u ? cur + 1u : MENU_MODES - 1u;
            break;
        default:
            break;
        }
    }
}
