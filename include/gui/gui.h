#ifndef GUI_H
#define GUI_H

#include "status_bar.h"
#include "st7735.h"

#define GUI_BG_COLOR ST7735_COLOR(6u, 12u, 6u)
#define GUI_SCR_WIDTH (ST7735_WIDTH - 2u)
#define GUI_SCR_HEIGHT ST7735_HEIGHT

/**
 * @brief Signifies the whole area of the display which is not covered by the
 * status bar.
 */
constexpr struct st7735_rect GUI_AREA = {
    .x = 0u, .y = STATUS_BAR_HEIGHT,
    .w = GUI_SCR_WIDTH, .h = GUI_SCR_HEIGHT - STATUS_BAR_HEIGHT
};

struct gui_text {
    const char* text;
    uint16_t len;
    color_t fg;
    color_t bg;
    uint16_t x;
    uint16_t y;
    uint8_t height_cutoff;
    uint8_t _scale;
    uint16_t _prev_len;
};

void gui_init(void);

void gui_text_init(struct gui_text* text, uint8_t scale);

inline static void gui_clear_bg(void) {
    st7735_output_rect(GUI_AREA, GUI_BG_COLOR);
}

void gui_draw_text(struct gui_text* text);

#endif // GUI_H
