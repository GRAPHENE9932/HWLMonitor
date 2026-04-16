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
    .x = 1u, .y = STATUS_BAR_HEIGHT,
    .w = GUI_SCR_WIDTH, .h = GUI_SCR_HEIGHT - STATUS_BAR_HEIGHT
};

enum gui_alignment : uint8_t {
    GUI_ALIGN_LEFT = 0u,
    GUI_ALIGN_RIGHT = 1u,
};

void gui_init(void);

inline static void gui_clear_bg(void) {
    st7735_output_rect(GUI_AREA, GUI_BG_COLOR);
}

void gui_draw_frame(struct st7735_rect rect, color_t col);

#endif // GUI_H
