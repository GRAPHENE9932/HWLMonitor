#include "gui/gui.h"
#include "font.h"
#include "utils.h"

void gui_init(void) {
    st7735_init();
    st7735_clear(GUI_BG_COLOR);
}

void gui_text_init(struct gui_text* text, uint8_t scale) {
    text->text = NULL;
    text->len = 0u;
    text->fg = 0u;
    text->bg = GUI_BG_COLOR;
    text->x = 0u;
    text->y = 0u;
    text->height_cutoff = FONT_HEIGHT;
    text->_prev_len = 0u;
    text->_scale = scale;
}

static uint32_t gui_x_to_st7735_x(uint32_t gui_x) {
    return gui_x + 1u;
}

static uint32_t gui_y_to_st7735_y(uint32_t gui_y) {
    return gui_y;
}

void gui_draw_text(struct gui_text* text) {
    const struct st7735_text dp_text = {
        .text = text->text,
        .len = text->len,
        .fg = text->fg,
        .bg = text->bg,
        .x = gui_x_to_st7735_x(text->x),
        .y = gui_y_to_st7735_y(text->y),
        .height_cutoff = text->height_cutoff,
        .scale = text->_scale
    };

    st7735_output_text(&dp_text);

    // Clear the space that was previously taken by this text.
    if (text->len < text->_prev_len) {
        const uint32_t char_h =
            min_u32(FONT_HEIGHT, text->height_cutoff) * text->_scale;
        const uint32_t char_w = FONT_WIDTH * text->_scale;

        const struct st7735_rect rect = {
            .x = dp_text.x + dp_text.len * char_w,
            .y = dp_text.y,
            .w = (text->_prev_len - text->len) * char_w,
            .h = char_h
        };

        st7735_output_rect(rect, GUI_BG_COLOR);
    }

    text->_prev_len = text->len;
}
