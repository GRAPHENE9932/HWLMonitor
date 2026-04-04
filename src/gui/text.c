#include "gui/text.h"
#include "font.h"
#include "utils.h"

void gui_text_init(struct gui_text* text) {
    text->_text = NULL;
    text->_len = 0u;
    text->_fg = 0u;
    text->_bg = GUI_BG_COLOR;
    text->_x = 0u;
    text->_y = 0u;
    text->_height_cutoff = FONT_HEIGHT;
    text->_alignment = GUI_ALIGN_LEFT;
    text->_scale = 1u;
    text->_drawn_once = false;
    text->_prev_len = 0u;
}

static uint16_t gui_x_to_st7735_x(uint16_t gui_x) {
    return gui_x + 1u;
}

static uint16_t gui_y_to_st7735_y(uint16_t gui_y) {
    return gui_y;
}

void gui_text_draw(struct gui_text* t) {
    if (t->_text == NULL) {
        return;
    }

    const uint32_t char_h = min_u32(FONT_HEIGHT, t->_height_cutoff) * t->_scale;
    const uint32_t char_w = FONT_WIDTH * t->_scale;

    uint16_t x_after_align = 0u;
    uint16_t y_after_align = 0u;
    switch (t->_alignment) {
    case GUI_ALIGN_LEFT:
        x_after_align = t->_x;
        y_after_align = t->_y;
        break;
    case GUI_ALIGN_RIGHT:
        x_after_align = t->_x - t->_len * char_w;
        y_after_align = t->_y;
        break;
    }

    const struct st7735_text dp_text = {
        .text = t->_text,
        .len = t->_len,
        .fg = t->_fg,
        .bg = t->_bg,
        .x = gui_x_to_st7735_x(x_after_align),
        .y = gui_y_to_st7735_y(y_after_align),
        .height_cutoff = t->_height_cutoff,
        .scale = t->_scale
    };

    st7735_output_text(&dp_text);

    // Clear the space that was previously taken by this text.
    if (t->_len < t->_prev_len) {
        struct st7735_rect rect = {
            .w = (t->_prev_len - t->_len) * char_w,
            .h = char_h
        };
        switch (t->_alignment) {
        case GUI_ALIGN_LEFT:
            rect.x = dp_text.x + dp_text.len * char_w;
            rect.y = dp_text.y;
            break;
        case GUI_ALIGN_RIGHT:
            rect.x = dp_text.x - (t->_prev_len - t->_len) * char_w;
            rect.y = dp_text.y;
            break;
        }

        st7735_output_rect(rect, GUI_BG_COLOR);
    }

    t->_prev_len = t->_len;
    t->_drawn_once = true;
}
