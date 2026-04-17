#include "gui/bar.h"
#include "gui/gui.h"

#define FRAME_THICKNESS 1u
#define GAP_THICKNESS 1u

void gui_bar_init(struct gui_bar* bar) {
    bar->_rect.x = 0u;
    bar->_rect.y = 0u;
    bar->_rect.w = 0u;
    bar->_rect.h = 0u;
    bar->_frame = 0u;
    bar->_bar = 0u;
    bar->_bg = GUI_BG_COLOR;
    bar->_fill = 0u;
    bar->_prev_fill = 0u;
    bar->_drawn_once = false;
}

static uint16_t map_fill(uint8_t fill, uint16_t full_range) {
    return ((uint32_t)full_range * fill) >> 8u;
}

static uint16_t bar_width(struct st7735_rect rect, uint8_t fill) {
    return map_fill(fill, rect.w - (FRAME_THICKNESS + GAP_THICKNESS) * 2u);
}

void gui_bar_draw(struct gui_bar* bar) {
    if (!bar->_drawn_once) {
        gui_bar_full_redraw(bar);
        return;
    }
    
    const uint16_t new_w = bar_width(bar->_rect, bar->_fill);
    const uint16_t old_w = bar_width(bar->_rect, bar->_prev_fill);
    if (new_w > old_w) {
        const struct st7735_rect bar_continuation = {
            .x = bar->_rect.x + FRAME_THICKNESS + GAP_THICKNESS + old_w,
            .y = bar->_rect.y + FRAME_THICKNESS + GAP_THICKNESS,
            .w = new_w - old_w,
            .h = bar->_rect.h - (FRAME_THICKNESS + GAP_THICKNESS) * 2u
        };
        st7735_output_rect(bar_continuation, bar->_bar);
    } else if (new_w < old_w) {
        const struct st7735_rect bar_shrinkage = {
            .x = bar->_rect.x + FRAME_THICKNESS + GAP_THICKNESS + new_w,
            .y = bar->_rect.y + FRAME_THICKNESS + GAP_THICKNESS,
            .w = old_w - new_w,
            .h = bar->_rect.h - (FRAME_THICKNESS + GAP_THICKNESS) * 2u
        };
        st7735_output_rect(bar_shrinkage, bar->_bg);
    }

    bar->_prev_fill = bar->_fill;
}

void gui_bar_full_redraw(struct gui_bar* bar) {
    st7735_output_rect(bar->_rect, bar->_frame);
    
    const struct st7735_rect bg_rect = { 
        .x = bar->_rect.x + FRAME_THICKNESS,
        .y = bar->_rect.y + FRAME_THICKNESS,
        .w = bar->_rect.w - FRAME_THICKNESS * 2u,
        .h = bar->_rect.h - FRAME_THICKNESS * 2u
    };
    st7735_output_rect(bg_rect, bar->_bg);

    const struct st7735_rect bar_rect = {
        .x = bar->_rect.x + FRAME_THICKNESS + GAP_THICKNESS,
        .y = bar->_rect.y + FRAME_THICKNESS + GAP_THICKNESS,
        .w = bar_width(bar->_rect, bar->_fill),
        .h = bar->_rect.h - (FRAME_THICKNESS + GAP_THICKNESS) * 2u
    };
    st7735_output_rect(bar_rect, bar->_bar);

    bar->_drawn_once = true;
    bar->_prev_fill = bar->_fill;
}
