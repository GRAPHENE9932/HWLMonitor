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

void gui_bar_draw(struct gui_bar* bar) {
    gui_bar_full_redraw(bar);
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
        .w = map_fill(
            bar->_fill, bar->_rect.w - (FRAME_THICKNESS + GAP_THICKNESS) * 2u
        ),
        .h = bar->_rect.h - (FRAME_THICKNESS + GAP_THICKNESS) * 2u
    };
    st7735_output_rect(bar_rect, bar->_bar);
}
