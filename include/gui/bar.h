#ifndef GUI_BAR_H
#define GUI_BAR_H

#include <stdint.h>

#include "st7735.h"
#include "FreeRTOSConfig.h"

/**
 * @brief A structure representing a gauge bar (much like a progress bar) on a
 * screen.
 *
 * This is a structure for representing a persistent object on a screen, not
 * just a parameter to the draw function.
 */
struct gui_bar {
    struct st7735_rect _rect;
    color_t _frame;
    color_t _bar;
    color_t _bg;
    uint8_t _fill;
    uint8_t _prev_fill;
    bool _drawn_once;
};

void gui_bar_init(struct gui_bar* bar);

static inline void gui_bar_set_pos(
    struct gui_bar* bar, uint16_t x, uint16_t y
) {
    configASSERT(!bar->_drawn_once);
    bar->_rect.x = x;
    bar->_rect.y = y;
}

static inline void gui_bar_set_size(
    struct gui_bar* bar, uint16_t w, uint16_t h
) {
    configASSERT(!bar->_drawn_once);
    bar->_rect.w = w;
    bar->_rect.h = h;
}

static inline void gui_bar_set_frame_color(struct gui_bar* bar, color_t c) {
    configASSERT(!bar->_drawn_once);
    bar->_frame = c;
}

static inline void gui_bar_set_bar_color(struct gui_bar* bar, color_t c) {
    configASSERT(!bar->_drawn_once);
    bar->_bar = c;
}

static inline void gui_bar_set_bg(struct gui_bar* bar, color_t c) {
    configASSERT(!bar->_drawn_once);
    bar->_bg = c;
}

/**
 * @brief Sets the fill of the status bar in the range 0 to 255, where 0 is
 * completely empty and 255 is completely full.
 */
static inline void gui_bar_set_fill(struct gui_bar* bar, uint8_t fill) {
    bar->_fill = fill;
}

void gui_bar_draw(struct gui_bar* bar);
void gui_bar_full_redraw(struct gui_bar* bar);

#endif // GUI_BAR_H
