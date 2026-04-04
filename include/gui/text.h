#ifndef GUI_TEXT_H
#define GUI_TEXT_H

#include "gui.h"
#include "FreeRTOSConfig.h"

/**
 * @brief A structure representing a text on a screen.
 *
 * It is not designed as a structure that plays a role of just a parameter for
 * the text drawing function, but as a persistent GUI text element. Most of the
 * geometric fields can not be changed after the first draw.
 */
struct gui_text {
    const char* _text;
    uint16_t _len;
    color_t _fg;
    color_t _bg;
    uint16_t _x;
    uint16_t _y;
    uint8_t _height_cutoff;
    enum gui_alignment _alignment;
    uint8_t _scale;
    bool _drawn_once;
    uint16_t _prev_len;
};

void gui_text_init(struct gui_text* text);

static inline void gui_text_set_text(
    struct gui_text* t, const char* ptr, uint16_t len
) {
    t->_text = ptr;
    t->_len = len;
}

static inline void gui_text_set_fg(struct gui_text* t, color_t c) {
    t->_fg = c;
}

static inline void gui_text_set_bg(struct gui_text* t, color_t c) {
    configASSERT(!t->_drawn_once);
    t->_bg = c;
}

static inline void gui_text_set_pos(
    struct gui_text* t, uint16_t x, uint16_t y
) {
    configASSERT(!t->_drawn_once);
    t->_x = x;
    t->_y = y;
}

static inline void gui_text_set_height_cutoff(
    struct gui_text* t, uint8_t height_cutoff
) {
    configASSERT(!t->_drawn_once);
    t->_height_cutoff = height_cutoff;
}

static inline void gui_text_set_alignment(
    struct gui_text* t, enum gui_alignment alignment
) {
    configASSERT(!t->_drawn_once);
    t->_alignment = alignment;
}

static inline void gui_text_set_scale(struct gui_text* t, uint8_t scale) {
    configASSERT(!t->_drawn_once);
    t->_scale = scale;
}

void gui_text_draw(struct gui_text* text);

#endif // GUI_TEXT_H
