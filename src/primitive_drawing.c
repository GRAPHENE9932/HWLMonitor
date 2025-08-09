#include "primitive_drawing.h"
#include "sh1106.h"
#include "font.h"

#include <FreeRTOS.h>

#include <string.h>

#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) > (y) ? (y) : (x))

static void draw_glyph(char glyph, uint32_t x, uint32_t y) {
    uint32_t width_to_draw = MIN(
        (int32_t)FONT_WIDTH,
        MAX((int32_t)SH1106_WIDTH - x, 0)
    );

    for (uint32_t rel_page = 0; rel_page < FONT_HEIGHT / 8; ++rel_page) {
        uint32_t page = rel_page + y / 8;
        memcpy(
            sh1106_frame_buffer + page * SH1106_WIDTH + x,
            FONT_GLYPHS[glyph - FONT_FIRST_ASCII] + rel_page * FONT_WIDTH,
            width_to_draw
        );
    }
}

void draw_text(const char* text, uint32_t x, uint32_t y) {
    // It is much simpler and more performant to draw text if it is aligned by
    // pages (8 pixel vertical). For now, unaligned text is not necessarily
    // needed. So, allow only aligned text.
    configASSERT(y % 8 == 0);

    char cur_char = 0;
    while ((cur_char = *(text++))) {
        draw_glyph(cur_char, x, y);
        x += FONT_WIDTH;
    };
}
