#include "primitive_drawing.h"
#include "sh1106.h"
#include "font.h"
#include "utils.h"

#include <FreeRTOS.h>

static void draw_glyph(char glyph, uint32_t x, uint32_t y, bool color) {
    uint32_t width_to_draw = MIN(
        (int32_t)FONT_WIDTH,
        MAX((int32_t)SH1106_WIDTH - x, 0)
    );

    for (uint32_t rel_page = 0; rel_page < FONT_HEIGHT / 8; ++rel_page) {
        uint32_t page = rel_page + y / 8;

        for (uint32_t rel_x = 0; rel_x < width_to_draw; ++rel_x) {
            uint8_t font_byte = FONT_GLYPHS[glyph - FONT_FIRST_ASCII]
                [rel_page * FONT_WIDTH + rel_x];
            
            if (color) {
                sh1106_frame_buffer[page * SH1106_WIDTH + x + rel_x] |= font_byte;
            }
            else {
                sh1106_frame_buffer[page * SH1106_WIDTH + x + rel_x] &= ~font_byte;
            }
        }
    }
}

void draw_text(const char* text, uint32_t x, uint32_t y, bool color) {
    // It is much simpler and more performant to draw text if it is aligned by
    // pages (8 pixel vertical). For now, unaligned text is not necessarily
    // needed. So, allow only aligned text.
    configASSERT(y % 8 == 0);

    char cur_char = 0;
    while ((cur_char = *(text++))) {
        draw_glyph(cur_char, x, y, color);
        x += FONT_WIDTH;
    };
}

static void draw_vertical_line(uint32_t x, uint32_t top, uint32_t bottom) {
    for (uint32_t page = top / 8; page <= bottom / 8; ++page) {
        uint8_t cur_byte = 0xFF;
        if (page == top / 8) {
            cur_byte &= ((uint8_t)0xFF) << (top % 8);
        }
        if (page == bottom / 8) {
            cur_byte &= ((uint8_t)0xFF) >> (7 - bottom % 8);
        }

        sh1106_frame_buffer[SH1106_WIDTH * page + x] |= cur_byte;
    }
}

static void draw_horizontal_line(uint32_t left, uint32_t right, uint32_t y) {
    uint32_t page = y / 8;
    uint8_t bit = 1 << (y % 8);

    for (uint32_t x = left; x <= right; ++x) {
        sh1106_frame_buffer[SH1106_WIDTH * page + x] |= bit;
    }
}

static void draw_filled_rectangle(
    uint32_t top, uint32_t left,
    uint32_t bottom, uint32_t right
) {
    for (uint32_t page = top / 8; page <= bottom / 8; ++page) {
        uint8_t cur_byte = 0xFF;
        if (page == top / 8) {
            cur_byte &= ((uint8_t)0xFF) << (top % 8);
        }
        if (page == bottom / 8) {
            cur_byte &= ((uint8_t)0xFF) >> (7 - bottom % 8);
        }

        for (uint32_t x = left; x <= right; ++x) {
            sh1106_frame_buffer[SH1106_WIDTH * page + x] |= cur_byte;
        }
    }
}

static const uint8_t SMALL_POINTER_UP_IMG[] = {
    5, // Width in pixels
    1, // Height in pages
    0b00000100,
    0b00000110,
    0b00000111,
    0b00000110,
    0b00000100,
};

static const uint8_t SMALL_POINTER_DOWN_IMG[] = {
    5, // Width in pixels
    1, // Height in pages
    0b00000001,
    0b00000011,
    0b00000111,
    0b00000011,
    0b00000001,
};

void draw_image(const uint8_t* image, uint32_t x, uint32_t y) {
    const uint32_t image_width = image[0];
    const uint32_t image_pages = image[1];

    for (uint32_t rel_page = 0; rel_page < image_pages; ++rel_page) {
        uint32_t higher_page = rel_page + y / 8;
        uint32_t lower_page = rel_page + (y + 7) / 8;

        for (uint32_t rel_x = 0; rel_x < image_width; ++rel_x) {
            sh1106_frame_buffer[SH1106_WIDTH * higher_page + x + rel_x] |=
                image[2 + image_width * rel_page + rel_x] << (y % 8);
            sh1106_frame_buffer[SH1106_WIDTH * lower_page + x + rel_x] |=
                image[2 + image_width * rel_page + rel_x] >> (8 - y % 8);
        }
    }
}

void draw_vertical_scrollbar(uint32_t total_entries, uint32_t position) {
    // Draw an empty 9xHEIGHT rectangle at the right edge.
    draw_vertical_line(SH1106_WIDTH - 9, 0, SH1106_HEIGHT - 1);
    draw_vertical_line(SH1106_WIDTH - 1, 0, SH1106_HEIGHT - 1);
    draw_horizontal_line(SH1106_WIDTH - 9, SH1106_WIDTH - 1, 0);
    draw_horizontal_line(SH1106_WIDTH - 9, SH1106_WIDTH - 1, SH1106_HEIGHT - 1);
    // Draw small arrows at the top and the bottom of the rectangle.
    draw_image(SMALL_POINTER_UP_IMG, SH1106_WIDTH - 7, 2);
    draw_image(SMALL_POINTER_DOWN_IMG, SH1106_WIDTH - 7, SH1106_HEIGHT - 5);

    // 1 pixel for the rectangle, 1 pixel for the gap between the rectangle and
    // a pointer, 3 pixels for the pointer, 1 pixel for the gap between pointer
    // and slider. Multiplying by two to account for both top and bottom and
    // we get 12.
    const uint32_t TOTAL_SLIDER_AREA_HEIGHT = SH1106_HEIGHT - 12;

    uint32_t slider_height = DIV_UP(TOTAL_SLIDER_AREA_HEIGHT, total_entries);
    uint32_t slider_y = MIN(
        TOTAL_SLIDER_AREA_HEIGHT * position / total_entries + 6,
        TOTAL_SLIDER_AREA_HEIGHT - 1 - slider_height + 6
    );

    draw_filled_rectangle(
        slider_y, SH1106_WIDTH - 7,
        slider_y + slider_height, SH1106_WIDTH - 3
    );
}
