#ifndef PRIMITIVE_DRAWING_H
#define PRIMITIVE_DRAWING_H

#include <stdint.h>
#include <stdbool.h>

void draw_text(const char* text, uint32_t x, uint32_t y, bool color);
void draw_vertical_scrollbar(uint32_t total_entries, uint32_t position);

#endif // PRIMITIVE_DRAWING_H
