#include "gui/gui.h"

void gui_init(void) {
    st7735_init();
    st7735_clear(GUI_BG_COLOR);
}

void gui_draw_frame(struct st7735_rect rect, color_t col) {
    struct st7735_rect edge;

    // Top edge.
    edge.x = rect.x;
    edge.y = rect.y;
    edge.w = rect.w;
    edge.h = 1u;
    st7735_output_rect(edge, col);

    // Bottom edge.
    edge.y = rect.y + rect.h - 1u;
    st7735_output_rect(edge, col);

    // Left edge.
    edge.x = rect.x;
    edge.y = rect.y;
    edge.w = 1u;
    edge.h = rect.h;
    st7735_output_rect(edge, col);

    // Right edge.
    edge.x = rect.x + rect.w - 1u;
    st7735_output_rect(edge, col);
}
