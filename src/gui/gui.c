#include "gui/gui.h"

void gui_init(void) {
    st7735_init();
    st7735_clear(GUI_BG_COLOR);
}
