#include "gui/mode_menu.h"
#include "primitive_drawing.h"

#define MODE_MENU_ENTRIES_COUNT 3
static const char* const MODE_MENU_ENTRIES[MODE_MENU_ENTRIES_COUNT] = {
    "PWM Analysis",
    "Delay",
    "Info",
};

static enum mode_menu_entry cur_entry = 0;

void mode_menu_take_user_input(void) {

}

void mode_menu_draw(void) {
    draw_text(
        MODE_MENU_ENTRIES[cur_entry],
        0, 48, true
    );

    draw_vertical_scrollbar(3, 2);
}

enum mode_menu_entry mode_menu_cur_entry(void) {
    return cur_entry;
}
