#ifndef MODE_MENU_H
#define MODE_MENU_H

#include <stdint.h>

#include "user_input.h"

enum mode_menu_entry : uint32_t {
    MODE_MENU_PWM_MEASUREMENT = 0,
    MODE_MENU_DELAY_MEASUREMENT = 1,
    MODE_MENU_INFO = 2,
};

void mode_menu_handle_input(enum user_input_action input);
void mode_menu_draw(void);
enum mode_menu_entry mode_menu_cur_entry(void);

#endif // MODE_MENU_H
