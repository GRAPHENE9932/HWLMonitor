#ifndef USER_INPUT_H
#define USER_INPUT_H

#include <stdint.h>

#define USER_INPUT_BUTTONS_AMOUNT 5
#define USER_INPUT_UP_INDEX 0
#define USER_INPUT_DOWN_INDEX 1
#define USER_INPUT_EXIT_INDEX 2
#define USER_INPUT_CONF_INDEX 3
#define USER_INPUT_OK_INDEX 4

enum user_input_action : uint8_t {
    USER_INPUT_NONE = 0,
    USER_INPUT_UP   = 1 << USER_INPUT_UP_INDEX,
    USER_INPUT_DOWN = 1 << USER_INPUT_DOWN_INDEX,
    USER_INPUT_EXIT = 1 << USER_INPUT_EXIT_INDEX,
    USER_INPUT_CONF = 1 << USER_INPUT_CONF_INDEX,
    USER_INPUT_OK   = 1 << USER_INPUT_OK_INDEX,
};

void user_input_task(void*);
enum user_input_action user_input_take(void);

#endif // USER_INPUT_H
