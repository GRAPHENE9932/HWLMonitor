#ifndef MENU_H
#define MENU_H

#include <stdint.h>

#define MENU_MODES 5u

enum menu_mode : uint8_t {
    MENU_STATISTICS = 0u,
    MENU_WAVEFORM,
    MENU_DELAY_MEAS,
    MENU_SETTINGS,
    MENU_SYSTEM_INFO,
};

enum menu_mode menu_start(enum menu_mode cur);

#endif // MENU_H
