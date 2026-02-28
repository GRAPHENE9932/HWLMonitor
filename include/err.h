#ifndef ERR_H
#define ERR_H

#include <stdint.h>

enum Err : int32_t {
    ETIMEOUT = -2,
    EDMA = -3,
    EUNKNOWN = -4,
    ERTOS = -5,
    EBUS = -6,          // Generic communication bus error.
    EOVR = -7,          // Overrun.
};

#endif // ERR_H
