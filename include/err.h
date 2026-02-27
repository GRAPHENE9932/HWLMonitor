#ifndef ERR_H
#define ERR_H

#include <stdint.h>

enum Err : int32_t {
    ETIMEOUT = 2,
    EDMA,
    EUNKNOWN,
    ERTOS
};

#endif // ERR_H
