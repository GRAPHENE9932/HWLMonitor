#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

// The argument value must be at least 2.
void delay_us_at_8mhz(uint32_t us);

#endif // DELAY_H
