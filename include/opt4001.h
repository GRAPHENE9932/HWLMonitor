#ifndef OPT4001_H
#define OPT4001_H

#include <stdint.h>

#define OPT4001_I2C_ADDR 0b1000100u

/**
 * This module takes over the I2C1 module.
 */

void opt4001_init(void);

/**
 * @brief Starts the continuous conversion with 200 ms conversion time.
 * 
 * Results can be read with opt4001_read_mlx().
 *
 * @sa opt4001_read_mlx.
 */
void opt4001_start_cont(void);

uint32_t opt4001_read_mlx(void);

void opt4001_standby(void);

#endif // OPT4001_H
