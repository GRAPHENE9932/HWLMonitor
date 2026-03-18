/*
 * This module is responsible for battery monitoring, shutdown into STOP mode
 * and the independent watchdog.
 *
 * Takes over IWDG and cooperates with UI (user input) module.
 */

#ifndef PWR_CTRL_H
#define PWR_CTRL_H

#include "fix32.h"
#include <stdnoreturn.h>

#define PWR_CTRL_IWDG_MS 2000u

// UI (user input) must be initialized before calling. ADC must be initialized
// at most PWR_CTRL_IWDG_MS/2 after.
void pwr_ctrl_init(void); // TODO: split into start_iwdg and start_task.

// Returns the last recorded battery voltage in volts.
fix32_t pwr_ctrl_bat_voltage(void);

noreturn void pwr_ctrl_power_off(void);

#endif // PWR_CTR_H
