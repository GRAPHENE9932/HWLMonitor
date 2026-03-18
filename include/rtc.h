#ifndef RTC_H
#define RTC_H

#include <stdint.h>

// Initializes and starts RTC from the 40 kHz LSI clock for generation of
// periodic wake up events.
void rtc_periodic_wakeup(uint32_t period_s);
void rtc_clear_event(void);
void rtc_domain_reset(void);

#endif // RTC_H
