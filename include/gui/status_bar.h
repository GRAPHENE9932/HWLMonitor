#ifndef STATUS_BAR_H
#define STATUS_BAR_H

#define STATUS_BAR_UPDATE_PERIOD_MS 1000

// Initializes and starts the status bar task which updates the status bar
// periodically.
void status_bar_init(void);
void status_bar_full_redraw(void);
void status_bar_set_text(const char* new_text);

#endif // STATUS_BAR_H
