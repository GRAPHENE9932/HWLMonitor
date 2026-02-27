#include "debug_log.h"

const char* debug_log_buf[DEBUG_LOG_DEPTH] = { 0 };
uint32_t debug_log_idx = 0;
