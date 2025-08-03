#include "drawing_task.h"
#include "sh1106.h"

uint16_t frame_n = 0;

void drawing_task(void*) {
    while (true) {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        for (uint16_t i = 0; i < 1024; i++) {
            uint8_t cur_bit = i % 128 / 8;
            sh1106_frame_buffer[i] = frame_n & (1 << cur_bit) ? 0xFF : 0x00;
        }

        frame_n++;

        sh1106_begin_sending();
    }
}
