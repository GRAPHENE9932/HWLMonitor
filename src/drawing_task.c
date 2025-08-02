#include "drawing_task.h"
#include "sh1106.h"

uint8_t frame_n = 1;

void drawing_task(void*) {
    while (true) {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        for (uint16_t i = 0; i < 1024; i++) {
            sh1106_frame_buffer[i] = i * frame_n;
        }

        if (frame_n == 255) {
            frame_n = 1;
        }
        frame_n++;

        sh1106_begin_sending();
    }
}
