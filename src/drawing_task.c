#include "drawing_task.h"
#include "primitive_drawing.h"
#include "sh1106.h"

void drawing_task(void*) {
    while (true) {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        draw_text("Lorem Ipsum do", 0, 0);
        draw_text("lor sit amet, ", 0, 16);
        draw_text("consectetur ad", 0, 32);
        draw_text("ipiscing elit.", 0, 48);

        sh1106_begin_sending();
    }
}
