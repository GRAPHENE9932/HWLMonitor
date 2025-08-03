#include "sh1106.h"
#include "i2c.h"

#include <stddef.h>
#include <string.h>

#define SH1106_ADDRESS 0x3C
#define SH1106_COMMAND 0x80
#define SH1106_LAST_COMMAND 0x00
#define SH1106_DATA 0xC0
#define SH1106_LAST_DATA 0x40
#define SH1106_SET_CURSOR_TX_SIZE 6
#define SH1106_TX_BUFFER_SIZE (SH1106_SET_CURSOR_TX_SIZE + SH1106_WIDTH * 2)

static const uint8_t INIT_COMMANDS[] = {
    SH1106_COMMAND, 0xAE, // Display OFF
    SH1106_COMMAND, 0x40, // Set display start line to 0
    SH1106_COMMAND, 0xC8, // Set common output scan direction to reverse (vertical flip)
    SH1106_COMMAND, 0xA1, // Set segment re-map to reverse direction (horizontal flip)
    SH1106_COMMAND, 0x81, // Set contrast control register to 0xFF (double byte command)
    SH1106_COMMAND, 0xFF,
    SH1106_LAST_COMMAND, 0xAF, // Display ON
};

uint8_t sh1106_frame_buffer[SH1106_FRAME_BUFFER_SIZE];
static uint8_t tx_buffer[SH1106_TX_BUFFER_SIZE];
static int8_t pages_sent = -1; // -1 means that not a single page was ever sent.
static TaskHandle_t drawing_task_handle = NULL;

static void page_transfer_over(void);
static void error_handler(void);

void sh1106_initialize(TaskHandle_t drawing_task_handle_arg) {
    drawing_task_handle = drawing_task_handle_arg;
    pages_sent = -1;
    // Let the drawing task fill the framebuffer right away.
    xTaskNotifyGive(drawing_task_handle);

    i2c_transfer_over_handler = page_transfer_over;
    i2c_error_handler = error_handler;
    i2c_dma_initialize();
    i2c_dma_send(SH1106_ADDRESS, INIT_COMMANDS, sizeof(INIT_COMMANDS));
}

static void sh1106_set_cursor_position(uint8_t* buffer, uint8_t x, uint8_t page, bool last_command) {
    x += 2;

    buffer[0] = SH1106_COMMAND;
    buffer[1] = 0xB0 + (page & 0x0F);
    buffer[2] = SH1106_COMMAND;
    buffer[3] = 0x00 + (x & 0x0F);
    buffer[4] = last_command ? SH1106_LAST_COMMAND : SH1106_COMMAND;
    buffer[5] = 0x10 + ((x >> 4) & 0x0F);
}

static void sh1106_send_page(uint8_t page) {
    i2c_dma_wait_for_transfer_to_complete();

    sh1106_set_cursor_position(tx_buffer, 0, page, false);

    for (uint8_t i = 0; i < SH1106_WIDTH; i++) {
        ptrdiff_t tx_buf_off = SH1106_SET_CURSOR_TX_SIZE + i * 2;
        ptrdiff_t frame_buf_off = SH1106_WIDTH * page + i;
        bool last = i == SH1106_WIDTH - 1;

        tx_buffer[tx_buf_off] = last ? SH1106_LAST_DATA : SH1106_DATA;
        tx_buffer[tx_buf_off + 1] = sh1106_frame_buffer[frame_buf_off];
    }

    i2c_dma_send(SH1106_ADDRESS, tx_buffer, sizeof(tx_buffer));
}

void sh1106_begin_sending(void) {
    sh1106_send_page(0);
}

static void page_transfer_over(void) {
    pages_sent++;

    if (pages_sent == 0) {
        return;
    }
    else if (pages_sent < 7) {
        sh1106_send_page(pages_sent);
    }
    else if (pages_sent == 7) {
        sh1106_send_page(pages_sent);
        // The last page is being sent from this moment with DMA. Now we can
        // let the drawing task modify the frame buffer while the last page
        // buffer is being used for the transfer.
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(drawing_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else if (pages_sent == 8) {
        pages_sent = 0; // Wrap around.
    }
}

static void error_handler(void) {
    sh1106_initialize(drawing_task_handle);
}
