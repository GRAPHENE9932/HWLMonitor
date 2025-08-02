#ifndef SH1106_H
#define SH1106_H

#include <FreeRTOS.h>
#include <task.h>

#include <stdint.h>

#define SH1106_WIDTH 128
#define SH1106_HEIGHT 64
#define SH1106_FRAME_BUFFER_SIZE (SH1106_WIDTH * SH1106_HEIGHT / 8)

/*
 * Storing a buffer for a whole 128x64 frame is expensive (2048 bytes with
 * double buffering). Instead, only one page will be double buffered. So,
 * 1024 bytes of framebuffer to draw on and >128 bytes of buffer for DMA
 * transfer. The drawback is that the frame must be drawn in less than 2,88 ms
 * to prevent framerate decrease.
 *
 * The display consists of 8 pages, each of size 128x8 pixels.
*/

// Each byte represents a vertical 8-pixel long line, from left to right.
extern uint8_t sh1106_frame_buffer[SH1106_FRAME_BUFFER_SIZE];

// The vTaskNotifyGiveFromISR will be used on the drawing_task_handle after the
// frame transfer has finished.
void sh1106_initialize(TaskHandle_t drawing_task_handle);
void sh1106_begin_sending(void);

#endif // SH1106_H
