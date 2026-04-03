#ifndef ST7735_H
#define ST7735_H

/**
 * Takes over the peripherals spi.h uses and GPIO pins defined below.
 * Uses 16-bit/pixel color depth and rotates the screen 90 deg cc.
 *
 * Has a queue for SPI transfers which is processed with a statically-allocated
 * separate sender task. Supports concurrent calls.
 */

#include <stdint.h>
#include <stddef.h>

#define ST7735_EN_GPIO GPIOB
#define ST7735_EN_PIN 1
#define ST7735_LED_GPIO GPIOA
#define ST7735_LED_PIN 3
#define ST7735_RES_GPIO GPIOA
#define ST7735_RES_PIN 4
#define ST7735_AO_GPIO GPIOA
#define ST7735_AO_PIN 6

#define ST7735_WIDTH 162
#define ST7735_HEIGHT 132

typedef uint16_t color_t;

// R and B must be in range from 0 to 31, G from 0 to 63.
#define ST7735_COLOR(r, g, b)\
    (((color_t)r << 11) | ((color_t)g << 5) | (color_t)b)

struct st7735_text {
    const char* text;
    uint16_t len;
    color_t fg;
    color_t bg;
    uint16_t x;
    uint16_t y;
    uint8_t height_cutoff;
    uint8_t scale;
};

struct st7735_rect {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
};

void st7735_init(void);
void st7735_clear(color_t c);
void st7735_output_text(const struct st7735_text* t);

/**
 * @brief Queues the image to render on the screen.
 *
 * Expects data in ST7735_COLOR format preceded with two half words indicating
 * image width and height.
 */
void st7735_output_image(const color_t* image, uint32_t x, uint32_t y);

void st7735_output_rect(struct st7735_rect rect, color_t color);

int32_t st7735_get_error(void);

#endif // ST7735_H
