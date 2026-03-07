#include "st7735.h"
#include "gpio.h"
#include "spi.h"
#include "font.h"
#include <FreeRTOS.h>
#include <task.h>

#define DRAW_BUFFER_SIZE 2048

static int32_t report_error = 0;

// Output to the display is double buffered (for parallelization, not tearing
// effect evasion) and due to the lack of SRAM it is done using "windows" -
// rectangles with x, y, width and height set in buf_win. Data for these windows
// is stored in these buffers.
static color_t buffer_1[DRAW_BUFFER_SIZE / sizeof(color_t)];
static color_t buffer_2[DRAW_BUFFER_SIZE / sizeof(color_t)];
static color_t* cur_buf = buffer_1;
static struct st7735_rect buf_win;

static inline void send_cmd_with_arg(uint8_t cmd, uint8_t arg) {
    ST7735_AO_GPIO->ODR &= ~(1 << ST7735_AO_PIN);
    spi1_tx_byte_sync(cmd);
    ST7735_AO_GPIO->ODR |= 1 << ST7735_AO_PIN;
    spi1_tx_byte_sync(arg);
}

void st7735_init(void) {
    gpio_init(
        ST7735_EN_GPIO, ST7735_EN_PIN, GPIO_OUTPUT | GPIO_SPEED_HIGH, 0);
    gpio_init(
        ST7735_LED_GPIO, ST7735_LED_PIN, GPIO_OUTPUT | GPIO_SPEED_HIGH, 0);
    gpio_init(
        ST7735_RES_GPIO, ST7735_RES_PIN, GPIO_OUTPUT | GPIO_SPEED_HIGH, 0);
    gpio_init(
        ST7735_AO_GPIO, ST7735_AO_PIN, GPIO_OUTPUT | GPIO_SPEED_HIGH, 0);

    spi1_init();

    ST7735_EN_GPIO->ODR |= 1 << ST7735_EN_PIN;

    ST7735_RES_GPIO->ODR &= ~(1 << ST7735_RES_PIN);
    vTaskDelay(pdMS_TO_TICKS(2));
    ST7735_RES_GPIO->ODR |= 1 << ST7735_RES_PIN;

    SPI1_CS_GPIO->ODR |= 1 << SPI1_CS_PIN;
    vTaskDelay(pdMS_TO_TICKS(2));
    SPI1_CS_GPIO->ODR &= ~(1 << SPI1_CS_PIN);

    ST7735_AO_GPIO->ODR &= ~(1 << ST7735_AO_PIN);

    // Sleep Out.
    spi1_tx_byte_sync(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    // Display On.
    spi1_tx_byte_sync(0x29);

    // Set 16-bit pixel format.
    send_cmd_with_arg(0x3A, 0x55);

    // Exchange X and Y, invert Y by setting MADCTL.
    send_cmd_with_arg(0x36, 0xA0);

    // Set gamma to Gamma Curve 2 (G2.5).
    send_cmd_with_arg(0x26, 0x02);

    int32_t spi_err = spi1_get_error();
    if (spi_err) {
        report_error = spi_err;
        return;
    }

    st7735_clear(ST7735_COLOR(0, 0, 0));

    ST7735_LED_GPIO->ODR |= 1 << ST7735_LED_PIN;
}

static void send_set_window(const struct st7735_rect* rect) {
    spi1_wait();

    ST7735_AO_GPIO->ODR &= ~(1 << ST7735_AO_PIN);
    spi1_tx_byte_sync(0x2A); // Column Address Set.
    ST7735_AO_GPIO->ODR |= 1 << ST7735_AO_PIN;
    spi1_tx_hword_sync(rect->x);
    spi1_tx_hword_sync(rect->x + rect->w - 1);

    ST7735_AO_GPIO->ODR &= ~(1 << ST7735_AO_PIN);
    spi1_tx_byte_sync(0x2B); // Row Address Set.
    ST7735_AO_GPIO->ODR |= 1 << ST7735_AO_PIN;
    spi1_tx_hword_sync(rect->y);
    spi1_tx_hword_sync(rect->y + rect->h - 1);
}

void st7735_clear(color_t c) {
    const struct st7735_rect rect = {
        .x = 0,
        .y = 0,
        .w = ST7735_WIDTH,
        .h = ST7735_HEIGHT
    };
    send_set_window(&rect);

    // Memory Write.
    ST7735_AO_GPIO->ODR &= ~(1 << ST7735_AO_PIN);
    spi1_tx_byte_sync(0x2C);

    ST7735_AO_GPIO->ODR |= 1 << ST7735_AO_PIN;
    spi1_tx_repeating_hword_async(c, ST7735_WIDTH * ST7735_HEIGHT);

    int32_t spi_err = spi1_get_error();
    if (spi_err) {
        report_error = spi_err;
        return;
    }
}

static void send_window(const struct st7735_rect* rect, const uint16_t* data) {
    send_set_window(rect);

    ST7735_AO_GPIO->ODR &= ~(1 << ST7735_AO_PIN);
    spi1_tx_byte_sync(0x2C); // Memory Write.
    ST7735_AO_GPIO->ODR |= 1 << ST7735_AO_PIN;
    spi1_tx_async(data, rect->w * rect->h);
}

static void exchange_buffers(void) {
    send_window(&buf_win, cur_buf);

    if (cur_buf == buffer_1) {
        cur_buf = buffer_2;
    } else {
        cur_buf = buffer_1;
    }
}

static void draw_glyph(char c, color_t bg, color_t fg, uint32_t x, uint32_t y) {
    for (uint32_t rel_y = 0; rel_y < FONT_HEIGHT; ++rel_y) {
        for (uint32_t rel_x = 0; rel_x < FONT_WIDTH; ++rel_x) {
            cur_buf[(rel_y + y) * buf_win.w + x + rel_x] = ((
                FONT_GLYPHS[c - FONT_FIRST_ASCII]
                [(rel_y * FONT_PADDED_WIDTH / 8 + rel_x / 8)] >>
                (rel_x % 8)
            ) & 1) ? fg : bg;
        }
    }
}

static void draw_empty_glyph(color_t bg, uint32_t x, uint32_t y) {
    for (uint32_t rel_y = 0; rel_y < FONT_HEIGHT; ++rel_y) {
        for (uint32_t rel_x = 0; rel_x < FONT_WIDTH; ++rel_x) {
            cur_buf[(rel_y + y) * buf_win.w + x + rel_x] = bg;
        }
    }
}

void st7735_output_text(struct st7735_text* text) {
    const uint32_t glyphs_to_draw =
        text->_prev_len > text->len ? text->_prev_len : text->len;
    
    buf_win.x = text->x;
    buf_win.y = text->y;
    buf_win.w = FONT_WIDTH * glyphs_to_draw;
    buf_win.h = FONT_HEIGHT;

    uint32_t bytes_left = DRAW_BUFFER_SIZE;
    uint32_t i_in_win = 0;

    for (uint32_t i = 0; i < glyphs_to_draw; ++i, ++i_in_win) {
        if (bytes_left < FONT_WIDTH * FONT_HEIGHT * sizeof(color_t)) {
            exchange_buffers();
            bytes_left = DRAW_BUFFER_SIZE;
            i_in_win = 0;
            break;
        }

        if (i < text->len) {
            draw_glyph(
                text->text[i], text->bg, text->fg, FONT_WIDTH * i_in_win, 0);
        } else {
            draw_empty_glyph(text->bg, FONT_WIDTH * i, 0);
        }
        
        bytes_left -= FONT_WIDTH * FONT_HEIGHT * sizeof(color_t);
    }

    exchange_buffers();
    text->_prev_len = text->len;
}

void st7735_output_image(const uint16_t* image, uint32_t x, uint32_t y) {
    const struct st7735_rect rect = {
        .x = x,
        .y = y,
        .w = image[0],
        .h = image[1]
    };

    send_window(&rect, image + 2);
}

int32_t st7735_get_error(void) {
    const int32_t tmp = report_error;
    report_error = 0;
    return tmp;
}
