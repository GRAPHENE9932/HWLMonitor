#include "st7735.h"
#include "gpio.h"
#include "spi.h"
#include "font.h"
#include "err.h"
#include "utils.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#define RAM_BUFFER_SIZE 2048
#define RAM_BUFFERS 2
#define TX_QUEUE_SIZE 4
#define SENDER_STACK_DEPTH 128

enum segment_kind : uint8_t {
    SEGMENT_FILLED,
    SEGMENT_RAM_BUF_IMAGE,
    SEGMENT_EXTERN_IMAGE,
};

struct segment {
    struct st7735_rect rect;
    enum segment_kind kind;

    union {
        color_t* ram_data;
        const color_t* ext_data;
        color_t fill_color;
    };
};

static int32_t report_error = 0;

static StaticQueue_t tx_queue_mem;
static struct segment tx_queue_storage[TX_QUEUE_SIZE];
static QueueHandle_t tx_queue = NULL;

static StaticQueue_t ram_buf_queue_mem;
static color_t* ram_buf_queue_storage[RAM_BUFFERS];
static QueueHandle_t ram_buf_queue = NULL;

static StackType_t sender_stack[SENDER_STACK_DEPTH];
static StaticTask_t sender_task_mem;
static TaskHandle_t sender_task_handle;

static color_t ram_buffers[RAM_BUFFERS][RAM_BUFFER_SIZE / sizeof(color_t)];

static void handle_error(int32_t err) {
    report_error = err;

    // Reset everything and try to recover.
    vTaskDelete(sender_task_handle);

    struct transmission* dummy = NULL;
    while (xQueueReceive(tx_queue, &dummy, 0) == pdPASS) {}
    while (xQueueReceive(ram_buf_queue, &dummy, 0) == pdPASS) {}

    st7735_init();
}

static void send_set_window(const struct st7735_rect* rect) {
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

static void sender_task(void*) {
    int32_t err = 0;

    while (true) {
        struct segment s;
        err = xQueueReceive(tx_queue, &s, portMAX_DELAY);
        configASSERT(err == pdPASS);

        send_set_window(&s.rect);

        ST7735_AO_GPIO->ODR &= ~(1 << ST7735_AO_PIN);
        spi1_tx_byte_sync(0x2C); // Memory Write.

        ST7735_AO_GPIO->ODR |= 1 << ST7735_AO_PIN;

        switch (s.kind) {
        case SEGMENT_FILLED:
            spi1_tx_repeating_hword(s.fill_color, s.rect.w * s.rect.h);
            break;
        case SEGMENT_EXTERN_IMAGE:
            spi1_tx(s.ext_data, s.rect.w * s.rect.h);
            break;
        case SEGMENT_RAM_BUF_IMAGE:
            spi1_tx(s.ram_data, s.rect.w * s.rect.h);
            err = xQueueSendToBack(ram_buf_queue, &s.ram_data, 0);
            configASSERT(err == pdPASS);
            break;
        default:
            configASSERT(false);
        }

        err = spi1_get_error();
        if (err < 0) {
            handle_error(err);
            return;
        }
    }
}

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

    ST7735_EN_GPIO->ODR |= 1 << ST7735_EN_PIN; // Power up the MOSFET.

    ST7735_RES_GPIO->ODR &= ~(1 << ST7735_RES_PIN); // Do a reset.
    vTaskDelay(MS_TO_TICKS(2));
    ST7735_RES_GPIO->ODR |= 1 << ST7735_RES_PIN;

    SPI1_CS_GPIO->ODR |= 1 << SPI1_CS_PIN; // Part of the reset sequence.
    vTaskDelay(MS_TO_TICKS(2));
    SPI1_CS_GPIO->ODR &= ~(1 << SPI1_CS_PIN);

    // Sleep Out.
    ST7735_AO_GPIO->ODR &= ~(1 << ST7735_AO_PIN);
    spi1_tx_byte_sync(0x11);
    vTaskDelay(MS_TO_TICKS(120));

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

    tx_queue = xQueueCreateStatic(TX_QUEUE_SIZE, sizeof(struct segment),
        (uint8_t*)tx_queue_storage, &tx_queue_mem);
    ram_buf_queue = xQueueCreateStatic(RAM_BUFFERS, sizeof(color_t*),
        (uint8_t*)ram_buf_queue_storage, &ram_buf_queue_mem);

    for (uint32_t i = 0; i < RAM_BUFFERS; ++i) {
        color_t* ptr = &ram_buffers[i][0];
        if (xQueueSendToBack(ram_buf_queue, &ptr, 0) != pdPASS) {
            report_error = ERTOS;
            return;
        }
    }

    sender_task_handle = xTaskCreateStatic(sender_task, "st7", 
        SENDER_STACK_DEPTH, NULL, 2, sender_stack, &sender_task_mem);

    if (sender_task_handle == NULL) {
        report_error = ERTOS;
        return;
    }

    ST7735_LED_GPIO->ODR |= 1 << ST7735_LED_PIN;
}

void st7735_clear(color_t c) {
    const struct segment s = {
        .rect = {
            .x = 0, .y = 0,
            .w = ST7735_WIDTH, .h = ST7735_HEIGHT
        },
        .kind = SEGMENT_FILLED,
        .fill_color = c
    };

    int32_t err = xQueueSendToBack(tx_queue, &s, portMAX_DELAY);
    configASSERT(err == pdPASS);
}

static void draw_glyph(const struct segment* s, char c, color_t bg, color_t fg,
    uint32_t x, uint32_t y) {
    for (uint32_t rel_y = 0; rel_y < s->rect.h; ++rel_y) {
        for (uint32_t rel_x = 0; rel_x < FONT_WIDTH; ++rel_x) {
            s->ram_data[(rel_y + y) * s->rect.w + x + rel_x] = ((
                FONT_GLYPHS[c - FONT_FIRST_ASCII]
                [(rel_y * FONT_PADDED_WIDTH / 8 + rel_x / 8)] >>
                (rel_x % 8)
            ) & 1) ? fg : bg;
        }
    }
}

static uint32_t output_text_segment(struct st7735_text* t, uint32_t off) {
    const uint8_t char_h =
        t->height_cutoff == 0 ? FONT_HEIGHT : t->height_cutoff;
    const uint32_t char_size = FONT_WIDTH * char_h * sizeof(color_t);
    const uint32_t chars_in_seg =
        min_u32(RAM_BUFFER_SIZE / char_size, t->len - off);
    configASSERT(chars_in_seg > 0);

    color_t* buf = NULL;
    int32_t err = xQueueReceive(ram_buf_queue, &buf, portMAX_DELAY);
    configASSERT(err == pdPASS);

    const struct segment s = {
        .rect = {
            .x = t->x + FONT_WIDTH * off, .y = t->y,
            .w = chars_in_seg * FONT_WIDTH, .h = char_h
        },
        .kind = SEGMENT_RAM_BUF_IMAGE,
        .ram_data = buf
    };

    for (uint32_t i = 0; i < chars_in_seg; ++i) {
        draw_glyph(&s, t->text[off + i], t->bg, t->fg, FONT_WIDTH * i, 0);
    }

    err = xQueueSendToBack(tx_queue, &s, portMAX_DELAY);
    configASSERT(err == pdPASS);

    return chars_in_seg;
}

void st7735_output_text(struct st7735_text* t) {
    uint32_t i = 0;
    while (i < t->len) {
        i += output_text_segment(t, i);
    }
    
    // Clear the space that was previously taken by this text.
    if (i < t->prev_len) {
        const uint8_t char_h =
            t->height_cutoff == 0 ? FONT_HEIGHT : t->height_cutoff;

        const struct segment s = {
            .rect = {
                .x = t->x + i * FONT_WIDTH, .y = t->y,
                .w = (t->prev_len - i) * FONT_WIDTH, .h = char_h
            },
            .kind = SEGMENT_FILLED,
            .fill_color = t->bg
        };

        const int32_t err = xQueueSendToBack(tx_queue, &s, portMAX_DELAY);
        configASSERT(err == pdPASS);
    }

    t->prev_len = t->len;
}

void st7735_output_image(const color_t* image, uint32_t x, uint32_t y) {
    const struct segment s = {
        .rect = {
            .x = x, .y = y,
            .w = image[0], .h = image[1]
        },
        .kind = SEGMENT_EXTERN_IMAGE,
        .ext_data = image + 2
    };

    int32_t err = xQueueSendToBack(tx_queue, &s, portMAX_DELAY);
    configASSERT(err == pdPASS);
}

void st7735_output_rect(struct st7735_rect rect, color_t color) {
    const struct segment s = {
        .rect = rect,
        .kind = SEGMENT_FILLED,
        .fill_color = color
    };

    int32_t err = xQueueSendToBack(tx_queue, &s, portMAX_DELAY);
    configASSERT(err == pdPASS);
}

int32_t st7735_get_error(void) {
    const int32_t tmp = report_error;
    report_error = 0;
    return tmp;
}
