#include "gui/mode_statistics.h"
#include "gui/gui.h"
#include "font.h"
#include "utils.h"
#include "st7735.h"
#include "opt4001.h"

#define TEXT_COLOR ST7735_COLOR(29u, 59u, 29u)
#define MILLI_THRESHOLD 1000u

struct lx_val {
    struct gui_text text;
    char buf[6u];
};

struct lx_unit {
    struct gui_text text;
    char buf[3u];
};

static void lx_val_init(struct lx_val* lv) {
    gui_text_init(&lv->text, 2u);
    for (uint32_t i = 0u; i < sizeof(lv->buf); ++i) {
        lv->buf[i] = '-';
    }
    lv->text.text = lv->buf;
    lv->text.len = 0u;
    lv->text.fg = TEXT_COLOR;
    lv->text.x = 2u;
    lv->text.y = STATUS_BAR_HEIGHT;
    lv->text.height_cutoff = FONT_HEIGHT;
}

static void lx_val_update(struct lx_val* lv, uint32_t mlx) {
    uint8_t decimals = 0u;
    uint32_t int_val = 0u;
    uint32_t dec_val = 0u;

    if (mlx < MILLI_THRESHOLD) {
        decimals = 0u;
        int_val = mlx;
        dec_val = 0u;
    } else {
        const uint8_t digits_lx = max_i32(1, (int32_t)u32_log10(mlx) - 2);
        decimals = clamp_i32(5 - digits_lx, 0, 3);
        int_val = mlx / 1000u;
        dec_val = (mlx - int_val * 1000u) / u32_pow10(max_i32(0, 3 - decimals));
    }

    uint32_t off = i32_to_str(lv->buf, sizeof(lv->buf), int_val);
    if (decimals != 0u && off < sizeof(lv->buf)) {
        lv->buf[off++] = ',';
        off += u32_to_str_padded(
            &lv->buf[off], min_i32(decimals, sizeof(lv->buf) - off), dec_val);
    }
    lv->text.len = off;
}

static void lx_val_output(struct lx_val* lv) {
    gui_draw_text(&lv->text);
}

static void lx_unit_init(struct lx_unit* lu) {
    gui_text_init(&lu->text, 2u);
    lu->buf[0] = 'l';
    lu->buf[1] = 'x';

    lu->text.text = lu->buf;
    lu->text.len = 2u;
    lu->text.fg = TEXT_COLOR;
    lu->text.x = GUI_SCR_WIDTH - FONT_WIDTH * 3u * 2u;
    lu->text.y = STATUS_BAR_HEIGHT;
    lu->text.height_cutoff = FONT_HEIGHT;
}

static void lx_unit_update(struct lx_unit* lu, uint32_t mlx) {
    if (mlx < MILLI_THRESHOLD) {
        lu->buf[0] = 'm';
        lu->buf[1] = 'l';
        lu->buf[2] = 'x';
        lu->text.len = 3u;
    } else {
        lu->buf[0] = 'l';
        lu->buf[1] = 'x';
        lu->text.len = 2u;
    }
}

static void lx_unit_output(struct lx_unit* lu) {
    gui_draw_text(&lu->text);
}

void mode_statistics_start(void) {
    struct lx_val lx_val = { 0 };
    struct lx_unit lx_unit = { 0 };

    opt4001_init();
    opt4001_start_cont();

    gui_clear_bg();
    status_bar_set_text("Statistics");
    
    lx_val_init(&lx_val);
    lx_unit_init(&lx_unit);

    while (true) {
        const uint32_t mlx = opt4001_read_mlx();
        lx_val_update(&lx_val, mlx);
        lx_val_output(&lx_val);
        lx_unit_update(&lx_unit, mlx);
        lx_unit_output(&lx_unit);
    }
}
