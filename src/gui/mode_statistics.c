#include "gui/mode_statistics.h"
#include "gui/gui.h"
#include "gui/text.h"
#include "utils.h"
#include "st7735.h"
#include "opt4001.h"

#define TEXT_COLOR ST7735_COLOR(29u, 59u, 29u)
#define MILLI_THRESHOLD 1000u
#define KILO_THRESHOLD 10'000'000u

struct lx_val_text {
    struct gui_text text;
    char buf[6u];
};

struct lx_unit_text {
    struct gui_text text;
};

enum lx_unit : uint8_t {
    UNIT_MILLILUX,
    UNIT_LUX,
    UNIT_KILOLUX,
};

static enum lx_unit choose_unit(uint32_t mlx) {
    if (mlx < MILLI_THRESHOLD) {
        return UNIT_MILLILUX;
    } else if (mlx >= KILO_THRESHOLD) {
        return UNIT_KILOLUX;
    } else {
        return UNIT_LUX;
    }
}

static void lx_val_text_init(struct lx_val_text* lv) {
    gui_text_init(&lv->text);
    for (uint32_t i = 0u; i < sizeof(lv->buf); ++i) {
        lv->buf[i] = '-';
    }
    gui_text_set_text(&lv->text, lv->buf, sizeof(lv->buf));
    gui_text_set_fg(&lv->text, TEXT_COLOR);
    gui_text_set_pos(&lv->text, 2u, STATUS_BAR_HEIGHT);
    gui_text_set_scale(&lv->text, 2u);
}

static void lx_val_text_update(
    struct lx_val_text* lv, uint32_t mlx, enum lx_unit unit
) {
    uint8_t decimals = 0u;
    uint32_t int_val = 0u;
    uint32_t dec_val = 0u;

    switch (unit) {
    case UNIT_MILLILUX:
        decimals = 0u;
        int_val = mlx;
        dec_val = 0u;
        break;
    case UNIT_LUX:
        const uint8_t digits_lx = max_i32(1, (int32_t)u32_log10(mlx) - 2);
        decimals = clamp_i32(5 - digits_lx, 0, 3);
        int_val = mlx / 1000u;
        dec_val = (mlx - int_val * 1000u) / u32_pow10(max_i32(0, 3 - decimals));
        break;
    case UNIT_KILOLUX:
        const uint8_t digits_klx = max_i32(1, (int32_t)u32_log10(mlx) - 5);
        decimals = clamp_i32(5 - digits_klx, 0, 3);
        int_val = mlx / 1'000'000u;
        dec_val =
            (mlx - int_val * 1'000'000u) / u32_pow10(max_i32(0, 6 - decimals));
        break;
    }

    uint32_t off = i32_to_str(lv->buf, sizeof(lv->buf), int_val);
    if (decimals != 0u && off < sizeof(lv->buf)) {
        lv->buf[off++] = ',';
        off += u32_to_str_padded(
            &lv->buf[off], min_i32(decimals, sizeof(lv->buf) - off), dec_val
        );
    }
    gui_text_set_text(&lv->text, lv->buf, off);
}

static void lx_val_text_output(struct lx_val_text* lv) {
    gui_text_draw(&lv->text);
}

static void lx_unit_text_init(struct lx_unit_text* lu) {
    gui_text_init(&lu->text);
    gui_text_set_text(&lu->text, "lx", 2u);
    gui_text_set_fg(&lu->text, TEXT_COLOR);
    gui_text_set_alignment(&lu->text, GUI_ALIGN_RIGHT);
    gui_text_set_pos(&lu->text, GUI_SCR_WIDTH, STATUS_BAR_HEIGHT);
    gui_text_set_scale(&lu->text, 2u);
}

static void lx_unit_text_update(struct lx_unit_text* lu, enum lx_unit unit) {
    switch (unit) {
    case UNIT_MILLILUX:
        gui_text_set_text(&lu->text, "mlx", 3u);
        break;
    case UNIT_LUX:
        gui_text_set_text(&lu->text, "lx", 2u);
        break;
    case UNIT_KILOLUX:
        gui_text_set_text(&lu->text, "klx", 3u);
        break;
    }
}

static void lx_unit_text_output(struct lx_unit_text* lu) {
    gui_text_draw(&lu->text);
}

void mode_statistics_start(void) {
    struct lx_val_text lx_val_text = { 0 };
    struct lx_unit_text lx_unit_text = { 0 };

    opt4001_init();
    opt4001_start_cont();

    gui_clear_bg();
    status_bar_set_text("Statistics");
    
    lx_val_text_init(&lx_val_text);
    lx_unit_text_init(&lx_unit_text);

    while (true) {
        const uint32_t mlx = opt4001_read_mlx();
        const enum lx_unit unit = choose_unit(mlx);
        lx_val_text_update(&lx_val_text, mlx, unit);
        lx_val_text_output(&lx_val_text);
        lx_unit_text_update(&lx_unit_text, unit);
        lx_unit_text_output(&lx_unit_text);
    }
}
