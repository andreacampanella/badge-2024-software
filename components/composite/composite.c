/*
 * Tildagon composite video output - PAL bring-up step B2.
 *
 * Full 288p PAL frame: 3 vsync lines + 19 blanking + 288 "active" + 2 post-blanking.
 * For now all "active" lines are still blank - we're proving the frame structure
 * causes the TV to lock vertically.
 *
 * Adapted from Phil Burgess / Adafruit's ESP32-S3 LCD blog post:
 *   https://blog.adafruit.com/2022/06/21/esp32uesday-more-s3-lcd-peripheral-hacking-with-code/
 */

#include "composite.h"
#include "esp_log.h"
#include "esp_rom_gpio.h"
#include "esp_heap_caps.h"
#include "driver/periph_ctrl.h"
#include "esp_private/gdma.h"
#include "hal/dma_types.h"
#include "hal/gpio_hal.h"
#include "soc/lcd_cam_struct.h"
#include "soc/gpio_sig_map.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <string.h>

// 4-bit DAC: HS_I (GPIO38) = MSB weight 8 / HS_H = 4 / HS_G = 2 / HS_F = 1
#define LUMA_SYNC      0       // 0V - sync tip (only used during sync interval)
#define LUMA_BLANK     4       // ~0.3V - blank/black level
#define LUMA_BLACK     4       // alias
#define LUMA_WHITE    15       // peak white

// PAL line at 13.33 MHz pclk (1.3% slow vs spec 13.5 MHz; PAL is tolerant)
#define LINE_LEN          864      // 64.8us per line
#define SYNC_LEN           63      //  4.7us
#define BACK_PORCH_LEN     76      //  5.7us
#define ACTIVE_LEN        690      // 51.7us
#define FRONT_PORCH_LEN   (LINE_LEN - SYNC_LEN - BACK_PORCH_LEN - ACTIVE_LEN)

// Frame structure (288p progressive, simplified)
#define FRAME_LINES       312
#define VSYNC_LINES         3
#define VBLANK_TOP_LINES   19      // lines 4..22
#define ACTIVE_LINES      288      // lines 23..310
#define VBLANK_BOT_LINES    2      // lines 311..312

// Pin mux table
static const struct {
    int8_t pin;
    uint8_t signal;
} mux[] = {
    { 35, LCD_DATA_OUT0_IDX },
    { 36, LCD_DATA_OUT1_IDX },
    { 37, LCD_DATA_OUT2_IDX },
    { 38, LCD_DATA_OUT3_IDX },
    { 11, LCD_DATA_OUT4_IDX },
    { 14, LCD_DATA_OUT5_IDX },
    { 13, LCD_DATA_OUT6_IDX },
    { 12, LCD_DATA_OUT7_IDX },
};

static gdma_channel_handle_t dma_chan;
static dma_descriptor_t *desc_chain = NULL;   // FRAME_LINES descriptors
static uint8_t *line_blank = NULL;
static uint8_t *line_vsync = NULL;
static uint8_t *line_active = NULL;           // currently same as blank

// Normal blanking line: sync pulse, then blank for the rest
static void build_blank_line(uint8_t *buf) {
    int i = 0;
    for (int n = 0; n < SYNC_LEN;        n++) buf[i++] = LUMA_SYNC;
    for (int n = 0; n < BACK_PORCH_LEN;  n++) buf[i++] = LUMA_BLANK;
    for (int n = 0; n < ACTIVE_LEN;      n++) buf[i++] = LUMA_BLACK;
    for (int n = 0; n < FRONT_PORCH_LEN; n++) buf[i++] = LUMA_BLANK;
}

// VSYNC line: inverted - mostly LOW, brief HIGH at end (broad pulse)
// PAL spec: line is LOW for ~30us, HIGH for ~2us, total = half-line + half-line.
// We're going to use a simplified version: LOW for whole line except the
// front-porch-equivalent gap.
static void build_vsync_line(uint8_t *buf) {
    int i = 0;
    // Most of the line at sync level
    for (int n = 0; n < LINE_LEN - FRONT_PORCH_LEN; n++) buf[i++] = LUMA_SYNC;
    // End of line at blank level (so the receiver sees a transition)
    for (int n = 0; n < FRONT_PORCH_LEN; n++) buf[i++] = LUMA_BLANK;
}

// Active line: for now identical to blank - we'll fill with pixel data later
static void build_active_line(uint8_t *buf) {
    int i = 0;
    for (int n = 0; n < SYNC_LEN;       n++) buf[i++] = LUMA_SYNC;
    for (int n = 0; n < BACK_PORCH_LEN; n++) buf[i++] = LUMA_BLANK;
    // 12 vertical bars of progressively lighter grey, values 4..15
    // (values 0..3 reserved for sync, must not appear in active video)
    for (int n = 0; n < ACTIVE_LEN;     n++) {
        int bar = 4 + (n * 12) / ACTIVE_LEN;   // 4..15
        buf[i++] = (uint8_t)bar;
    }
    for (int n = 0; n < FRONT_PORCH_LEN; n++) buf[i++] = LUMA_BLANK;
}

static void lcd_task(void *arg) {
    vTaskDelay(8000 / portTICK_PERIOD_MS);

    // Allocate the three line templates in DMA-capable internal SRAM
    line_blank  = heap_caps_malloc(LINE_LEN, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    line_vsync  = heap_caps_malloc(LINE_LEN, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    line_active = heap_caps_malloc(LINE_LEN, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!line_blank || !line_vsync || !line_active) {
        while (1) vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
    build_blank_line(line_blank);
    build_vsync_line(line_vsync);
    build_active_line(line_active);

    // Allocate descriptor chain (one per scanline)
    desc_chain = heap_caps_malloc(sizeof(dma_descriptor_t) * FRAME_LINES,
                                  MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!desc_chain) {
        while (1) vTaskDelay(60000 / portTICK_PERIOD_MS);
    }

    // Wire up the chain: each descriptor points to next, last loops to first
    for (int i = 0; i < FRAME_LINES; i++) {
        uint8_t *buf;
        if (i < VSYNC_LINES) {
            buf = line_vsync;
        } else if (i < VSYNC_LINES + VBLANK_TOP_LINES) {
            buf = line_blank;
        } else if (i < VSYNC_LINES + VBLANK_TOP_LINES + ACTIVE_LINES) {
            buf = line_active;
        } else {
            buf = line_blank;
        }
        desc_chain[i].dw0.size    = LINE_LEN;
        desc_chain[i].dw0.length  = LINE_LEN;
        desc_chain[i].dw0.suc_eof = 0;
        desc_chain[i].dw0.owner   = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        desc_chain[i].buffer      = buf;
        desc_chain[i].next        = &desc_chain[(i + 1) % FRAME_LINES];
    }

    // Enable & reset LCD_CAM peripheral
    periph_module_enable(PERIPH_LCD_CAM_MODULE);
    periph_module_reset(PERIPH_LCD_CAM_MODULE);

    // Allocate GDMA TX channel
    gdma_channel_alloc_config_t dma_cfg = {
        .sibling_chan = NULL,
        .direction = GDMA_CHANNEL_DIRECTION_TX,
        .flags = { .reserve_sibling = 0 },
    };
    if (gdma_new_channel(&dma_cfg, &dma_chan) != ESP_OK) {
        while (1) vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
    gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
    gdma_strategy_config_t strat = { .owner_check = false, .auto_update_desc = false };
    gdma_apply_strategy(dma_chan, &strat);

    // Route GPIOs to LCD peripheral
    for (int i = 0; i < 8; i++) {
        esp_rom_gpio_connect_out_signal(mux[i].pin, mux[i].signal, false, false);
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[mux[i].pin], PIN_FUNC_GPIO);
        gpio_set_drive_capability((gpio_num_t)mux[i].pin, (gpio_drive_cap_t)3);
    }

    // Configure LCD_CAM clock for 13.33 MHz pclk
    LCD_CAM.lcd_clock.clk_en = 1;
    LCD_CAM.lcd_clock.lcd_clk_sel = 3;
    LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
    LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
    LCD_CAM.lcd_clock.lcd_clkm_div_num = 6;
    LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
    LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;
    LCD_CAM.lcd_clock.lcd_ck_idle_edge = 1;
    LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;

    // 8-bit continuous mode
    LCD_CAM.lcd_user.lcd_8bits_order = 0;
    LCD_CAM.lcd_user.lcd_bit_order   = 0;
    LCD_CAM.lcd_user.lcd_byte_order  = 0;
    LCD_CAM.lcd_user.lcd_2byte_en    = 0;
    LCD_CAM.lcd_user.lcd_dummy       = 0;
    LCD_CAM.lcd_user.lcd_cmd         = 0;
    LCD_CAM.lcd_user.lcd_dout        = 1;
    LCD_CAM.lcd_user.lcd_always_out_en = 1;
    LCD_CAM.lcd_user.lcd_dout_cyclelen = LINE_LEN - 1;
    LCD_CAM.lcd_misc.lcd_bk_en       = 1;

    // Start DMA at first descriptor and kick off LCD
    gdma_start(dma_chan, (intptr_t)&desc_chain[0]);
    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start  = 1;

    while (1) vTaskDelay(60000 / portTICK_PERIOD_MS);
}

void composite_init(void) {
    xTaskCreatePinnedToCore(lcd_task, "lcd_task", 8192, NULL, 5, NULL, 1);
}

void composite_submit_fb(const uint8_t *fb) { (void)fb; }
