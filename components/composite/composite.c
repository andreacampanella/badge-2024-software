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

// Internal-SRAM-friendly count of unique active line buffers.
// 64 * 864 = ~55KB, fits in internal SRAM.
#define NUM_ACTIVE_BUFS    48

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
static uint8_t *active_bufs[NUM_ACTIVE_BUFS];  // 64 unique active line buffers

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
    if (!line_blank || !line_vsync) {
        while (1) vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
    build_blank_line(line_blank);
    build_vsync_line(line_vsync);
    for (int i = 0; i < NUM_ACTIVE_BUFS; i++) {
        active_bufs[i] = heap_caps_malloc(LINE_LEN, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!active_bufs[i]) { while (1) vTaskDelay(60000 / portTICK_PERIOD_MS); }
        build_active_line(active_bufs[i]);   // start with the bars pattern
    }

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
            int active_idx = i - (VSYNC_LINES + VBLANK_TOP_LINES);   // 0..287
            int buf_idx = (active_idx * NUM_ACTIVE_BUFS) / ACTIVE_LINES; // 0..63
            buf = active_bufs[buf_idx];
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

volatile uint32_t composite_submit_count = 0;
volatile const uint8_t *composite_last_fb = NULL;

// RGB565 byteswapped (high byte first in memory) -> 4-bit luma
static inline uint8_t rgb565_to_luma4(uint8_t hi, uint8_t lo) {
    uint16_t p = ((uint16_t)hi << 8) | lo;
    uint8_t r = (p >> 11) & 0x1F;
    uint8_t g = (p >>  5) & 0x3F;
    uint8_t b =  p        & 0x1F;
    uint32_t y = 77u * (r << 1) + 150u * g + 29u * (b << 1);
    uint32_t level = (y * 12u) / 16321u;
    return (uint8_t)(LUMA_BLACK + level);
}

// Source framebuffer geometry (matches drivers/gc9a01/display.c)
#define FB_WIDTH   240
#define FB_HEIGHT  240
#define FB_BPP       2

static void render_task(void *arg) {
    vTaskDelay(10000 / portTICK_PERIOD_MS);   // wait for lcd_task to finish init

    // Horizontal layout in active region: letterbox 240 columns in the middle.
    // ACTIVE_LEN = 690; padding each side = (690-240)/2 = 225.
    const int h_pad = (ACTIVE_LEN - FB_WIDTH) / 2;
    const int active_off = SYNC_LEN + BACK_PORCH_LEN;

    while (1) {
        vTaskDelay(33 / portTICK_PERIOD_MS);   // ~30 Hz max; cheap if no fb yet
        const uint8_t *fb = (const uint8_t *)composite_last_fb;
        if (!fb) continue;

        for (int row = 0; row < NUM_ACTIVE_BUFS; row++) {
            // Which source row maps to this output row?
            int y_src = (row * FB_HEIGHT) / NUM_ACTIVE_BUFS;
            const uint8_t *src = &fb[y_src * FB_WIDTH * FB_BPP];
            uint8_t *dst = active_bufs[row] + active_off;
            // Left padding stays at LUMA_BLANK from build_active_line; just
            // overwrite the centre 240 columns.
            // (The bars to either side will remain visible until first frame.)
            for (int x = 0; x < h_pad; x++)        dst[x] = LUMA_BLANK;
            for (int x = 0; x < FB_WIDTH; x++)    dst[h_pad + x] = rgb565_to_luma4(src[x*2], src[x*2+1]);
            for (int x = h_pad + FB_WIDTH; x < ACTIVE_LEN; x++) dst[x] = LUMA_BLANK;
        }
    }
}

void composite_init(void) {
    xTaskCreatePinnedToCore(lcd_task, "lcd_task", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(render_task, "render", 4096, NULL, 4, NULL, 1);
}

void composite_submit_fb(const uint8_t *fb) {
    composite_submit_count++;
    composite_last_fb = fb;
}
