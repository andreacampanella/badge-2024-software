/*
 * Tildagon composite video output - direct LCD_CAM peripheral programming.
 *
 * Adapted from Phil Burgess / Adafruit's ESP32-S3 LCD peripheral blog post:
 *   https://blog.adafruit.com/2022/06/21/esp32uesday-more-s3-lcd-peripheral-hacking-with-code/
 * Used as a reference for register-level setup of LCD_CAM + GDMA on ESP32-S3.
 *
 * Modified for: continuous circular DMA loop (no one-shot), 8 data lines,
 * with 3 routed to slot B HS pins for a 3-bit luma DAC.
 */

#include "composite.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
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

static gdma_channel_handle_t dma_chan;
static dma_descriptor_t desc __attribute__((aligned(4)));

// Output pin assignments. Slot B HS1/HS2/HS3 = active luma bits.
// Other 5 are parked on hexpansion slots that should be empty.
static const struct {
    int8_t pin;
    uint8_t signal;
} mux[] = {
    { 35, LCD_DATA_OUT0_IDX },  // J2 HS_F - luma LSB
    { 36, LCD_DATA_OUT1_IDX },  // J2 HS_G - luma mid
    { 37, LCD_DATA_OUT2_IDX },  // J2 HS_H - luma MSB
    { 38, LCD_DATA_OUT3_IDX },  // J2 HS_I - parked
    { 11, LCD_DATA_OUT4_IDX },  // J4 HS_F - parked
    { 14, LCD_DATA_OUT5_IDX },  // J4 HS_G - parked
    { 13, LCD_DATA_OUT6_IDX },  // J4 HS_H - parked
    { 12, LCD_DATA_OUT7_IDX },  // J4 HS_I - parked
};

// Test pattern: 256 bytes, value = index. So D0 toggles every byte (1/2 pclk),
// D1 every 2 bytes, D2 every 4 bytes, etc. Easy to spot on scope.
#define BUF_LEN 256
static uint8_t *dma_buf = NULL;

// Heartbeat for visibility (separate from DMA output)
static int heartbeat_pin = 35;  // will be reassigned after LCD takes over

static void blink_count(int n) {
    for (int i = 0; i < n; i++) {
        gpio_set_level(heartbeat_pin, 1);
        vTaskDelay(80 / portTICK_PERIOD_MS);
        gpio_set_level(heartbeat_pin, 0);
        vTaskDelay(180 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1500 / portTICK_PERIOD_MS);
}

static void lcd_task(void *arg) {
    // Use GPIO 35 for heartbeat blinks ONLY before LCD takes over
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << 35),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = 0,
    };
    gpio_config(&cfg);

    vTaskDelay(8000 / portTICK_PERIOD_MS);
    blink_count(1);

    // Allocate DMA-capable buffer
    dma_buf = heap_caps_malloc(BUF_LEN, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!dma_buf) { blink_count(2); while (1) vTaskDelay(60000 / portTICK_PERIOD_MS); }
    for (int i = 0; i < BUF_LEN; i++) dma_buf[i] = (uint8_t)i;
    blink_count(3);

    // Enable & reset LCD_CAM peripheral
    periph_module_enable(PERIPH_LCD_CAM_MODULE);
    periph_module_reset(PERIPH_LCD_CAM_MODULE);
    blink_count(4);

    // Configure DMA descriptor for circular loop: descriptor points to itself
    desc.dw0.size = BUF_LEN;
    desc.dw0.length = BUF_LEN;
    desc.dw0.suc_eof = 0;       // not last - it loops
    desc.dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
    desc.buffer = dma_buf;
    desc.next = &desc;          // circular: points back to itself

    // Allocate GDMA TX channel and connect to LCD
    gdma_channel_alloc_config_t dma_cfg = {
        .sibling_chan = NULL,
        .direction = GDMA_CHANNEL_DIRECTION_TX,
        .flags = { .reserve_sibling = 0 },
    };
    if (gdma_new_channel(&dma_cfg, &dma_chan) != ESP_OK) {
        blink_count(5); while (1) vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
    gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
    gdma_strategy_config_t strat = { .owner_check = false, .auto_update_desc = false };
    gdma_apply_strategy(dma_chan, &strat);
    blink_count(6);

    // Now hand GPIO 35..(38),11..14 over to the LCD peripheral via GPIO matrix
    for (int i = 0; i < 8; i++) {
        esp_rom_gpio_connect_out_signal(mux[i].pin, mux[i].signal, false, false);
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[mux[i].pin], PIN_FUNC_GPIO);
        gpio_set_drive_capability((gpio_num_t)mux[i].pin, (gpio_drive_cap_t)3);
    }
    blink_count(7);
    // After this point, GPIO 35 is owned by LCD_CAM. blinks via gpio_set_level
    // won't reach the pin reliably. Use a different pin if more heartbeats needed.

    // Configure LCD_CAM clock: source = PLL_F160M (selector 3), divisor = 160 -> 1 MHz pclk
    LCD_CAM.lcd_clock.clk_en = 1;
    LCD_CAM.lcd_clock.lcd_clk_sel = 3;       // PLL_F160M_CLK
    LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
    LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
    LCD_CAM.lcd_clock.lcd_clkm_div_num = 160;  // 160 MHz / 160 = 1 MHz
    LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
    LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;       // PCLK = LCDclk / (clkcnt_n+1) ; minimum=1 -> /2
    LCD_CAM.lcd_clock.lcd_ck_idle_edge = 1;
    LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;

    // 8-bit mode, no command/dummy phase
    LCD_CAM.lcd_user.lcd_8bits_order = 0;
    LCD_CAM.lcd_user.lcd_bit_order   = 0;
    LCD_CAM.lcd_user.lcd_byte_order  = 0;
    LCD_CAM.lcd_user.lcd_2byte_en    = 0;
    LCD_CAM.lcd_user.lcd_dummy       = 0;
    LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;
    LCD_CAM.lcd_user.lcd_cmd         = 0;
    LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;
    LCD_CAM.lcd_user.lcd_dout        = 1;       // do output data
    LCD_CAM.lcd_user.lcd_always_out_en = 1;     // continuous output
    LCD_CAM.lcd_user.lcd_dout_cyclelen = BUF_LEN - 1;
    LCD_CAM.lcd_misc.lcd_bk_en       = 1;       // continuous DMA mode

    blink_count(8);

    // Start GDMA, then start the LCD output
    gdma_start(dma_chan, (intptr_t)&desc);
    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start  = 1;

    blink_count(9);
    // After this, GPIO 35-37 should show the pattern from dma_buf.

    while (1) vTaskDelay(60000 / portTICK_PERIOD_MS);
}

void composite_init(void) {
    xTaskCreatePinnedToCore(lcd_task, "lcd_task", 8192, NULL, 5, NULL, 1);
}

void composite_submit_fb(const uint8_t *fb) { (void)fb; }
