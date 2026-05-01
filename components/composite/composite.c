#include "composite.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "composite";

static void pin_cycle_task(void *arg) {
    vTaskDelay(500 / portTICK_PERIOD_MS);

    ESP_LOGW(TAG, "configuring GPIO 35-38 as outputs");
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << 35) | (1ULL << 36) | (1ULL << 37) | (1ULL << 38),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    ESP_LOGW(TAG, "gpio_config returned %d", err);

    int pins[] = {35, 36, 37, 38};
    int idx = 0;
    while (1) {
        for (int i = 0; i < 4; i++) gpio_set_level(pins[i], 0);
        gpio_set_level(pins[idx], 1);
        ESP_LOGW(TAG, "GPIO%d HIGH", pins[idx]);
        idx = (idx + 1) % 4;
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void composite_init(void) {
    ESP_LOGW(TAG, "composite_init entered");
    xTaskCreatePinnedToCore(pin_cycle_task, "pin_cycle", 4096, NULL, 5, NULL, 1);
    ESP_LOGW(TAG, "composite_init returned");
}

void composite_submit_fb(const uint8_t *fb) { (void)fb; }
