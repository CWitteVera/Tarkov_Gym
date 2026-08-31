#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TarkovGym";

#define BLINK_GPIO GPIO_NUM_2

void app_main(void)
{
    esp_err_t err = gpio_reset_pin(BLINK_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset GPIO%u: %s", BLINK_GPIO, esp_err_to_name(err));
        return;
    }

    err = gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO%u as output: %s", BLINK_GPIO, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Tarkov Gym app started on GPIO%u", BLINK_GPIO);

    while (1) {
        err = gpio_set_level(BLINK_GPIO, 1);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to drive GPIO%u high: %s", BLINK_GPIO, esp_err_to_name(err));
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        err = gpio_set_level(BLINK_GPIO, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to drive GPIO%u low: %s", BLINK_GPIO, esp_err_to_name(err));
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
