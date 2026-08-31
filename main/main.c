#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "tarkov_gym";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-IDF app started");

    while (1) {
        ESP_LOGI(TAG, "Running...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
