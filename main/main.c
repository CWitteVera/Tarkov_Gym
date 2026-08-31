#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "tarkov_gym";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP-IDF app started");

    while (1) {
        /*
         * Keep the loop as lean as possible. Logging on every pass is a major
         * performance bottleneck, and a 2s delay makes frame processing far too
         * slow for high-throughput workloads. Yield briefly so the scheduler can
         * keep the system responsive without throttling the app unnecessarily.
         */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
