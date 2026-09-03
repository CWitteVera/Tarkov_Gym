#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

#ifdef CONFIG_ENABLE_CAMERA_DEBUG
#include "camera.h"
#include "stream_uart.h"
#endif

static const char *TAG = "TarkovGym";

#define LED_GPIO_PRIMARY GPIO_NUM_21
#define LED_USE_SECONDARY_GPIO 1
#define LED_GPIO_SECONDARY GPIO_NUM_2
#define LED_ACTIVE_LOW 0
#define STREAM_FRAME_INTERVAL_MS 100
#define BLINK_TOGGLE_INTERVAL_MS 500
#define LED_CLICK_ON_MS 700
#define LED_CLICK_OFF_HOLD_MS 280
#define HOST_CMD_FLASH_LED '!'
#define HOST_CMD_LED_CAMERA_READY 'C'
#define HOST_CMD_LED_HEX_DETECTED 'H'
#define HOST_CMD_LED_MATCH_SOLID 'M'
#define HOST_CMD_LED_ERROR 'E'

#define LED_BLINK_DETECTED_MS 220

typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_HEX_DETECTED,
} led_mode_t;

static esp_err_t apply_led_mode(led_mode_t mode, TickType_t now, TickType_t *last_toggle, bool *led_on);

static int led_physical_level(int logical_on)
{
    if (LED_ACTIVE_LOW) {
        return logical_on ? 0 : 1;
    }
    return logical_on ? 1 : 0;
}

static esp_err_t set_led_state(int level)
{
    int phys = led_physical_level(level);
    esp_err_t err = gpio_set_level(LED_GPIO_PRIMARY, phys);
    if (err != ESP_OK) {
        return err;
    }
#if LED_USE_SECONDARY_GPIO
    err = gpio_set_level(LED_GPIO_SECONDARY, phys);
    if (err != ESP_OK) {
        return err;
    }
#endif
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err = gpio_reset_pin(LED_GPIO_PRIMARY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset GPIO%u: %s", LED_GPIO_PRIMARY, esp_err_to_name(err));
        return;
    }

#if LED_USE_SECONDARY_GPIO
    err = gpio_reset_pin(LED_GPIO_SECONDARY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset GPIO%u: %s", LED_GPIO_SECONDARY, esp_err_to_name(err));
        return;
    }
#endif

    err = gpio_set_direction(LED_GPIO_PRIMARY, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO%u as output: %s", LED_GPIO_PRIMARY, esp_err_to_name(err));
        return;
    }

#if LED_USE_SECONDARY_GPIO
    err = gpio_set_direction(LED_GPIO_SECONDARY, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO%u as output: %s", LED_GPIO_SECONDARY, esp_err_to_name(err));
        return;
    }
#endif

#if LED_USE_SECONDARY_GPIO
    ESP_LOGI(TAG, "Tarkov Gym app started (LED GPIO%u primary, GPIO%u secondary, active_low=%d)",
             LED_GPIO_PRIMARY, LED_GPIO_SECONDARY, LED_ACTIVE_LOW);
#else
    ESP_LOGI(TAG, "Tarkov Gym app started (LED GPIO%u, active_low=%d)", LED_GPIO_PRIMARY, LED_ACTIVE_LOW);
#endif

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_blink_toggle = last_wake;
    TickType_t click_on_until = 0;
    TickType_t click_off_until = 0;
    TickType_t last_flash_status = 0;
    TickType_t last_mode_status = 0;
    bool click_active = false;
    bool click_phase_on = false;
    bool led_on = false;
    led_mode_t led_mode = LED_MODE_OFF;

#ifdef CONFIG_ENABLE_CAMERA_DEBUG
    ESP_LOGI(TAG, "Initializing camera and UART stream...");
    bool camera_ready = false;

    err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed");
        stream_uart_send_status("Camera Init Failed");
        led_mode = LED_MODE_OFF;
    } else {
        err = stream_uart_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART stream initialization failed");
            led_mode = LED_MODE_OFF;
        } else {
            camera_ready = true;
            led_mode = LED_MODE_OFF;
            stream_uart_send_status("Camera Stream Ready - Connect PC viewer at 115200 baud");
            // Stop log output once streaming starts because logs share the same serial link as JPEG frames.
            esp_log_level_set("*", ESP_LOG_NONE);
        }
    }
#endif

    err = set_led_state(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set initial LED state: %s", esp_err_to_name(err));
        return;
    }

    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 256,
    };
    err = usb_serial_jtag_driver_install(&usb_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install USB Serial/JTAG driver: %s", esp_err_to_name(err));
        return;
    }

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Host commands over USB Serial/JTAG: ! C H M E
        uint8_t cmd_buf[16];
        int cmd_len = usb_serial_jtag_read_bytes(cmd_buf, sizeof(cmd_buf), 0);
        if (cmd_len > 0) {
            for (int i = 0; i < cmd_len; i++) {
                if (cmd_buf[i] == HOST_CMD_FLASH_LED) {
                    click_active = true;
                    click_phase_on = true;
                    click_on_until = now + pdMS_TO_TICKS(LED_CLICK_ON_MS);
                    click_off_until = 0;
                    err = set_led_state(1);
                    if (err == ESP_OK) {
                        led_on = true;
                    }
#ifdef CONFIG_ENABLE_CAMERA_DEBUG
                    if ((now - last_flash_status) >= pdMS_TO_TICKS(500)) {
                        stream_uart_send_status("LED click simulation triggered");
                        last_flash_status = now;
                    }
#endif
                } else if (cmd_buf[i] == HOST_CMD_LED_CAMERA_READY || cmd_buf[i] == HOST_CMD_LED_MATCH_SOLID || cmd_buf[i] == HOST_CMD_LED_ERROR) {
                    if (led_mode != LED_MODE_OFF) {
                        led_mode = LED_MODE_OFF;
#ifdef CONFIG_ENABLE_CAMERA_DEBUG
                        if ((now - last_mode_status) >= pdMS_TO_TICKS(200)) {
                            stream_uart_send_status("LED mode: OFF");
                            last_mode_status = now;
                        }
#endif
                    }
                    last_blink_toggle = now;
                } else if (cmd_buf[i] == HOST_CMD_LED_HEX_DETECTED) {
                    if (led_mode != LED_MODE_HEX_DETECTED) {
                        led_mode = LED_MODE_HEX_DETECTED;
#ifdef CONFIG_ENABLE_CAMERA_DEBUG
                        if ((now - last_mode_status) >= pdMS_TO_TICKS(200)) {
                            stream_uart_send_status("LED mode: HEX_DETECTED");
                            last_mode_status = now;
                        }
#endif
                    }
                    last_blink_toggle = now;
                }
            }
        }

        if (click_active) {
            if (click_phase_on && now >= click_on_until) {
                err = set_led_state(0);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to drive LED pins: %s", esp_err_to_name(err));
                    esp_restart();
                }
                led_on = false;
                click_phase_on = false;
                click_off_until = now + pdMS_TO_TICKS(LED_CLICK_OFF_HOLD_MS);
            } else if (!click_phase_on && now >= click_off_until) {
                click_active = false;
                // Restart blink timing so the held-off period is preserved.
                last_blink_toggle = now;
            }
        } else {
            err = apply_led_mode(led_mode, now, &last_blink_toggle, &led_on);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to drive LED pins: %s", esp_err_to_name(err));
                esp_restart();
            }
        }

#ifdef CONFIG_ENABLE_CAMERA_DEBUG
        // Capture and stream camera frame
        if (camera_ready) {
            camera_fb_t *fb = camera_capture_frame();
            if (fb) {
                stream_uart_send_frame(fb);
                camera_release_frame(fb);
            }
        }
#endif

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STREAM_FRAME_INTERVAL_MS));
    }
}

static esp_err_t apply_led_mode(led_mode_t mode, TickType_t now, TickType_t *last_toggle, bool *led_on)
{
    TickType_t interval = pdMS_TO_TICKS(BLINK_TOGGLE_INTERVAL_MS);

    if (mode == LED_MODE_OFF) {
        if (*led_on) {
            esp_err_t err = set_led_state(0);
            if (err != ESP_OK) {
                return err;
            }
            *led_on = false;
        }
        return ESP_OK;
    }

    if (mode == LED_MODE_HEX_DETECTED) {
        interval = pdMS_TO_TICKS(LED_BLINK_DETECTED_MS);
    }

    if ((now - *last_toggle) >= interval) {
        *led_on = !*led_on;
        esp_err_t err = set_led_state(*led_on ? 1 : 0);
        if (err != ESP_OK) {
            return err;
        }
        *last_toggle = now;
    }

    return ESP_OK;
}
