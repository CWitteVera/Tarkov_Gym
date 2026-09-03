#ifndef STREAM_UART_H
#define STREAM_UART_H

#include "esp_err.h"
#include "esp_camera.h"

/**
 * @brief Initialize UART streaming (USB CDC serial)
 * @return ESP_OK on success
 */
esp_err_t stream_uart_init(void);

/**
 * @brief Send a camera frame over UART as JPEG
 * @param fb Camera frame buffer (JPEG format)
 * @return ESP_OK on success
 */
esp_err_t stream_uart_send_frame(camera_fb_t *fb);

/**
 * @brief Send a status message to the serial monitor
 * @param message Status message
 */
void stream_uart_send_status(const char *message);

#endif // STREAM_UART_H
