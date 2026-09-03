#ifndef CAMERA_H
#define CAMERA_H

#include "esp_camera.h"
#include "esp_err.h"

/**
 * @brief Initialize the camera
 * @return ESP_OK on success
 */
esp_err_t camera_init(void);

/**
 * @brief Capture a frame from the camera
 * @return Pointer to camera_fb_t frame buffer, NULL on failure
 */
camera_fb_t *camera_capture_frame(void);

/**
 * @brief Return a frame buffer to the camera driver
 */
void camera_release_frame(camera_fb_t *fb);

/**
 * @brief Get frame statistics
 * @return Frame number (incremented each capture)
 */
uint32_t camera_get_frame_number(void);

#endif // CAMERA_H
