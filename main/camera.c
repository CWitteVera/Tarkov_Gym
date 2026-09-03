#include "camera.h"
#include "esp_log.h"

static const char *TAG = "Camera";
static uint32_t frame_number = 0;

// Camera pin definitions for Seeed Studio XIAO ESP32-S3 (Sense camera profile)
// Matches components/esp_camera/examples/camera_example/main/camera_pinout.h
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_VSYNC 38
#define CAMERA_PIN_HREF 47
#define CAMERA_PIN_PCLK 13
#define CAMERA_PIN_XCLK 10
#define CAMERA_PIN_SIOD 40
#define CAMERA_PIN_SIOC 39
#define CAMERA_PIN_D0 15
#define CAMERA_PIN_D1 17
#define CAMERA_PIN_D2 18
#define CAMERA_PIN_D3 16
#define CAMERA_PIN_D4 14
#define CAMERA_PIN_D5 12
#define CAMERA_PIN_D6 11
#define CAMERA_PIN_D7 48

esp_err_t camera_init(void)
{
    camera_config_t camera_config = {
        .pin_pwdn = CAMERA_PIN_PWDN,
        .pin_reset = CAMERA_PIN_RESET,
        .pin_xclk = CAMERA_PIN_XCLK,
        .pin_sscb_sda = CAMERA_PIN_SIOD,
        .pin_sscb_scl = CAMERA_PIN_SIOC,
        .pin_d7 = CAMERA_PIN_D7,
        .pin_d6 = CAMERA_PIN_D6,
        .pin_d5 = CAMERA_PIN_D5,
        .pin_d4 = CAMERA_PIN_D4,
        .pin_d3 = CAMERA_PIN_D3,
        .pin_d2 = CAMERA_PIN_D2,
        .pin_d1 = CAMERA_PIN_D1,
        .pin_d0 = CAMERA_PIN_D0,
        .pin_vsync = CAMERA_PIN_VSYNC,
        .pin_href = CAMERA_PIN_HREF,
        .pin_pclk = CAMERA_PIN_PCLK,

        .xclk_freq_hz = 10000000,  // 10 MHz for stability on Sense modules
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,  // JPEG for efficient streaming
        .frame_size = FRAMESIZE_QQVGA,   // 160x120 keeps bitrate within serial limits
        .jpeg_quality = 36,              // More compression for higher serial throughput
        .fb_count = 1,                   // Single frame buffer
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };

    // Initialize camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    // Get camera sensor handle
    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return ESP_FAIL;
    }

    // Configure camera sensor for brighter automatic exposure on Sense modules.
    s->set_brightness(s, 1);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_special_effect(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_ae_level(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_32X);

    ESP_LOGI(TAG, "Camera initialized successfully (160x120 JPEG, XCLK=10MHz, XTAL=40MHz)");
    return ESP_OK;
}

camera_fb_t *camera_capture_frame(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        return NULL;
    }
    frame_number++;
    return fb;
}

void camera_release_frame(camera_fb_t *fb)
{
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

uint32_t camera_get_frame_number(void)
{
    return frame_number;
}
