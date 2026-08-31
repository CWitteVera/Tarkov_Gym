#include "camera_manager.h"

#include <esp_camera.h>

#include "camera_pins.h"

namespace {
constexpr int kCameraPixelFormat = PIXFORMAT_GRAYSCALE;
constexpr framesize_t kCameraFrameSize = FRAMESIZE_QVGA;

camera_fb_t* gFrame = nullptr;
uint32_t gFramesCaptured = 0;
uint32_t gLastCaptureMs = 0;
uint32_t gLastFpsSampleMs = 0;
uint32_t gAverageProcessingMs = 0;
uint32_t gMaximumProcessingMs = 0;
uint32_t gProcessingSamples = 0;
float gCameraFps = 0.0f;
float gProcessingFps = 0.0f;

bool isCameraConfigured = false;

camera_config_t buildCameraConfig() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = kCameraPixelFormat;
  config.frame_size = kCameraFrameSize;
  config.jpeg_quality = 12;
  config.fb_count = config::CAMERA_FRAME_BUFFER_COUNT;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  return config;
}
}  // namespace

namespace camera_manager {
bool initialize() {
  if (isCameraConfigured) {
    return true;
  }

  camera_config_t config = buildCameraConfig();
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: %s\n", esp_err_to_name(err));
    return false;
  }

  isCameraConfigured = true;
  gLastCaptureMs = millis();
  gLastFpsSampleMs = gLastCaptureMs;
  return true;
}

bool captureFrame() {
  if (!isCameraConfigured) {
    return false;
  }

  gFrame = esp_camera_fb_get();
  if (gFrame == nullptr) {
    return false;
  }

  const uint32_t now = millis();
  gFramesCaptured++;
  if (gLastCaptureMs != 0) {
    const uint32_t delta = now - gLastCaptureMs;
    if (delta > 0) {
      gCameraFps = 1000.0f / static_cast<float>(delta);
    }
  }
  gLastCaptureMs = now;

  if (gLastFpsSampleMs == 0) {
    gLastFpsSampleMs = now;
  }
  return true;
}

uint8_t* frameBuffer() {
  if (gFrame == nullptr) {
    return nullptr;
  }
  return gFrame->buf;
}

uint16_t width() { return config::CAMERA_WIDTH; }
uint16_t height() { return config::CAMERA_HEIGHT; }

float cameraFps() { return gCameraFps; }
float processingFps() { return gProcessingFps; }
uint32_t averageProcessingMs() { return gAverageProcessingMs; }
uint32_t maximumProcessingMs() { return gMaximumProcessingMs; }
uint32_t frameCount() { return gFramesCaptured; }

void setProcessingTiming(uint32_t processingTimeMs) {
  gProcessingSamples++;
  gAverageProcessingMs = (gAverageProcessingMs * (gProcessingSamples - 1) + processingTimeMs) / gProcessingSamples;
  if (processingTimeMs > gMaximumProcessingMs) {
    gMaximumProcessingMs = processingTimeMs;
  }
  if (processingTimeMs > 0) {
    gProcessingFps = 1000.0f / static_cast<float>(processingTimeMs);
  }
}

void printStatus() {
  Serial.print("Camera FPS: ");
  Serial.print(cameraFps(), 1);
  Serial.print(" | processing FPS: ");
  Serial.print(processingFps(), 1);
  Serial.print(" | average processing ms: ");
  Serial.print(averageProcessingMs());
  Serial.print(" | maximum processing ms: ");
  Serial.println(maximumProcessingMs());
}
}  // namespace camera_manager
