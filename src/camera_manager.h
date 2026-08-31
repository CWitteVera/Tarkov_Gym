#pragma once

#include <Arduino.h>

#include "configuration.h"

namespace camera_manager {
bool initialize();
bool captureFrame();
uint8_t* frameBuffer();
uint16_t width();
uint16_t height();
float cameraFps();
float processingFps();
uint32_t averageProcessingMs();
uint32_t maximumProcessingMs();
uint32_t frameCount();
void setProcessingTiming(uint32_t processingTimeMs);
void printStatus();
}  // namespace camera_manager
