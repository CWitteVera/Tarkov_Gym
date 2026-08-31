#pragma once

#include <Arduino.h>

#include "configuration.h"

namespace debug {
void setLevel(config::DebugLevel level);
config::DebugLevel getLevel();
bool enabled(config::DebugLevel level);
void printStatusLine(const __FlashStringHelper* text);
void printStatusLine(const char* text);
void printCameraStartupSummary();
void printDetectionSummary(int frameIndex, int outerRadius, int innerRadius, int difference,
                          uint8_t matchConfirmationFrames, bool matchReady, bool clickEnabled,
                          uint16_t roundNumber);
void printFrameMetrics(float cameraFps, float processingFps, uint32_t avgMs, uint32_t maxMs,
                       uint32_t frameCount);
void printSerialHelp();
}  // namespace debug
