#include "debug.h"

namespace {
config::DebugLevel gDebugLevel = config::DEBUG_LEVEL;
}

namespace debug {
void setLevel(config::DebugLevel level) { gDebugLevel = level; }
config::DebugLevel getLevel() { return gDebugLevel; }
bool enabled(config::DebugLevel level) { return gDebugLevel >= level; }

void printStatusLine(const __FlashStringHelper* text) { Serial.println(text); }
void printStatusLine(const char* text) { Serial.println(text); }

void printCameraStartupSummary() {
  Serial.println("Camera initialized");
  Serial.print("Resolution: ");
  Serial.print(config::CAMERA_WIDTH);
  Serial.print("x");
  Serial.println(config::CAMERA_HEIGHT);
  Serial.println("ROI active: x=40 y=20 w=240 h=200");
}

void printDetectionSummary(int frameIndex, int outerRadius, int innerRadius, int difference,
                          uint8_t matchConfirmationFrames, bool matchReady, bool clickEnabled,
                          uint16_t roundNumber) {
  if (!enabled(config::DEBUG_DETECTION)) {
    return;
  }
  Serial.print("Frame ");
  Serial.print(frameIndex);
  Serial.print(" | outer=");
  Serial.print(outerRadius);
  Serial.print(" inner=");
  Serial.print(innerRadius);
  Serial.print(" diff=");
  Serial.print(difference);
  Serial.print(" confirm=");
  Serial.print(matchConfirmationFrames);
  Serial.print(" match=");
  Serial.print(matchReady ? "YES" : "NO");
  Serial.print(" clickEnabled=");
  Serial.print(clickEnabled ? "YES" : "NO");
  Serial.print(" round=");
  Serial.println(roundNumber);
}

void printFrameMetrics(float cameraFps, float processingFps, uint32_t avgMs, uint32_t maxMs,
                       uint32_t frameCount) {
  if (!enabled(config::DEBUG_BASIC)) {
    return;
  }
  Serial.print("Camera FPS: ");
  Serial.print(cameraFps, 1);
  Serial.print(" | Processing FPS: ");
  Serial.print(processingFps, 1);
  Serial.print(" | avg_ms=");
  Serial.print(avgMs);
  Serial.print(" | max_ms=");
  Serial.print(maxMs);
  Serial.print(" | frames=");
  Serial.println(frameCount);
}

void printSerialHelp() {
  Serial.println("Commands:");
  Serial.println("  CLICK ON");
  Serial.println("  CLICK OFF");
  Serial.println("  STATUS");
  Serial.println("  DEBUG 0/1/2/3");
  Serial.println("  CALIBRATE");
}
}  // namespace debug
