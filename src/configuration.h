#pragma once

#include <Arduino.h>

namespace config {
constexpr uint16_t CAMERA_WIDTH = 320;
constexpr uint16_t CAMERA_HEIGHT = 240;
constexpr uint8_t CAMERA_FRAME_BUFFER_COUNT = 2;

constexpr uint16_t ROI_X = 40;
constexpr uint16_t ROI_Y = 20;
constexpr uint16_t ROI_WIDTH = 240;
constexpr uint16_t ROI_HEIGHT = 200;

constexpr uint8_t BRIGHTNESS_THRESHOLD = 200;
constexpr uint16_t MIN_HEXAGON_SIZE = 18;
constexpr uint16_t MAX_HEXAGON_SIZE = 220;
constexpr uint16_t MATCH_TOLERANCE_PIXELS = 4;
constexpr uint8_t MATCH_CONFIRMATION_FRAMES = 3;
constexpr uint32_t CLICK_DURATION_MS = 30;
constexpr uint32_t CLICK_COOLDOWN_MS = 250;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 5000;

enum DebugLevel {
  DEBUG_NONE = 0,
  DEBUG_BASIC = 1,
  DEBUG_DETECTION = 2,
  DEBUG_VERBOSE = 3
};

constexpr DebugLevel DEBUG_LEVEL = DEBUG_BASIC;

struct RoiRect {
  int x;
  int y;
  int w;
  int h;
};

inline RoiRect makeRoi() {
  return {ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT};
}
}  // namespace config
