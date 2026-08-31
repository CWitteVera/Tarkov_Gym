#pragma once

#include <Arduino.h>

#include "configuration.h"

struct CalibrationState {
  int roiX = config::ROI_X;
  int roiY = config::ROI_Y;
  int roiWidth = config::ROI_WIDTH;
  int roiHeight = config::ROI_HEIGHT;
  uint8_t brightnessThreshold = config::BRIGHTNESS_THRESHOLD;
  uint16_t matchTolerance = config::MATCH_TOLERANCE_PIXELS;
  uint8_t matchConfirmationFrames = config::MATCH_CONFIRMATION_FRAMES;
};

namespace calibration {
void resetToDefaults(CalibrationState& calibration);
void printStatus(const CalibrationState& calibration);
bool parseCommand(const String& command, CalibrationState& calibration);
}  // namespace calibration
