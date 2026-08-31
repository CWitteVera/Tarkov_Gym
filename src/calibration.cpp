#include "calibration.h"

namespace calibration {
void resetToDefaults(CalibrationState& calibration) {
  calibration.roiX = config::ROI_X;
  calibration.roiY = config::ROI_Y;
  calibration.roiWidth = config::ROI_WIDTH;
  calibration.roiHeight = config::ROI_HEIGHT;
  calibration.brightnessThreshold = config::BRIGHTNESS_THRESHOLD;
  calibration.matchTolerance = config::MATCH_TOLERANCE_PIXELS;
  calibration.matchConfirmationFrames = config::MATCH_CONFIRMATION_FRAMES;
}

void printStatus(const CalibrationState& calibration) {
  Serial.println("Calibration state:");
  Serial.print("  ROI: x=");
  Serial.print(calibration.roiX);
  Serial.print(" y=");
  Serial.print(calibration.roiY);
  Serial.print(" w=");
  Serial.print(calibration.roiWidth);
  Serial.print(" h=");
  Serial.print(calibration.roiHeight);
  Serial.print(" | brightness threshold=");
  Serial.print(calibration.brightnessThreshold);
  Serial.print(" | match tolerance=");
  Serial.print(calibration.matchTolerance);
  Serial.print(" | confirm frames=");
  Serial.println(calibration.matchConfirmationFrames);
}

bool parseCommand(const String& command, CalibrationState& calibration) {
  if (command.startsWith("ROI ")) {
    int x = 0, y = 0, w = 0, h = 0;
    char* cursor = nullptr;
    String raw = command.substring(4);
    x = raw.toInt();
    y = raw.substring(raw.indexOf(' ') + 1).toInt();
    w = raw.substring(raw.lastIndexOf(' ') + 1).toInt();
    // Simple parse fallback for the common case: ROI x y w h
    int firstSpace = raw.indexOf(' ');
    int secondSpace = raw.indexOf(' ', firstSpace + 1);
    int thirdSpace = raw.indexOf(' ', secondSpace + 1);
    if (firstSpace >= 0 && secondSpace >= 0 && thirdSpace >= 0) {
      x = raw.substring(0, firstSpace).toInt();
      y = raw.substring(firstSpace + 1, secondSpace).toInt();
      w = raw.substring(secondSpace + 1, thirdSpace).toInt();
      h = raw.substring(thirdSpace + 1).toInt();
      calibration.roiX = x;
      calibration.roiY = y;
      calibration.roiWidth = w;
      calibration.roiHeight = h;
      return true;
    }
    return false;
  }

  if (command.startsWith("THRESH ")) {
    calibration.brightnessThreshold = static_cast<uint8_t>(command.substring(7).toInt());
    return true;
  }

  if (command.startsWith("MATCH ")) {
    calibration.matchTolerance = static_cast<uint16_t>(command.substring(6).toInt());
    return true;
  }

  if (command.startsWith("CONFIRM ")) {
    calibration.matchConfirmationFrames = static_cast<uint8_t>(command.substring(8).toInt());
    return true;
  }

  return false;
}
}  // namespace calibration
