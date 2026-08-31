#pragma once

#include <Arduino.h>

#include "configuration.h"
#include "hexagon_detector.h"

struct VisionResult {
  bool valid = false;
  HexagonMeasurement inner;
  HexagonMeasurement outer;
  int difference = 0;
  int frameIndex = 0;
  bool matchReady = false;
};

class VisionProcessor {
 public:
  VisionProcessor();
  void configureRoi(int x, int y, int width, int height);
  void setThreshold(uint8_t threshold);
  void setTolerance(uint16_t pixels);
  void setMatchConfirmationFrames(uint8_t frames);
  void resetMatchState();
  bool processFrame(const uint8_t* frame, uint16_t width, uint16_t height, VisionResult& result);

 private:
  HexagonDetector detector_;
  int roiX_ = config::ROI_X;
  int roiY_ = config::ROI_Y;
  int roiWidth_ = config::ROI_WIDTH;
  int roiHeight_ = config::ROI_HEIGHT;
  uint8_t threshold_ = config::BRIGHTNESS_THRESHOLD;
  uint16_t tolerance_ = config::MATCH_TOLERANCE_PIXELS;
  uint8_t confirmationFrames_ = config::MATCH_CONFIRMATION_FRAMES;
  uint8_t matchFrames_ = 0;
  bool lastMatchState_ = false;
};
