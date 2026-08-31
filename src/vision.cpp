#include "vision.h"

#include <Arduino.h>

VisionProcessor::VisionProcessor() {
  detector_.configure(roiX_, roiY_, roiWidth_, roiHeight_, threshold_, config::MIN_HEXAGON_SIZE,
                     config::MAX_HEXAGON_SIZE);
}

void VisionProcessor::configureRoi(int x, int y, int width, int height) {
  roiX_ = x;
  roiY_ = y;
  roiWidth_ = width;
  roiHeight_ = height;
  detector_.configure(roiX_, roiY_, roiWidth_, roiHeight_, threshold_, config::MIN_HEXAGON_SIZE,
                     config::MAX_HEXAGON_SIZE);
}

void VisionProcessor::setThreshold(uint8_t threshold) {
  threshold_ = threshold;
  detector_.configure(roiX_, roiY_, roiWidth_, roiHeight_, threshold_, config::MIN_HEXAGON_SIZE,
                     config::MAX_HEXAGON_SIZE);
}

void VisionProcessor::setTolerance(uint16_t pixels) {
  tolerance_ = pixels;
}

void VisionProcessor::setMatchConfirmationFrames(uint8_t frames) {
  confirmationFrames_ = frames;
}

void VisionProcessor::resetMatchState() {
  matchFrames_ = 0;
  lastMatchState_ = false;
}

bool VisionProcessor::processFrame(const uint8_t* frame, uint16_t width, uint16_t height,
                                  VisionResult& result) {
  DetectionSnapshot snapshot;
  const bool detected = detector_.detect(frame, width, height, snapshot);

  result.valid = false;
  result.matchReady = false;
  result.difference = 0;

  if (!detected) {
    if (matchFrames_ > 0) {
      matchFrames_ = 0;
    }
    return false;
  }

  const int difference = snapshot.outer.radius - snapshot.inner.radius;
  const bool matchCondition = difference <= static_cast<int>(tolerance_);

  if (matchCondition) {
    ++matchFrames_;
  } else {
    matchFrames_ = 0;
  }

  result.valid = true;
  result.inner = snapshot.inner;
  result.outer = snapshot.outer;
  result.difference = difference;
  result.frameIndex = snapshot.frameIndex;
  result.matchReady = matchFrames_ >= confirmationFrames_;
  lastMatchState_ = result.matchReady;

  return true;
}
