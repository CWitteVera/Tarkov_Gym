#pragma once

#include <Arduino.h>

struct HexagonMeasurement {
  bool valid = false;
  int centerX = 0;
  int centerY = 0;
  int radius = 0;
  int width = 0;
  int height = 0;
  int thickness = 1;
  int brightPixelCount = 0;
};

struct DetectionSnapshot {
  HexagonMeasurement inner;
  HexagonMeasurement outer;
  bool valid = false;
  int difference = 0;
  int frameIndex = 0;
};

class HexagonDetector {
 public:
  HexagonDetector();
  void configure(int roiX, int roiY, int roiWidth, int roiHeight, uint8_t threshold,
                 uint16_t minHexagonSize, uint16_t maxHexagonSize);
  bool detect(const uint8_t* frame, uint16_t width, uint16_t height, DetectionSnapshot& snapshot);

 private:
  HexagonMeasurement findCandidate(const uint8_t* frame, uint16_t width, uint16_t height,
                                  int roiX, int roiY, int roiWidth, int roiHeight,
                                  const HexagonMeasurement* excludedRegion,
                                  uint8_t threshold) const;

  int roiX_ = 0;
  int roiY_ = 0;
  int roiWidth_ = 0;
  int roiHeight_ = 0;
  uint8_t threshold_ = 200;
  uint16_t minHexagonSize_ = 18;
  uint16_t maxHexagonSize_ = 220;
};
