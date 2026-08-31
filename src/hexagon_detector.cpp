#include "hexagon_detector.h"

#include <algorithm>

namespace {
inline int clampToRange(int value, int low, int high) {
  return std::max(low, std::min(value, high));
}
}  // namespace

HexagonDetector::HexagonDetector() = default;

void HexagonDetector::configure(int roiX, int roiY, int roiWidth, int roiHeight, uint8_t threshold,
                                uint16_t minHexagonSize, uint16_t maxHexagonSize) {
  roiX_ = roiX;
  roiY_ = roiY;
  roiWidth_ = roiWidth;
  roiHeight_ = roiHeight;
  threshold_ = threshold;
  minHexagonSize_ = minHexagonSize;
  maxHexagonSize_ = maxHexagonSize;
}

HexagonMeasurement HexagonDetector::findCandidate(const uint8_t* frame, uint16_t width,
                                                  uint16_t height, int roiX, int roiY,
                                                  int roiWidth, int roiHeight,
                                                  const HexagonMeasurement* excludedRegion,
                                                  uint8_t threshold) const {
  if (frame == nullptr || width == 0 || height == 0) {
    return {};
  }

  int minX = width;
  int minY = height;
  int maxX = -1;
  int maxY = -1;
  uint32_t sumX = 0;
  uint32_t sumY = 0;
  uint32_t pixels = 0;

  const int x0 = std::max(0, roiX);
  const int y0 = std::max(0, roiY);
  const int x1 = std::min(static_cast<int>(width), roiX + roiWidth);
  const int y1 = std::min(static_cast<int>(height), roiY + roiHeight);

  for (int y = y0; y < y1; ++y) {
    const int rowIndex = y * width;
    for (int x = x0; x < x1; ++x) {
      if (frame[rowIndex + x] < threshold) {
        continue;
      }

      if (excludedRegion != nullptr && excludedRegion->valid) {
        const int exclusionRadius = std::max(10, excludedRegion->radius + 12);
        const int dx = x - excludedRegion->centerX;
        const int dy = y - excludedRegion->centerY;
        if (dx * dx + dy * dy < exclusionRadius * exclusionRadius) {
          continue;
        }
      }

      minX = std::min(minX, x);
      minY = std::min(minY, y);
      maxX = std::max(maxX, x);
      maxY = std::max(maxY, y);
      sumX += x;
      sumY += y;
      pixels++;
    }
  }

  if (pixels < 12) {
    return {};
  }

  const int centerX = static_cast<int>(sumX / pixels);
  const int centerY = static_cast<int>(sumY / pixels);
  const int boxWidth = std::max(1, maxX - minX + 1);
  const int boxHeight = std::max(1, maxY - minY + 1);
  const int radius = std::max(boxWidth, boxHeight) / 2;

  if (radius < minHexagonSize_ || radius > maxHexagonSize_) {
    return {};
  }

  HexagonMeasurement candidate;
  candidate.valid = true;
  candidate.centerX = centerX;
  candidate.centerY = centerY;
  candidate.radius = radius;
  candidate.width = boxWidth;
  candidate.height = boxHeight;
  candidate.thickness = std::max(1, std::min(boxWidth, boxHeight) / 18);
  candidate.brightPixelCount = static_cast<int>(pixels);
  return candidate;
}

bool HexagonDetector::detect(const uint8_t* frame, uint16_t width, uint16_t height,
                            DetectionSnapshot& snapshot) {
  const HexagonMeasurement largest = findCandidate(frame, width, height, roiX_, roiY_, roiWidth_, roiHeight_,
                                                  nullptr, threshold_);
  if (!largest.valid) {
    snapshot = {};
    return false;
  }

  const HexagonMeasurement second = findCandidate(frame, width, height, roiX_, roiY_, roiWidth_, roiHeight_,
                                                  &largest, threshold_);

  HexagonMeasurement outer = largest;
  HexagonMeasurement inner = second;
  if (second.valid && second.radius > largest.radius) {
    outer = second;
    inner = largest;
  }

  if (!inner.valid) {
    snapshot = {};
    return false;
  }

  snapshot.inner = inner;
  snapshot.outer = outer;
  snapshot.valid = true;
  snapshot.difference = outer.radius - inner.radius;
  snapshot.frameIndex = millis();
  return true;
}
