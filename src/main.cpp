#include <Arduino.h>

#include "camera_manager.h"
#include "calibration.h"
#include "configuration.h"
#include "debug.h"
#include "mouse_hid.h"
#include "vision.h"

namespace {
constexpr uint32_t kStatusDelayMs = 2000;

VisionProcessor gVisionProcessor;
CalibrationState gCalibration;
bool gClickTriggeredThisRound = false;
uint16_t gRoundNumber = 1;
uint32_t gLastSerialStatusAt = 0;
uint32_t gLastFrameAt = 0;
int gPreviousOuterRadius = 0;
int gMatchConfirmationCount = 0;
bool gMatchDetectedThisFrame = false;

void applyCalibration() {
  gVisionProcessor.configureRoi(gCalibration.roiX, gCalibration.roiY, gCalibration.roiWidth,
                               gCalibration.roiHeight);
  gVisionProcessor.setThreshold(gCalibration.brightnessThreshold);
  gVisionProcessor.setTolerance(gCalibration.matchTolerance);
  gVisionProcessor.setMatchConfirmationFrames(gCalibration.matchConfirmationFrames);
}

void handleSerialCommand(const String& command) {
  String normalized = command;
  normalized.trim();
  normalized.toUpperCase();

  if (normalized == "CLICK ON") {
    mouse_hid::setClickEnabled(true);
    Serial.println("CLICKING ENABLED");
    return;
  }

  if (normalized == "CLICK OFF") {
    mouse_hid::setClickEnabled(false);
    Serial.println("CLICKING DISABLED");
    return;
  }

  if (normalized == "STATUS") {
    Serial.print("Click enabled: ");
    Serial.println(mouse_hid::isClickEnabled() ? "YES" : "NO");
    Serial.print("Round: ");
    Serial.println(gRoundNumber);
    Serial.print("Tolerance: ");
    Serial.println(gCalibration.matchTolerance);
    Serial.print("Threshold: ");
    Serial.println(gCalibration.brightnessThreshold);
    return;
  }

  if (normalized == "CALIBRATE") {
    calibration::printStatus(gCalibration);
    return;
  }

  if (normalized.startsWith("DEBUG ")) {
    const String valueText = normalized.substring(6);
    const int level = valueText.toInt();
    if (level >= config::DEBUG_NONE && level <= config::DEBUG_VERBOSE) {
      debug::setLevel(static_cast<config::DebugLevel>(level));
      Serial.print("Debug level set to ");
      Serial.println(level);
    }
    return;
  }

  if (normalized.startsWith("ROI ")) {
    if (calibration::parseCommand(normalized, gCalibration)) {
      applyCalibration();
      Serial.println("ROI updated");
      return;
    }
  }

  if (normalized.startsWith("THRESH ")) {
    if (calibration::parseCommand(normalized, gCalibration)) {
      applyCalibration();
      Serial.println("Threshold updated");
      return;
    }
  }

  if (normalized.startsWith("MATCH ")) {
    if (calibration::parseCommand(normalized, gCalibration)) {
      applyCalibration();
      Serial.println("Match tolerance updated");
      return;
    }
  }

  if (normalized.startsWith("CONFIRM ")) {
    if (calibration::parseCommand(normalized, gCalibration)) {
      applyCalibration();
      Serial.println("Confirmation window updated");
      return;
    }
  }

  if (normalized == "HELP") {
    debug::printSerialHelp();
  }
}

void printPeriodicStatus(uint32_t now) {
  if (now - gLastSerialStatusAt < kStatusDelayMs) {
    return;
  }
  gLastSerialStatusAt = now;
  camera_manager::printStatus();
  mouse_hid::printStatus();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  debug::printSerialHelp();
  mouse_hid::initialize();
  mouse_hid::setClickEnabled(false);

  calibration::resetToDefaults(gCalibration);
  applyCalibration();

  if (!camera_manager::initialize()) {
    Serial.println("Camera initialization failed. Check camera wiring and pin configuration.");
    while (true) {
      delay(1000);
    }
  }

  debug::printCameraStartupSummary();
  Serial.println("System ready. Clicking is disabled by default. Use 'CLICK ON' to allow USB mouse clicks.");
  gLastFrameAt = millis();
}

void loop() {
  while (Serial.available() > 0) {
    const String command = Serial.readStringUntil('\n');
    handleSerialCommand(command);
  }

  const uint32_t startTimestamp = millis();
  const bool frameReady = camera_manager::captureFrame();
  if (!frameReady) {
    delay(5);
    return;
  }

  uint8_t* frame = camera_manager::frameBuffer();
  if (frame == nullptr) {
    delay(5);
    return;
  }

  VisionResult result;
  const bool detected = gVisionProcessor.processFrame(frame, config::CAMERA_WIDTH, config::CAMERA_HEIGHT,
                                                     result);

  const uint32_t processingTimeMs = millis() - startTimestamp;
  camera_manager::setProcessingTiming(processingTimeMs);

  if (detected) {
    debug::printDetectionSummary(result.frameIndex, result.outer.radius, result.inner.radius,
                                result.difference, gCalibration.matchConfirmationFrames,
                                result.matchReady, mouse_hid::isClickEnabled(), gRoundNumber);

    if (result.outer.radius > gPreviousOuterRadius + 12 && gPreviousOuterRadius != 0) {
      gClickTriggeredThisRound = false;
      gRoundNumber++;
    }

    if (result.matchReady && !gClickTriggeredThisRound) {
      gMatchConfirmationCount++;
      if (gMatchConfirmationCount >= 1) {
        if (mouse_hid::isClickEnabled()) {
          Serial.println("TRIGGERED CLICK");
          mouse_hid::clickLeft(config::CLICK_DURATION_MS);
        } else {
          Serial.println("Match found but click is disabled.");
        }
        gClickTriggeredThisRound = true;
      }
    } else if (!result.matchReady) {
      gMatchConfirmationCount = 0;
    }

    gPreviousOuterRadius = result.outer.radius;
    gMatchDetectedThisFrame = result.matchReady;
  } else {
    gMatchDetectedThisFrame = false;
    gMatchConfirmationCount = 0;
  }

  printPeriodicStatus(millis());
  const float cameraFps = camera_manager::cameraFps();
  const float processingFps = camera_manager::processingFps();
  static uint32_t lastMetricsPrint = 0;
  if (millis() - lastMetricsPrint > config::DEBUG_PRINT_INTERVAL_MS) {
    lastMetricsPrint = millis();
    debug::printFrameMetrics(cameraFps, processingFps, camera_manager::averageProcessingMs(),
                             camera_manager::maximumProcessingMs(), camera_manager::frameCount());
  }

  if (gClickTriggeredThisRound && !detected) {
    gClickTriggeredThisRound = false;
    gRoundNumber++;
  }
}
