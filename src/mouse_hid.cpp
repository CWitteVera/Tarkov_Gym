#include "mouse_hid.h"

#include <USB.h>
#include <USBHIDMouse.h>

namespace {
bool gClickEnabled = false;
USBHIDMouse gMouse;
}

namespace mouse_hid {
void initialize() {
  USB.begin();
  gMouse.begin();
  gClickEnabled = false;
}

void setClickEnabled(bool enabled) {
  gClickEnabled = enabled;
}

bool isClickEnabled() {
  return gClickEnabled;
}

void clickLeft(uint32_t durationMs) {
  if (!gClickEnabled) {
    return;
  }
  gMouse.press(MOUSE_LEFT);
  delay(durationMs);
  gMouse.release(MOUSE_LEFT);
}

void printStatus() {
  Serial.print("Click enabled: ");
  Serial.println(gClickEnabled ? "YES" : "NO");
}
}  // namespace mouse_hid
