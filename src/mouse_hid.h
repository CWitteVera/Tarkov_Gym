#pragma once

#include <Arduino.h>

namespace mouse_hid {
void initialize();
void setClickEnabled(bool enabled);
bool isClickEnabled();
void clickLeft(uint32_t durationMs = 30);
void printStatus();
}  // namespace mouse_hid
