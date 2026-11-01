#pragma once

#include <Arduino.h>

namespace SandClock {

// Call once after MAX7219_Init(). Starts with the top chamber full.
void begin();

// Reset the hourglass and generate a new random source/destination pairing.
void reset(uint32_t nowMs);

// Call repeatedly from the main loop while LEVEL mode is active.
void update(uint32_t nowMs);

} // namespace SandClock
