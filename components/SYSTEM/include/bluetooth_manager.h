#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <Arduino.h>
#include <stddef.h>

namespace BluetoothManager {
bool begin();
void update();
bool isConnected();
void setStatus(float pitchDeg, float rollDeg, bool wifiConnected,
               bool timeValid, const char *modeName, uint8_t brightness);
bool takeCommand(char *out, size_t outSize);
}

#endif
