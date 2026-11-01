#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <time.h>

namespace DisplayManager {

struct Status {
    bool timeValid = false;
    struct tm localTime = {};

    bool mpuReady = false;
    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;

    bool wifiConnected = false;
    bool ntpSynced = false;

    const char *ipAddress = "-";
    const char *modeName = "WATER";

    uint8_t page = 0;
    uint8_t brightness = 5;

    bool externalPowerPresent = false;
    int batteryPercent = -1;    
};

void begin();
void showStartupMessage(const char *message);
void logMessage(const char *format, ...);
void update(const Status &status);


}

#endif