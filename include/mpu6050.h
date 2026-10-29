#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <Arduino.h>

namespace MPU6050 {
struct Data {
    int16_t axRaw = 0, ayRaw = 0, azRaw = 0;
    int16_t temperatureRaw = 0;
    int16_t gxRaw = 0, gyRaw = 0, gzRaw = 0;
    float axG = 0.0f, ayG = 0.0f, azG = 0.0f;
    float gxDps = 0.0f, gyDps = 0.0f, gzDps = 0.0f;
    float temperatureC = 0.0f;
    float pitchDeg = 0.0f;
    float rollDeg = 0.0f;
    uint32_t timestampMs = 0;
    bool valid = false;
};

bool begin();
void update();
bool isReady();
uint8_t address();
const Data &getData();
}

#endif
