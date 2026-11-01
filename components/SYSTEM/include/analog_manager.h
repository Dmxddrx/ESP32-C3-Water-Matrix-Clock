
#pragma once

#include <Arduino.h>

namespace AnalogManager {

    // Hardware configuration
    constexpr uint8_t BATTERY_PIN = 1;       // GPIO1 / ADC1_CH1
    constexpr uint8_t POWER_DETECT_PIN = 0;  // GPIO0

    // Battery divider: equal resistors, midpoint output
    constexpr float BATTERY_DIVIDER_RATIO = 2.0f;

    void begin();

    uint32_t readBatteryAdcMilliVolts();
    float readBatteryVoltage();

    uint8_t voltageToPercentage(float voltage);

    bool isExternalPowerPresent();
}
  