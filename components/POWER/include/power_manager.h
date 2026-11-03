#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include "mpu6050.h"

namespace PowerManager {

    // Initialize inactivity and motion tracking.
    void begin(uint32_t nowMs);

    // Call when a physical button or remote command is used.
    void notifyUserActivity(uint32_t nowMs);

    // Call when the user manually changes the OLED page.
    // This also refreshes the inactivity timer.
    void notifyPageInteraction(uint32_t nowMs);

    // Check whether the device is in display-idle mode.
    bool isIdle();
    
    // Update motion detection, automatic page return and sleep.
    void update(
        uint32_t nowMs,
        uint8_t &displayPage,
        const MPU6050::Data &imu
    );

    
}

#endif