#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>

namespace WifiManager {

    void begin();

    void update();

    bool isConnected();

    bool isTimeValid();

    bool getLocalTime(struct tm &out);

    const char *ipAddress();

    const char *ssid();

    // Save new Wi-Fi credentials and start a connection attempt.
    bool setCredentials(
        const char *ssid,
        const char *password
    );

    // Disable Wi-Fi before ESP32 deep sleep.
void shutdown();

}

#endif