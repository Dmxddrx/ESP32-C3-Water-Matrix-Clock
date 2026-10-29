#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <time.h>

namespace WifiManager {
void begin();
void update();
bool isConnected();
bool isTimeValid();
bool getLocalTime(struct tm &out);
const char *ipAddress();
const char *ssid();
}

#endif
