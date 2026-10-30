#include "wifi_manager.h"
#include "display_manager.h"
#include <WiFi.h>
#include <time.h>
#include <string.h>

#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
#else
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif

namespace WifiManager {
namespace {
constexpr char TIME_ZONE[] = "<+0530>-5:30"; // Sri Lanka Standard Time (UTC+05:30)
const char *NTP_SERVER_1 = "pool.ntp.org";
const char *NTP_SERVER_2 = "time.google.com";
const char *NTP_SERVER_3 = "time.cloudflare.com";
uint32_t lastConnectAttemptMs = 0;
bool startRequested = false;
bool ntpConfigured = false;

bool hasCredentials()
{
    return WIFI_SSID[0] != '\0';
}
}

void begin()
{
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    if (!hasCredentials()) {
        DisplayManager::logMessage("WiFi: add local arduino_secrets.h with WIFI_SSID/WIFI_PASSWORD");
        return;
    }
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    startRequested = true;
    lastConnectAttemptMs = millis();
    DisplayManager::logMessage("WiFi: connecting to '%s' in background...\n", WIFI_SSID);
}

void update()
{
    const uint32_t now = millis();
    if (WiFi.status() == WL_CONNECTED) {
        if (!ntpConfigured) {
            configTzTime(TIME_ZONE, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
            ntpConfigured = true;
            DisplayManager::logMessage("WiFi: connected; NTP synchronization started");
        }
        return;
    }

    // Keep reconnect attempts asynchronous; no long waits in the main loop.
    if (hasCredentials() && (!startRequested || (uint32_t)(now - lastConnectAttemptMs) >= 15000U)) {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        startRequested = true;
        lastConnectAttemptMs = now;
        DisplayManager::logMessage("WiFi: retrying connection");
    }
}

bool isConnected() { return WiFi.status() == WL_CONNECTED; }

bool isTimeValid()
{
    if (!ntpConfigured) return false;
    const time_t now = time(nullptr);
    // Once synchronized, system time continues while Wi-Fi is temporarily offline.
    return now > 1700000000; // Reject an unset/reset clock near January 1970.
}

bool getLocalTime(struct tm &out)
{
    if (!isTimeValid()) return false;
    const time_t now = time(nullptr);
    return localtime_r(&now, &out) != nullptr;
}

const char *ipAddress()
{
    static char ip[16];
    if (!isConnected()) return "-";
    const IPAddress addr = WiFi.localIP();
    snprintf(ip, sizeof(ip), "%u.%u.%u.%u", addr[0], addr[1], addr[2], addr[3]);
    return ip;
}

const char *ssid()
{
    return isConnected() ? WIFI_SSID : "-";
}
}
