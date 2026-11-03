#include "wifi_manager.h"
#include "display_manager.h"

#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <string.h>
#include <stdio.h>


#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
#else
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif


namespace WifiManager {
namespace {

// ------------------------------------------------------------
// WIFI CONFIGURATION
// ------------------------------------------------------------

constexpr char TIME_ZONE[] = "<+0530>-5:30";

const char *NTP_SERVER_1 = "pool.ntp.org";
const char *NTP_SERVER_2 = "time.google.com";
const char *NTP_SERVER_3 = "time.cloudflare.com";

constexpr size_t MAX_SSID_LENGTH = 32;
constexpr size_t MAX_PASSWORD_LENGTH = 63;


// ------------------------------------------------------------
// ACTIVE WIFI CREDENTIALS
// ------------------------------------------------------------

char activeSsid[MAX_SSID_LENGTH + 1] = {};
char activePassword[MAX_PASSWORD_LENGTH + 1] = {};

bool credentialsAvailable = false;

bool wifiPaused = false;


// ------------------------------------------------------------
// CONNECTION STATE
// ------------------------------------------------------------

uint32_t lastConnectAttemptMs = 0;

bool startRequested = false;
bool ntpConfigured = false;


// ------------------------------------------------------------
// COPY CREDENTIAL
// ------------------------------------------------------------

bool copyCredential(
    char *destination,
    size_t destinationSize,
    const char *source,
    size_t maximumLength
)
{
    if (destination == nullptr ||
        source == nullptr ||
        destinationSize == 0)
    {
        return false;
    }

    const size_t length = strlen(source);

    if (length > maximumLength ||
        length >= destinationSize)
    {
        return false;
    }

    memcpy(destination, source, length);
    destination[length] = '\0';

    return true;
}


// ------------------------------------------------------------
// CHECK WIFI CREDENTIALS
// ------------------------------------------------------------

bool hasCredentials()
{
    return credentialsAvailable &&
           activeSsid[0] != '\0';
}


// ------------------------------------------------------------
// LOAD SAVED WIFI CREDENTIALS
// ------------------------------------------------------------

/*
 * LOAD WIFI CREDENTIALS
 *
 * Priority:
 *
 * 1. Previously saved credentials from ESP32 NVS.
 * 2. Compile-time credentials from arduino_secrets.h.
 *
 * This allows the device to remember its Wi-Fi network
 * after a restart.
 */
void loadCredentials()
{
    memset(activeSsid, 0, sizeof(activeSsid));
    memset(activePassword, 0, sizeof(activePassword));

    credentialsAvailable = false;

    // --------------------------------------------------------
    // 1. LOAD SAVED NVS CREDENTIALS
    // --------------------------------------------------------

    Preferences preferences;

    if (preferences.begin("wmclock", true))
    {
        String storedSsid =
            preferences.getString("ssid", "");

        String storedPassword =
            preferences.getString("pass", "");

        preferences.end();

        if (storedSsid.length() > 0 &&
            storedSsid.length() <= MAX_SSID_LENGTH &&
            storedPassword.length() <= MAX_PASSWORD_LENGTH)
        {
            storedSsid.toCharArray(
                activeSsid,
                sizeof(activeSsid)
            );

            storedPassword.toCharArray(
                activePassword,
                sizeof(activePassword)
            );

            credentialsAvailable = true;

            DisplayManager::logMessage(
                "WiFi: saved credentials loaded"
            );

            return;
        }
    }


    // --------------------------------------------------------
    // 2. FALLBACK TO COMPILE-TIME CREDENTIALS
    // --------------------------------------------------------

    if (strlen(WIFI_SSID) > 0)
    {
        const bool ssidValid = copyCredential(
            activeSsid,
            sizeof(activeSsid),
            WIFI_SSID,
            MAX_SSID_LENGTH
        );

        const bool passwordValid = copyCredential(
            activePassword,
            sizeof(activePassword),
            WIFI_PASSWORD,
            MAX_PASSWORD_LENGTH
        );

        if (ssidValid && passwordValid)
        {
            credentialsAvailable = true;

            DisplayManager::logMessage(
                "WiFi: compile-time credentials loaded"
            );
        }
    }
}

} // namespace


// ------------------------------------------------------------
// PUBLIC API
// ------------------------------------------------------------

void begin()
{
    wifiPaused = false;

    WiFi.mode(WIFI_STA);

    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    // Load saved or compile-time credentials.
    loadCredentials();

    if (!hasCredentials())
    {
        DisplayManager::logMessage(
            "WiFi: no saved credentials; waiting for provisioning"
        );

        return;
    }

    // Begin connecting without blocking general_run().
    WiFi.begin(
        activeSsid,
        activePassword
    );

    startRequested = true;
    lastConnectAttemptMs = millis();

    DisplayManager::logMessage(
        "WiFi: connecting to '%s' in background",
        activeSsid
    );
}


// ------------------------------------------------------------
// UPDATE WIFI CONNECTION
// ------------------------------------------------------------

void update()
{
    const uint32_t now = millis();


    if (wifiPaused) {
        return;
    }

    // --------------------------------------------------------
    // WIFI CONNECTED
    // --------------------------------------------------------

    if (WiFi.status() == WL_CONNECTED)
    {
        if (!ntpConfigured)
        {
            configTzTime(
                TIME_ZONE,
                NTP_SERVER_1,
                NTP_SERVER_2,
                NTP_SERVER_3
            );

            ntpConfigured = true;

            DisplayManager::logMessage(
                "WiFi: connected; NTP synchronization started"
            );
        }

        return;
    }


    // --------------------------------------------------------
    // WIFI DISCONNECTED
    // --------------------------------------------------------

    // Keep reconnect attempts asynchronous.
    if (hasCredentials() &&
        (!startRequested ||
         (uint32_t)(now - lastConnectAttemptMs) >= 15000U))
    {
        WiFi.begin(
            activeSsid,
            activePassword
        );

        startRequested = true;
        lastConnectAttemptMs = now;

        DisplayManager::logMessage(
            "WiFi: retrying connection"
        );
    }
}


// ------------------------------------------------------------
// SET WIFI CREDENTIALS
// ------------------------------------------------------------

/*
 * SET WIFI CREDENTIALS
 *
 * Called after receiving WIFI_CONNECT through BLE.
 *
 * Process:
 *
 * 1. Validate SSID and password lengths.
 * 2. Save both values to ESP32 NVS.
 * 3. Replace the active credentials.
 * 4. Disconnect from the previous network.
 * 5. Start a new asynchronous connection attempt.
 *
 * The password is never printed to the debug log.
 */
bool setCredentials(
    const char *ssid,
    const char *password
)
{
    if (ssid == nullptr || password == nullptr) {
        return false;
    }

    const size_t ssidLength = strlen(ssid);
    const size_t passwordLength = strlen(password);

    if (ssidLength == 0 ||
        ssidLength > MAX_SSID_LENGTH ||
        passwordLength > MAX_PASSWORD_LENGTH)
    {
        DisplayManager::logMessage(
            "WiFi: invalid credential length"
        );

        return false;
    }


    // --------------------------------------------------------
    // SAVE CREDENTIALS TO NVS
    // --------------------------------------------------------

    Preferences preferences;

    if (!preferences.begin("wmclock", false))
    {
        DisplayManager::logMessage(
            "WiFi: unable to open credential storage"
        );

        return false;
    }

    const size_t ssidStored =
        preferences.putString("ssid", ssid);

    const size_t passwordStored =
        preferences.putString("pass", password);

    preferences.end();

    if (ssidStored == 0 || passwordStored == 0)
    {
        DisplayManager::logMessage(
            "WiFi: credential storage failed"
        );

        return false;
    }


    // --------------------------------------------------------
    // UPDATE ACTIVE CREDENTIALS
    // --------------------------------------------------------

    if (!copyCredential(
            activeSsid,
            sizeof(activeSsid),
            ssid,
            MAX_SSID_LENGTH))
    {
        return false;
    }

    if (!copyCredential(
            activePassword,
            sizeof(activePassword),
            password,
            MAX_PASSWORD_LENGTH))
    {
        return false;
    }

    credentialsAvailable = true;


    // --------------------------------------------------------
    // START NEW WIFI CONNECTION
    // --------------------------------------------------------

    wifiPaused = false;

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    // Disconnect from the previous network without powering off Wi-Fi.
    WiFi.disconnect(false, false);

    // Reset NTP configuration so it is initialized after reconnection.
    ntpConfigured = false;

    WiFi.begin(
        activeSsid,
        activePassword
    );

    startRequested = true;
    lastConnectAttemptMs = millis();

    DisplayManager::logMessage(
        "WiFi: new credentials saved; connection attempt started"
    );

    return true;
}


// ------------------------------------------------------------
// CONNECTION INFORMATION
// ------------------------------------------------------------

bool isConnected()
{
    return WiFi.status() == WL_CONNECTED;
}


bool isTimeValid()
{
    // System time remains valid even while Wi-Fi is disabled,
    // provided it was synchronized previously.
    const time_t now = time(nullptr);

    return now > 1700000000;
}


bool getLocalTime(struct tm &out)
{
    if (!isTimeValid()) {
        return false;
    }

    const time_t now = time(nullptr);

    return localtime_r(&now, &out) != nullptr;
}


const char *ipAddress()
{
    static char ip[16];

    if (!isConnected()) {
        return "-";
    }

    const IPAddress addr = WiFi.localIP();

    snprintf(
        ip,
        sizeof(ip),
        "%u.%u.%u.%u",
        addr[0],
        addr[1],
        addr[2],
        addr[3]
    );

    return ip;
}


const char *ssid()
{
    static char currentSsid[MAX_SSID_LENGTH + 1];

    if (!isConnected()) {
        return "-";
    }

    const String connectedSsid = WiFi.SSID();

    connectedSsid.toCharArray(
        currentSsid,
        sizeof(currentSsid)
    );

    return currentSsid;
}

// ------------------------------------------------------------
// WIFI SHUTDOWN
// ------------------------------------------------------------

void shutdown()
{
    if (wifiPaused) {
        return;
    }

    wifiPaused = true;

    // Disconnect and turn off the Wi-Fi radio.
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);

    startRequested = false;

    // NTP time already synchronized remains available through
    // the ESP32 system clock. We will configure NTP again when
    // Wi-Fi is resumed.
    ntpConfigured = false;

    DisplayManager::logMessage(
        "WiFi: radio disabled for idle mode"
    );
}


// ------------------------------------------------------------
// WIFI RESUME
// ------------------------------------------------------------

void resume()
{
    if (!wifiPaused) {
        return;
    }

    wifiPaused = false;

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    if (!hasCredentials()) {
        DisplayManager::logMessage(
            "WiFi: no credentials available after resume"
        );

        return;
    }

    WiFi.begin(
        activeSsid,
        activePassword
    );

    startRequested = true;
    lastConnectAttemptMs = millis();

    DisplayManager::logMessage(
        "WiFi: reconnecting after idle mode"
    );
}

} // namespace WifiManager