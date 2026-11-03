#include "bluetooth_manager.h"
#include "display_manager.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <string.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace BluetoothManager {
namespace {

// ------------------------------------------------------------
// BLE DEVICE CONFIGURATION
// ------------------------------------------------------------

constexpr char DEVICE_NAME[] = "WaterMatrixClock";

constexpr char SERVICE_UUID[] =
    "4fafc201-1fb5-459e-8fcc-c5c9c331914b";

constexpr char COMMAND_UUID[] =
    "beb5483e-36e1-4688-b7f5-ea07361b26a8";

constexpr char STATUS_UUID[] =
    "e3223119-9445-4e96-a4a1-85358c4046a2";

// New characteristics for Wi-Fi provisioning.
constexpr char WIFI_SSID_UUID[] =
    "3f7a1001-8c42-4c6b-9a11-2f6b4d8e1001";

constexpr char WIFI_PASSWORD_UUID[] =
    "3f7a1002-8c42-4c6b-9a11-2f6b4d8e1001";

constexpr size_t MAX_COMMAND_LENGTH = 47;
constexpr size_t MAX_SSID_LENGTH = 32;
constexpr size_t MAX_PASSWORD_LENGTH = 63;


// ------------------------------------------------------------
// BLE OBJECTS
// ------------------------------------------------------------

BLEServer *server = nullptr;

BLECharacteristic *commandCharacteristic = nullptr;
BLECharacteristic *statusCharacteristic = nullptr;

BLECharacteristic *ssidCharacteristic = nullptr;
BLECharacteristic *passwordCharacteristic = nullptr;


// ------------------------------------------------------------
// BLE STATE
// ------------------------------------------------------------

bool advertisingEnabled = false;
bool previouslyConnected = false;


// ------------------------------------------------------------
// COMMAND BUFFER
// ------------------------------------------------------------

// Commands are transferred from the BLE callback to general_run().
// This prevents application logic from executing inside BLE callbacks.
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;

char pendingCommand[MAX_COMMAND_LENGTH + 1] = {};

bool commandPending = false;


// ------------------------------------------------------------
// WIFI CREDENTIAL STAGING
// ------------------------------------------------------------

// Wi-Fi credentials are received independently through two
// BLE characteristics before WIFI_CONNECT is submitted.

char stagedSsid[MAX_SSID_LENGTH + 1] = {};
char stagedPassword[MAX_PASSWORD_LENGTH + 1] = {};

bool ssidReceived = false;
bool passwordReceived = false;
bool credentialInputInvalid = false;


// ------------------------------------------------------------
// COMMAND QUEUE
// ------------------------------------------------------------

void enqueueCommand(const char *data, size_t length)
{
    if (data == nullptr || length == 0) {
        return;
    }

    // Remove an optional trailing null character.
    while (length > 0 && data[length - 1] == '\0') {
        --length;
    }

    if (length == 0) {
        return;
    }

    if (length > MAX_COMMAND_LENGTH) {
        length = MAX_COMMAND_LENGTH;
    }

    portENTER_CRITICAL(&commandMux);

    memcpy(pendingCommand, data, length);
    pendingCommand[length] = '\0';

    commandPending = true;

    portEXIT_CRITICAL(&commandMux);
}


// ------------------------------------------------------------
// WIFI SSID STAGING
// ------------------------------------------------------------

void stageSsid(const char *data, size_t length)
{
    if (data == nullptr) {
        return;
    }

    // Remove an optional null terminator.
    while (length > 0 && data[length - 1] == '\0') {
        --length;
    }

    const bool valid =
        length > 0 && length <= MAX_SSID_LENGTH;

    portENTER_CRITICAL(&commandMux);

    // A new SSID starts a new credential transaction.
    memset(stagedSsid, 0, sizeof(stagedSsid));
    memset(stagedPassword, 0, sizeof(stagedPassword));

    ssidReceived = false;
    passwordReceived = false;

    credentialInputInvalid = !valid;

    if (valid)
    {
        memcpy(stagedSsid, data, length);
        stagedSsid[length] = '\0';

        ssidReceived = true;
    }

    portEXIT_CRITICAL(&commandMux);
}


// ------------------------------------------------------------
// WIFI PASSWORD STAGING
// ------------------------------------------------------------

void stagePassword(const char *data, size_t length)
{
    if (data == nullptr) {
        return;
    }

    // Remove an optional null terminator.
    while (length > 0 && data[length - 1] == '\0') {
        --length;
    }

    // An empty password is allowed for open Wi-Fi networks.
    const bool valid = length <= MAX_PASSWORD_LENGTH;

    portENTER_CRITICAL(&commandMux);

    memset(stagedPassword, 0, sizeof(stagedPassword));

    if (valid)
    {
        memcpy(stagedPassword, data, length);
        stagedPassword[length] = '\0';

        passwordReceived = true;
    }
    else
    {
        passwordReceived = false;
        credentialInputInvalid = true;
    }

    portEXIT_CRITICAL(&commandMux);
}


// ------------------------------------------------------------
// BLE COMMAND CALLBACK
// ------------------------------------------------------------

class CommandCallbacks final : public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic *characteristic) override
    {
        const String value = characteristic->getValue();

        enqueueCommand(
            value.c_str(),
            value.length()
        );
    }
};


// ------------------------------------------------------------
// BLE SSID CALLBACK
// ------------------------------------------------------------

class SsidCallbacks final : public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic *characteristic) override
    {
        const String value = characteristic->getValue();

        stageSsid(
            value.c_str(),
            value.length()
        );
    }
};


// ------------------------------------------------------------
// BLE PASSWORD CALLBACK
// ------------------------------------------------------------

class PasswordCallbacks final : public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic *characteristic) override
    {
        const String value = characteristic->getValue();

        stagePassword(
            value.c_str(),
            value.length()
        );
    }
};

} // namespace


// ------------------------------------------------------------
// PUBLIC API
// ------------------------------------------------------------

bool begin()
{
    BLEDevice::init(DEVICE_NAME);

    // Allow the app to request a larger MTU for Wi-Fi credentials.
    BLEDevice::setMTU(100);

    server = BLEDevice::createServer();

    if (server == nullptr)
    {
        DisplayManager::logMessage("BLE: server creation failed");
        return false;
    }

    BLEService *service =
        server->createService(SERVICE_UUID);

    if (service == nullptr)
    {
        DisplayManager::logMessage("BLE: service creation failed");
        return false;
    }


    // --------------------------------------------------------
    // COMMAND CHARACTERISTIC
    // --------------------------------------------------------

    commandCharacteristic = service->createCharacteristic(
        COMMAND_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
    );

    commandCharacteristic->setValue(String("MODE=WATER"));

    commandCharacteristic->setCallbacks(
        new CommandCallbacks()
    );


    // --------------------------------------------------------
    // STATUS CHARACTERISTIC
    // --------------------------------------------------------

    statusCharacteristic = service->createCharacteristic(
        STATUS_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    statusCharacteristic->setValue(String("booting"));


    // --------------------------------------------------------
    // WIFI SSID CHARACTERISTIC
    // --------------------------------------------------------

    ssidCharacteristic = service->createCharacteristic(
        WIFI_SSID_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );

    ssidCharacteristic->setCallbacks(
        new SsidCallbacks()
    );


    // --------------------------------------------------------
    // WIFI PASSWORD CHARACTERISTIC
    // --------------------------------------------------------

    passwordCharacteristic = service->createCharacteristic(
        WIFI_PASSWORD_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );

    passwordCharacteristic->setCallbacks(
        new PasswordCallbacks()
    );


    // --------------------------------------------------------
    // START BLE SERVICE
    // --------------------------------------------------------

    service->start();


    // --------------------------------------------------------
    // CONFIGURE ADVERTISING
    // --------------------------------------------------------

    BLEAdvertising *advertising =
        BLEDevice::getAdvertising();

    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);

    // Do not advertise immediately at startup.
    advertisingEnabled = false;
    previouslyConnected = false;

    DisplayManager::logMessage(
        "BLE: initialized as %s; advertising disabled",
        DEVICE_NAME
    );

    return true;
}


// ------------------------------------------------------------
// BLE ADVERTISING CONTROL
// ------------------------------------------------------------

void setEnabled(bool enabled)
{
    if (server == nullptr) {
        return;
    }

    if (advertisingEnabled == enabled) {
        return;
    }

    advertisingEnabled = enabled;

    if (enabled)
    {
        previouslyConnected = false;

        BLEDevice::startAdvertising();

        DisplayManager::logMessage(
            "BLE: advertising enabled"
        );
    }
    else
    {
        BLEAdvertising *advertising =
            BLEDevice::getAdvertising();

        if (advertising != nullptr) {
            advertising->stop();
        }

        DisplayManager::logMessage(
            "BLE: advertising disabled"
        );
    }
}


bool isEnabled()
{
    return advertisingEnabled;
}


// ------------------------------------------------------------
// BLE UPDATE
// ------------------------------------------------------------

void update()
{
    if (server == nullptr || !advertisingEnabled) {
        return;
    }

    const bool connected =
        server->getConnectedCount() > 0;

    if (connected)
    {
        previouslyConnected = true;
        return;
    }

    // Restart advertising after a connected client disconnects.
    if (previouslyConnected)
    {
        previouslyConnected = false;

        BLEDevice::startAdvertising();

        DisplayManager::logMessage(
            "BLE: restarting advertising after disconnect"
        );
    }
}


// ------------------------------------------------------------
// CONNECTION STATUS
// ------------------------------------------------------------

bool isConnected()
{
    return (server != nullptr) &&
           (server->getConnectedCount() > 0);
}


// ------------------------------------------------------------
// DEVICE STATUS NOTIFICATION
// ------------------------------------------------------------

void setStatus(
    float pitchDeg,
    float rollDeg,
    bool wifiConnected,
    bool timeValid,
    const char *modeName,
    uint8_t brightness
)
{
    // Do not generate or update application status data
    // when no BLE client is connected.
    if (statusCharacteristic == nullptr || !isConnected()) {
        return;
    }

    char status[100];

    snprintf(
        status,
        sizeof(status),
        "P=%.1f,R=%.1f,WIFI=%d,TIME=%d,MODE=%s,BRIGHT=%u",
        pitchDeg,
        rollDeg,
        wifiConnected ? 1 : 0,
        timeValid ? 1 : 0,
        (modeName != nullptr) ? modeName : "?",
        brightness
    );

    // Include a null terminator for the app's string notification reader.
    statusCharacteristic->setValue(
        (uint8_t *)status,
        strlen(status) + 1
    );

    // Send status only to a connected BLE client.
    statusCharacteristic->notify();
}


// ------------------------------------------------------------
// TAKE COMMAND
// ------------------------------------------------------------

bool takeCommand(char *out, size_t outSize)
{
    if (out == nullptr || outSize == 0) {
        return false;
    }

    bool received = false;

    portENTER_CRITICAL(&commandMux);

    if (commandPending)
    {
        strncpy(
            out,
            pendingCommand,
            outSize - 1
        );

        out[outSize - 1] = '\0';

        commandPending = false;
        received = true;
    }

    portEXIT_CRITICAL(&commandMux);

    return received;
}


// ------------------------------------------------------------
// TAKE WIFI CREDENTIALS
// ------------------------------------------------------------

bool takeWifiCredentials(
    char *ssid,
    size_t ssidSize,
    char *password,
    size_t passwordSize
)
{
    if (ssid == nullptr ||
        password == nullptr ||
        ssidSize < MAX_SSID_LENGTH + 1 ||
        passwordSize < MAX_PASSWORD_LENGTH + 1)
    {
        return false;
    }

    bool valid = false;

    portENTER_CRITICAL(&commandMux);

    if (ssidReceived &&
        passwordReceived &&
        !credentialInputInvalid)
    {
        strncpy(
            ssid,
            stagedSsid,
            ssidSize - 1
        );

        ssid[ssidSize - 1] = '\0';

        strncpy(
            password,
            stagedPassword,
            passwordSize - 1
        );

        password[passwordSize - 1] = '\0';

        valid = true;
    }

    // Clear staging after every WIFI_CONNECT request.
    memset(stagedSsid, 0, sizeof(stagedSsid));
    memset(stagedPassword, 0, sizeof(stagedPassword));

    ssidReceived = false;
    passwordReceived = false;
    credentialInputInvalid = false;

    portEXIT_CRITICAL(&commandMux);

    return valid;
}

} // namespace BluetoothManager