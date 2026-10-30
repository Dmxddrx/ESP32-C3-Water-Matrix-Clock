#include "bluetooth_manager.h"
#include "display_manager.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace BluetoothManager {
namespace {
constexpr char DEVICE_NAME[] = "WaterMatrixClock";
constexpr char SERVICE_UUID[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char COMMAND_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
constexpr char STATUS_UUID[]  = "e3223119-9445-4e96-a4a1-85358c4046a2";

BLEServer *server = nullptr;
BLECharacteristic *commandCharacteristic = nullptr;
BLECharacteristic *statusCharacteristic = nullptr;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
char pendingCommand[48] = {0};
bool commandPending = false;
uint32_t lastAdvertiseCheckMs = 0;

void enqueueCommand(const char *data, size_t length)
{
    if (data == nullptr || length == 0) return;
    if (length >= sizeof(pendingCommand)) length = sizeof(pendingCommand) - 1;
    portENTER_CRITICAL(&commandMux);
    memcpy(pendingCommand, data, length);
    pendingCommand[length] = '\0';
    commandPending = true;
    portEXIT_CRITICAL(&commandMux);
}

class CommandCallbacks final : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override
    {
        const String value = characteristic->getValue();
        enqueueCommand(value.c_str(), value.length());
    }
};
}

bool begin()
{
    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setMTU(100);
    server = BLEDevice::createServer();
    if (server == nullptr) {
        DisplayManager::logMessage("BLE: server creation failed");
        return false;
    }
    BLEService *service = server->createService(SERVICE_UUID);
    if (service == nullptr) {
        DisplayManager::logMessage("BLE: service creation failed");
        return false;
    }

    commandCharacteristic = service->createCharacteristic(
        COMMAND_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    commandCharacteristic->setValue(String("MODE=WATER"));
    commandCharacteristic->setCallbacks(new CommandCallbacks());

    statusCharacteristic = service->createCharacteristic(
        STATUS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    statusCharacteristic->addDescriptor(new BLE2902());
    statusCharacteristic->setValue(String("booting"));

    service->start();
    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->start();
    DisplayManager::logMessage(("BLE: advertising as %s"), DEVICE_NAME);
    return true;
}

void update()
{
    // Restart advertising after a client disconnects, without requiring callbacks
    // whose signatures differ between ESP32 Arduino core releases.
    const uint32_t now = millis();
    if (server != nullptr && server->getConnectedCount() == 0 &&
        (uint32_t)(now - lastAdvertiseCheckMs) >= 5000U) {
        lastAdvertiseCheckMs = now;
        BLEDevice::startAdvertising();
    }
}

bool isConnected()
{
    return (server != nullptr) && (server->getConnectedCount() > 0);
}

void setStatus(float pitchDeg, float rollDeg, bool wifiConnected,
               bool timeValid, const char *modeName, uint8_t brightness)
{
    if (statusCharacteristic == nullptr) return;
    char status[100];
    snprintf(status, sizeof(status), "P=%.1f,R=%.1f,WIFI=%d,TIME=%d,MODE=%s,BRIGHT=%u",
             pitchDeg, rollDeg, wifiConnected ? 1 : 0, timeValid ? 1 : 0,
             (modeName != nullptr) ? modeName : "?", brightness);
    statusCharacteristic->setValue(String(status));
    if (isConnected()) statusCharacteristic->notify();
}

bool takeCommand(char *out, size_t outSize)
{
    if (out == nullptr || outSize == 0) return false;
    bool received = false;
    portENTER_CRITICAL(&commandMux);
    if (commandPending) {
        strncpy(out, pendingCommand, outSize - 1);
        out[outSize - 1] = '\0';
        commandPending = false;
        received = true;
    }
    portEXIT_CRITICAL(&commandMux);
    return received;
}
}
