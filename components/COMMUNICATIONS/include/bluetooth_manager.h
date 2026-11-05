#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <stdint.h>
#include <stddef.h>

namespace BluetoothManager {

    // Initialize the BLE service and characteristics.
    // Advertising remains disabled until explicitly enabled.
    bool begin();

    // Maintain BLE advertising after a client disconnects.
    void update();

    // Enable or disable BLE advertising.
    void setEnabled(bool enabled);

    // Check whether BLE advertising has been enabled.
    bool isEnabled();

    // Check whether a phone is currently connected.
    bool isConnected();

    // Update the status characteristic.
    void setStatus(
        float pitchDeg,
        float rollDeg,
        bool wifiConnected,
        bool timeValid,
        const char *modeName,
        uint8_t brightness
    );

    enum class WifiProvisionState : uint8_t {
        WAITING,
        SSID_RECEIVED,
        PASSWORD_RECEIVED,
        CONNECTING,
        CONNECTED,
        ERROR
    };

    // Track the progress of Wi-Fi provisioning over BLE.
    void setWifiProvisionState(WifiProvisionState state);
    WifiProvisionState getWifiProvisionState();
    const char *wifiProvisionStateText();

    // Retrieve a command written by the connected phone.
    bool takeCommand(char *out, size_t outSize);

    // Retrieve the staged Wi-Fi credentials.
    // Both SSID and password must have been received.
    bool takeWifiCredentials(
        char *ssid,
        size_t ssidSize,
        char *password,
        size_t passwordSize
    );

}

#endif