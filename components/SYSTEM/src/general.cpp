#include "general.h"
#include "project_config.h"
#include "max7219.h"
#include "icons.h"
#include "mpu6050.h"
#include "display_manager.h"
#include "wifi_manager.h"
#include "bluetooth_manager.h"
#include "analog_manager.h"

#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

namespace {
struct ButtonState {
    bool rawState = HIGH;
    bool stableState = HIGH;
    uint32_t rawChangedMs = 0;
};

ButtonState pageButton;
ButtonState modeButton;
uint8_t displayPage = 0;
uint8_t brightness = ProjectConfig::MATRIX_DEFAULT_BRIGHTNESS;
MatrixMode matrixMode = MatrixMode::HEART;
uint32_t lastMatrixFrameMs = 0;
uint32_t lastDisplayRefreshMs = 0;
uint32_t lastBleStatusMs = 0;
uint32_t frameCounter = 0;

// Power and battery state
bool externalPowerPresent = false;
float batteryVoltage = 0.0f;
int batteryPercent = -1;
bool batteryDataValid = false;
uint32_t lastBatteryUpdateMs = 0;

const char *modeName(MatrixMode mode)
{
    switch (mode) {
        case MatrixMode::WATER: return "WATER";
        case MatrixMode::HEART: return "HEART";
        case MatrixMode::LEVEL: return "LEVEL";
        default: return "UNKNOWN";
    }
}

void cycleMatrixMode()
{
    const uint8_t next = ((uint8_t)matrixMode + 1U) % 3U;
    matrixMode = (MatrixMode)next;
    DisplayManager::logMessage("Matrix mode: %s", modeName(matrixMode));
}

void onButtonPressed(uint8_t pin)
{
    if (pin == ProjectConfig::BUTTON_PAGE_PIN) {
        displayPage = (displayPage + 1U) % 3U;
        DisplayManager::logMessage("Display page: %u", displayPage);
    } else if (pin == ProjectConfig::BUTTON_MODE_PIN) {
        cycleMatrixMode();
    }
}

void updateButton(uint8_t pin, ButtonState &button, uint32_t now)
{
    const bool raw = digitalRead(pin);
    if (raw != button.rawState) {
        button.rawState = raw;
        button.rawChangedMs = now;
    }
    if ((uint32_t)(now - button.rawChangedMs) >= ProjectConfig::BUTTON_DEBOUNCE_MS &&
        button.stableState != button.rawState) {
        button.stableState = button.rawState;
        if (button.stableState == LOW) onButtonPressed(pin);
    }
}

void applyBluetoothCommand(const char *command)
{
    if (command == nullptr || command[0] == '\0') return;

    if (strcmp(command, "MODE=WATER") == 0) matrixMode = MatrixMode::WATER;
    else if (strcmp(command, "MODE=HEART") == 0) matrixMode = MatrixMode::HEART;
    else if (strcmp(command, "MODE=LEVEL") == 0) matrixMode = MatrixMode::LEVEL;
    else if (strncmp(command, "PAGE=", 5) == 0) {
        const int value = atoi(command + 5);
        if (value >= 0 && value <= 2) displayPage = (uint8_t)value;
    } else if (strncmp(command, "BRIGHT=", 7) == 0) {
        const int value = atoi(command + 7);
        if (value >= 0 && value <= 15) {
            brightness = (uint8_t)value;
            MAX7219_SetBrightness(brightness);
        }
    } else if (strcmp(command, "NEXTMODE") == 0) {
        cycleMatrixMode();
    } else {
        DisplayManager::logMessage("BLE command not recognized: %s", command);
        return;
    }
    DisplayManager::logMessage("BLE command applied: %s", command);
}

void drawWaterFrame(uint32_t nowMs)
{
    //MAX7219_Clear();

    const MPU6050::Data &imu = MPU6050::getData();
    const bool imuValid = imu.valid && MPU6050::isReady();

    float pitch = imuValid ? imu.pitchDeg : 0.0f;
    float roll  = imuValid ? imu.rollDeg  : 0.0f;

    // Limit extreme angles to keep the liquid visible in 16x8 pixels.
    if (pitch < -30.0f) pitch = -30.0f;
    if (pitch >  30.0f) pitch =  30.0f;
    if (roll  < -30.0f) roll  = -30.0f;
    if (roll  >  30.0f) roll  =  30.0f;

    const float seconds = nowMs * 0.001f;
    const float radians = roll * 0.01745329252f;

    // Pitch changes the mean fill height.
    const float baseSurface = 4.0f - pitch * 0.035f;

    // Roll tilts the liquid surface across the 16-pixel width.
    const float slope = -tanf(radians) * 0.60f;

    float surfaceRows[16] = {};

    // Draw water inside the tank, leaving space for its walls.
    for (int16_t x = 1; x < 15; ++x) {
        float ripple = 0.0f;

        if (matrixMode == MatrixMode::WATER) {
            ripple = 0.30f * sinf(
                x * 0.75f + seconds * 2.2f
            );
        }

        float surfaceY =
            baseSurface + ((float)x - 7.5f) * slope + ripple;

        if (surfaceY < 1.0f) surfaceY = 1.0f;
        if (surfaceY > 6.0f) surfaceY = 6.0f;

        surfaceRows[x] = surfaceY;

        // Fill the pixels below the local water surface.
        for (int16_t y = 1; y <= 6; ++y) {
            if ((float)y >= surfaceY) {
                MAX7219_SetPixel(x, y, 1);
            }
        }
    }

    // Tank side walls.
    for (int16_t y = 1; y <= 7; ++y) {
        MAX7219_SetPixel(0,  y, 1);
        MAX7219_SetPixel(15, y, 1);
    }

    // Tank bottom.
    for (int16_t x = 0; x < 16; ++x) {
        MAX7219_SetPixel(x, 7, 1);
    }

    // Moving bubbles, only when they lie below the water surface.
    if (matrixMode == MatrixMode::WATER) {
        const int16_t bubbleX1 =
            (int16_t)((frameCounter / 4U) % 12U) + 2;

        const int16_t bubbleY1 =
            2 + (int16_t)((frameCounter / 15U) % 4U);

        const int16_t bubbleX2 =
            14 - (int16_t)((frameCounter / 6U) % 12U);

        const int16_t bubbleY2 =
            2 + (int16_t)((frameCounter / 18U) % 4U);

        if ((float)bubbleY1 >= surfaceRows[bubbleX1]) {
            MAX7219_SetPixel(bubbleX1, bubbleY1, 0);
        }

        if ((float)bubbleY2 >= surfaceRows[bubbleX2]) {
            MAX7219_SetPixel(bubbleX2, bubbleY2, 0);
        }
    }

    //MAX7219_Update();
}

void drawMatrixFrame(uint32_t nowMs)
{
    // Clear the previous frame before drawing a new one.
    MAX7219_Clear();

    const bool heartMode = (matrixMode == MatrixMode::HEART);

    if (heartMode) {

        MAX7219_DrawIcon(&Icon_Heart, 0, 0);
        MAX7219_DrawIcon(&Icon_Heart, 8, 0);

        const uint8_t pulse =
            ((frameCounter / 8U) % 2U)
                ? brightness
                : (brightness > 1 ? brightness - 1 : 1);

        MAX7219_SetBrightness(pulse);

    } else {

        // WATER and LEVEL both use the tank renderer.
        drawWaterFrame(nowMs);
    }

    // Exactly one complete framebuffer transmission per frame.
    MAX7219_Update();

    // Restore normal brightness after the heart pulse.
    if (heartMode) {
        MAX7219_SetBrightness(brightness);
    }

    ++frameCounter;
}

void updateDisplay(uint32_t now)
{
    if ((uint32_t)(now - lastDisplayRefreshMs) < ProjectConfig::DISPLAY_REFRESH_MS) return;
    lastDisplayRefreshMs = now;

    DisplayManager::Status status;
    status.timeValid = WifiManager::getLocalTime(status.localTime);
    status.mpuReady = MPU6050::isReady() && MPU6050::getData().valid;
    status.pitchDeg = MPU6050::getData().pitchDeg;
    status.rollDeg = MPU6050::getData().rollDeg;
    status.wifiConnected = WifiManager::isConnected();
    status.ntpSynced = WifiManager::isTimeValid();
    status.ipAddress = WifiManager::ipAddress();

    status.modeName = modeName(matrixMode);
    status.page = displayPage;
    status.brightness = brightness;

    status.externalPowerPresent = externalPowerPresent;
    status.batteryPercent = batteryPercent;

    DisplayManager::update(status);
}

void updateBleStatus(uint32_t now)
{
    if ((uint32_t)(now - lastBleStatusMs) < ProjectConfig::BLE_STATUS_REFRESH_MS) return;
    lastBleStatusMs = now;
    const MPU6050::Data &imu = MPU6050::getData();
    BluetoothManager::setStatus(imu.valid ? imu.pitchDeg : 0.0f,
                                imu.valid ? imu.rollDeg : 0.0f,
                                WifiManager::isConnected(), WifiManager::isTimeValid(),
                                modeName(matrixMode), brightness);
}

void testMatrixRefresh()
{
    // Stage 1: Completely OFF.
    MAX7219_Clear();
    MAX7219_Update();
    delay(1000);

    // Stage 2: Checkerboard.
    for (int16_t x = 0; x < 16; ++x) {
        for (int16_t y = 0; y < 8; ++y) {
            if (((x + y) & 1) == 0) {
                MAX7219_SetPixel(x, y, 1);
            }
        }
    }

    MAX7219_Update();
    delay(1500);

    // Stage 3: Completely OFF again.
    MAX7219_Clear();
    MAX7219_Update();
    delay(1000);
}
void updatePowerMonitor(uint32_t now)
{
    // GPIO0 is checked continuously.
    externalPowerPresent =
        AnalogManager::isExternalPowerPresent();

    // External power detected.
    if (externalPowerPresent)
    {
        // Do not read GPIO1 while external power is present.
        batteryVoltage = 0.0f;
        batteryPercent = -1;

        batteryDataValid = false;

        return;
    }

    // No external power: read battery once per second.
    if (!batteryDataValid ||
        (uint32_t)(now - lastBatteryUpdateMs) >= 1000)
    {
        lastBatteryUpdateMs = now;

        // GPIO1 is read only in this branch.
        batteryVoltage =
            AnalogManager::readBatteryVoltage();

        batteryPercent =
            AnalogManager::voltageToPercentage(batteryVoltage);

        batteryDataValid = true;

        DisplayManager::logMessage(
            "Battery: %.2fV %d%%",
            batteryVoltage,
            batteryPercent
        );
    }
}
}

void general_init(void)
{
    AnalogManager::begin();

    DisplayManager::logMessage("ESP32-C3 Water Matrix Clock");
    DisplayManager::logMessage("Starting peripherals...");

    pinMode(ProjectConfig::BUTTON_PAGE_PIN, INPUT_PULLUP);
    pinMode(ProjectConfig::BUTTON_MODE_PIN, INPUT_PULLUP);

    const uint32_t now = millis();

    pageButton.rawState = pageButton.stableState =
        digitalRead(ProjectConfig::BUTTON_PAGE_PIN);
    pageButton.rawChangedMs = now;

    modeButton.rawState = modeButton.stableState =
        digitalRead(ProjectConfig::BUTTON_MODE_PIN);
    modeButton.rawChangedMs = now;

    Wire.begin(
        ProjectConfig::I2C_SDA_PIN,
        ProjectConfig::I2C_SCL_PIN,
        ProjectConfig::I2C_FREQUENCY_HZ
    );
    Wire.setTimeOut(50);

    DisplayManager::logMessage("Initializing MAX7219...");
    MAX7219_Init();

    // Use the same brightness variable used by the animation.
    brightness = 8;
    MAX7219_SetBrightness(brightness);

    DisplayManager::logMessage("Initializing OLED...");
    DisplayManager::begin();

    DisplayManager::logMessage("Initializing MPU6050...");
    MPU6050::begin();

    DisplayManager::logMessage("Initializing Wi-Fi...");
    WifiManager::begin();

    DisplayManager::logMessage("Initializing Bluetooth...");
    BluetoothManager::begin();

    lastMatrixFrameMs = millis();
    lastDisplayRefreshMs =
        millis() - ProjectConfig::DISPLAY_REFRESH_MS;
    lastBleStatusMs =
        millis() - ProjectConfig::BLE_STATUS_REFRESH_MS;

    DisplayManager::logMessage("Drawing initial matrix frame...");
    testMatrixRefresh();

    drawMatrixFrame(millis());

    DisplayManager::logMessage("Initialization complete.");
}

void general_run(void)
{
    const uint32_t now = millis();
    WifiManager::update();
    BluetoothManager::update();
    MPU6050::update();

    updateButton(ProjectConfig::BUTTON_PAGE_PIN, pageButton, now);
    updateButton(ProjectConfig::BUTTON_MODE_PIN, modeButton, now);

    char command[48];
    if (BluetoothManager::takeCommand(command, sizeof(command))) {
        applyBluetoothCommand(command);
    }

    if ((uint32_t)(now - lastMatrixFrameMs) >= ProjectConfig::MATRIX_FRAME_MS) {
        lastMatrixFrameMs = now;
        drawMatrixFrame(now);
    }

    updatePowerMonitor(now);

    updateDisplay(now);
    updateBleStatus(now);
}

MatrixMode general_getMatrixMode(void) { return matrixMode; }
const char *general_getMatrixModeName(void) { return modeName(matrixMode); }
