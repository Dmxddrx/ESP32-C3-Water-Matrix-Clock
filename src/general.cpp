#include "general.h"
#include "project_config.h"
#include "max7219.h"
#include "icons.h"
#include "mpu6050.h"
#include "display_manager.h"
#include "wifi_manager.h"
#include "bluetooth_manager.h"
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
MatrixMode matrixMode = MatrixMode::WATER;
uint32_t lastMatrixFrameMs = 0;
uint32_t lastDisplayRefreshMs = 0;
uint32_t lastBleStatusMs = 0;
uint32_t frameCounter = 0;

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
    Serial.printf("Matrix mode: %s\n", modeName(matrixMode));
}

void onButtonPressed(uint8_t pin)
{
    if (pin == ProjectConfig::BUTTON_PAGE_PIN) {
        displayPage = (displayPage + 1U) % 3U;
        Serial.printf("Display page: %u\n", displayPage);
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
        Serial.printf("BLE command not recognized: %s\n", command);
        return;
    }
    Serial.printf("BLE command applied: %s\n", command);
}

void drawWaterFrame(uint32_t nowMs)
{
    MAX7219_Clear();
    const MPU6050::Data &imu = MPU6050::getData();
    const float pitch = (imu.valid && MPU6050::isReady()) ? imu.pitchDeg : 0.0f;
    const float roll = (imu.valid && MPU6050::isReady()) ? imu.rollDeg : 0.0f;
    const float seconds = nowMs * 0.001f;

    for (int16_t x = 0; x < 16; ++x) {
        float wave = 0.0f;
        if (matrixMode == MatrixMode::WATER) {
            wave = 0.38f * sinf((x * 0.60f) + seconds * 2.0f);
        }
        // Pitch changes the average fill level; roll tilts the liquid line.
        float surfaceY = 4.0f - (pitch * 0.065f) + ((x - 7.5f) * roll * 0.012f) + wave;
        if (surfaceY < 0.0f) surfaceY = 0.0f;
        if (surfaceY > 7.0f) surfaceY = 7.0f;

        for (int16_t y = 0; y < 8; ++y) {
            if ((float)y >= surfaceY) MAX7219_SetPixel(x, y, 1);
        }
    }

    // Add small hollow bubbles only in water mode. They move slowly and are
    // intentionally sparse so the liquid silhouette remains readable.
    if (matrixMode == MatrixMode::WATER) {
        const int16_t bubbleX1 = (int16_t)((frameCounter / 4U) % 13U) + 1;
        const int16_t bubbleY1 = 6 - (int16_t)((frameCounter / 15U) % 2U);
        const int16_t bubbleX2 = 14 - (int16_t)((frameCounter / 6U) % 11U);
        const int16_t bubbleY2 = 5 - (int16_t)((frameCounter / 18U) % 2U);
        MAX7219_SetPixel(bubbleX1, bubbleY1, 0);
        MAX7219_SetPixel(bubbleX2, bubbleY2, 0);
    }
    MAX7219_Update();
}

void drawMatrixFrame(uint32_t nowMs)
{
    if (matrixMode == MatrixMode::HEART) {
        MAX7219_Clear();
        // Two hearts fill the 16x8 canvas, matching the supplied STM32 icon art.
        MAX7219_DrawIcon(&Icon_Heart, 0, 0);
        MAX7219_DrawIcon(&Icon_Heart, 8, 0);
        // A small pulse effect is achieved by briefly changing brightness.
        const uint8_t pulse = ((frameCounter / 8U) % 2U) ? brightness : (brightness > 1 ? brightness - 1 : 1);
        MAX7219_SetBrightness(pulse);
        MAX7219_Update();
        MAX7219_SetBrightness(brightness);
    } else {
        drawWaterFrame(nowMs);
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
}

void general_init(void)
{
    Serial.println();
    Serial.println(F("ESP32-C3 Water Matrix Clock"));
    Serial.println(F("Starting peripherals..."));

    pinMode(ProjectConfig::BUTTON_PAGE_PIN, INPUT_PULLUP);
    pinMode(ProjectConfig::BUTTON_MODE_PIN, INPUT_PULLUP);
    const uint32_t now = millis();
    pageButton.rawState = pageButton.stableState = digitalRead(ProjectConfig::BUTTON_PAGE_PIN);
    pageButton.rawChangedMs = now;
    modeButton.rawState = modeButton.stableState = digitalRead(ProjectConfig::BUTTON_MODE_PIN);
    modeButton.rawChangedMs = now;

    Wire.begin(ProjectConfig::I2C_SDA_PIN, ProjectConfig::I2C_SCL_PIN,
               ProjectConfig::I2C_FREQUENCY_HZ);
    Wire.setTimeOut(50);
    Serial.printf("I2C: SDA=GPIO%u, SCL=GPIO%u at %lu Hz\n",
                  ProjectConfig::I2C_SDA_PIN, ProjectConfig::I2C_SCL_PIN,
                  (unsigned long)ProjectConfig::I2C_FREQUENCY_HZ);

    MAX7219_Init();
    MAX7219_SetBrightness(brightness);
    DisplayManager::begin();
    MPU6050::begin();
    WifiManager::begin();
    BluetoothManager::begin();

    lastMatrixFrameMs = millis();
    lastDisplayRefreshMs = millis() - ProjectConfig::DISPLAY_REFRESH_MS;
    lastBleStatusMs = millis() - ProjectConfig::BLE_STATUS_REFRESH_MS;
    drawMatrixFrame(millis());
    Serial.println(F("Initialization complete."));
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

    updateDisplay(now);
    updateBleStatus(now);
}

MatrixMode general_getMatrixMode(void) { return matrixMode; }
const char *general_getMatrixModeName(void) { return modeName(matrixMode); }
