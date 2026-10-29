#pragma once

#include <Arduino.h>

// ESP32-C3 Super Mini project pin configuration.
// Recommended safer matrix wiring avoids the GPIO8/GPIO9 boot-strapping pins.
// GPIO4/GPIO5 are also JTAG-capable pins; they are used here as ordinary GPIOs,
// so external JTAG debugging is not available while these assignments are active.
namespace ProjectConfig {
constexpr uint8_t I2C_SDA_PIN = 6;
constexpr uint8_t I2C_SCL_PIN = 7;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;

constexpr uint8_t MATRIX_CLK_PIN = 5;
constexpr uint8_t MATRIX_MOSI_PIN = 10;
constexpr uint8_t MATRIX_CS_PIN = 4;
constexpr uint8_t MATRIX_MODULES = 2;
constexpr uint8_t MATRIX_DEFAULT_BRIGHTNESS = 2;

constexpr uint8_t BUTTON_PAGE_PIN = 0;
constexpr uint8_t BUTTON_MODE_PIN = 3;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;

// This build assumes a 128x64 SSD1306 I2C OLED at 0x3C.
// Change these if your display uses a different controller, resolution, or address.
constexpr uint8_t OLED_I2C_ADDRESS = 0x3C;
constexpr int OLED_WIDTH = 128;
constexpr int OLED_HEIGHT = 64;

constexpr uint8_t MPU6050_ADDRESS_LOW = 0x68;
constexpr uint8_t MPU6050_ADDRESS_HIGH = 0x69;
constexpr uint32_t SENSOR_SAMPLE_MS = 10;
constexpr uint32_t MATRIX_FRAME_MS = 70;
constexpr uint32_t DISPLAY_REFRESH_MS = 250;
constexpr uint32_t BLE_STATUS_REFRESH_MS = 1000;
}
