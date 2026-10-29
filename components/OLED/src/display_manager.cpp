#include "display_manager.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>
#include <string.h>

namespace DisplayManager {
namespace {

constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 64;
constexpr int OLED_RESET = -1;
constexpr uint8_t OLED_ADDRESS = 0x3C;

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    OLED_RESET
);

bool displayReady = false;

// Width of the built-in Adafruit GFX font.
int16_t textWidth(const char *text, uint8_t size)
{
    if (text == nullptr) {
        return 0;
    }

    return (int16_t)strlen(text) * 6 * size;
}

void drawCenteredText(
    const char *text,
    int16_t y,
    uint8_t size
)
{
    if (text == nullptr) {
        return;
    }

    int16_t width = textWidth(text, size);
    int16_t x = (SCREEN_WIDTH - width) / 2;

    if (x < 0) {
        x = 0;
    }

    display.setTextSize(size);
    display.setCursor(x, y);
    display.print(text);
}

void drawDateHeader(const Status &status)
{
    char dateText[24];

    if (status.timeValid) {
        strftime(
            dateText,
            sizeof(dateText),
            "%a %d %b %Y",
            &status.localTime
        );
    } else {
        snprintf(
            dateText,
            sizeof(dateText),
            "WAITING FOR NTP"
        );
    }

    // OLED rows 0-15: physical yellow section.
    display.setTextColor(SSD1306_WHITE);
    drawCenteredText(dateText, 4, 1);
}

void drawClockPage(const Status &status)
{
    char hourMinute[8];
    char seconds[4];

    if (status.timeValid) {
        snprintf(
            hourMinute,
            sizeof(hourMinute),
            "%02d:%02d",
            status.localTime.tm_hour,
            status.localTime.tm_min
        );

        snprintf(
            seconds,
            sizeof(seconds),
            "%02d",
            status.localTime.tm_sec
        );
    } else {
        snprintf(hourMinute, sizeof(hourMinute), "--:--");
        snprintf(seconds, sizeof(seconds), "--");
    }

    // OLED rows 16-63: physical blue section.
    // Large hours and minutes.
    display.setTextSize(3);
    display.setCursor(6, 29);
    display.print(hourMinute);

    // Smaller seconds, aligned with the main clock.
    display.setTextSize(2);
    display.setCursor(98, 32);
    display.print(seconds);
}

void drawNetworkPage(const Status &status)
{
    display.setTextSize(1);

    display.setCursor(0, 19);
    display.print("WiFi: ");
    display.println(
        status.wifiConnected ? "CONNECTED" : "OFFLINE"
    );

    display.setCursor(0, 29);
    display.print("NTP: ");
    display.println(
        status.ntpSynced ? "SYNCHRONIZED" : "WAITING"
    );

    display.setCursor(0, 39);
    display.print("IP: ");
    display.println(
        status.ipAddress != nullptr
            ? status.ipAddress
            : "-"
    );

    display.setCursor(0, 49);
    display.print("Mode: ");
    display.println(
        status.modeName != nullptr
            ? status.modeName
            : "-"
    );
}

void drawImuPage(const Status &status)
{
    char line[32];

    display.setTextSize(1);

    display.setCursor(0, 17);
    display.print("MPU6050: ");
    display.println(
        status.mpuReady ? "READY" : "NOT READY"
    );

    display.setCursor(0, 25);
    if (status.mpuReady) {
        snprintf(
            line,
            sizeof(line),
            "Pitch: %+.1f deg",
            (double)status.pitchDeg
        );
    } else {
        snprintf(line, sizeof(line), "Pitch: --");
    }
    display.println(line);

    display.setCursor(0, 33);
    if (status.mpuReady) {
        snprintf(
            line,
            sizeof(line),
            "Roll: %+.1f deg",
            (double)status.rollDeg
        );
    } else {
        snprintf(line, sizeof(line), "Roll: --");
    }
    display.println(line);

    display.setCursor(0, 41);
    display.print("Mode: ");
    display.println(
        status.modeName != nullptr
            ? status.modeName
            : "-"
    );

    display.setCursor(0, 49);
    display.print("WiFi: ");
    display.println(
        status.wifiConnected ? "ON" : "OFF"
    );

    display.setCursor(0, 57);
    display.print("Brightness: ");
    display.println(status.brightness);
}

} // namespace

void begin()
{
    displayReady = display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS
    );

    if (!displayReady) {
        Serial.println(
            F("OLED: initialization failed; check I2C wiring/address")
        );
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.display();

    Serial.println(F("OLED: SSD1306 128x64 initialized"));
}

void update(const Status &status)
{
    if (!displayReady) {
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);

    // Keep the date in the yellow section on every page.
    drawDateHeader(status);

    switch (status.page) {
        case 0:
            drawClockPage(status);
            break;

        case 1:
            drawNetworkPage(status);
            break;

        case 2:
            drawImuPage(status);
            break;

        default:
            drawClockPage(status);
            break;
    }

    // Send the completed framebuffer to the OLED.
    display.display();
}

} // namespace DisplayManager