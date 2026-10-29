#include "display_manager.h"
#include "project_config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>

namespace DisplayManager {
namespace {
Adafruit_SSD1306 oled(ProjectConfig::OLED_WIDTH, ProjectConfig::OLED_HEIGHT, &Wire, -1);
bool available = false;
}

bool begin()
{
    available = oled.begin(SSD1306_SWITCHCAPVCC, ProjectConfig::OLED_I2C_ADDRESS);
    if (!available) {
        Serial.println(F("OLED: SSD1306 not detected; check controller/address/wiring"));
        return false;
    }
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextWrap(false);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println(F("Water Matrix Clock"));
    oled.println(F("Starting..."));
    oled.display();
    Serial.println(F("OLED: SSD1306 initialized"));
    return true;
}

bool isAvailable() { return available; }

void update(const Status &s)
{
    if (!available) return;
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    if (s.page == 0) {
        char timeText[16] = "--:--:--";
        char dateText[28] = "Waiting for NTP";
        if (s.timeValid) {
            strftime(timeText, sizeof(timeText), "%H:%M:%S", &s.localTime);
            strftime(dateText, sizeof(dateText), "%a, %d %b %Y", &s.localTime);
        }
        oled.setTextSize(2);
        int16_t x1, y1;
        uint16_t w, h;
        oled.getTextBounds(timeText, 0, 0, &x1, &y1, &w, &h);
        oled.setCursor((ProjectConfig::OLED_WIDTH - (int)w) / 2, 8);
        oled.print(timeText);
        oled.drawFastHLine(0, 34, ProjectConfig::OLED_WIDTH, SSD1306_WHITE);
        oled.setTextSize(1);
        oled.getTextBounds(dateText, 0, 0, &x1, &y1, &w, &h);
        oled.setCursor((ProjectConfig::OLED_WIDTH - (int)w) / 2, 40);
        oled.print(dateText);
        oled.setCursor(0, 55);
        oled.print(s.wifiConnected ? F("WiFi OK") : F("WiFi offline"));
        oled.setCursor(77, 55);
        oled.print(s.ntpSynced ? F("NTP sync") : F("NTP wait"));
    } else if (s.page == 1) {
        oled.setTextSize(1);
        oled.setCursor(0, 0);
        oled.println(F("ORIENTATION / MPU6050"));
        oled.drawFastHLine(0, 11, ProjectConfig::OLED_WIDTH, SSD1306_WHITE);
        if (s.mpuReady) {
            char pitchText[20];
            char rollText[20];
            snprintf(pitchText, sizeof(pitchText), "P %6.1f", s.pitchDeg);
            snprintf(rollText, sizeof(rollText), "R %6.1f", s.rollDeg);
            oled.setTextSize(2);
            oled.setCursor(0, 19);
            oled.print(pitchText);
            oled.setCursor(0, 40);
            oled.print(rollText);
            oled.setTextSize(1);
            oled.setCursor(99, 23);
            oled.print(F("deg"));
            oled.setCursor(99, 44);
            oled.print(F("deg"));
        } else {
            oled.setCursor(0, 27);
            oled.println(F("MPU6050 not found"));
            oled.println(F("Check SDA/SCL and 3V3"));
        }
    } else {
        oled.setTextSize(1);
        oled.setCursor(0, 0);
        oled.println(F("DEVICE STATUS"));
        oled.drawFastHLine(0, 11, ProjectConfig::OLED_WIDTH, SSD1306_WHITE);
        oled.setCursor(0, 17);
        oled.print(F("WiFi: "));
        oled.println(s.wifiConnected ? F("connected") : F("offline"));
        oled.setCursor(0, 28);
        oled.print(F("IP: "));
        oled.println((s.ipAddress != nullptr) ? s.ipAddress : "-");
        oled.setCursor(0, 39);
        oled.print(F("Clock: "));
        oled.println(s.ntpSynced ? F("synchronized") : F("waiting NTP"));
        oled.setCursor(0, 50);
        oled.print(F("Mode: "));
        oled.print((s.modeName != nullptr) ? s.modeName : "-");
        oled.print(F("  Bright: "));
        oled.print(s.brightness);
    }

    oled.display();
}
}
