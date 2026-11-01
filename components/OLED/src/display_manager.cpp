#include "display_manager.h"
#include "fonts.h"
#include "icons.h"

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

constexpr uint8_t LOG_QUEUE_SIZE = 8;
constexpr size_t LOG_LINE_SIZE = 22;
constexpr uint32_t LOG_HOLD_MS = 1400;

char logQueue[LOG_QUEUE_SIZE][LOG_LINE_SIZE] = {};
char activeLog[LOG_LINE_SIZE] = "> STARTING";

uint8_t logHead = 0;
uint8_t logCount = 0;
uint32_t lastLogRotateMs = 0;

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

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Date aligned to the left.
    display.setCursor(0, 4);
    display.print(dateText);

    // Charging / external power indication.
    if (status.externalPowerPresent)
    {
        // Your custom 8x12 lightning icon.
        display.drawBitmap(
            SCREEN_WIDTH - Icon_Power.width - 2,
            2,
            Icon_Power.data,
            Icon_Power.width,
            Icon_Power.height,
            SSD1306_WHITE
        );
    }
    else if (status.batteryPercent >= 0)
    {
        // Show battery percentage when not charging.
        char batteryText[6];

        snprintf(
            batteryText,
            sizeof(batteryText),
            "%d%%",
            status.batteryPercent
        );

        const int16_t width = textWidth(batteryText, 1);

        display.setCursor(
            SCREEN_WIDTH - width,
            4
        );

        display.print(batteryText);
    }
}

void drawBitmapChar(
    char ch,
    int16_t x,
    int16_t y,
    const FontDef_t *font
)
{
    if (font == nullptr || font->data == nullptr) return;
    if (ch < 32 || ch > 126) return;

    const uint16_t offset =
        (uint16_t)(ch - 32) * font->FontHeight;

    for (uint8_t row = 0; row < font->FontHeight; ++row) {

        const uint16_t rowBits = font->data[offset + row];

        for (uint8_t col = 0; col < font->FontWidth && col < 16; ++col) {

            if (rowBits & (uint16_t)(0x8000U >> col)) {

                const int16_t px = x + col;
                const int16_t py = y + row;

                if (px >= 0 && px < SCREEN_WIDTH &&
                    py >= 0 && py < SCREEN_HEIGHT) {

                    display.drawPixel(
                        px,
                        py,
                        SSD1306_WHITE
                    );
                }
            }
        }
    }
}

void drawBitmapString(
    const char *text,
    int16_t x,
    int16_t y,
    const FontDef_t *font
)
{
    if (text == nullptr || font == nullptr) return;

    while (*text) {

        drawBitmapChar(*text, x, y, font);

        x += font->FontWidth;
        ++text;
    }
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

    // Main clock: custom 16x26 font.
    drawBitmapString(
        hourMinute,
        17,
        18,
        &Font_16x26
    );

    // Seconds: small 7x10 font.
    drawBitmapString(
        seconds,
        101,
        30,
        &Font_7x10
    );
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
    display.print("WiFi: ");
    display.println(
        status.wifiConnected ? "ON" : "OFF"
    );

    display.setCursor(0, 49);
    display.print("Brightness: ");
    display.println(status.brightness);
}
void drawModeFooter(const Status &status)
{
    char footer[24];

    snprintf(
        footer,
        sizeof(footer),
        "%s",
        status.modeName != nullptr ? status.modeName : "UNKNOWN"
    );

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    int16_t x = (SCREEN_WIDTH - textWidth(footer, 1)) / 2;
    if (x < 0) x = 0;

    display.setCursor(x, 47);
    display.print(footer);
}

void drawLogLine()
{
    const uint32_t now = millis();

    if (logCount > 0 &&
        (uint32_t)(now - lastLogRotateMs) >= LOG_HOLD_MS) {

        strncpy(
            activeLog,
            logQueue[logHead],
            sizeof(activeLog) - 1
        );
        activeLog[sizeof(activeLog) - 1] = '\0';

        logHead = (logHead + 1) % LOG_QUEUE_SIZE;
        --logCount;
        lastLogRotateMs = now;
    }

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 56);
    display.print(activeLog);
}
} // namespace

void begin()
{
    displayReady = display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS
    );

    if (!displayReady) {
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.display();
}

void showStartupMessage(const char *message)
{
    if (!displayReady || message == nullptr || message[0] == '\0') {
        return;
    }

    display.setTextWrap(false);
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    const int16_t messageWidth = textWidth(message, 1);

    // Scroll once from the right edge to beyond the left edge.
    for (int16_t x = SCREEN_WIDTH;
         x >= -messageWidth;
         x -= 3) {

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(x, 28);
        display.print(message);
        display.display();

        delay(8);
    }

    display.clearDisplay();
    display.display();

}

void logMessage(const char *format, ...)
{
    if (format == nullptr) return;

    char message[96];

    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    char shortMessage[LOG_LINE_SIZE];

    // Reserve two characters for the visible log marker.
    snprintf(shortMessage, sizeof(shortMessage), "> %.19s", message);

    // If full, discard the oldest queued message.
    if (logCount >= LOG_QUEUE_SIZE) {
        logHead = (logHead + 1) % LOG_QUEUE_SIZE;
        --logCount;
    }

    const uint8_t tail =
        (logHead + logCount) % LOG_QUEUE_SIZE;

    strncpy(
        logQueue[tail],
        shortMessage,
        LOG_LINE_SIZE - 1
    );
    logQueue[tail][LOG_LINE_SIZE - 1] = '\0';

    ++logCount;
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
    drawModeFooter(status);
    drawLogLine();
    display.display();
}

} // namespace DisplayManager