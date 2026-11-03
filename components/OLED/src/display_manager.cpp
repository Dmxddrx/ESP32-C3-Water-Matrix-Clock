#include "display_manager.h"
#include "fonts.h"
#include "icons.h"
#include "mpu6050.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdio.h>
#include <string.h>

namespace DisplayManager {
namespace {


constexpr int DISPLAY_NATIVE_WIDTH = 128;
constexpr int DISPLAY_NATIVE_HEIGHT = 64;
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
    DISPLAY_NATIVE_WIDTH,
    DISPLAY_NATIVE_HEIGHT,
    &Wire,
    OLED_RESET
);

bool displayReady = false;


// ------------------------------------------------------------
// AUTOMATIC DEVICE POSITION DETECTION
// ------------------------------------------------------------

enum class DevicePosition : uint8_t {
    UNKNOWN,
    POSITION_1,
    POSITION_2,
    POSITION_3,
    POSITION_4
};

// Current confirmed device position.
DevicePosition currentPosition = DevicePosition::UNKNOWN;

// Candidate position used for stability checking.
DevicePosition candidatePosition = DevicePosition::UNKNOWN;

uint32_t candidatePositionSinceMs = 0;

// Position must remain stable for this duration.
constexpr uint32_t POSITION_STABLE_MS = 400;

// Accelerometer raw thresholds.
constexpr int16_t GRAVITY_THRESHOLD = 10000;
constexpr int16_t DIAGONAL_THRESHOLD2 = 1500;
constexpr int16_t DIAGONAL_THRESHOLD = 9000;

uint8_t currentDisplayRotation = 0;


/*
 * Detect device position using stationary gravity direction.
 *
 * POSITION 1:
 *     Negative Z gravity.
 *
 * POSITION 2:
 *     Positive Z gravity.
 *
 * POSITION 3:
 *     Negative Y gravity.
 *
 * POSITION 4:
 *     Positive X and positive Y together.
 *
 * Gyroscope values are deliberately not used here.
 */
DevicePosition detectDevicePosition(const MPU6050::Data &imu)
{
    // Position 4: both X and Y are strongly positive.
    // Check this first because it is a diagonal orientation.
    if (imu.axRaw > DIAGONAL_THRESHOLD2 &&
        imu.ayRaw > DIAGONAL_THRESHOLD)
    {
        return DevicePosition::POSITION_4;
    }

    // Position 1: Z axis points in the negative direction.
    if (imu.azRaw < -GRAVITY_THRESHOLD)
    {
        return DevicePosition::POSITION_1;
    }

    // Position 2: Z axis points in the positive direction.
    if (imu.azRaw > GRAVITY_THRESHOLD)
    {
        return DevicePosition::POSITION_2;
    }

    // Position 3: Y axis points in the negative direction.
    if (imu.ayRaw < -GRAVITY_THRESHOLD)
    {
        return DevicePosition::POSITION_3;
    }

    return DevicePosition::UNKNOWN;
}


/*
 * Convert the detected physical position into OLED rotation.
 *
 * Position 1: 90 degrees anticlockwise.
 * Position 2: 90 degrees clockwise.
 * Position 3: No rotation.
 * Position 4: 180 degrees.
 */
uint8_t getRotationForPosition(DevicePosition position)
{
    switch (position)
    {
        case DevicePosition::POSITION_1:
            return 3;

        case DevicePosition::POSITION_2:
            return 1;

        case DevicePosition::POSITION_3:
            return 0;

        case DevicePosition::POSITION_4:
            return 2;

        default:
            return currentDisplayRotation;
    }
}


/*
 * Update OLED orientation only after a stable position is detected.
 *
 * During movement, unknown or changing accelerometer readings do not
 * immediately rotate the display.
 */
void updateAutomaticRotation()
{
    if (!MPU6050::isReady()) {
        return;
    }

    const MPU6050::Data &imu = MPU6050::getData();

    if (!imu.valid) {
        return;
    }

    const uint32_t now = millis();

    const DevicePosition detected =
        detectDevicePosition(imu);

    // Device is between known positions.
    // Keep the previous display orientation.
    if (detected == DevicePosition::UNKNOWN)
    {
        candidatePosition = DevicePosition::UNKNOWN;
        return;
    }

    // A new candidate position has appeared.
    if (detected != candidatePosition)
    {
        candidatePosition = detected;
        candidatePositionSinceMs = now;
        return;
    }

    // Wait until the candidate position remains stable.
    if ((uint32_t)(now - candidatePositionSinceMs) <
        POSITION_STABLE_MS)
    {
        return;
    }

    // Position is confirmed.
    if (detected != currentPosition)
    {
        currentPosition = detected;

        const uint8_t newRotation =
            getRotationForPosition(currentPosition);

        // Apply rotation only when it actually changes.
        if (newRotation != currentDisplayRotation)
        {
            currentDisplayRotation = newRotation;

            display.setRotation(currentDisplayRotation);

            // Clear the old framebuffer after changing orientation.
            display.clearDisplay();
        }
    }
}


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
    int16_t x = (display.width() - width) / 2;

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
            display.width() - Icon_Power.width - 2,
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
            display.width() - width,
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

                if (px >= 0 && px < display.width() &&
                    py >= 0 && py < display.height()) {

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

// --------------------------------------------------
// PORTRAIT CLOCK PAGE (64 x 128)
// Used for physical positions 1 and 2.
// --------------------------------------------------

void drawPortraitClockPage(const Status &status)
{
    char dateText[20];
    char hourText[3];
    char minuteText[3];
    char batteryText[12];

    const int16_t screenW = display.width();

    // --------------------------------------------------
    // DATE
    // --------------------------------------------------

    if (status.timeValid)
    {
        strftime(
            dateText,
            sizeof(dateText),
            "%a %d %b",
            &status.localTime
        );
    }
    else
    {
        snprintf(dateText, sizeof(dateText), "WAITING NTP");
    }

    drawCenteredText(dateText, 2, 1);


    // --------------------------------------------------
    // BATTERY / CHARGING STATUS
    // --------------------------------------------------

    if (status.externalPowerPresent)
    {
        display.drawBitmap(
            3,
            17,
            Icon_Power.data,
            Icon_Power.width,
            Icon_Power.height,
            SSD1306_WHITE
        );

        display.setCursor(14, 19);
        display.print("CHARGING");
    }
    else if (status.batteryPercent >= 0)
    {
        snprintf(
            batteryText,
            sizeof(batteryText),
            "BAT: %d%%",
            status.batteryPercent
        );

        drawCenteredText(batteryText, 20, 1);
    }


    // --------------------------------------------------
    // TIME
    // Large vertical HH / MM clock
    // --------------------------------------------------

    if (status.timeValid)
    {
        snprintf(
            hourText,
            sizeof(hourText),
            "%02d",
            status.localTime.tm_hour
        );

        snprintf(
            minuteText,
            sizeof(minuteText),
            "%02d",
            status.localTime.tm_min
        );
    }
    else
    {
        snprintf(hourText, sizeof(hourText), "--");
        snprintf(minuteText, sizeof(minuteText), "--");
    }

    // Font_16x26: two digits, centered horizontally.
    const int16_t clockWidth =
        2 * Font_16x26.FontWidth;

    const int16_t clockX =
        (screenW - clockWidth) / 2;

    // HOURS
    drawBitmapString(
        hourText,
        clockX,
        39,
        &Font_16x26
    );

    // MINUTES
    drawBitmapString(
        minuteText,
        clockX,
        68,
        &Font_16x26
    );


    // --------------------------------------------------
    // NETWORK STATUS (CENTERED)
    // --------------------------------------------------

    display.setTextSize(1);

    char wifiText[16];
    char ntpText[16];

    snprintf(
        wifiText,
        sizeof(wifiText),
        "WiFi: %s",
        status.wifiConnected ? "ON" : "OFF"
    );

    snprintf(
        ntpText,
        sizeof(ntpText),
        "NTP: %s",
        status.ntpSynced ? "SYNC" : "WAIT"
    );

    drawCenteredText(wifiText, 101, 1);
    drawCenteredText(ntpText, 111, 1);


    // --------------------------------------------------
    // MODE FOOTER
    // --------------------------------------------------

    drawCenteredText(
        status.modeName != nullptr ? status.modeName : "UNKNOWN",
        120,
        1
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

// --------------------------------------------------
// PORTRAIT NETWORK PAGE (64 x 128)
// --------------------------------------------------

void drawPortraitNetworkPage(const Status &status)
{
    display.setTextSize(1);

    drawCenteredText("NETWORK", 3, 1);

    display.setCursor(3, 24);
    display.print("WiFi: ");
    display.println(status.wifiConnected ? "ON" : "OFF");

    display.setCursor(3, 38);
    display.print("NTP: ");
    display.println(status.ntpSynced ? "SYNC" : "WAIT");

    display.setCursor(3, 55);
    display.println("IP ADDRESS:");

    // An IPv4 address can be wider than the 64-pixel display.
    // Split it into two lines instead of clipping the address.
    if (status.ipAddress != nullptr)
    {
        char ipFirst[11] = {};
        char ipSecond[11] = {};

        const size_t ipLength = strlen(status.ipAddress);
        const size_t firstLength =
            (ipLength > 10) ? 10 : ipLength;

        strncpy(ipFirst, status.ipAddress, firstLength);

        if (ipLength > firstLength)
        {
            strncpy(
                ipSecond,
                status.ipAddress + firstLength,
                sizeof(ipSecond) - 1
            );
        }

        display.setCursor(3, 68);
        display.print(ipFirst);

        display.setCursor(3, 80);
        display.print(ipSecond);
    }

    display.setCursor(3, 103);
    display.print("Mode:");

    drawCenteredText(
        status.modeName != nullptr ? status.modeName : "UNKNOWN",
        116,
        1
    );
}

void drawImuPage(const Status &status)
{
    char line[32];

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // --------------------------------------------------
    // MPU6050 STATUS
    // --------------------------------------------------

    display.setCursor(0, 17);

    if (!status.mpuReady) {
        display.println("MPU6050: NOT READY");
        return;
    }

    const MPU6050::Data &imu = MPU6050::getData();

    if (!imu.valid) {
        display.println("MPU6050: NO DATA");
        return;
    }

    display.println("MPU6050: RAW STREAM");

    // --------------------------------------------------
    // RAW ACCELEROMETER DATA
    // --------------------------------------------------

    // Raw accelerometer X and Y.
    snprintf(
        line,
        sizeof(line),
        "AX:%+6d AY:%+6d",
        (int)imu.axRaw,
        (int)imu.ayRaw
    );

    display.setCursor(0, 25);
    display.println(line);

    // Raw accelerometer Z and temperature.
    snprintf(
        line,
        sizeof(line),
        "AZ:%+6d T:%4.1fC",
        (int)imu.azRaw,
        (double)imu.temperatureC
    );

    display.setCursor(0, 33);
    display.println(line);

    // --------------------------------------------------
    // RAW GYROSCOPE DATA
    // --------------------------------------------------

    // Raw gyroscope X and Y.
    snprintf(
        line,
        sizeof(line),
        "GX:%+6d GY:%+6d",
        (int)imu.gxRaw,
        (int)imu.gyRaw
    );

    display.setCursor(0, 41);
    display.println(line);

    // Raw gyroscope Z.
    snprintf(
        line,
        sizeof(line),
        "GZ:%+6d",
        (int)imu.gzRaw
    );

    display.setCursor(0, 49);
    display.println(line);
}

// --------------------------------------------------
// PORTRAIT IMU PAGE (64 x 128)
// --------------------------------------------------

void drawPortraitImuPage(const Status &status)
{
    char line[24];

    display.setTextSize(1);

    drawCenteredText("IMU RAW DATA", 2, 1);

    if (!status.mpuReady)
    {
        drawCenteredText("MPU NOT READY", 40, 1);
        return;
    }

    const MPU6050::Data &imu = MPU6050::getData();

    if (!imu.valid)
    {
        drawCenteredText("NO SENSOR DATA", 40, 1);
        return;
    }

    // Raw accelerometer X.
    snprintf(line, sizeof(line), "AX:%+6d", (int)imu.axRaw);
    display.setCursor(3, 19);
    display.println(line);

    // Raw accelerometer Y.
    snprintf(line, sizeof(line), "AY:%+6d", (int)imu.ayRaw);
    display.setCursor(3, 30);
    display.println(line);

    // Raw accelerometer Z.
    snprintf(line, sizeof(line), "AZ:%+6d", (int)imu.azRaw);
    display.setCursor(3, 41);
    display.println(line);

    // Raw gyroscope X.
    snprintf(line, sizeof(line), "GX:%+6d", (int)imu.gxRaw);
    display.setCursor(3, 52);
    display.println(line);

    // Raw gyroscope Y.
    snprintf(line, sizeof(line), "GY:%+6d", (int)imu.gyRaw);
    display.setCursor(3, 63);
    display.println(line);

    // Raw gyroscope Z.
    snprintf(line, sizeof(line), "GZ:%+6d", (int)imu.gzRaw);
    display.setCursor(3, 74);
    display.println(line);

    // Complementary-filter angles.
    snprintf(
        line,
        sizeof(line),
        "P:%+.1f",
        (double)imu.pitchDeg
    );
    display.setCursor(3, 88);
    display.println(line);

    snprintf(
        line,
        sizeof(line),
        "R:%+.1f",
        (double)imu.rollDeg
    );
    display.setCursor(3, 99);
    display.println(line);

    // Temperature.
    snprintf(
        line,
        sizeof(line),
        "T:%4.1fC",
        (double)imu.temperatureC
    );
    display.setCursor(3, 110);
    display.println(line);
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

    int16_t x = (display.width() - textWidth(footer, 1)) / 2;
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

    display.setRotation(0);

    currentDisplayRotation = 0;
    currentPosition = DevicePosition::UNKNOWN;
    candidatePosition = DevicePosition::UNKNOWN;
    candidatePositionSinceMs = 0;

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
    for (int16_t x = display.width();
         x >= -messageWidth;
         x -= 3) {

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(x, (display.height() - 8) / 2);
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

    // Detect the physical device position and update OLED rotation.
    updateAutomaticRotation();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);

    // --------------------------------------------------
    // PORTRAIT MODE: 64 x 128
    // --------------------------------------------------

    if (display.height() > display.width())
    {
        switch (status.page)
        {
            case 0:
                drawPortraitClockPage(status);
                break;

            case 1:
                drawPortraitNetworkPage(status);
                break;

            case 2:
                drawPortraitImuPage(status);
                break;

            default:
                drawPortraitClockPage(status);
                break;
        }
    }

    // --------------------------------------------------
    // LANDSCAPE MODE: 128 x 64
    // --------------------------------------------------

    else
    {
        // Date header remains in the yellow section.
        drawDateHeader(status);

        switch (status.page)
        {
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
        //drawLogLine();
    }

    display.display();
}


// ------------------------------------------------------------
// PREPARE OLED FOR DEEP SLEEP
// ------------------------------------------------------------

void prepareForSleep()
{
    if (!displayReady) {
        return;
    }

    display.clearDisplay();
    display.display();

    // Send SSD1306 display OFF command.
    display.ssd1306_command(SSD1306_DISPLAYOFF);
}

} // namespace DisplayManager