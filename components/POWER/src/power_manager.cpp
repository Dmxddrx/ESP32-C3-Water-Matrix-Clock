
#include "power_manager.h"

#include "project_config.h"
#include "max7219.h"
#include "display_manager.h"
#include "wifi_manager.h"
#include "bluetooth_manager.h"
#include "mpu6050.h"

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include <esp_sleep.h>
#include <esp_idf_version.h>
#include <driver/gpio.h>

namespace PowerManager {
namespace {

// ------------------------------------------------------------
// POWER MANAGEMENT TIMING
// ------------------------------------------------------------

// Return to the clock page after one minute without page interaction.
constexpr uint32_t CLOCK_PAGE_TIMEOUT_MS = 60UL * 1000UL;

// Enter deep sleep after 15 minutes without user activity or motion.
constexpr uint32_t DEEP_SLEEP_TIMEOUT_MS = 15UL * 60UL * 1000UL;

// ------------------------------------------------------------
// MOTION DETECTION
// ------------------------------------------------------------

// Minimum gyro speed considered significant physical movement.
constexpr float MOTION_GYRO_THRESHOLD_DPS = 12.0f;

// Minimum difference between consecutive accelerometer samples.
// At +/-2g, 2500 counts is approximately 0.153g.
constexpr int32_t MOTION_ACCEL_DELTA_THRESHOLD = 2500;

// Avoid repeatedly processing motion events too frequently.
constexpr uint32_t MOTION_EVENT_COOLDOWN_MS = 1500;

// Do not allow sleep if the MPU6050 stops providing valid samples.
constexpr uint32_t IMU_VALID_TIMEOUT_MS = 500;

// ------------------------------------------------------------
// ACTIVITY TIMERS
// ------------------------------------------------------------

uint32_t lastUserActivityMs = 0;
uint32_t lastPageInteractionMs = 0;
uint32_t lastMotionEventMs = 0;
uint32_t lastValidImuMs = 0;

bool initialized = false;

// ------------------------------------------------------------
// MPU6050 MOTION TRACKING
// ------------------------------------------------------------

bool previousMotionSampleValid = false;

int16_t previousAxRaw = 0;
int16_t previousAyRaw = 0;
int16_t previousAzRaw = 0;

uint32_t lastProcessedImuTimestamp = 0;

// ------------------------------------------------------------
// DEEP-SLEEP WAKE-UP CONFIGURATION
// ------------------------------------------------------------

bool wakeupPinsValid = false;
bool wakeupConfigurationFailed = false;

// ------------------------------------------------------------
// ABSOLUTE DIFFERENCE
// ------------------------------------------------------------

int32_t absoluteDifference(int32_t a, int32_t b)
{
    const int32_t difference = a - b;

    return (difference < 0)
        ? -difference
        : difference;
}

// ------------------------------------------------------------
// INITIALIZE WAKE-UP PINS
// ------------------------------------------------------------

/*
 * CHECK DEEP-SLEEP WAKE-UP PINS
 *
 * ESP32-C3 GPIO0..GPIO5 support deep-sleep GPIO wake-up.
 *
 * The project buttons are active LOW:
 *
 *     Not pressed = HIGH
 *     Pressed     = LOW
 *
 * Both buttons are configured as wake-up sources.
 */
bool validateWakeupPins()
{
    const uint8_t pagePin = ProjectConfig::BUTTON_PAGE_PIN;
    const uint8_t modePin = ProjectConfig::BUTTON_MODE_PIN;

    if (pagePin == modePin)
    {
        DisplayManager::logMessage(
            "Power: wake-up buttons cannot share a GPIO"
        );

        return false;
    }

    if (!esp_sleep_is_valid_wakeup_gpio((gpio_num_t)pagePin) ||
        !esp_sleep_is_valid_wakeup_gpio((gpio_num_t)modePin))
    {
        DisplayManager::logMessage(
            "Power: button GPIOs are not valid deep-sleep wake pins"
        );

        DisplayManager::logMessage(
            "Power: ESP32-C3 requires RTC GPIO0..GPIO5"
        );

        return false;
    }

    return true;
}

// ------------------------------------------------------------
// DETECT SIGNIFICANT PHYSICAL MOTION
// ------------------------------------------------------------

/*
 * DETECT LARGE MOTION
 *
 * Motion is detected using two independent measurements:
 *
 * 1. Gyroscope:
 *      Detects significant angular movement.
 *
 * 2. Accelerometer:
 *      Detects sudden changes in physical acceleration.
 *
 * The initial accelerometer sample is used as a reference.
 * Consecutive samples are then compared.
 *
 * The MPU6050 sample timestamp prevents processing the
 * same measurement repeatedly when general_run() executes
 * faster than the sensor sampling interval.
 *
 * Returns true when a new significant motion event occurs.
 */
bool detectLargeMotion(
    const MPU6050::Data &imu,
    uint32_t nowMs
)
{
    if (!imu.valid) {
        previousMotionSampleValid = false;
        return false;
    }

    // Process each MPU6050 sample only once.
    if (imu.timestampMs == lastProcessedImuTimestamp) {
        return false;
    }

    lastProcessedImuTimestamp = imu.timestampMs;
    lastValidImuMs = nowMs;

    // --------------------------------------------------------
    // GYROSCOPE MOTION DETECTION
    // --------------------------------------------------------

    const float gyroMagnitudeSquared =
        (imu.gxDps * imu.gxDps) +
        (imu.gyDps * imu.gyDps) +
        (imu.gzDps * imu.gzDps);

    const float gyroThresholdSquared =
        MOTION_GYRO_THRESHOLD_DPS *
        MOTION_GYRO_THRESHOLD_DPS;

    bool motionDetected =
        gyroMagnitudeSquared >= gyroThresholdSquared;


    // --------------------------------------------------------
    // ACCELEROMETER CHANGE DETECTION
    // --------------------------------------------------------

    if (previousMotionSampleValid)
    {
        const int32_t deltaX =
            absoluteDifference(imu.axRaw, previousAxRaw);

        const int32_t deltaY =
            absoluteDifference(imu.ayRaw, previousAyRaw);

        const int32_t deltaZ =
            absoluteDifference(imu.azRaw, previousAzRaw);

        if (deltaX >= MOTION_ACCEL_DELTA_THRESHOLD ||
            deltaY >= MOTION_ACCEL_DELTA_THRESHOLD ||
            deltaZ >= MOTION_ACCEL_DELTA_THRESHOLD)
        {
            motionDetected = true;
        }
    }

    // Save this sample for the next comparison.
    previousAxRaw = imu.axRaw;
    previousAyRaw = imu.ayRaw;
    previousAzRaw = imu.azRaw;

    previousMotionSampleValid = true;

    if (!motionDetected) {
        return false;
    }

    // Prevent repeated activity notifications during continuous motion.
    if ((uint32_t)(nowMs - lastMotionEventMs) <
        MOTION_EVENT_COOLDOWN_MS)
    {
        return false;
    }

    lastMotionEventMs = nowMs;

    return true;
}

// ------------------------------------------------------------
// ENTER DEEP SLEEP
// ------------------------------------------------------------

/*
 * ENTER DEEP SLEEP
 *
 * Before sleeping:
 *
 * 1. Configure both buttons as active-LOW wake-up sources.
 * 2. Turn off MAX7219 displays.
 * 3. Turn off the OLED.
 * 4. Put MPU6050 into sleep mode.
 * 5. Disable Wi-Fi.
 * 6. Disable Bluetooth.
 *
 * On button wake-up, the ESP32-C3 restarts from setup().
 *
 * The regular initialization sequence then restores all peripherals.
 */
void enterDeepSleep()
{
    if (!wakeupPinsValid || wakeupConfigurationFailed) {
        return;
    }

    const uint8_t pagePin = ProjectConfig::BUTTON_PAGE_PIN;
    const uint8_t modePin = ProjectConfig::BUTTON_MODE_PIN;

    // Do not enter sleep while either button is being held.
    if (digitalRead(pagePin) == LOW ||
        digitalRead(modePin) == LOW)
    {
        return;
    }

    // Ensure both RTC-capable pins are configured as inputs.
    pinMode(pagePin, INPUT_PULLUP);
    pinMode(modePin, INPUT_PULLUP);

    // Build a wake-up mask containing both button GPIOs.
    const uint64_t wakeupMask =
        (1ULL << pagePin) |
        (1ULL << modePin);

    // --------------------------------------------------------
    // CONFIGURE ACTIVE-LOW DEEP-SLEEP WAKE-UP
    // --------------------------------------------------------

    esp_err_t wakeupResult;

#if ESP_IDF_VERSION_MAJOR >= 6

    // ESP-IDF 6.x and later.
    wakeupResult =
        esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(
            wakeupMask,
            ESP_GPIO_WAKEUP_GPIO_LOW
        );

#else

    // ESP-IDF versions before 6.x.
    wakeupResult =
        esp_deep_sleep_enable_gpio_wakeup(
            wakeupMask,
            ESP_GPIO_WAKEUP_GPIO_LOW
        );

#endif

    if (wakeupResult != ESP_OK)
    {
        DisplayManager::logMessage(
            "Power: deep-sleep wake-up configuration failed: %d",
            (int)wakeupResult
        );

        // Avoid repeatedly attempting an invalid sleep configuration.
        wakeupConfigurationFailed = true;

        return;
    }

    // --------------------------------------------------------
    // TURN OFF MATRIX DISPLAYS
    // --------------------------------------------------------

    MAX7219_Clear();
    MAX7219_Update();

    // Use the MAX7219 hardware shutdown register.
    MAX7219_SetShutdown(true);

    // --------------------------------------------------------
    // TURN OFF OLED
    // --------------------------------------------------------

    DisplayManager::prepareForSleep();

    // --------------------------------------------------------
    // PUT MPU6050 INTO SLEEP
    // --------------------------------------------------------

    MPU6050::sleep();

    // --------------------------------------------------------
    // DISABLE WIRELESS PERIPHERALS
    // --------------------------------------------------------

    BluetoothManager::shutdown();
    WifiManager::shutdown();

    delay(20);

    // --------------------------------------------------------
    // START DEEP SLEEP
    // --------------------------------------------------------

    Serial.println("Power: entering deep sleep");

    Serial.flush();

    esp_deep_sleep_start();

}

// ------------------------------------------------------------
// PUBLIC API
// ------------------------------------------------------------

void begin(uint32_t nowMs)
{
    initialized = true;

    lastUserActivityMs = nowMs;
    lastPageInteractionMs = nowMs;

    // Permit a motion event immediately after initialization.
    lastMotionEventMs =
        nowMs - MOTION_EVENT_COOLDOWN_MS;

    lastValidImuMs = nowMs;

    previousMotionSampleValid = false;
    lastProcessedImuTimestamp = 0;

    wakeupConfigurationFailed = false;

    wakeupPinsValid = validateWakeupPins();

    if (wakeupPinsValid)
    {
        DisplayManager::logMessage(
            "Power: deep-sleep wake-up pins ready"
        );
    }

}

// ------------------------------------------------------------
// REGISTER USER ACTIVITY
// ------------------------------------------------------------

void notifyUserActivity(uint32_t nowMs)
{
    lastUserActivityMs = nowMs;
}


// ------------------------------------------------------------
// REGISTER PAGE INTERACTION
// ------------------------------------------------------------

void notifyPageInteraction(uint32_t nowMs)
{
    lastPageInteractionMs = nowMs;

    // Changing the OLED page is also user activity.
    notifyUserActivity(nowMs);
}


// ------------------------------------------------------------
// UPDATE POWER MANAGEMENT
// ------------------------------------------------------------

void update(
    uint32_t nowMs,
    uint8_t &displayPage,
    const MPU6050::Data &imu
)
{
    if (!initialized) {
        begin(nowMs);
        return;
    }

    // --------------------------------------------------------
    // 1. DETECT LARGE PHYSICAL MOTION
    // --------------------------------------------------------

    if (detectLargeMotion(imu, nowMs))
    {
        // Significant movement restarts the inactivity timer.
        notifyUserActivity(nowMs);
    }

    // --------------------------------------------------------
    // 2. AUTOMATICALLY RETURN TO CLOCK PAGE
    // --------------------------------------------------------

    if (displayPage != 0 &&
        (uint32_t)(nowMs - lastPageInteractionMs) >=
            CLOCK_PAGE_TIMEOUT_MS)
    {
        displayPage = 0;

        // The clock page is now active.
        DisplayManager::logMessage(
            "Power: automatically returned to clock page"
        );
    }

    // --------------------------------------------------------
    // 3. CHECK DEEP-SLEEP INACTIVITY
    // --------------------------------------------------------

    if ((uint32_t)(nowMs - lastUserActivityMs) <
        DEEP_SLEEP_TIMEOUT_MS)
    {
        return;
    }

    // Do not sleep if the MPU6050 has stopped providing
    // valid sensor samples. Motion detection must be available.
    if ((uint32_t)(nowMs - lastValidImuMs) >
        IMU_VALID_TIMEOUT_MS)
    {
        return;
    }

    // --------------------------------------------------------
    // 4. ENTER DEEP SLEEP
    // --------------------------------------------------------

    enterDeepSleep();
}

} // namespace PowerManager
