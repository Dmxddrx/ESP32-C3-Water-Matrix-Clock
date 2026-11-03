
#include "power_manager.h"

#include "max7219.h"
#include "display_manager.h"
#include "wifi_manager.h"
#include "bluetooth_manager.h"
#include "mpu6050.h"

#include <Arduino.h>
#include <math.h>
#include <stdint.h>

namespace PowerManager {
namespace {

// ------------------------------------------------------------
// POWER MANAGEMENT TIMING
// ------------------------------------------------------------

// Return to the clock page after one minute without page interaction.
constexpr uint32_t CLOCK_PAGE_TIMEOUT_MS = 60UL * 1000UL;

// Enter display idle mode after 15 minutes without user activity or motion.
constexpr uint32_t DISPLAY_IDLE_TIMEOUT_MS = 15UL * 60UL * 1000UL;

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

// Do not allow idle mode if the MPU6050 stops providing valid samples.
constexpr uint32_t IMU_VALID_TIMEOUT_MS = 500;


// ------------------------------------------------------------
// ACTIVITY TIMERS
// ------------------------------------------------------------

uint32_t lastUserActivityMs = 0;
uint32_t lastPageInteractionMs = 0;
uint32_t lastMotionEventMs = 0;
uint32_t lastValidImuMs = 0;

bool initialized = false;

// True when OLED and MAX7219 displays are switched off.
bool idleMode = false;


// ------------------------------------------------------------
// MPU6050 MOTION TRACKING
// ------------------------------------------------------------

bool previousMotionSampleValid = false;

int16_t previousAxRaw = 0;
int16_t previousAyRaw = 0;
int16_t previousAzRaw = 0;

uint32_t lastProcessedImuTimestamp = 0;


// ------------------------------------------------------------
// BLUETOOTH CONNECTION TRACKING
// ------------------------------------------------------------

// Used to detect when a phone connects while the clock is idle.
bool previousBleConnected = false;


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
// ENTER DISPLAY IDLE MODE
// ------------------------------------------------------------

/*
 * ENTER DISPLAY IDLE MODE
 *
 * After 15 minutes without user activity:
 *
 * 1. Turn OFF both MAX7219 modules.
 * 2. Turn OFF the OLED panel.
 * 3. Disconnect Wi-Fi and disable its radio.
 *
 * Bluetooth remains advertising and connected clients
 * remain available.
 *
 * MPU6050 remains active for motion detection.
 *
 * The ESP32 system clock continues running without Wi-Fi.
 *
 * IMPORTANT:
 *     This is NOT ESP32 deep sleep.
 *     The CPU and application loop continue running.
 *
 *     The MAX7219 display RAM is preserved during shutdown.
 *     This allows the previous matrix frame to reappear
 *     immediately when normal operation resumes.
 */
void enterIdleMode()
{
    if (idleMode) {
        return;
    }

    // --------------------------------------------------------
    // TURN OFF MAX7219 MATRICES
    // --------------------------------------------------------

    // Do not clear the MAX7219 framebuffer here.
    // Hardware shutdown preserves its display RAM.
    MAX7219_SetShutdown(true);


    // --------------------------------------------------------
    // TURN OFF OLED
    // --------------------------------------------------------

    DisplayManager::prepareForSleep();


    // --------------------------------------------------------
    // DISCONNECT WIFI
    // --------------------------------------------------------

    WifiManager::shutdown();


    // --------------------------------------------------------
    // KEEP BLUETOOTH AND MPU6050 RUNNING
    // --------------------------------------------------------

    // Do NOT call BluetoothManager::shutdown().
    // Do NOT call MPU6050::sleep().
    // Do NOT call esp_deep_sleep_start().

    idleMode = true;

    DisplayManager::logMessage(
        "Power: display idle mode entered; BLE remains active"
    );
}


// ------------------------------------------------------------
// EXIT DISPLAY IDLE MODE
// ------------------------------------------------------------

/*
 * EXIT DISPLAY IDLE MODE
 *
 * Restore the displays and resume Wi-Fi.
 *
 * This is called when:
 *     - A physical button is pressed.
 *     - Significant device movement is detected.
 *     - A phone connects through BLE.
 *     - A Bluetooth command is received.
 */
void exitIdleMode()
{
    if (!idleMode) {
        return;
    }

    idleMode = false;


    // --------------------------------------------------------
    // RESTORE MAX7219 MATRICES
    // --------------------------------------------------------

    // Restore the preserved MAX7219 framebuffer.
    MAX7219_SetShutdown(false);


    // --------------------------------------------------------
    // RESTORE OLED
    // --------------------------------------------------------

    DisplayManager::wakeFromSleep();


    // --------------------------------------------------------
    // RESUME WIFI
    // --------------------------------------------------------

    WifiManager::resume();


    DisplayManager::logMessage(
        "Power: display idle mode exited"
    );
}

} // namespace (closes the anonymous namespace)


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

    idleMode = false;

    // Record the current BLE connection state.
    previousBleConnected = BluetoothManager::isConnected();
}


// ------------------------------------------------------------
// REGISTER USER ACTIVITY
// ------------------------------------------------------------

/*
 * Any physical button, BLE command or significant movement
 * counts as user activity.
 *
 * If the clock is in display-idle mode, restore the displays
 * and resume Wi-Fi.
 */
void notifyUserActivity(uint32_t nowMs)
{
    lastUserActivityMs = nowMs;

    // Restore the device when activity is detected.
    exitIdleMode();
}


// ------------------------------------------------------------
// REGISTER PAGE INTERACTION
// ------------------------------------------------------------

/*
 * A manual OLED page change restarts both:
 *
 * 1. The one-minute clock-page timer.
 * 2. The 15-minute inactivity timer.
 */
void notifyPageInteraction(uint32_t nowMs)
{
    lastPageInteractionMs = nowMs;

    notifyUserActivity(nowMs);
}


// ------------------------------------------------------------
// CHECK DISPLAY-IDLE STATE
// ------------------------------------------------------------

/*
 * Returns true when OLED and MAX7219 displays are switched OFF.
 *
 * general.cpp should use this function to prevent normal
 * matrix animation rendering while the clock is idle.
 */
bool isIdle()
{
    return idleMode;
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
    // 1. DETECT BLUETOOTH CONNECTION
    // --------------------------------------------------------

    const bool bleConnected =
        BluetoothManager::isConnected();

    // A newly connected phone wakes the clock.
    if (bleConnected && !previousBleConnected)
    {
        notifyUserActivity(nowMs);

        DisplayManager::logMessage(
            "Power: mobile phone connected; waking display"
        );
    }

    previousBleConnected = bleConnected;


    // --------------------------------------------------------
    // 2. DETECT LARGE PHYSICAL MOTION
    // --------------------------------------------------------

    if (detectLargeMotion(imu, nowMs))
    {
        // Significant movement restarts the inactivity timer.
        notifyUserActivity(nowMs);
    }


    // --------------------------------------------------------
    // 3. AUTOMATICALLY RETURN TO CLOCK PAGE
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
    // 4. CHECK DISPLAY-IDLE INACTIVITY
    // --------------------------------------------------------

    if (idleMode) {
        return;
    }

    if ((uint32_t)(nowMs - lastUserActivityMs) <
        DISPLAY_IDLE_TIMEOUT_MS)
    {
        return;
    }

    // Do not enter idle mode if the MPU6050 has stopped
    // providing valid samples. Movement detection must remain active.
    if ((uint32_t)(nowMs - lastValidImuMs) >
        IMU_VALID_TIMEOUT_MS)
    {
        return;
    }


    // --------------------------------------------------------
    // 5. ENTER DISPLAY IDLE MODE
    // --------------------------------------------------------

    enterIdleMode();
}

} // namespace PowerManager
