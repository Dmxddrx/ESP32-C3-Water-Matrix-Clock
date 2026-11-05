#include "general.h"
#include "project_config.h"
#include "max7219.h"
#include "icons.h"
#include "mpu6050.h"
#include "display_manager.h"
#include "wifi_manager.h"
#include "bluetooth_manager.h"
#include "analog_manager.h"
#include "sand_clock.h"
#include "water_animation.h"
#include "power_manager.h"

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
        case MatrixMode::HEART: return "MANSALA & UMESH";
        case MatrixMode::LEVEL: return "LEVEL";
        default: return "UNKNOWN";
    }
}

void cycleMatrixMode()
{
    switch (matrixMode)
    {
        case MatrixMode::HEART:

            matrixMode = MatrixMode::LEVEL;

            // Generate a fresh SandClock transfer plan.
            SandClock::reset(millis());

            break;


        case MatrixMode::LEVEL:

            matrixMode = MatrixMode::WATER;

            // Reset water distribution.
            WaterAnimation::reset(millis());

            break;


        case MatrixMode::WATER:

            matrixMode = MatrixMode::HEART;

            break;
    }

    // Allow the selected mode to render on the next loop.
    lastMatrixFrameMs = 0;

    DisplayManager::logMessage(
        "Matrix mode changed: %s",
        modeName(matrixMode)
    );
}

void onButtonPressed(uint8_t pin)
{
    const uint32_t now = millis();

    // Every physical button press counts as user activity.
    PowerManager::notifyUserActivity(now);

    if (pin == ProjectConfig::BUTTON_PAGE_PIN)
    {
        displayPage = (displayPage + 1U) % 3U;

        // Restart the one-minute automatic clock-page timer.
        PowerManager::notifyPageInteraction(now);

        DisplayManager::logMessage(
            "Display page: %u",
            displayPage
        );
    }
    else if (pin == ProjectConfig::BUTTON_MODE_PIN)
    {
        cycleMatrixMode();
    }
}

void updateButton(uint8_t pin, ButtonState &button, uint32_t now)
{
    const bool raw = digitalRead(pin);

    if (raw != button.rawState)
    {
        button.rawState = raw;
        button.rawChangedMs = now;
    }

    // Apply button debouncing.
    if ((uint32_t)(now - button.rawChangedMs) >=
            ProjectConfig::BUTTON_DEBOUNCE_MS &&
        button.stableState != button.rawState)
    {
        button.stableState = button.rawState;

        if (button.stableState == LOW)
        {
            onButtonPressed(pin);
        }
    }
}


void applyBluetoothCommand(const char *command)
{
    if (command == nullptr || command[0] == '\0') {
        return;
    }

    // A valid command from the mobile app counts as user activity.
    PowerManager::notifyUserActivity(millis());

    if (strcmp(command, "MODE=WATER") == 0)
    {
        matrixMode = MatrixMode::WATER;
        WaterAnimation::reset(millis());
    }
    else if (strcmp(command, "MODE=HEART") == 0)
    {
        matrixMode = MatrixMode::HEART;
    }
    else if (strcmp(command, "MODE=LEVEL") == 0)
    {
        matrixMode = MatrixMode::LEVEL;
        SandClock::reset(millis());
    }

    else if (strcmp(command, "WIFI_CONNECT") == 0)
    {
        char receivedSsid[33] = {};
        char receivedPassword[64] = {};

        // Retrieve the SSID and password received through BLE.
        if (!BluetoothManager::takeWifiCredentials(
                receivedSsid,
                sizeof(receivedSsid),
                receivedPassword,
                sizeof(receivedPassword)))
        {
            DisplayManager::logMessage(
                "BLE: WiFi provisioning rejected; credentials missing or invalid"
            );

            return;
        }

        // Save credentials and start connecting asynchronously.
        if (WifiManager::setCredentials(
                receivedSsid,
                receivedPassword))
        {
            DisplayManager::logMessage(
                "BLE: WiFi credentials accepted; connecting..."
            );
        }
        else
        {
            DisplayManager::logMessage(
                "BLE: unable to save WiFi credentials"
            );
        }
    }

    else if (strncmp(command, "PAGE=", 5) == 0) {
        const int value = atoi(command + 5);
        if (value >= 0 && value <= 2)
        {
            displayPage = (uint8_t)value;

            // Restart the automatic clock-page timer.
            PowerManager::notifyPageInteraction(millis());
        }
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

void drawMatrixFrame(uint32_t nowMs)
{
    if (matrixMode == MatrixMode::HEART)
    {
        MAX7219_Clear();

        MAX7219_DrawIcon(&Icon_Heart, 0, 0);
        MAX7219_DrawIcon(&Icon_Heart, 8, 0);

        const uint8_t pulse =
            ((frameCounter / 8U) % 2U)
                ? brightness
                : (brightness > 1 ? brightness - 1 : 1);

        MAX7219_SetBrightness(pulse);
        MAX7219_Update();

        MAX7219_SetBrightness(brightness);
    }
    else if (matrixMode == MatrixMode::WATER)
    {
        WaterAnimation::update(nowMs);
    }
    else if (matrixMode == MatrixMode::LEVEL)
    {
        // SandClock manages its own framebuffer and MAX7219 updates.
        SandClock::update(nowMs);
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

void testSandFlowPattern()
{
    constexpr uint16_t DIAGONAL_STEP_MS = 95;
    constexpr uint16_t FILL_STEP_MS = 100;
    constexpr uint16_t EMPTY_STEP_MS = 75;

    // --------------------------------------------------------
    // STARTUP: CLEAR BOTH MATRICES
    // --------------------------------------------------------

    MAX7219_Clear();
    MAX7219_Update();

    delay(300);


    // --------------------------------------------------------
    // STAGE 1: SIMULTANEOUS DIAGONAL SWEEP
    // --------------------------------------------------------

    /*
     * TOP SLAVE
     * Physical LEFT -> RIGHT
     * Logical (0,0) -> (7,7)
     *
     * BOTTOM MASTER
     * Physical LEFT -> RIGHT
     * Logical (8,7) -> (15,0)
     *
     * Both diagonal patterns illuminate simultaneously.
     * Previously illuminated pixels remain ON, creating
     * a growing diagonal trail.
     */

    for (int8_t i = 0; i < 8; ++i)
    {
        // Top slave diagonal.
        MAX7219_SetPixel(i, i, 1);

        // Bottom master diagonal.
        MAX7219_SetPixel(8 + i, 7 - i, 1);

        MAX7219_Update();

        delay(DIAGONAL_STEP_MS);
    }

    delay(300);


    // --------------------------------------------------------
    // STAGE 2: CENTER-OUTWARD MATRIX EXPANSION
    // --------------------------------------------------------

    /*
     * Begin filling from the two central rows.
     *
     * Expansion sequence:
     *
     *     Rows 3 and 4
     *     Rows 2 and 5
     *     Rows 1 and 6
     *     Rows 0 and 7
     *
     * Both 8x8 modules expand simultaneously.
     *
     * When this stage finishes, all 128 LEDs are ON.
     */

    for (int8_t offset = 0; offset < 4; ++offset)
    {
        const uint8_t upperRow = 3 - offset;
        const uint8_t lowerRow = 4 + offset;

        // Fill both rows across the complete 16x8 framebuffer.
        for (uint8_t x = 0; x < 16; ++x)
        {
            MAX7219_SetPixel(x, upperRow, 1);
            MAX7219_SetPixel(x, lowerRow, 1);
        }

        MAX7219_Update();

        delay(FILL_STEP_MS);
    }

    // Hold the completely illuminated display.
    delay(500);


    // --------------------------------------------------------
    // STAGE 3: CENTER-OUTWARD RECTANGULAR CLEARING
    // --------------------------------------------------------

    /*
    * Clear the complete 16x8 framebuffer using expanding
    * rectangular layers.
    *
    * Clearing sequence:
    *
    *     1. Central 2x2 region
    *     2. First surrounding rectangular layer
    *     3. Second surrounding rectangular layer
    *     4. Continue expanding towards the outer edges
    *
    * The clearing begins at the centre of both matrices
    * and gradually spreads towards all four edges.
    *
    * Every LED is switched OFF exactly once.
    */

    for (uint8_t layer = 0; layer < 8; ++layer)
    {
        for (uint8_t y = 0; y < 8; ++y)
        {
            for (uint8_t x = 0; x < 16; ++x)
            {
                // Calculate distance from the central 2x2 region.
                const uint8_t distanceX =
                    (x < 8) ? (7 - x) : (x - 8);

                const uint8_t distanceY =
                    (y < 4) ? (3 - y) : (y - 4);

                // Chebyshev distance creates rectangular layers.
                const uint8_t distance =
                    (distanceX > distanceY)
                        ? distanceX
                        : distanceY;

                // Turn OFF only the current rectangular layer.
                if (distance == layer)
                {
                    MAX7219_SetPixel(x, y, 0);
                }
            }
        }

        MAX7219_Update();

        delay(EMPTY_STEP_MS);
    }
    MAX7219_Update();
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
    // --------------------------------------------------------
    // 1. INITIALIZE ANALOG / POWER MONITORING
    // --------------------------------------------------------

    AnalogManager::begin();


    // --------------------------------------------------------
    // 2. INITIALIZE BUTTONS
    // --------------------------------------------------------

    pinMode(ProjectConfig::BUTTON_PAGE_PIN, INPUT_PULLUP);
    pinMode(ProjectConfig::BUTTON_MODE_PIN, INPUT_PULLUP);   

    const uint32_t now = millis();

    pageButton.rawState = pageButton.stableState =
        digitalRead(ProjectConfig::BUTTON_PAGE_PIN);
    pageButton.rawChangedMs = now;

    modeButton.rawState = modeButton.stableState =
        digitalRead(ProjectConfig::BUTTON_MODE_PIN);
    modeButton.rawChangedMs = now;

  

    // --------------------------------------------------------
    // 3. INITIALIZE I2C
    // --------------------------------------------------------

    Wire.begin(
        ProjectConfig::I2C_SDA_PIN,
        ProjectConfig::I2C_SCL_PIN,
        ProjectConfig::I2C_FREQUENCY_HZ
    );

    Wire.setTimeOut(50);


    // --------------------------------------------------------
    // 4. INITIALIZE OLED DISPLAY
    // --------------------------------------------------------

    DisplayManager::begin();

    DisplayManager::logMessage("ESP32-C3 Water Matrix Clock");
    DisplayManager::logMessage("Starting peripherals...");


    // --------------------------------------------------------
    // 5. INITIALIZE MAX7219 MATRIX
    // --------------------------------------------------------

    DisplayManager::logMessage("Initializing MAX7219...");

    MAX7219_Init();

    // Use the same brightness variable used by the animations.
    brightness = 1;
    MAX7219_SetBrightness(brightness);


    // --------------------------------------------------------
    // 6. INITIALIZE MPU6050
    // --------------------------------------------------------

    DisplayManager::logMessage("Initializing MPU6050...");

    MPU6050::begin();


    // --------------------------------------------------------
    // 7. INITIALIZE MATRIX ANIMATIONS
    // --------------------------------------------------------

    // Initialize water animation after the MPU6050.
    DisplayManager::logMessage("Initializing water animation...");
    WaterAnimation::begin();

    // Initialize digital sand clock after the MPU6050.
    DisplayManager::logMessage("Initializing sand clock...");
    SandClock::begin();


    // --------------------------------------------------------
    // 8. INITIALIZE WI-FI
    // --------------------------------------------------------

    DisplayManager::logMessage("Initializing Wi-Fi...");

    WifiManager::begin();
    // Start tracking the Wi-Fi offline duration.
    

    // --------------------------------------------------------
    // 9. INITIALIZE BLUETOOTH
    // --------------------------------------------------------

    DisplayManager::logMessage("Initializing Bluetooth...");

    if (BluetoothManager::begin())
    {
        DisplayManager::logMessage(
            "BLE: continuous advertising enabled"
        );
    }
    else
    {
        DisplayManager::logMessage(
            "BLE: initialization failed"
        );
    }


    // --------------------------------------------------------
    // 10. INITIALIZE PERIODIC UPDATE TIMERS
    // --------------------------------------------------------

    lastMatrixFrameMs = millis();

    lastDisplayRefreshMs =
        millis() - ProjectConfig::DISPLAY_REFRESH_MS;

    lastBleStatusMs =
        millis() - ProjectConfig::BLE_STATUS_REFRESH_MS;


    // --------------------------------------------------------
    // 11. TEST MAX7219 LED ORIENTATION
    // --------------------------------------------------------

    DisplayManager::logMessage("Testing sand flow direction...");

    testSandFlowPattern();


    // --------------------------------------------------------
    // 12. DRAW INITIAL MATRIX FRAME
    // --------------------------------------------------------

    DisplayManager::logMessage("Drawing initial matrix frame...");

    drawMatrixFrame(millis());


    // --------------------------------------------------------
    // INITIALIZE POWER MANAGEMENT
    // --------------------------------------------------------

    // Begin the inactivity, motion and automatic page timers.
    // Start the timer only after all peripherals are initialized.
    PowerManager::begin(millis());

    // --------------------------------------------------------
    // INITIALIZATION COMPLETE
    // --------------------------------------------------------

    DisplayManager::logMessage("Initialization complete.");
}


void general_run(void)
{
    const uint32_t now = millis();

    // --------------------------------------------------------
    // UPDATE CONNECTIVITY
    // --------------------------------------------------------

    WifiManager::update();
    BluetoothManager::update();


    // --------------------------------------------------------
    // UPDATE MPU6050
    // --------------------------------------------------------

    MPU6050::update();


    // --------------------------------------------------------
    // UPDATE PHYSICAL BUTTONS
    // --------------------------------------------------------

    updateButton(
        ProjectConfig::BUTTON_PAGE_PIN,
        pageButton,
        now
    );

    updateButton(
        ProjectConfig::BUTTON_MODE_PIN,
        modeButton,
        now
    );


    // --------------------------------------------------------
    // PROCESS BLUETOOTH COMMANDS
    // --------------------------------------------------------

    char command[48];

    if (BluetoothManager::takeCommand(
            command,
            sizeof(command)))
    {
        applyBluetoothCommand(command);
    }


    // --------------------------------------------------------
    // UPDATE POWER MANAGEMENT
    // --------------------------------------------------------

    PowerManager::update(
        now,
        displayPage,
        MPU6050::getData()
    );


    // --------------------------------------------------------
    // UPDATE OLED AND BLUETOOTH STATUS
    // --------------------------------------------------------

    // Do not update the OLED framebuffer while the panel is OFF.
    if (!PowerManager::isIdle())
    {
        updateDisplay(now);
    }

    // BLE status remains available while the OLED is idle.
    updateBleStatus(now);


    // --------------------------------------------------------
    // UPDATE POWER MONITOR
    // --------------------------------------------------------

    updatePowerMonitor(now);


    // --------------------------------------------------------
    // UPDATE OLED AND BLUETOOTH STATUS
    // --------------------------------------------------------

    // Do not redraw the OLED while the panel is switched OFF.
    if (!PowerManager::isIdle())
    {
        updateDisplay(now);
    }

    // BLE status remains active even during display-idle mode.
    updateBleStatus(now);
}

MatrixMode general_getMatrixMode(void) { return matrixMode; }
const char *general_getMatrixModeName(void) { return modeName(matrixMode); }
