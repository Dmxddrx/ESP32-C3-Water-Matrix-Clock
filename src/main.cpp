#include <Arduino.h>

#include "general.h"

#include <esp_task_wdt.h>
#include <esp_err.h>

// ------------------------------------------------------------
// APPLICATION TASK WATCHDOG
// ------------------------------------------------------------

// Restart the ESP32 if the main application loop does not
// feed the watchdog within this interval.
constexpr uint32_t LOOP_WATCHDOG_TIMEOUT_MS = 10000;

bool watchdogReady = false;


// ------------------------------------------------------------
// INITIALIZE WATCHDOG
// ------------------------------------------------------------

void initializeWatchdog()
{
    esp_task_wdt_config_t watchdogConfig = {};

    watchdogConfig.timeout_ms = LOOP_WATCHDOG_TIMEOUT_MS;

    // ESP32-C3 is a single-core processor (CPU0).
    watchdogConfig.idle_core_mask = 1;

    // Trigger the panic handler when the watchdog expires.
    watchdogConfig.trigger_panic = true;

    // Try initializing the Task Watchdog.
    esp_err_t result =
        esp_task_wdt_init(&watchdogConfig);

    // The Arduino core may have initialized it already.
    // In that case, update its configuration instead.
    if (result == ESP_ERR_INVALID_STATE)
    {
        result = esp_task_wdt_reconfigure(
            &watchdogConfig
        );
    }

    if (result != ESP_OK)
    {
        Serial.printf(
            "WDT: initialization failed: %d\n",
            (int)result
        );

        return;
    }

    // Check whether the current Arduino loop task is already
    // subscribed to the watchdog.
    result = esp_task_wdt_status(NULL);

    if (result != ESP_OK)
    {
        // Subscribe the current loop task.
        result = esp_task_wdt_add(NULL);
    }

    if (result == ESP_OK ||
        esp_task_wdt_status(NULL) == ESP_OK)
    {
        watchdogReady = true;

        Serial.println("WDT: initialized successfully");
    }
    else
    {
        Serial.printf(
            "WDT: unable to subscribe loop task: %d\n",
            (int)result
        );
    }
}


// ------------------------------------------------------------
// ARDUINO SETUP
// ------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Initialize watchdog before starting the application.
    initializeWatchdog();

    general_init();
}


// ------------------------------------------------------------
// ARDUINO MAIN LOOP
// ------------------------------------------------------------

void loop()
{
    // Execute the main application.
    general_run();

    // Feed watchdog only after the main application completes.
    // If general_run() hangs, this line will not execute.
    if (watchdogReady)
    {
        esp_task_wdt_reset();
    }
}