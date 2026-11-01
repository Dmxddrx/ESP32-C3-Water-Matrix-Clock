#include "analog_manager.h"
#include <math.h>

// ADC configuration
static constexpr uint8_t ADC_RESOLUTION = 12;
static constexpr uint8_t ADC_SAMPLE_COUNT = 20;

// External power detection debounce
static constexpr uint32_t POWER_DEBOUNCE_MS = 50;


// Typical single-cell 4.2V Li-ion OCV/SOC curve.
// Voltage is in volts; percentage is state of charge.
struct BatteryCurvePoint {
    float voltage;
    uint8_t percentage;
};

static constexpr BatteryCurvePoint BATTERY_CURVE[] = {
    {3.305545f,   0},
    {3.674776f,   5},
    {3.686654f,  10},
    {3.709800f,  15},
    {3.741018f,  20},
    {3.762185f,  25},
    {3.775129f,  30},
    {3.783504f,  35},
    {3.793250f,  40},
    {3.805737f,  45},
    {3.820965f,  50},
    {3.841219f,  55},
    {3.884009f,  60},
    {3.917968f,  65},
    {3.945074f,  70},
    {3.974769f,  75},
    {4.008118f,  80},
    {4.045427f,  85},
    {4.085934f,  90},
    {4.129486f,  95},
    {4.177454f, 100}
};

static constexpr uint8_t BATTERY_CURVE_SIZE =
    sizeof(BATTERY_CURVE) / sizeof(BATTERY_CURVE[0]);


static bool powerStableState = false;
static bool lastPowerReading = false;

static uint32_t lastPowerChangeMs = 0;


// --------------------------------------------------
// INITIALIZATION
// --------------------------------------------------

void AnalogManager::begin()
{
    analogReadResolution(ADC_RESOLUTION);

    pinMode(BATTERY_PIN, INPUT);

    analogSetPinAttenuation(
        BATTERY_PIN,
        ADC_11db
    );

    pinMode(POWER_DETECT_PIN, INPUT);

    lastPowerReading =
        (digitalRead(POWER_DETECT_PIN) == HIGH);

    powerStableState = lastPowerReading;
    lastPowerChangeMs = millis();
}


// --------------------------------------------------
// BATTERY ADC READING
// --------------------------------------------------

uint32_t AnalogManager::readBatteryAdcMilliVolts()
{
    uint32_t total = 0;

    for (uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++)
    {
        total += analogReadMilliVolts(BATTERY_PIN);
        delayMicroseconds(200);
    }

    return total / ADC_SAMPLE_COUNT;
}


// --------------------------------------------------
// ACTUAL BATTERY VOLTAGE
// --------------------------------------------------

float AnalogManager::readBatteryVoltage()
{
    const uint32_t adcMilliVolts =
        readBatteryAdcMilliVolts();

    const float adcVoltage =
        adcMilliVolts / 1000.0f;

    return adcVoltage * BATTERY_DIVIDER_RATIO;
}


// --------------------------------------------------
// VOLTAGE TO BATTERY PERCENTAGE
// --------------------------------------------------

uint8_t AnalogManager::voltageToPercentage(float voltage)
{
    if (!isfinite(voltage)) {
        return 0;
    }

    // Below the lowest curve voltage.
    if (voltage <= BATTERY_CURVE[0].voltage) {
        return 0;
    }

    // Above the highest curve voltage.
    if (voltage >= BATTERY_CURVE[BATTERY_CURVE_SIZE - 1].voltage) {
        return 100;
    }

    // Find the two surrounding curve points.
    for (uint8_t i = 1; i < BATTERY_CURVE_SIZE; i++)
    {
        if (voltage <= BATTERY_CURVE[i].voltage)
        {
            const float v1 = BATTERY_CURVE[i - 1].voltage;
            const float v2 = BATTERY_CURVE[i].voltage;

            const float p1 = BATTERY_CURVE[i - 1].percentage;
            const float p2 = BATTERY_CURVE[i].percentage;

            // Linear interpolation between the two points.
            const float ratio = (voltage - v1) / (v2 - v1);

            const float percentage =
                p1 + ratio * (p2 - p1);

            return (uint8_t)(percentage + 0.5f);
        }
    }

    return 0;
}


// --------------------------------------------------
// EXTERNAL POWER DETECTION
// --------------------------------------------------

bool AnalogManager::isExternalPowerPresent()
{
    const bool currentReading =
        (digitalRead(POWER_DETECT_PIN) == HIGH);

    if (currentReading != lastPowerReading)
    {
        lastPowerReading = currentReading;
        lastPowerChangeMs = millis();
    }

    if ((uint32_t)(millis() - lastPowerChangeMs)
        >= POWER_DEBOUNCE_MS)
    {
        powerStableState = lastPowerReading;
    }

    return powerStableState;
}