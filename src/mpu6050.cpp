#include "mpu6050.h"
#include "project_config.h"
#include <Wire.h>
#include <math.h>

namespace MPU6050 {
namespace {
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;
constexpr float ACCEL_SCALE_G = 1.0f / 16384.0f;  // +/-2 g
constexpr float GYRO_SCALE_DPS = 1.0f / 131.0f;   // +/-250 deg/s
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr uint16_t CALIBRATION_SAMPLES = 100;

bool ready = false;
uint8_t deviceAddress = ProjectConfig::MPU6050_ADDRESS_LOW;
Data latest;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
uint32_t lastSampleMs = 0;
bool filterInitialized = false;

bool writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t *buffer, size_t length)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    const size_t received = Wire.requestFrom(deviceAddress, length, true);
    if (received != length) {
        while (Wire.available()) (void)Wire.read();
        return false;
    }
    for (size_t i = 0; i < length; ++i) buffer[i] = (uint8_t)Wire.read();
    return true;
}

int16_t readSigned16(const uint8_t highByte, const uint8_t lowByte)
{
    return (int16_t)(((uint16_t)highByte << 8) | lowByte);
}

bool readRaw(Data &out)
{
    uint8_t b[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
    out.axRaw = readSigned16(b[0], b[1]);
    out.ayRaw = readSigned16(b[2], b[3]);
    out.azRaw = readSigned16(b[4], b[5]);
    out.temperatureRaw = readSigned16(b[6], b[7]);
    out.gxRaw = readSigned16(b[8], b[9]);
    out.gyRaw = readSigned16(b[10], b[11]);
    out.gzRaw = readSigned16(b[12], b[13]);
    return true;
}

void calibrateGyro()
{
    int64_t sx = 0, sy = 0, sz = 0;
    uint16_t good = 0;
    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
        Data sample;
        if (readRaw(sample)) {
            sx += sample.gxRaw;
            sy += sample.gyRaw;
            sz += sample.gzRaw;
            ++good;
        }
        delay(5);
        yield();
    }
    if (good > 0) {
        gyroBiasX = (float)sx / good;
        gyroBiasY = (float)sy / good;
        gyroBiasZ = (float)sz / good;
    }
}
}

bool begin()
{
    ready = false;
    uint8_t id = 0;
    bool found = false;
    const uint8_t candidates[] = {ProjectConfig::MPU6050_ADDRESS_LOW,
                                  ProjectConfig::MPU6050_ADDRESS_HIGH};
    for (uint8_t addr : candidates) {
        deviceAddress = addr;
        if (readRegisters(REG_WHO_AM_I, &id, 1) &&
            (id == 0x68 || id == 0x70 || id == 0x71 || id == 0x73)) {
            found = true;
            break;
        }
    }
    if (!found) {
        Serial.println(F("MPU6050: not detected at 0x68 or 0x69"));
        return false;
    }

    // Reset, wake with the X-gyro PLL, then configure a 100 Hz sample rate.
    if (!writeRegister(REG_PWR_MGMT_1, 0x80)) return false;
    delay(100);
    if (!writeRegister(REG_PWR_MGMT_1, 0x01) ||
        !writeRegister(REG_SMPLRT_DIV, 0x09) ||
        !writeRegister(REG_CONFIG, 0x03) ||
        !writeRegister(REG_GYRO_CONFIG, 0x00) ||
        !writeRegister(REG_ACCEL_CONFIG, 0x00)) {
        Serial.println(F("MPU6050: configuration write failed"));
        return false;
    }
    delay(50);
    Serial.println(F("MPU6050: keep device still; calibrating gyro bias..."));
    calibrateGyro();
    latest = Data{};
    filterInitialized = false;
    lastSampleMs = millis();
    ready = true;
    Serial.printf("MPU6050: ready at 0x%02X (WHO_AM_I=0x%02X)\n", deviceAddress, id);
    return true;
}

void update()
{
    if (!ready) return;
    const uint32_t now = millis();
    if ((uint32_t)(now - lastSampleMs) < ProjectConfig::SENSOR_SAMPLE_MS) return;

    Data sample;
    if (!readRaw(sample)) {
        latest.valid = false;
        return;
    }

    sample.axG = sample.axRaw * ACCEL_SCALE_G;
    sample.ayG = sample.ayRaw * ACCEL_SCALE_G;
    sample.azG = sample.azRaw * ACCEL_SCALE_G;
    sample.gxDps = (sample.gxRaw - gyroBiasX) * GYRO_SCALE_DPS;
    sample.gyDps = (sample.gyRaw - gyroBiasY) * GYRO_SCALE_DPS;
    sample.gzDps = (sample.gzRaw - gyroBiasZ) * GYRO_SCALE_DPS;
    sample.temperatureC = (sample.temperatureRaw / 340.0f) + 36.53f;

    const float accelRoll = atan2f(sample.ayG, sample.azG) * RAD_TO_DEG_F;
    const float accelPitch = atan2f(-sample.axG,
        sqrtf(sample.ayG * sample.ayG + sample.azG * sample.azG)) * RAD_TO_DEG_F;
    const float dt = (now - lastSampleMs) / 1000.0f;

    if (!filterInitialized || dt <= 0.0f || dt > 0.25f) {
        sample.rollDeg = accelRoll;
        sample.pitchDeg = accelPitch;
        filterInitialized = true;
    } else {
        // Complementary filter: gyro provides responsive motion; accelerometer
        // corrects long-term pitch/roll drift when gravity is the main acceleration.
        sample.rollDeg = 0.98f * (latest.rollDeg + sample.gxDps * dt) + 0.02f * accelRoll;
        sample.pitchDeg = 0.98f * (latest.pitchDeg + sample.gyDps * dt) + 0.02f * accelPitch;
    }

    sample.timestampMs = now;
    sample.valid = true;
    latest = sample;
    lastSampleMs = now;
}

bool isReady() { return ready; }
uint8_t address() { return deviceAddress; }
const Data &getData() { return latest; }
}
