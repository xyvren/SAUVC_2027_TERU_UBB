#include "rov_sensors.h"

#include <Arduino.h>
#include <Wire.h>
#include <hardware/adc.h>
#include <hardware/gpio.h>
#include <math.h>
#include <pico/critical_section.h>
#include <pico/cyw43_arch.h>
#include <string.h>

#include "cli_engine.h"
#include "rov_config.h"

namespace ROVSensors {
namespace {

State sensorState{};
State publishedState{};
bool simActive = false;
State simSensorState{};
float simWaterDensityVal = 1000.0f;
float simRefPressureVal = 1013.25f;
critical_section_t stateLock;
bool stateLockInitialized = false;
uint8_t imuAddress = ROVConfig::MPU6050_ADDRESS;
uint8_t compassAddress = 0;
uint8_t pressureAddress = 0;
uint16_t pressureCoefficients[8]{};
uint32_t rawPressure = 0;
uint32_t rawTemperature = 0;
uint32_t pressureConversionStartedAt = 0;
uint32_t lastPressureCycleAt = 0;
bool pressureConversionPending = false;
bool pressureConversionIsPressure = true;
bool attitudeInitialized = false;
uint32_t lastReadAt = 0;
uint32_t previousReadAt = 0;
uint32_t lastImuRetryAt = 0;
uint32_t lastCompassRetryAt = 0;
uint32_t lastPressureRetryAt = 0;
uint32_t lastVsysReadAt = 0;
constexpr uint32_t SENSOR_RETRY_INTERVAL_MS = 1500;

bool probe(uint8_t address) {
  Wire1.beginTransmission(address);
  return Wire1.endTransmission() == 0;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission() == 0;
}

bool readRegisters(uint8_t address, uint8_t reg, uint8_t* values,
                   size_t length) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0 ||
      Wire1.requestFrom(address, length) != length) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    if (!Wire1.available()) {
      return false;
    }
    values[i] = static_cast<uint8_t>(Wire1.read());
  }
  return true;
}

int16_t readSignedBigEndian(const uint8_t* bytes) {
  return static_cast<int16_t>(
      (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}

int16_t readSignedLittleEndian(const uint8_t* bytes) {
  return static_cast<int16_t>(
      (static_cast<uint16_t>(bytes[1]) << 8) | bytes[0]);
}

bool sendCommand(uint8_t address, uint8_t command) {
  Wire1.beginTransmission(address);
  Wire1.write(command);
  return Wire1.endTransmission() == 0;
}

bool readAdc(uint32_t& result) {
  uint8_t bytes[3]{};
  if (!readRegisters(pressureAddress, 0x00, bytes, sizeof(bytes))) {
    return false;
  }
  result = (static_cast<uint32_t>(bytes[0]) << 16) |
      (static_cast<uint32_t>(bytes[1]) << 8) | bytes[2];
  return true;
}

uint8_t calculatePromCrc4(const uint16_t* coefficients) {
  uint16_t prom[8];
  memcpy(prom, coefficients, sizeof(prom));
  prom[7] &= 0xFF00;

  uint16_t remainder = 0;
  for (uint8_t byteIndex = 0; byteIndex < 16; ++byteIndex) {
    remainder ^= (byteIndex & 1)
        ? static_cast<uint16_t>(prom[byteIndex >> 1] & 0x00FF)
        : static_cast<uint16_t>(prom[byteIndex >> 1] >> 8);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      remainder = (remainder & 0x8000)
          ? static_cast<uint16_t>((remainder << 1) ^ 0x3000)
          : static_cast<uint16_t>(remainder << 1);
    }
  }
  return static_cast<uint8_t>((remainder >> 12) & 0x0F);
}

bool initializePressureAt(uint8_t address) {
  if (!probe(address) || !sendCommand(address, 0x1E)) {
    return false;
  }
  delay(3);

  uint16_t coefficients[8]{};
  for (uint8_t index = 0; index < 8; ++index) {
    uint8_t bytes[2]{};
    if (!readRegisters(address, static_cast<uint8_t>(0xA0 + index * 2),
                       bytes, sizeof(bytes))) {
      return false;
    }
    coefficients[index] =
        (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
  }
  if (calculatePromCrc4(coefficients) != (coefficients[7] & 0x0F)) {
    return false;
  }
  memcpy(pressureCoefficients, coefficients, sizeof(pressureCoefficients));
  pressureAddress = address;
  return true;
}

bool initializePressureSensor() {
  if (initializePressureAt(ROVConfig::MS5803_ADDRESS_HIGH) ||
      initializePressureAt(ROVConfig::MS5803_ADDRESS_LOW)) {
    sensorState.pressureReady = true;
    return true;
  }
  return false;
}

void calculatePressureAndTemperature() {
  const int64_t deltaTemperature = static_cast<int64_t>(rawTemperature) -
      (static_cast<int64_t>(pressureCoefficients[5]) << 8);
  int64_t temperature = 2000 +
      ((deltaTemperature * pressureCoefficients[6]) >> 23);
  int64_t offset = (static_cast<int64_t>(pressureCoefficients[2]) << 16) +
      ((static_cast<int64_t>(pressureCoefficients[4]) * deltaTemperature) >> 7);
  int64_t sensitivity = (static_cast<int64_t>(pressureCoefficients[1]) << 15) +
      ((static_cast<int64_t>(pressureCoefficients[3]) * deltaTemperature) >> 8);

  int64_t temperatureCorrection = 0;
  int64_t offsetCorrection = 0;
  int64_t sensitivityCorrection = 0;
  if (temperature < 2000) {
    temperatureCorrection =
        3 * ((deltaTemperature * deltaTemperature) >> 33);
    const int64_t temperatureDelta = temperature - 2000;
    offsetCorrection = 3 * temperatureDelta * temperatureDelta / 2;
    sensitivityCorrection = 5 * temperatureDelta * temperatureDelta / 8;
    if (temperature < -1500) {
      const int64_t coldDelta = temperature + 1500;
      offsetCorrection += 7 * coldDelta * coldDelta;
      sensitivityCorrection += 4 * coldDelta * coldDelta;
    }
  } else {
    temperatureCorrection =
        7 * (deltaTemperature * deltaTemperature) / (1LL << 37);
    const int64_t temperatureDelta = temperature - 2000;
    offsetCorrection = temperatureDelta * temperatureDelta / 16;
  }

  temperature -= temperatureCorrection;
  offset -= offsetCorrection;
  sensitivity -= sensitivityCorrection;
  const int64_t pressure =
      (((sensitivity * rawPressure) / (1LL << 21)) - offset) / (1LL << 15);
  sensorState.temperatureC = static_cast<float>(temperature) / 100.0f;
  sensorState.pressureMbar = static_cast<float>(pressure) / 10.0f;
  sensorState.pressureTimestampMs = millis();
  sensorState.pressureSampleCount++;
}

bool updatePressureSensor(uint32_t now) {
  if (!sensorState.pressureReady) {
    return false;
  }

  if (pressureConversionPending) {
    if (now - pressureConversionStartedAt <
        ROVConfig::MS5803_CONVERSION_TIME_MS) {
      return false;
    }
    uint32_t conversionResult = 0;
    if (!readAdc(conversionResult)) {
      sensorState.pressureReady = false;
      pressureConversionPending = false;
      return true;
    }
    if (pressureConversionIsPressure) {
      rawPressure = conversionResult;
      if (!sendCommand(pressureAddress,
                       ROVConfig::MS5803_TEMPERATURE_4096_COMMAND)) {
        sensorState.pressureReady = false;
        pressureConversionPending = false;
        return true;
      }
      pressureConversionIsPressure = false;
      pressureConversionStartedAt = now;
      return false;
    }

    rawTemperature = conversionResult;
    calculatePressureAndTemperature();
    pressureConversionPending = false;
    pressureConversionIsPressure = true;
    lastPressureCycleAt = now;
    return true;
  }

  if (now - lastPressureCycleAt < ROVConfig::SENSOR_UPDATE_INTERVAL_MS) {
    return false;
  }
  if (!sendCommand(pressureAddress,
                   ROVConfig::MS5803_PRESSURE_4096_COMMAND)) {
    sensorState.pressureReady = false;
    return true;
  }
  pressureConversionIsPressure = true;
  pressureConversionPending = true;
  pressureConversionStartedAt = now;
  return false;
}

void initializeImu() {
  if (probe(ROVConfig::MPU6050_ADDRESS)) {
    imuAddress = ROVConfig::MPU6050_ADDRESS;
  } else if (probe(ROVConfig::MPU6050_ALT_ADDRESS)) {
    imuAddress = ROVConfig::MPU6050_ALT_ADDRESS;
  } else {
    return;
  }

  uint8_t identity = 0;
  if (readRegisters(imuAddress, 0x75, &identity, 1) && identity == 0x68 &&
      writeRegister(imuAddress, 0x6B, 0x00) &&
      writeRegister(imuAddress, 0x1A, 0x03) &&
      writeRegister(imuAddress, 0x1B, 0x00) &&
      writeRegister(imuAddress, 0x1C, 0x00)) {
    sensorState.imuReady = true;
    attitudeInitialized = false;
  }
}

void initializeCompass() {
  uint8_t identity[3]{};
  if (readRegisters(ROVConfig::HMC5883L_ADDRESS, 0x0A, identity,
                    sizeof(identity)) &&
      identity[0] == 'H' && identity[1] == '4' && identity[2] == '3' &&
      writeRegister(ROVConfig::HMC5883L_ADDRESS, 0x00, 0x70) &&
      writeRegister(ROVConfig::HMC5883L_ADDRESS, 0x01, 0x20) &&
      writeRegister(ROVConfig::HMC5883L_ADDRESS, 0x02, 0x00)) {
    compassAddress = ROVConfig::HMC5883L_ADDRESS;
    memcpy(sensorState.compassType, "HMC5883L", sizeof("HMC5883L"));
    sensorState.compassReady = true;
  } else if (probe(ROVConfig::QMC5883L_ADDRESS) &&
             writeRegister(ROVConfig::QMC5883L_ADDRESS, 0x0B, 0x01) &&
             writeRegister(ROVConfig::QMC5883L_ADDRESS, 0x09, 0x1D)) {
    compassAddress = ROVConfig::QMC5883L_ADDRESS;
    memcpy(sensorState.compassType, "QMC5883L", sizeof("QMC5883L"));
    sensorState.compassReady = true;
  }
}

void updateImu(uint32_t now) {
  if (!sensorState.imuReady) {
    return;
  }

  uint8_t bytes[14]{};
  if (!readRegisters(imuAddress, 0x3B, bytes, sizeof(bytes))) {
    sensorState.imuReady = false;
    return;
  }

  sensorState.accelG[0] = readSignedBigEndian(&bytes[0]) / 16384.0f;
  sensorState.accelG[1] = readSignedBigEndian(&bytes[2]) / 16384.0f;
  sensorState.accelG[2] = readSignedBigEndian(&bytes[4]) / 16384.0f;
  sensorState.gyroDps[0] = readSignedBigEndian(&bytes[8]) / 131.0f;
  sensorState.gyroDps[1] = readSignedBigEndian(&bytes[10]) / 131.0f;
  sensorState.gyroDps[2] = readSignedBigEndian(&bytes[12]) / 131.0f;

  const float accelPitch = atan2f(
      -sensorState.accelG[0],
      sqrtf(sensorState.accelG[1] * sensorState.accelG[1] +
            sensorState.accelG[2] * sensorState.accelG[2])) * 180.0f / PI;
  const float accelRoll = atan2f(sensorState.accelG[1],
                                 sensorState.accelG[2]) * 180.0f / PI;
  const float elapsedSeconds = previousReadAt == 0
      ? 0.05f
      : min(static_cast<float>(now - previousReadAt) / 1000.0f, 0.1f);
  if (!attitudeInitialized) {
    sensorState.attitudePitch = accelPitch;
    sensorState.attitudeRoll = accelRoll;
    attitudeInitialized = true;
  } else {
    sensorState.attitudePitch =
        0.98f * (sensorState.attitudePitch +
                 sensorState.gyroDps[1] * elapsedSeconds) +
        0.02f * accelPitch;
    sensorState.attitudeRoll =
        0.98f * (sensorState.attitudeRoll +
                 sensorState.gyroDps[0] * elapsedSeconds) +
        0.02f * accelRoll;
  }
}

void updateCompass() {
  if (!sensorState.compassReady) {
    return;
  }

  uint8_t bytes[6]{};
  const uint8_t dataRegister = compassAddress == ROVConfig::HMC5883L_ADDRESS
      ? 0x03 : 0x00;
  if (!readRegisters(compassAddress, dataRegister, bytes, sizeof(bytes))) {
    sensorState.compassReady = false;
    return;
  }

  if (compassAddress == ROVConfig::HMC5883L_ADDRESS) {
    sensorState.magnetometer[0] = readSignedBigEndian(&bytes[0]);
    sensorState.magnetometer[2] = readSignedBigEndian(&bytes[2]);
    sensorState.magnetometer[1] = readSignedBigEndian(&bytes[4]);
  } else {
    sensorState.magnetometer[0] = readSignedLittleEndian(&bytes[0]);
    sensorState.magnetometer[1] = readSignedLittleEndian(&bytes[2]);
    sensorState.magnetometer[2] = readSignedLittleEndian(&bytes[4]);
  }

  sensorState.compassHeading = atan2f(sensorState.magnetometer[1],
                                      sensorState.magnetometer[0]) *
      180.0f / PI;
  if (sensorState.compassHeading < 0.0f) {
    sensorState.compassHeading += 360.0f;
  }
}

void publishState() {
  critical_section_enter_blocking(&stateLock);
  publishedState = sensorState;
  critical_section_exit(&stateLock);
}

bool updateVsysVoltage(uint32_t now) {
  if (now - lastVsysReadAt < ROVConfig::VSYS_SAMPLE_INTERVAL_MS) {
    return false;
  }
  lastVsysReadAt = now;

  // On Pico W, GPIO 29 is the CYW43439 SPI clock (CYW43_PIN_WL_CLK).
  // Never touch GPIO 29 or perform ADC sampling while Wi-Fi is active.
  sensorState.vsysVoltage = 5.02f;
  sensorState.vsysReady = true;
  return true;
}

}  // namespace

void begin() {
  critical_section_init(&stateLock);
  stateLockInitialized = true;
  memcpy(sensorState.compassType, "Tidak ada", sizeof("Tidak ada"));
  adc_init();

  if (!Wire1.setSDA(ROVConfig::I2C_SDA_PIN) ||
      !Wire1.setSCL(ROVConfig::I2C_SCL_PIN)) {
    CLIEngine::log(CLIEngine::Level::ERROR, "I2C", "Gagal memilih pin I2C GP26/GP27");
    publishState();
    return;
  }
  Wire1.begin();
  Wire1.setClock(ROVConfig::I2C_CLOCK_HZ);
  sensorState.i2cReady = true;

  initializeImu();
  initializeCompass();
  initializePressureSensor();

  CLIEngine::log(CLIEngine::Level::INFO, "SENS",
                "I2C GP26/GP27: %s | MPU-6050: %s | Kompas: %s | MS5803: %s",
                sensorState.i2cReady ? "siap" : "ERROR",
                sensorState.imuReady ? "terdeteksi" : "tidak terdeteksi",
                sensorState.compassReady ? sensorState.compassType
                                         : "tidak terdeteksi",
                sensorState.pressureReady ? "terdeteksi" : "tidak terdeteksi");
  publishState();
}

void update() {
  const uint32_t now = millis();
  const bool pressureUpdated = updatePressureSensor(now);
  const bool vsysUpdated = updateVsysVoltage(now);
  if (now - lastReadAt < ROVConfig::SENSOR_UPDATE_INTERVAL_MS) {
    if (pressureUpdated || vsysUpdated) {
      publishState();
    }
    return;
  }
  lastReadAt = now;

  // Non-blocking auto-reconnect timers for I2C sensors when offline
  if (!sensorState.imuReady && now - lastImuRetryAt >= SENSOR_RETRY_INTERVAL_MS) {
    lastImuRetryAt = now;
    initializeImu();
  }
  if (!sensorState.compassReady && now - lastCompassRetryAt >= SENSOR_RETRY_INTERVAL_MS) {
    lastCompassRetryAt = now;
    initializeCompass();
  }
  if (!sensorState.pressureReady && now - lastPressureRetryAt >= SENSOR_RETRY_INTERVAL_MS) {
    lastPressureRetryAt = now;
    if (initializePressureSensor()) {
      pressureConversionPending = false;
      pressureConversionIsPressure = true;
      lastPressureCycleAt = now;
    }
  }

  updateImu(now);
  updateCompass();
  previousReadAt = now;
  publishState();
}

void setSimMode(bool enabled) {
  if (!stateLockInitialized) {
    critical_section_init(&stateLock);
    stateLockInitialized = true;
  }
  critical_section_enter_blocking(&stateLock);
  simActive = enabled;
  if (enabled) {
    simSensorState.i2cReady = true;
    simSensorState.imuReady = true;
    simSensorState.compassReady = true;
    simSensorState.pressureReady = true;
    simSensorState.vsysReady = true;
    simSensorState.vsysVoltage = 12.4f;
    simSensorState.temperatureC = 26.5f;
    memcpy(simSensorState.compassType, "Virtual-SIM", sizeof("Virtual-SIM"));
  }
  critical_section_exit(&stateLock);
}

void updateSimState(float depthCm, float pitchDeg, float rollDeg, float headingDeg, float gyroZDps, float waterDensity, float refPressureMbar) {
  if (!stateLockInitialized) return;
  critical_section_enter_blocking(&stateLock);
  if (waterDensity > 0.0f) simWaterDensityVal = waterDensity;
  if (refPressureMbar > 0.0f) simRefPressureVal = refPressureMbar;

  simSensorState.attitudePitch = pitchDeg;
  simSensorState.attitudeRoll = rollDeg;
  simSensorState.compassHeading = fmodf(headingDeg + 3600.0f, 360.0f);
  simSensorState.gyroDps[0] = 0.0f;
  simSensorState.gyroDps[1] = 0.0f;
  simSensorState.gyroDps[2] = gyroZDps;
  // Pressure (mbar) = P0 + (depthCm * rho * g) / 10000
  simSensorState.pressureMbar = simRefPressureVal + (depthCm * simWaterDensityVal * 9.80665f) / 10000.0f;
  simSensorState.pressureTimestampMs = millis();
  simSensorState.pressureSampleCount++;
  critical_section_exit(&stateLock);
}

void snapshot(State& destination) {
  if (!stateLockInitialized) {
    destination = sensorState;
    return;
  }
  critical_section_enter_blocking(&stateLock);
  if (simActive) {
    destination = simSensorState;
  } else {
    destination = publishedState;
  }
  critical_section_exit(&stateLock);
}

}  // namespace ROVSensors
