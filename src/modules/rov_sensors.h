#pragma once

#include <stdint.h>

namespace ROVSensors {

struct State {
  bool i2cReady;
  bool imuReady;
  bool compassReady;
  bool pressureReady;
  bool vsysReady;
  char compassType[12];
  float accelG[3];
  float gyroDps[3];
  float magnetometer[3];
  float attitudePitch;
  float attitudeRoll;
  float compassHeading;
  float pressureMbar;
  float temperatureC;
  float vsysVoltage;
  uint32_t pressureTimestampMs;
  uint32_t pressureSampleCount;
};

void begin();
void update();
void snapshot(State& destination);
void setSimMode(bool enabled);
void updateSimState(float depthCm, float pitchDeg, float rollDeg, float headingDeg, float gyroZDps, float waterDensity = 1000.0f, float refPressureMbar = 1013.25f);

}  // namespace ROVSensors
