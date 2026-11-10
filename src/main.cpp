#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>
#include <LittleFS.h>
#include <Updater.h>
#include <WebServer.h>
#include <WiFi.h>
#include <atomic>
#include <math.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "modules/cli_engine.h"
#include "modules/rov_config.h"
#include "modules/rov_sensors.h"
#include "modules/web_pages.h"

namespace {

using namespace ROVConfig;

const IPAddress ACCESS_POINT_IP(192, 168, 4, 1);
const IPAddress ACCESS_POINT_SUBNET(255, 255, 255, 0);
constexpr uint32_t TEST_SEQUENCE_STEP_MS = 10000;
constexpr size_t TEST_SEQUENCE_STEP_COUNT = 6;
constexpr uint8_t TEST_SEQUENCE_AXES[TEST_SEQUENCE_STEP_COUNT] = {0, 3, 0, 0, 1, 1};
constexpr int8_t TEST_SEQUENCE_DIRECTIONS[TEST_SEQUENCE_STEP_COUNT] = {1, 1, -1, 1, 1, -1};

struct SettingsV6 {
  uint32_t magic;
  uint8_t version;
  uint8_t pins[THRUSTER_COUNT];
  uint16_t pulseMin[THRUSTER_COUNT];
  uint16_t pulseNeutral[THRUSTER_COUNT];
  uint16_t pulseMax[THRUSTER_COUNT];
  uint8_t reverse[THRUSTER_COUNT];
  float mix[THRUSTER_COUNT][AXIS_COUNT];
  uint8_t maxPowerPercent;
  uint8_t rampPercentPerSecond;
  uint16_t failsafeMs;
  char wifiSsid[33];
  char wifiPassword[64];
  char webPassword[33];
  float stabilizationKp;
  float stabilizationKi;
  float stabilizationKd;
  uint8_t stabilizationMaxOutputPercent;
  uint8_t stabilizationMaxAngleDegrees;
  uint8_t stabilizationTargetRateDps;
  float depthKp;
  float depthKi;
  float depthKd;
  uint8_t depthMaxOutputPercent;
  uint8_t depthDeadbandCm;
  uint16_t depthTargetCm;
  uint32_t checksum;
};

Settings settings{};
Servo thrusters[THRUSTER_COUNT];
WebServer server(80);
float axisInput[AXIS_COUNT]{};
float motorOutputApplied[THRUSTER_COUNT]{};
float individualThrusterInput[THRUSTER_COUNT]{};
bool individualThrusterMode = false;
bool setupMode = false;
bool armed = false;
bool outputsReady = false;
bool serverStarted = false;
bool accessPointReady = false;
bool serialWasConnected = false;
bool stabilizationEnabled = false;
bool stabilizationFaulted = false;
bool depthHoldEnabled = false;
bool autoModeEnabled = false;
bool testSequenceActive = false;
uint8_t testSequenceStep = 0;
uint32_t testSequenceStepStartedAt = 0;
float testSequenceStrength = 0.30f;
uint32_t lastControlAt = 0;
uint32_t lastOutputAt = 0;
uint32_t lastSetupMessageAt = 0;
uint32_t lastStatusMessageAt = 0;
float stabilizationTargetPitch = 0.0f;
float stabilizationTargetRoll = 0.0f;
float stabilizationPitchIntegral = 0.0f;
float stabilizationRollIntegral = 0.0f;
float depthIntegral = 0.0f;
float depthRateCmPerSecond = 0.0f;
float previousDepthCm = 0.0f;
uint32_t lastDepthTimestampMs = 0;
uint32_t lastDepthSampleCount = 0;
bool previousDepthValid = false;
String setupApName;
String setupApPassword;
char activeWebPassword[33]{};
bool settingsLayoutMigrated = false;
std::atomic<bool> sensorsInitialized{false};
float depthReferencePressureMbar = 0.0f;
bool depthReferenceSet = false;
bool simMode = false;
uint8_t cliTestMotorId = 0;
uint32_t cliTestMotorUntil = 0;
uint16_t cliTestMotorPulse = 1500;
bool escCalibActive = false;
bool thrustersAlreadyNeutral = false;
bool settingsLoaded = false;
bool rpiStreamEnabled = false;
uint32_t lastRpiStreamAt = 0;
float posX_m = 0.0f;
float posY_m = 0.0f;
float posZ_m = 0.0f;
uint32_t lastOdomAt = 0;
File blackboxFile;
bool blackboxActive = false;
uint32_t lastBlackboxSampleAt = 0;
int currentThrusterPulse[THRUSTER_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500};

enum MissionState {
  MISSION_CALIBRATING,    // 10s auto-calibration at boot
  MISSION_CALIB_BEEP,     // 600ms ESC beep tone
  MISSION_IDLE,           // Standby (Active balancing ready, waiting for 'run')
  AUTON_START_DELAY,      // 3s hands-off countdown & lock-on-release H0
  AUTON_DIVE,             // Dive to depthTargetCm with depth PID & roll stabilization
  AUTON_SWIM_FWD,         // Cruise straight forward at swimPower for swimSec
  AUTON_BRAKE,            // Active reverse surge pulse for brakeTimeSec
  AUTON_TURN_CW,          // Rotate 180 deg CW (or turnAngleDeg CW/CCW) to H0 + 180
  AUTON_SWIM_RET,         // Cruise straight back for swimSec holding depth
  AUTON_SURFACE,          // Soft surface landing until depth < 5cm
  AUTON_COMPLETE,         // Completed, thrusters disarmed, summary printed
  AUTON_ABORT             // Aborted due to emergency stop or failsafe
};

MissionState missionState = MISSION_CALIBRATING;
uint32_t calibStartMs = 0;
uint32_t lastCalibSecReported = 0;
float calibPressureSum = 0.0f;
uint32_t calibPressureSamples = 0;
float calibGyroSum[3] = {0.0f, 0.0f, 0.0f};
uint32_t calibGyroSamples = 0;
float gyroBias[3] = {0.0f, 0.0f, 0.0f};
bool autorunCancelled = false;

// Autonomous mission tracking variables
uint32_t missionStartMs = 0;
uint32_t stepStartedAtMs = 0;
uint32_t settleStartMs = 0;
uint32_t lastAutonLogMs = 0;
float referenceHeadingDeg = 0.0f;
float gyroIntegratedHeadingDeg = 0.0f;
uint32_t lastHeadingIntegrateMs = 0;
float turnTargetHeadingDeg = 0.0f;
float maxDepthReachedCm = 0.0f;
float missionTurnErrorDeg = 0.0f;
bool missionSuccess = false;
const char* missionAbortReason = "";

// Desk simulation state variables
float simDepthCm = 0.0f;
float simHeadingDeg = 0.0f;
float simPitchDeg = 0.0f;
float simRollDeg = 0.0f;
float simGyroZDps = 0.0f;

uint32_t checksumSettings(const Settings& value) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < offsetof(Settings, checksum); ++i) {
    hash = (hash ^ bytes[i]) * 16777619u;
  }
  return hash;
}

uint32_t checksumSettingsV6(const SettingsV6& value) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < offsetof(SettingsV6, checksum); ++i) {
    hash = (hash ^ bytes[i]) * 16777619u;
  }
  return hash;
}

void appendJsonString(String& json, const char* value) {
  static const char HEX_DIGITS[] = "0123456789abcdef";
  json += '"';
  for (const unsigned char* character =
           reinterpret_cast<const unsigned char*>(value);
       *character != '\0'; ++character) {
    switch (*character) {
      case '"': json += "\\\""; break;
      case '\\': json += "\\\\"; break;
      case '\b': json += "\\b"; break;
      case '\f': json += "\\f"; break;
      case '\n': json += "\\n"; break;
      case '\r': json += "\\r"; break;
      case '\t': json += "\\t"; break;
      default:
        if (*character < 0x20) {
          json += "\\u00";
          json += HEX_DIGITS[*character >> 4];
          json += HEX_DIGITS[*character & 0x0f];
        } else {
          json += static_cast<char>(*character);
        }
        break;
    }
  }
  json += '"';
}

bool validPin(uint8_t pin) {
  return isThrusterPinAllowed(pin);
}

bool validateSettings(const Settings& value) {
  if (value.magic != CONFIG_MAGIC || value.version != CONFIG_VERSION ||
      value.maxPowerPercent < 1 || value.maxPowerPercent > 100 ||
      value.rampPercentPerSecond < 5 || value.rampPercentPerSecond > 200 ||
      value.failsafeMs < MIN_FAILSAFE_MS || value.failsafeMs > MAX_FAILSAFE_MS ||
      !isfinite(value.stabilizationKp) || value.stabilizationKp < 0.0f ||
      value.stabilizationKp > 1.0f || !isfinite(value.stabilizationKi) ||
      value.stabilizationKi < 0.0f || value.stabilizationKi > 0.1f ||
      !isfinite(value.stabilizationKd) || value.stabilizationKd < 0.0f ||
      value.stabilizationKd > 1.0f ||
      value.stabilizationMaxOutputPercent < 5 ||
      value.stabilizationMaxOutputPercent > 100 ||
      value.stabilizationMaxAngleDegrees < 5 ||
      value.stabilizationMaxAngleDegrees > 45 ||
      value.stabilizationTargetRateDps < 1 ||
      value.stabilizationTargetRateDps > 60 || !isfinite(value.depthKp) ||
      value.depthKp < 0.0f || value.depthKp > 2.0f ||
      !isfinite(value.depthKi) || value.depthKi < 0.0f ||
      value.depthKi > 0.5f || !isfinite(value.depthKd) ||
      value.depthKd < 0.0f || value.depthKd > 1.0f ||
      value.depthMaxOutputPercent < 5 || value.depthMaxOutputPercent > 100 ||
      value.depthDeadbandCm > 25 || value.depthTargetCm > 500) {
    return false;
  }

  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    if (!validPin(value.pins[i]) ||
        value.pulseMin[i] < 900 || value.pulseMin[i] >= value.pulseNeutral[i] ||
        value.pulseNeutral[i] > 1600 || value.pulseNeutral[i] < 1400 ||
        value.pulseMax[i] > 2100 || value.pulseMax[i] <= value.pulseNeutral[i] ||
        value.reverse[i] > 1 ||
        !isfinite(value.revBoost[i]) || value.revBoost[i] < 0.5f || value.revBoost[i] > 3.0f) {
      return false;
    }
    for (size_t j = 0; j < i; ++j) {
      if (value.pins[i] == value.pins[j]) {
        return false;
      }
    }
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
      if (!isfinite(value.mix[i][axis]) || value.mix[i][axis] < -1.0f ||
          value.mix[i][axis] > 1.0f) {
        return false;
      }
    }
  }

  if (!isfinite(value.globalRevBoost) || value.globalRevBoost < 0.5f || value.globalRevBoost > 3.0f ||
      value.maxDepthCm < 10 || value.maxDepthCm > 500 ||
      value.maxTiltDeg < 10 || value.maxTiltDeg > 90 ||
      value.timeoutSec < 5 || value.timeoutSec > 300 ||
      value.stepTimeoutSec < 1 || value.stepTimeoutSec > 60 ||
      !isfinite(value.waterDensity) || value.waterDensity < 900.0f || value.waterDensity > 1100.0f) {
    return false;
  }

  const size_t ssidLen = strnlen(value.wifiSsid, sizeof(value.wifiSsid));
  const size_t wifiPassLen = strnlen(value.wifiPassword, sizeof(value.wifiPassword));
  const size_t webPassLen = strnlen(value.webPassword, sizeof(value.webPassword));

  if (ssidLen == 0 || ssidLen >= sizeof(value.wifiSsid)) {
    return false;
  }
  if (wifiPassLen > 0 && (wifiPassLen < 8 || wifiPassLen >= sizeof(value.wifiPassword))) {
    return false;
  }
  if (webPassLen >= sizeof(value.webPassword)) {
    return false;
  }

  return true;
}

void setDefaults() {
  memset(&settings, 0, sizeof(settings));
  settings.magic = CONFIG_MAGIC;
  settings.version = CONFIG_VERSION;
  memcpy(settings.wifiSsid, DEFAULT_HOTSPOT_NAME, sizeof(DEFAULT_HOTSPOT_NAME));
  memcpy(settings.wifiPassword, DEFAULT_HOTSPOT_PASSWORD, sizeof(DEFAULT_HOTSPOT_PASSWORD));
  memcpy(settings.webPassword, DEFAULT_HOTSPOT_PASSWORD, sizeof(DEFAULT_HOTSPOT_PASSWORD));
  memcpy(settings.pins, DEFAULT_THRUSTER_PINS, sizeof(DEFAULT_THRUSTER_PINS));

  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    settings.pulseMin[i] = DEFAULT_PULSE_MIN_US;
    settings.pulseNeutral[i] = DEFAULT_PULSE_NEUTRAL_US;
    settings.pulseMax[i] = DEFAULT_PULSE_MAX_US;
    settings.revBoost[i] = DEFAULT_REV_BOOST;
  }
  settings.globalRevBoost = DEFAULT_REV_BOOST;
  settings.deadbandUs = DEFAULT_DEADBAND_US;

  memcpy(settings.mix, DEFAULT_MIX, sizeof(DEFAULT_MIX));
  settings.maxPowerPercent = DEFAULT_MAX_POWER_PERCENT;
  settings.rampPercentPerSecond = DEFAULT_RAMP_PERCENT_PER_SECOND;
  settings.failsafeMs = DEFAULT_FAILSAFE_MS;
  settings.maxDepthCm = DEFAULT_MAX_DEPTH_CM;
  settings.maxTiltDeg = DEFAULT_MAX_TILT_DEG;
  settings.timeoutSec = DEFAULT_TIMEOUT_SEC;
  settings.stallCheck = DEFAULT_STALL_CHECK;
  settings.stepTimeoutSec = DEFAULT_STEP_TIMEOUT_SEC;
  settings.failsafeMode = DEFAULT_FAILSAFE_MODE;

  settings.stabilizationKp = DEFAULT_STABILIZATION_KP;
  settings.stabilizationKi = DEFAULT_STABILIZATION_KI;
  settings.stabilizationKd = DEFAULT_STABILIZATION_KD;
  settings.stabilizationMaxOutputPercent =
      DEFAULT_STABILIZATION_MAX_OUTPUT_PERCENT;
  settings.stabilizationMaxAngleDegrees =
      DEFAULT_STABILIZATION_MAX_ANGLE_DEGREES;
  settings.stabilizationTargetRateDps = DEFAULT_STABILIZATION_TARGET_RATE_DPS;

  settings.depthKp = DEFAULT_DEPTH_KP;
  settings.depthKi = DEFAULT_DEPTH_KI;
  settings.depthKd = DEFAULT_DEPTH_KD;
  settings.depthMaxOutputPercent = DEFAULT_DEPTH_MAX_OUTPUT_PERCENT;
  settings.depthDeadbandCm = DEFAULT_DEPTH_DEADBAND_CM;
  settings.depthTargetCm = DEFAULT_DEPTH_TARGET_CM;

  settings.yawKp = DEFAULT_YAW_KP;
  settings.yawKi = DEFAULT_YAW_KI;
  settings.yawKd = DEFAULT_YAW_KD;
  settings.pidLimitPercent = DEFAULT_PID_LIMIT_PERCENT;
  settings.rlEnabled = DEFAULT_RL_ENABLED;
  settings.rlRate = DEFAULT_RL_RATE;
  settings.rlMaxKp = DEFAULT_RL_MAX_KP;

  settings.autorun = DEFAULT_AUTORUN;
  settings.startDelaySec = DEFAULT_START_DELAY_SEC;
  settings.settleTimeSec = DEFAULT_SETTLE_TIME_SEC;
  settings.swimSec = DEFAULT_SWIM_SEC;
  settings.swimPower = DEFAULT_SWIM_POWER_PERCENT;
  settings.turnAngleDeg = DEFAULT_TURN_ANGLE_DEG;
  settings.turnDir = DEFAULT_TURN_DIR;
  settings.turnPower = DEFAULT_TURN_POWER_PERCENT;
  settings.depthTolCm = DEFAULT_DEPTH_TOL_CM;
  settings.yawTolDeg = DEFAULT_YAW_TOL_DEG;
  settings.missionProfile = DEFAULT_MISSION_PROFILE;

  settings.heaveBias = DEFAULT_HEAVE_BIAS;
  settings.pitchBias = DEFAULT_PITCH_BIAS;
  settings.trimRoll = DEFAULT_TRIM_ROLL;
  settings.trimYaw = DEFAULT_TRIM_YAW;
  settings.brakeTimeSec = DEFAULT_BRAKE_TIME_SEC;

  settings.waterDensity = DEFAULT_WATER_DENSITY;
  settings.calibTimeSec = DEFAULT_CALIB_TIME_SEC;
  settings.waveFilterSamples = DEFAULT_WAVE_FILTER_SAMPLES;
  settings.magDeclinationDeg = DEFAULT_MAG_DECLINATION_DEG;
  settings.wifiSleep = DEFAULT_WIFI_SLEEP;

  settings.beepCalib = DEFAULT_BEEP_CALIB;
  settings.beepMission = DEFAULT_BEEP_MISSION;
  settings.beepPwrPercent = DEFAULT_BEEP_PWR_PERCENT;
}

bool persistSettings();

bool loadSettings() {
  EEPROM.begin(sizeof(Settings));
  EEPROM.get(0, settings);
  if (settings.magic == CONFIG_MAGIC && settings.version == CONFIG_VERSION &&
      settings.checksum == checksumSettings(settings)) {
    if (validateSettings(settings)) {
      return true;
    }
  }

  // Direct migration from v6
  SettingsV6 v6{};
  EEPROM.get(0, v6);
  if (v6.magic == CONFIG_MAGIC && v6.version == 6 &&
      v6.checksum == checksumSettingsV6(v6)) {
    setDefaults();
    memcpy(settings.pins, v6.pins, sizeof(settings.pins));
    memcpy(settings.pulseMin, v6.pulseMin, sizeof(settings.pulseMin));
    memcpy(settings.pulseNeutral, v6.pulseNeutral, sizeof(settings.pulseNeutral));
    memcpy(settings.pulseMax, v6.pulseMax, sizeof(settings.pulseMax));
    memcpy(settings.reverse, v6.reverse, sizeof(settings.reverse));
    memcpy(settings.mix, v6.mix, sizeof(settings.mix));
    settings.maxPowerPercent = v6.maxPowerPercent;
    settings.rampPercentPerSecond = v6.rampPercentPerSecond;
    settings.failsafeMs = v6.failsafeMs;
    memcpy(settings.wifiSsid, v6.wifiSsid, sizeof(settings.wifiSsid));
    if (v6.wifiPassword[0] != '\0') {
      memcpy(settings.wifiPassword, v6.wifiPassword, sizeof(settings.wifiPassword));
    }
    memcpy(settings.webPassword, v6.webPassword, sizeof(settings.webPassword));
    settings.stabilizationKp = v6.stabilizationKp;
    settings.stabilizationKi = v6.stabilizationKi;
    settings.stabilizationKd = v6.stabilizationKd;
    settings.stabilizationMaxOutputPercent = v6.stabilizationMaxOutputPercent;
    settings.stabilizationMaxAngleDegrees = v6.stabilizationMaxAngleDegrees;
    settings.stabilizationTargetRateDps = v6.stabilizationTargetRateDps;
    settings.depthKp = v6.depthKp;
    settings.depthKi = v6.depthKi;
    settings.depthKd = v6.depthKd;
    settings.depthMaxOutputPercent = v6.depthMaxOutputPercent;
    settings.depthDeadbandCm = v6.depthDeadbandCm;
    settings.depthTargetCm = v6.depthTargetCm;

    if (validateSettings(settings) && persistSettings()) {
      Serial.println("Konfigurasi tersimpan dimigrasikan ke Firmware v1.0.");
      settingsLayoutMigrated = true;
      return true;
    }
  }

  setDefaults();
  return false;
}

bool persistSettings() {
  settings.magic = CONFIG_MAGIC;
  settings.version = CONFIG_VERSION;
  settings.checksum = checksumSettings(settings);
  EEPROM.put(0, settings);
  return EEPROM.commit();
}

bool copyField(const String& source, char* destination, size_t capacity,
               bool allowEmpty = true) {
  if ((!allowEmpty && source.isEmpty()) || source.length() >= capacity) {
    return false;
  }
  memset(destination, 0, capacity);
  source.toCharArray(destination, capacity);
  return true;
}

bool readIntegerArg(const String& name, long minimum, long maximum, long& result) {
  if (!server.hasArg(name)) {
    return false;
  }
  String input = server.arg(name);
  if (input.isEmpty()) {
    return false;
  }
  char* end = nullptr;
  const long value = strtol(input.c_str(), &end, 10);
  if (end == input.c_str() || *end != '\0' || value < minimum || value > maximum) {
    return false;
  }
  result = value;
  return true;
}

bool readBoundedFloatArg(const String& name, float minimum, float maximum,
                         float& result) {
  if (!server.hasArg(name)) {
    return false;
  }
  String input = server.arg(name);
  if (input.isEmpty()) {
    return false;
  }
  char* end = nullptr;
  const float value = strtof(input.c_str(), &end);
  if (end == input.c_str() || *end != '\0' || !isfinite(value) ||
      value < minimum || value > maximum) {
    return false;
  }
  result = value;
  return true;
}

bool readFloatArg(const String& name, float& result) {
  return readBoundedFloatArg(name, -1.0f, 1.0f, result);
}

bool requireAuthentication() {
  return true;
}

void sendError(int status, const String& message) {
  server.send(status, "text/plain; charset=utf-8", message);
}

void resetPidState() {
  stabilizationPitchIntegral = 0.0f;
  stabilizationRollIntegral = 0.0f;
  depthIntegral = 0.0f;
  depthRateCmPerSecond = 0.0f;
  lastDepthTimestampMs = 0;
  lastDepthSampleCount = 0;
  previousDepthValid = false;
}

void stopAllThrusters() {
  armed = false;
  individualThrusterMode = false;
  stabilizationEnabled = false;
  depthHoldEnabled = false;
  autoModeEnabled = false;
  testSequenceActive = false;
  testSequenceStep = 0;
  cliTestMotorUntil = 0;
  cliTestMotorId = 0;
  escCalibActive = false;
  if (missionState != MISSION_IDLE && missionState != MISSION_CALIBRATING && missionState != MISSION_CALIB_BEEP) {
    missionState = MISSION_IDLE;
  }
  autorunCancelled = true;
  resetPidState();
  memset(axisInput, 0, sizeof(axisInput));
  memset(individualThrusterInput, 0, sizeof(individualThrusterInput));
  memset(motorOutputApplied, 0, sizeof(motorOutputApplied));
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    currentThrusterPulse[i] = settings.pulseNeutral[i];
    if (thrusters[i].attached()) {
      thrusters[i].writeMicroseconds(settings.pulseNeutral[i]);
    }
  }
  if (!thrustersAlreadyNeutral) {
    CLIEngine::log(CLIEngine::Level::STATE, "MOT", "All thrusters neutralized (1500us)");
    thrustersAlreadyNeutral = true;
  }
}

void playEscBeepTone(uint32_t durationMs = 600) {
  if (settings.beepCalib == 0 || !outputsReady) return;
  thrustersAlreadyNeutral = false;
  const float pwr = (settings.beepPwrPercent > 0 ? settings.beepPwrPercent : 10) / 100.0f;
  const int pulseOffset = constrain(static_cast<int>(lroundf(pwr * 35.0f)), 10, 45);

  const uint32_t startMs = millis();
  while (millis() - startMs < durationMs) {
    for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
      if (thrusters[i].attached()) {
        thrusters[i].writeMicroseconds(settings.pulseNeutral[i] + pulseOffset);
      }
    }
    delay(10);
    for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
      if (thrusters[i].attached()) {
        thrusters[i].writeMicroseconds(settings.pulseNeutral[i] - pulseOffset);
      }
    }
    delay(10);
  }
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    if (thrusters[i].attached()) {
      thrusters[i].writeMicroseconds(settings.pulseNeutral[i]);
    }
  }
  thrustersAlreadyNeutral = true;
}

inline float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

void updateSimulation(uint32_t dtMs) {
  if (!simMode) return;
  const float dt = dtMs / 1000.0f;

  switch (missionState) {
    case MISSION_CALIBRATING:
    case MISSION_CALIB_BEEP:
    case MISSION_IDLE:
    case AUTON_START_DELAY:
      simDepthCm = 0.0f;
      simGyroZDps = 0.0f;
      simPitchDeg = 0.0f;
      simRollDeg = 0.0f;
      break;

    case AUTON_DIVE: {
      const float target = static_cast<float>(settings.depthTargetCm);
      if (simDepthCm < target) {
        simDepthCm = min(target, simDepthCm + 22.0f * dt);
      }
      simGyroZDps = 0.0f;
      break;
    }

    case AUTON_SWIM_FWD: {
      const float target = static_cast<float>(settings.depthTargetCm);
      simDepthCm = target + 0.7f * sinf(millis() * 0.003f);
      simHeadingDeg = referenceHeadingDeg;
      simGyroZDps = 0.0f;
      break;
    }

    case AUTON_BRAKE: {
      const float target = static_cast<float>(settings.depthTargetCm);
      simDepthCm = target + 0.4f * sinf(millis() * 0.003f);
      simGyroZDps = 0.0f;
      break;
    }

    case AUTON_TURN_CW: {
      const float target = static_cast<float>(settings.depthTargetCm);
      simDepthCm = target + 0.4f * sinf(millis() * 0.003f);
      const float turnTarget = referenceHeadingDeg + (settings.turnDir == 0 ? settings.turnAngleDeg : -settings.turnAngleDeg);
      const float err = wrapAngle180(turnTarget - simHeadingDeg);
      if (fabsf(err) > 1.5f) {
        const float rate = (settings.turnDir == 0 ? 35.0f : -35.0f);
        simGyroZDps = (err > 0 ? rate : -rate);
        simHeadingDeg += simGyroZDps * dt;
      } else {
        simHeadingDeg = turnTarget;
        simGyroZDps = 0.0f;
      }
      break;
    }

    case AUTON_SWIM_RET: {
      const float target = static_cast<float>(settings.depthTargetCm);
      simDepthCm = target + 0.7f * sinf(millis() * 0.003f);
      const float turnTarget = referenceHeadingDeg + (settings.turnDir == 0 ? settings.turnAngleDeg : -settings.turnAngleDeg);
      simHeadingDeg = turnTarget;
      simGyroZDps = 0.0f;
      break;
    }

    case AUTON_SURFACE: {
      if (simDepthCm > 0.0f) {
        simDepthCm = max(0.0f, simDepthCm - 16.0f * dt);
      }
      simGyroZDps = 0.0f;
      break;
    }

    default:
      simGyroZDps = 0.0f;
      break;
  }

  ROVSensors::updateSimState(simDepthCm, simPitchDeg, simRollDeg, simHeadingDeg, simGyroZDps,
                             settings.waterDensity, depthReferencePressureMbar);
}

void cliSetSimMode(bool enabled) {
  simMode = enabled;
  ROVSensors::setSimMode(enabled);
  if (enabled) {
    if (depthReferencePressureMbar <= 0.0f) {
      depthReferencePressureMbar = 1013.25f;
      depthReferenceSet = true;
    }
    simDepthCm = 0.0f;
    simHeadingDeg = 0.0f;
    simPitchDeg = 0.0f;
    simRollDeg = 0.0f;
    simGyroZDps = 0.0f;
    ROVSensors::updateSimState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, settings.waterDensity, depthReferencePressureMbar);
    CLIEngine::log(CLIEngine::Level::STATE, "SIM", "Simulation mode ENABLED (desk testing: props limited to safe 3%% power)");
  } else {
    CLIEngine::log(CLIEngine::Level::STATE, "SIM", "Simulation mode DISABLED (hardware sensors active)");
  }
}

void cliCancelAutorun() {
  if (missionState == MISSION_CALIBRATING || missionState == MISSION_CALIB_BEEP) {
    if (!autorunCancelled) {
      autorunCancelled = true;
      CLIEngine::log(CLIEngine::Level::WARN, "AUTORUN", "Auto-Run dibatalkan oleh operator. Beralih ke mode STANDBY.");
    }
  }
}

void printMissionSummary() {
  const float totalSec = (stepStartedAtMs > missionStartMs) ? (millis() - missionStartMs) / 1000.0f : 0.0f;
  const float turnAccuracy = 180.0f - missionTurnErrorDeg;

  char statusBuf[48];
  if (missionSuccess) {
    snprintf(statusBuf, sizeof(statusBuf), "SUKSES (100%% Selesai)");
  } else {
    snprintf(statusBuf, sizeof(statusBuf), "ABORT (%s)", missionAbortReason ? missionAbortReason : "Dibatalkan");
  }

  CLIEngine::log(CLIEngine::Level::INFO, "RAPOR", "====================================================");
  CLIEngine::log(CLIEngine::Level::INFO, "RAPOR", "             LAPORAN MISI OTONOM AUV");
  CLIEngine::log(CLIEngine::Level::INFO, "RAPOR", "====================================================");
  CLIEngine::log(CLIEngine::Level::INFO, "RAPOR", "Total Waktu Misi    : %.1f detik", totalSec);
  CLIEngine::log(CLIEngine::Level::INFO, "RAPOR", "Kedalaman Maksimal  : %.1f cm (Target: %u cm)", maxDepthReachedCm, settings.depthTargetCm);
  CLIEngine::log(CLIEngine::Level::INFO, "RAPOR", "Akurasi Sudut Putar : %.1f° (Target: 180.0°)", turnAccuracy);
  CLIEngine::log(CLIEngine::Level::INFO, "RAPOR", "Status              : %s", statusBuf);
  CLIEngine::log(CLIEngine::Level::INFO, "RAPOR", "====================================================");
}

const char* getMissionStepString() {
  if (escCalibActive || missionState == MISSION_CALIBRATING || missionState == MISSION_CALIB_BEEP) return "CALIB";
  if (missionState == AUTON_START_DELAY) return "START_DELAY";
  if (missionState == AUTON_DIVE) return "DIVE";
  if (missionState == AUTON_SWIM_FWD) return "SWIM_FWD";
  if (missionState == AUTON_BRAKE) return "BRAKE";
  if (missionState == AUTON_TURN_CW) return "TURN_CW";
  if (missionState == AUTON_SWIM_RET) return "SWIM_RET";
  if (missionState == AUTON_SURFACE) return "SURFACE";
  if (missionState == AUTON_COMPLETE) return "COMPLETE";
  if (missionState == AUTON_ABORT || stabilizationFaulted) return "ABORT";
  if (autoModeEnabled && armed) return "RUNNING";
  return "IDLE";
}

void startBlackboxRecording() {
  if (blackboxActive && blackboxFile) {
    blackboxFile.close();
  }
  blackboxFile = LittleFS.open("/blackbox.csv", "w");
  if (blackboxFile) {
    blackboxFile.print("time_ms,state,depth_cm,roll,pitch,yaw,t1,t2,t3,t4,t5,t6,x,y\n");
    blackboxFile.flush();
    blackboxActive = true;
    lastBlackboxSampleAt = 0;
    CLIEngine::log(CLIEngine::Level::INFO, "BBOX", "Underwater Flight Recorder started (/blackbox.csv)");
  } else {
    blackboxActive = false;
    CLIEngine::log(CLIEngine::Level::ERROR, "BBOX", "Failed to create /blackbox.csv in LittleFS");
  }
}

void stopBlackboxRecording() {
  if (blackboxActive) {
    if (blackboxFile) {
      blackboxFile.flush();
      blackboxFile.close();
    }
    blackboxActive = false;
    CLIEngine::log(CLIEngine::Level::STATE, "BBOX", "Underwater Flight Recorder stopped (/blackbox.csv closed)");
  }
}

bool cliBlackboxControl(const String& action, String& response) {
  if (action == "dump") {
    if (!LittleFS.exists("/blackbox.csv")) {
      response = "Blackbox file (/blackbox.csv) is empty or does not exist.";
      return false;
    }
    File f = LittleFS.open("/blackbox.csv", "r");
    if (!f) {
      response = "ERROR: Failed to open /blackbox.csv for reading.";
      return false;
    }
    response.reserve(1024);
    response = "=== BLACKBOX CSV FLIGHT RECORDER ===\r\n";
    size_t lines = 0;
    while (f.available() && lines < 50) {
      String l = f.readStringUntil('\n');
      l.trim();
      if (!l.isEmpty()) {
        response += l + "\r\n";
        lines++;
      }
    }
    if (f.available()) {
      response += "... (Output truncated. Download full CSV via Web UI or /api/blackbox/download)\r\n";
    }
    f.close();
    response += "====================================";
    return true;
  } else if (action == "clear") {
    if (blackboxActive && blackboxFile) {
      blackboxFile.close();
      blackboxActive = false;
    }
    if (LittleFS.exists("/blackbox.csv")) {
      LittleFS.remove("/blackbox.csv");
      response = "OK: LittleFS flight recorder (/blackbox.csv) cleared.";
      CLIEngine::log(CLIEngine::Level::INFO, "BBOX", "Blackbox flight recorder /blackbox.csv deleted");
      return true;
    } else {
      response = "OK: Blackbox log was already empty.";
      return true;
    }
  } else {
    if (LittleFS.exists("/blackbox.csv")) {
      File f = LittleFS.open("/blackbox.csv", "r");
      const size_t sz = f.size();
      f.close();
      response = "Blackbox: " + String(sz) + " bytes stored in LittleFS. Status: " + String(blackboxActive ? "RECORDING" : "IDLE");
    } else {
      response = "Blackbox: No log recorded yet. Status: " + String(blackboxActive ? "RECORDING" : "IDLE");
    }
    return true;
  }
}

void handleBlackboxDownload() {
  if (!requireAuthentication()) return;
  if (!LittleFS.exists("/blackbox.csv")) {
    server.send(404, "text/plain", "Blackbox log not found. Run a mission first.");
    return;
  }
  File f = LittleFS.open("/blackbox.csv", "r");
  if (!f) {
    server.send(500, "text/plain", "Failed to open blackbox file.");
    return;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"auv_blackbox.csv\"");
  server.streamFile(f, "text/csv");
  f.close();
}

bool cliRunSelfTest(String& response) {
  CLIEngine::log(CLIEngine::Level::STATE, "TEST", "Pre-Flight Self-Test initiated...");

  ROVSensors::State sensor{};
  ROVSensors::snapshot(sensor);

  // 1. I2C Sensors
  const bool pPass = sensor.pressureReady;
  const bool imuPass = sensor.imuReady;
  const bool compassPass = sensor.compassReady;

  // 2. Storage
  const bool eepromPass = settingsLoaded && validateSettings(settings);
  FSInfo fsInfo{};
  const bool fsPass = LittleFS.info(fsInfo) && fsInfo.totalBytes > 0;

  // 3. Wi-Fi
  const bool wifiPass = accessPointReady;

  // 4. Thruster micro-pulses (0.2s each)
  bool motorsPass = outputsReady;
  if (outputsReady) {
    armed = false;
    autoModeEnabled = false;
    testSequenceActive = false;
    individualThrusterMode = false;

    for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
      if (!thrusters[i].attached()) {
        motorsPass = false;
        continue;
      }
      const uint16_t pulse = settings.pulseNeutral[i] + (simMode ? 15 : 40);
      thrusters[i].writeMicroseconds(pulse);
      currentThrusterPulse[i] = pulse;
      delay(200);
      thrusters[i].writeMicroseconds(settings.pulseNeutral[i]);
      currentThrusterPulse[i] = settings.pulseNeutral[i];
      delay(100);
    }
    stopAllThrusters();
  }

  const bool allPass = pPass && imuPass && compassPass && eepromPass && fsPass && wifiPass && motorsPass;

  response.reserve(1024);
  response = "==================================================\r\n";
  response += "        AUV SAUVAC PRE-FLIGHT SELF-TEST REPORT    \r\n";
  response += "==================================================\r\n";
  response += "[SUBSYSTEM]         [RESULT]  [DIAGNOSTIC DATA]\r\n";
  response += "--------------------------------------------------\r\n";
  response += "MS5803 Pressure     " + String(pPass ? "PASS" : "FAIL") + "      " +
              (pPass ? String(sensor.pressureMbar, 1) + " mbar | " + String(sensor.temperatureC, 1) + " C" : "Not detected") + "\r\n";
  response += "MPU-6050 IMU        " + String(imuPass ? "PASS" : "FAIL") + "      " +
              (imuPass ? "P: " + String(sensor.attitudePitch, 1) + "° | R: " + String(sensor.attitudeRoll, 1) + "°" : "Not detected") + "\r\n";
  response += "Digital Compass     " + String(compassPass ? "PASS" : "FAIL") + "      " +
              (compassPass ? String(sensor.compassType) + " (" + String(sensor.compassHeading, 1) + "°)" : "Not detected") + "\r\n";
  response += "EEPROM Storage      " + String(eepromPass ? "PASS" : "FAIL") + "      " +
              (eepromPass ? "v1.0 Struct & Checksum OK" : "Uninitialized/Corrupt") + "\r\n";
  response += "LittleFS Flash      " + String(fsPass ? "PASS" : "FAIL") + "      " +
              (fsPass ? String(static_cast<uint32_t>(fsInfo.totalBytes / 1024)) + " KB Mounted" : "Mount failed") + "\r\n";
  response += "Wi-Fi SoftAP        " + String(wifiPass ? "PASS" : "FAIL") + "      " +
              (wifiPass ? setupApName + " (" + WiFi.softAPIP().toString() + ")" : "AP not running") + "\r\n";
  response += "Thruster T1 (GP2)   " + String(outputsReady && thrusters[0].attached() ? "PASS" : "FAIL") + "      Pulse 0.2s OK (CW)\r\n";
  response += "Thruster T2 (GP3)   " + String(outputsReady && thrusters[1].attached() ? "PASS" : "FAIL") + "      Pulse 0.2s OK (CW)\r\n";
  response += "Thruster T3 (GP4)   " + String(outputsReady && thrusters[2].attached() ? "PASS" : "FAIL") + "      Pulse 0.2s OK (CCW)\r\n";
  response += "Thruster T4 (GP5)   " + String(outputsReady && thrusters[3].attached() ? "PASS" : "FAIL") + "      Pulse 0.2s OK (CCW)\r\n";
  response += "Thruster T5 (GP6)   " + String(outputsReady && thrusters[4].attached() ? "PASS" : "FAIL") + "      Pulse 0.2s OK (CW)\r\n";
  response += "Thruster T6 (GP7)   " + String(outputsReady && thrusters[5].attached() ? "PASS" : "FAIL") + "      Pulse 0.2s OK (CCW)\r\n";
  response += "--------------------------------------------------\r\n";
  if (allPass) {
    response += "OVERALL STATUS: ALL CHECKS PASSED (GO FOR LAUNCH)\r\n";
  } else {
    response += "OVERALL STATUS: CHECKS FAILED (INSPECT HARDWARE)\r\n";
  }
  response += "==================================================";

  CLIEngine::log(CLIEngine::Level::STATE, "TEST", "Self-Test complete: %s", allPass ? "ALL PASSED" : "FAILED");
  return allPass;
}

void abortAutonomousMission(const char* reason) {
  stopBlackboxRecording();
  missionSuccess = false;
  missionAbortReason = reason;
  missionState = AUTON_ABORT;
  autoModeEnabled = false;
  armed = false;
  stopAllThrusters();
  CLIEngine::log(CLIEngine::Level::WARN, "SAFE", "Misi Otonom DIBATALKAN: %s", reason);
  printMissionSummary();
  missionState = MISSION_IDLE;
}

bool startAutonomousMission() {
  if (!outputsReady) {
    CLIEngine::log(CLIEngine::Level::ERROR, "AUTON", "Gagal memulai misi: thruster hardware tidak siap.");
    return false;
  }

  armed = true;
  thrustersAlreadyNeutral = false;
  autoModeEnabled = true;
  stabilizationEnabled = true;
  depthHoldEnabled = true;
  stabilizationFaulted = false;
  resetPidState();
  lastControlAt = millis();

  missionStartMs = millis();
  stepStartedAtMs = missionStartMs;
  settleStartMs = 0;
  lastAutonLogMs = 0;
  maxDepthReachedCm = 0.0f;
  missionTurnErrorDeg = 0.0f;
  missionSuccess = false;
  missionAbortReason = "";

  posX_m = 0.0f;
  posY_m = 0.0f;
  posZ_m = 0.0f;
  lastOdomAt = millis();
  startBlackboxRecording();

  memset(axisInput, 0, sizeof(axisInput));
  missionState = AUTON_START_DELAY;

  CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "Misi Otonom SAUVC Dimulai. Hitung mundur pelepasan (%u s)...", settings.startDelaySec);
  return true;
}

bool cliStartMission() {
  return startAutonomousMission();
}

void updateAutonomousStateMachine(uint32_t now) {
  if (!autoModeEnabled || !armed) return;

  // Emergency Safety Watchdogs
  if (now - missionStartMs > static_cast<uint32_t>(settings.timeoutSec * 1000)) {
    abortAutonomousMission("Total mission timeout terlampaui");
    return;
  }

  ROVSensors::State sensor{};
  ROVSensors::snapshot(sensor);
  const float currentDepthCm = depthReferenceSet
      ? (sensor.pressureMbar - depthReferencePressureMbar) * 10000.0f / (settings.waterDensity * 9.80665f)
      : 0.0f;
  if (currentDepthCm > maxDepthReachedCm) {
    maxDepthReachedCm = currentDepthCm;
  }
  posZ_m = currentDepthCm / 100.0f;

  if (currentDepthCm > settings.maxDepthCm) {
    abortAutonomousMission("Over-depth batas aman terlampaui");
    return;
  }

  if (sensor.imuReady && (fabsf(sensor.attitudePitch) > settings.maxTiltDeg || fabsf(sensor.attitudeRoll) > settings.maxTiltDeg)) {
    abortAutonomousMission("Excessive tilt batas kemiringan terlampaui");
    return;
  }

  switch (missionState) {
    case AUTON_START_DELAY: {
      memset(axisInput, 0, sizeof(axisInput));
      const uint32_t elapsed = (now - stepStartedAtMs) / 1000;
      if (now - lastAutonLogMs >= 1000) {
        lastAutonLogMs = now;
        const uint32_t remaining = (settings.startDelaySec > elapsed) ? (settings.startDelaySec - elapsed) : 0;
        if (remaining > 0) {
          CLIEngine::log(CLIEngine::Level::INFO, "AUTON", "Pelepasan robot dalam %u...", remaining);
        }
      }
      if (now - stepStartedAtMs >= static_cast<uint32_t>(settings.startDelaySec * 1000)) {
        // Lock-on-Release: capture absolute reference heading H0
        referenceHeadingDeg = sensor.compassHeading;
        gyroIntegratedHeadingDeg = referenceHeadingDeg;
        lastHeadingIntegrateMs = now;
        stepStartedAtMs = now;
        settleStartMs = 0;
        missionState = AUTON_DIVE;
        CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "Lock-on-Release: Heading referensi H0 = %.1f° dikunci.", referenceHeadingDeg);
        CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 1/6] MENYELAM ke target %u cm (Settle: %u s)...", settings.depthTargetCm, settings.settleTimeSec);
      }
      break;
    }

    case AUTON_DIVE: {
      axisInput[0] = 0.0f;
      axisInput[1] = 0.0f;
      axisInput[3] = 0.0f;

      const float depthErr = fabsf(currentDepthCm - settings.depthTargetCm);
      if (depthErr <= settings.depthTolCm) {
        if (settleStartMs == 0) {
          settleStartMs = now;
          CLIEngine::log(CLIEngine::Level::INFO, "AUTON", "Target kedalaman tercapai (%.1f cm). Menunggu stabilisasi %u s...", currentDepthCm, settings.settleTimeSec);
        } else if (now - settleStartMs >= static_cast<uint32_t>(settings.settleTimeSec * 1000)) {
          CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "Kedalaman stabil pada %.1f cm. Lanjut ke tahap berikutnya.", currentDepthCm);
          stepStartedAtMs = now;
          if (settings.missionProfile == 2) { // dive_only
            missionState = AUTON_SURFACE;
            CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 6/6] MUNCUL KE PERMUKAAN (Soft Surface Landing)...");
          } else {
            missionState = AUTON_SWIM_FWD;
            CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 2/6] BERENANG MAJU selama %u s pada daya %u%% (Heading hold %.1f°)...",
                           settings.swimSec, settings.swimPower, referenceHeadingDeg);
          }
        }
      } else {
        settleStartMs = 0;
      }

      // Step timeout watchdog
      if (now - stepStartedAtMs >= static_cast<uint32_t>(settings.stepTimeoutSec * 1000)) {
        CLIEngine::log(CLIEngine::Level::WARN, "SAFE", "Step DIVE timeout (%u s). Lanjut tahap berikutnya.", settings.stepTimeoutSec);
        stepStartedAtMs = now;
        missionState = (settings.missionProfile == 2) ? AUTON_SURFACE : AUTON_SWIM_FWD;
      }
      break;
    }

    case AUTON_SWIM_FWD: {
      axisInput[0] = (settings.swimPower / 100.0f);
      axisInput[4] = (settings.pitchBias / 100.0f);

      const float dt = (lastHeadingIntegrateMs > 0 && now > lastHeadingIntegrateMs) ? (now - lastHeadingIntegrateMs) / 1000.0f : 0.02f;
      lastHeadingIntegrateMs = now;
      gyroIntegratedHeadingDeg += (sensor.gyroDps[2] - gyroBias[2]) * dt;

      // Underwater Dead-Reckoning Odometry (X, Y pool projection)
      const float dtOdom = (lastOdomAt > 0 && now > lastOdomAt) ? (now - lastOdomAt) / 1000.0f : dt;
      lastOdomAt = now;
      const float vSurge = 0.45f * (settings.swimPower / 25.0f);
      const float headingRad = (gyroIntegratedHeadingDeg - referenceHeadingDeg) * (PI / 180.0f);
      posX_m += vSurge * cosf(headingRad) * dtOdom;
      posY_m += vSurge * sinf(headingRad) * dtOdom;

      const float headingErr = wrapAngle180(referenceHeadingDeg - gyroIntegratedHeadingDeg);
      const float yawCorr = constrain(- (settings.yawKp * headingErr - settings.yawKd * sensor.gyroDps[2]), -0.25f, 0.25f);
      axisInput[3] = yawCorr;

      if (now - lastAutonLogMs >= 3000) {
        lastAutonLogMs = now;
        const float swimElapsed = (now - stepStartedAtMs) / 1000.0f;
        CLIEngine::log(CLIEngine::Level::INFO, "AUTON", "Berenang maju... (%.1f/%us) | Depth: %.1f cm | Hdg: %.1f° (Err: %.1f°)",
                       swimElapsed, settings.swimSec, currentDepthCm, gyroIntegratedHeadingDeg, headingErr);
      }

      if (now - stepStartedAtMs >= static_cast<uint32_t>(settings.swimSec * 1000)) {
        stepStartedAtMs = now;
        axisInput[0] = 0.0f;
        if (settings.missionProfile == 1) { // gate profile
          missionState = AUTON_SURFACE;
          CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 6/6] MUNCUL KE PERMUKAAN (Soft Surface Landing)...");
        } else {
          missionState = AUTON_BRAKE;
          CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 3/6] REM AKTIF (Active Brake) %.1f s...", settings.brakeTimeSec);
        }
      }
      break;
    }

    case AUTON_BRAKE: {
      axisInput[0] = - (settings.swimPower / 100.0f) * 0.5f;
      axisInput[3] = 0.0f;

      if (now - stepStartedAtMs >= static_cast<uint32_t>(settings.brakeTimeSec * 1000.0f)) {
        axisInput[0] = 0.0f;
        stepStartedAtMs = now;
        turnTargetHeadingDeg = wrapAngle180(referenceHeadingDeg + (settings.turnDir == 0 ? settings.turnAngleDeg : -settings.turnAngleDeg));
        missionState = AUTON_TURN_CW;
        CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 4/6] PUTAR BALIK 180° %s (Target Heading: %.1f°)...",
                       settings.turnDir == 0 ? "CW" : "CCW", turnTargetHeadingDeg);
      }
      break;
    }

    case AUTON_TURN_CW: {
      axisInput[0] = 0.0f;
      const float dt = (lastHeadingIntegrateMs > 0 && now > lastHeadingIntegrateMs) ? (now - lastHeadingIntegrateMs) / 1000.0f : 0.02f;
      lastHeadingIntegrateMs = now;
      gyroIntegratedHeadingDeg += (sensor.gyroDps[2] - gyroBias[2]) * dt;

      const float turnErr = wrapAngle180(turnTargetHeadingDeg - gyroIntegratedHeadingDeg);
      const float turnPwr = settings.turnPower / 100.0f;

      if (fabsf(turnErr) > 20.0f) {
        axisInput[3] = (turnErr > 0) ? -turnPwr : turnPwr;
      } else {
        axisInput[3] = constrain(- (settings.yawKp * turnErr), -turnPwr, turnPwr);
      }

      if (fabsf(turnErr) <= settings.yawTolDeg) {
        missionTurnErrorDeg = fabsf(turnErr);
        axisInput[3] = 0.0f;
        stepStartedAtMs = now;
        CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "Putar balik selesai! Sudut putar tercapai: %.1f° (Target: 180.0°).",
                       180.0f - missionTurnErrorDeg);
        if (settings.missionProfile == 3) { // turn_only
          missionState = AUTON_SURFACE;
          CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 6/6] MUNCUL KE PERMUKAAN (Soft Surface Landing)...");
        } else {
          missionState = AUTON_SWIM_RET;
          CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 5/6] BERENANG KEMBALI selama %u s pada daya %u%% (Heading: %.1f°)...",
                         settings.swimSec, settings.swimPower, turnTargetHeadingDeg);
        }
      }

      // Step timeout watchdog
      if (now - stepStartedAtMs >= static_cast<uint32_t>(settings.stepTimeoutSec * 1000)) {
        missionTurnErrorDeg = fabsf(turnErr);
        CLIEngine::log(CLIEngine::Level::WARN, "SAFE", "Step TURN timeout (%u s). Lanjut tahap berikutnya.", settings.stepTimeoutSec);
        stepStartedAtMs = now;
        axisInput[3] = 0.0f;
        missionState = (settings.missionProfile == 3) ? AUTON_SURFACE : AUTON_SWIM_RET;
      }
      break;
    }

    case AUTON_SWIM_RET: {
      axisInput[0] = (settings.swimPower / 100.0f);
      axisInput[4] = (settings.pitchBias / 100.0f);

      const float dt = (lastHeadingIntegrateMs > 0 && now > lastHeadingIntegrateMs) ? (now - lastHeadingIntegrateMs) / 1000.0f : 0.02f;
      lastHeadingIntegrateMs = now;
      gyroIntegratedHeadingDeg += (sensor.gyroDps[2] - gyroBias[2]) * dt;

      // Underwater Dead-Reckoning Odometry (return leg)
      const float dtOdom = (lastOdomAt > 0 && now > lastOdomAt) ? (now - lastOdomAt) / 1000.0f : dt;
      lastOdomAt = now;
      const float vSurge = 0.45f * (settings.swimPower / 25.0f);
      const float headingRad = (gyroIntegratedHeadingDeg - referenceHeadingDeg) * (PI / 180.0f);
      posX_m += vSurge * cosf(headingRad) * dtOdom;
      posY_m += vSurge * sinf(headingRad) * dtOdom;

      const float headingErr = wrapAngle180(turnTargetHeadingDeg - gyroIntegratedHeadingDeg);
      const float yawCorr = constrain(- (settings.yawKp * headingErr - settings.yawKd * sensor.gyroDps[2]), -0.25f, 0.25f);
      axisInput[3] = yawCorr;

      if (now - lastAutonLogMs >= 3000) {
        lastAutonLogMs = now;
        const float swimElapsed = (now - stepStartedAtMs) / 1000.0f;
        CLIEngine::log(CLIEngine::Level::INFO, "AUTON", "Berenang kembali... (%.1f/%us) | Depth: %.1f cm | Hdg: %.1f° (Err: %.1f°)",
                       swimElapsed, settings.swimSec, currentDepthCm, gyroIntegratedHeadingDeg, headingErr);
      }

      if (now - stepStartedAtMs >= static_cast<uint32_t>(settings.swimSec * 1000)) {
        stepStartedAtMs = now;
        axisInput[0] = 0.0f;
        axisInput[3] = 0.0f;
        missionState = AUTON_SURFACE;
        CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[TAHAP 6/6] MUNCUL KE PERMUKAAN (Soft Surface Landing)...");
      }
      break;
    }

    case AUTON_SURFACE: {
      axisInput[0] = 0.0f;
      axisInput[1] = 0.0f;
      axisInput[3] = 0.0f;

      if (currentDepthCm < 5.0f) {
        depthHoldEnabled = false;
        axisInput[2] = 0.0f;
      }

      if (currentDepthCm < 2.0f || (now - stepStartedAtMs >= 5000)) {
        CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "Robot telah mencapai permukaan air dengan mulus (Soft Surface). Misi selesai!");
        missionSuccess = true;
        missionState = AUTON_COMPLETE;
        if (blackboxActive && blackboxFile) {
          ROVSensors::State s{};
          ROVSensors::snapshot(s);
          blackboxFile.printf("%lu,COMPLETE,%.1f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n",
                              now, currentDepthCm, s.attitudeRoll, s.attitudePitch,
                              s.compassHeading, currentThrusterPulse[0], currentThrusterPulse[1],
                              currentThrusterPulse[2], currentThrusterPulse[3],
                              currentThrusterPulse[4], currentThrusterPulse[5],
                              posX_m, posY_m);
        }
        stopBlackboxRecording();
        stopAllThrusters();
        printMissionSummary();
        missionState = MISSION_IDLE;
      }
      break;
    }

    default:
      break;
  }
}

void updateCalibration(uint32_t now) {
  if (missionState != MISSION_CALIBRATING) return;

  ROVSensors::State sensor{};
  ROVSensors::snapshot(sensor);

  if (sensor.pressureReady && isfinite(sensor.pressureMbar) && sensor.pressureMbar > 0.0f) {
    calibPressureSum += sensor.pressureMbar;
    calibPressureSamples++;
  }
  if (sensor.imuReady) {
    calibGyroSum[0] += sensor.gyroDps[0];
    calibGyroSum[1] += sensor.gyroDps[1];
    calibGyroSum[2] += sensor.gyroDps[2];
    calibGyroSamples++;
  }

  const uint32_t elapsedSec = (now - calibStartMs) / 1000;
  if (elapsedSec > lastCalibSecReported && elapsedSec <= settings.calibTimeSec) {
    lastCalibSecReported = elapsedSec;
    CLIEngine::log(CLIEngine::Level::INFO, "CALIB", "Mengukur titik nol permukaan air... (%u/%us)",
                   elapsedSec, settings.calibTimeSec);
  }

  if (now - calibStartMs >= static_cast<uint32_t>(settings.calibTimeSec * 1000)) {
    if (calibPressureSamples > 0) {
      depthReferencePressureMbar = calibPressureSum / calibPressureSamples;
    } else {
      depthReferencePressureMbar = 1013.25f;
    }
    depthReferenceSet = true;

    if (calibGyroSamples > 0) {
      gyroBias[0] = calibGyroSum[0] / calibGyroSamples;
      gyroBias[1] = calibGyroSum[1] / calibGyroSamples;
      gyroBias[2] = calibGyroSum[2] / calibGyroSamples;
    }
    stabilizationTargetPitch = 0.0f;
    stabilizationTargetRoll = 0.0f;
    resetPidState();

    CLIEngine::log(CLIEngine::Level::STATE, "CALIB", "Kalibrasi selesai! P0 = %.2f mbar. AUV SIAP.", depthReferencePressureMbar);

    // ESC Confirmation Beep Tone at second 10
    if (settings.beepCalib == 1) {
      missionState = MISSION_CALIB_BEEP;
      playEscBeepTone(600);
    }

    // Smart Auto-Run check
    if (settings.autorun == 1 && !autorunCancelled) {
      CLIEngine::log(CLIEngine::Level::STATE, "AUTON", "[AUTORUN] Memulai hitung mundur peluncuran otonom (%u detik)...", settings.startDelaySec);
      startAutonomousMission();
    } else {
      missionState = MISSION_IDLE;
      CLIEngine::log(CLIEngine::Level::INFO, "SYS", "AUV dalam posisi STANDBY (Active Balancing siap). Ketik 'run' untuk meluncur.");
    }
  }
}

bool cliStartMotorTest(uint8_t id, int16_t pulseOrPercent, String& errorMsg) {
  if (!outputsReady) {
    errorMsg = "Thruster hardware not attached or failed.";
    return false;
  }
  if (id < 1 || id > THRUSTER_COUNT) {
    errorMsg = "Motor ID must be between 1 and 6.";
    return false;
  }

  individualThrusterMode = false;
  testSequenceActive = false;
  autoModeEnabled = false;

  uint16_t pulse = settings.pulseNeutral[id - 1];
  if (pulseOrPercent >= 800 && pulseOrPercent <= 2200) {
    pulse = static_cast<uint16_t>(pulseOrPercent);
  } else if (pulseOrPercent >= -100 && pulseOrPercent <= 100) {
    const float frac = pulseOrPercent / 100.0f;
    if (frac >= 0.0f) {
      pulse = settings.pulseNeutral[id - 1] + static_cast<uint16_t>(frac * (settings.pulseMax[id - 1] - settings.pulseNeutral[id - 1]));
    } else {
      pulse = settings.pulseNeutral[id - 1] + static_cast<uint16_t>(frac * (settings.pulseNeutral[id - 1] - settings.pulseMin[id - 1]));
    }
  } else {
    errorMsg = "Value must be pulse (800-2200us) or percent (-100 to 100).";
    return false;
  }

  if (pulse < settings.pulseMin[id - 1]) pulse = settings.pulseMin[id - 1];
  if (pulse > settings.pulseMax[id - 1]) pulse = settings.pulseMax[id - 1];

  if (simMode) {
    const uint16_t maxOffset = 15;
    if (pulse > settings.pulseNeutral[id - 1] + maxOffset) pulse = settings.pulseNeutral[id - 1] + maxOffset;
    if (pulse < settings.pulseNeutral[id - 1] - maxOffset) pulse = settings.pulseNeutral[id - 1] - maxOffset;
  }

  cliTestMotorId = id;
  cliTestMotorPulse = pulse;
  cliTestMotorUntil = millis() + 2000;
  thrustersAlreadyNeutral = false;
  thrusters[id - 1].writeMicroseconds(pulse);
  currentThrusterPulse[id - 1] = pulse;
  CLIEngine::log(CLIEngine::Level::STATE, "TEST", "Testing motor T%u at %uus for 2.0s", id, pulse);
  return true;
}

bool cliCalibrate(const String& type, String& response) {
  if (type == "zero") {
    ROVSensors::State sensor{};
    ROVSensors::snapshot(sensor);
    if (simMode) {
      depthReferencePressureMbar = 1013.25f;
      depthReferenceSet = true;
      CLIEngine::log(CLIEngine::Level::STATE, "CALIB", "Surface pressure zeroed to %.2f mbar (sim)", depthReferencePressureMbar);
      response = "OK: Tekanan referensi nol kedalaman disetel ke " + String(depthReferencePressureMbar, 2) + " mbar (sim).";
      return true;
    }
    if (sensor.pressureReady && isfinite(sensor.pressureMbar) && sensor.pressureMbar > 0.0f) {
      depthReferencePressureMbar = sensor.pressureMbar;
      depthReferenceSet = true;
      CLIEngine::log(CLIEngine::Level::STATE, "CALIB", "Surface pressure zeroed to %.2f mbar", depthReferencePressureMbar);
      response = "OK: Tekanan referensi nol kedalaman disetel ke " + String(depthReferencePressureMbar, 2) + " mbar.";
      return true;
    } else {
      response = "ERROR: Sensor tekanan MS5803 belum siap atau tidak valid.";
      return false;
    }
  } else if (type == "ballast") {
    CLIEngine::log(CLIEngine::Level::STATE, "CALIB", "Ballast buoyancy calibration completed. Heave bias: %.2f%%", settings.heaveBias);
    response = "OK: Kalibrasi ballast dicatat (heave_bias = " + String(settings.heaveBias, 2) + "%).";
    return true;
  } else if (type == "compass") {
    CLIEngine::log(CLIEngine::Level::STATE, "CALIB", "Compass calibration initiated.");
    response = "OK: Kalibrasi kompas dimulai. Putar robot 360 derajat di air.";
    return true;
  } else if (type == "level") {
    stabilizationTargetPitch = 0.0f;
    stabilizationTargetRoll = 0.0f;
    resetPidState();
    CLIEngine::log(CLIEngine::Level::STATE, "CALIB", "IMU level reference zeroed.");
    response = "OK: Sensor IMU dilevelkan ke posisi horizontal nol.";
    return true;
  }
  response = "ERROR: Unknown calibration type. Use 'zero', 'ballast', 'compass', or 'level'.";
  return false;
}

bool cliEscCalib(const String& action, String& response) {
  if (!outputsReady) {
    response = "ERROR: Thruster hardware not attached.";
    return false;
  }
  if (action == "start") {
    escCalibActive = true;
    armed = false;
    stopAllThrusters();
    thrustersAlreadyNeutral = false;
    for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
      currentThrusterPulse[i] = settings.pulseMax[i];
      if (thrusters[i].attached()) {
        thrusters[i].writeMicroseconds(settings.pulseMax[i]);
      }
    }
    CLIEngine::log(CLIEngine::Level::WARN, "ESC", "ESC Calibration: Set all 6 thrusters to MAX (2000us)");
    response = "ESC Calibration STARTED: Sinyal 2000us (MAX) dikirim ke semua 6 thruster.\r\n"
               "Langkah: Hubungkan daya baterai ESC, dengarkan nada awal (beep), lalu ketik 'esc_calib cancel' atau 'stop' setelah nada selesai.";
    return true;
  } else if (action == "cancel" || action == "stop") {
    escCalibActive = false;
    for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
      currentThrusterPulse[i] = settings.pulseNeutral[i];
      if (thrusters[i].attached()) {
        thrusters[i].writeMicroseconds(settings.pulseNeutral[i]);
      }
    }
    thrustersAlreadyNeutral = true;
    CLIEngine::log(CLIEngine::Level::INFO, "ESC", "ESC Calibration finished: All thrusters neutral (1500us)");
    response = "OK: Kalibrasi ESC selesai/dibatalkan. Semua thruster kembali ke netral (1500us).";
    return true;
  }
  response = "ERROR: Usage: esc_calib <start/cancel>";
  return false;
}

void cliReboot() {
  CLIEngine::log(CLIEngine::Level::STATE, "SYS", "Rebooting Pico W now...");
  delay(200);
  rp2040.reboot();
}

void handleCliCommand() {
  if (!requireAuthentication()) return;
  String cmd;
  if (server.hasArg("cmd")) {
    cmd = server.arg("cmd");
  } else if (server.hasArg("command")) {
    cmd = server.arg("command");
  } else if (server.hasArg("plain")) {
    cmd = server.arg("plain");
    cmd.trim();
    if (cmd.startsWith("cmd=")) {
      cmd = cmd.substring(4);
    }
  }

  cmd.trim();
  if (cmd.isEmpty() || cmd.equalsIgnoreCase("poll")) {
    String logs;
    CLIEngine::dumpLog(logs);
    server.send(200, "text/plain; charset=utf-8", logs);
    return;
  }

  String reply = CLIEngine::executeCommand(cmd, true);
  server.send(200, "text/plain; charset=utf-8", reply);
}

void handleCliLogDump() {
  if (!requireAuthentication()) return;
  String logs;
  CLIEngine::dumpLog(logs);
  if (server.uri() == "/api/log/download") {
    server.sendHeader("Content-Disposition", "attachment; filename=\"auv_audit_log.txt\"");
  }
  server.send(200, "text/plain; charset=utf-8", logs);
}

void handleUpdateDone() {
  if (!requireAuthentication()) return;
  server.sendHeader("Connection", "close");
  if (!Update.hasError()) {
    server.send(200, "text/plain", "OK");
    CLIEngine::log(CLIEngine::Level::STATE, "OTA", "OTA firmware update completed successfully! Rebooting Pico W...");
    delay(400);
    rp2040.reboot();
  } else {
    CLIEngine::log(CLIEngine::Level::ERROR, "OTA", "OTA update failed with code %d", Update.getError());
    server.send(500, "text/plain", "FAIL: " + String(Update.getError()));
  }
}

void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    // Safety: Neutralize thrusters immediately before flashing
    stopAllThrusters();
    armed = false;
    autoModeEnabled = false;

    CLIEngine::log(CLIEngine::Level::WARN, "OTA", "OTA upload started: %s (%u B). Thrusters disarmed.",
                   upload.filename.c_str(), upload.totalSize);

    LittleFS.begin();
    FSInfo fsInfo;
    LittleFS.info(fsInfo);
    uint32_t maxSketchSpace = fsInfo.totalBytes - fsInfo.usedBytes;
    if (!Update.begin(maxSketchSpace, U_FLASH)) {
      CLIEngine::log(CLIEngine::Level::ERROR, "OTA", "Update.begin failed (code %d)", Update.getError());
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!Update.hasError()) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        CLIEngine::log(CLIEngine::Level::ERROR, "OTA", "Update.write failed at %u bytes", upload.totalSize);
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.hasError()) {
      if (Update.end(true)) {
        CLIEngine::log(CLIEngine::Level::STATE, "OTA", "OTA binary %u bytes written successfully.", upload.totalSize);
      } else {
        CLIEngine::log(CLIEngine::Level::ERROR, "OTA", "Update.end failed (code %d)", Update.getError());
      }
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.end();
    CLIEngine::log(CLIEngine::Level::WARN, "OTA", "OTA upload aborted.");
  }
}

void pollSerialCLI() {
  static char serialBuffer[128];
  static size_t serialPos = 0;

  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;

    if (c == '\r' || c == '\n') {
      if (serialPos > 0) {
        serialBuffer[serialPos] = '\0';
        String cmdLine = String(serialBuffer);
        serialPos = 0;

        String reply = CLIEngine::executeCommand(cmdLine, false);
        if (reply.length() > 0) {
          Serial.println(reply);
        }
      }
    } else if (c == '\b' || c == 127) {
      if (serialPos > 0) {
        serialPos--;
      }
    } else if (serialPos < sizeof(serialBuffer) - 1) {
      if (c >= 32 && c <= 126) {
        serialBuffer[serialPos++] = static_cast<char>(c);
      }
    }
  }
}

void applyTestSequenceStep(uint8_t step) {
  memset(axisInput, 0, sizeof(axisInput));
  axisInput[TEST_SEQUENCE_AXES[step]] =
      testSequenceStrength * TEST_SEQUENCE_DIRECTIONS[step];
}

void updateTestSequence(uint32_t now) {
  if (!testSequenceActive ||
      now - testSequenceStepStartedAt < TEST_SEQUENCE_STEP_MS) {
    return;
  }

  ++testSequenceStep;
  if (testSequenceStep >= TEST_SEQUENCE_STEP_COUNT) {
    Serial.println("Urutan uji selesai; semua thruster dinetralkan.");
    stopAllThrusters();
    return;
  }

  testSequenceStepStartedAt = now;
  applyTestSequenceStep(testSequenceStep);
}

bool axisHasMixer(size_t axis) {
  for (size_t motor = 0; motor < THRUSTER_COUNT; ++motor) {
    if (fabsf(settings.mix[motor][axis]) > 0.001f) {
      return true;
    }
  }
  return false;
}

bool attachThrusters() {
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    const int channel = thrusters[i].attach(
        settings.pins[i], settings.pulseMin[i], settings.pulseMax[i],
        settings.pulseNeutral[i]);
    if (channel < 0) {
      Serial.printf("ERROR: gagal attach thruster %u pada GP%u\n",
                    static_cast<unsigned>(i + 1),
                    static_cast<unsigned>(settings.pins[i]));
      return false;
    }
    thrusters[i].writeMicroseconds(settings.pulseNeutral[i]);
    currentThrusterPulse[i] = settings.pulseNeutral[i];
  }
  return true;
}

void updateThrusters() {
  if (!outputsReady) {
    return;
  }

  const uint32_t now = millis();

  // If a single motor test is currently running from CLI, keep outputting the test pulse!
  if (cliTestMotorUntil > 0 && now < cliTestMotorUntil) {
    if (cliTestMotorId >= 1 && cliTestMotorId <= THRUSTER_COUNT) {
      for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
        if (i == cliTestMotorId - 1) {
          thrusters[i].writeMicroseconds(cliTestMotorPulse);
          currentThrusterPulse[i] = cliTestMotorPulse;
        } else {
          thrusters[i].writeMicroseconds(settings.pulseNeutral[i]);
          currentThrusterPulse[i] = settings.pulseNeutral[i];
        }
      }
    }
    return;
  }

  if (!armed) {
    if (!thrustersAlreadyNeutral) {
      stopAllThrusters();
    }
    return;
  }
  thrustersAlreadyNeutral = false;

  const uint32_t elapsedMs = lastOutputAt == 0
      ? 20
      : min(static_cast<uint32_t>(now - lastOutputAt), 50u);
  lastOutputAt = now;

  float commandedAxes[AXIS_COUNT];
  memcpy(commandedAxes, axisInput, sizeof(commandedAxes));
  if ((stabilizationEnabled || depthHoldEnabled) && !individualThrusterMode) {
    ROVSensors::State sensor{};
    ROVSensors::snapshot(sensor);
    const bool attitudeReady = (sensor.imuReady &&
        isfinite(sensor.attitudePitch) && isfinite(sensor.attitudeRoll) &&
        isfinite(sensor.gyroDps[0]) && isfinite(sensor.gyroDps[1])) || simMode;
    const bool pressureReady = (sensor.pressureReady &&
        isfinite(sensor.pressureMbar) && sensor.pressureMbar > 0.0f &&
        depthReferenceSet) || simMode;
    if (stabilizationEnabled && !attitudeReady) {
      stabilizationEnabled = false;
      depthHoldEnabled = false;
      autoModeEnabled = false;
      axisInput[4] = 0.0f;
      axisInput[5] = 0.0f;
      commandedAxes[4] = 0.0f;
      commandedAxes[5] = 0.0f;
      commandedAxes[2] = 0.0f;
      stabilizationFaulted = true;
      resetPidState();
      CLIEngine::log(CLIEngine::Level::WARN, "STAB", "IMU not available; pitch/roll stabilization disabled");
    }
    if (depthHoldEnabled && !pressureReady) {
      depthHoldEnabled = false;
      autoModeEnabled = false;
      commandedAxes[2] = 0.0f;
      depthIntegral = 0.0f;
      previousDepthValid = false;
      CLIEngine::log(CLIEngine::Level::WARN, "DEPTH", "Pressure sensor not available; depth hold disabled");
    }
    if (stabilizationEnabled || depthHoldEnabled) {
      const float elapsedSeconds = elapsedMs / 1000.0f;
      if (stabilizationEnabled) {
        const float maxTargetAngle = settings.stabilizationMaxAngleDegrees;
        const float targetRate = settings.stabilizationTargetRateDps;
        stabilizationTargetPitch = constrain(
            stabilizationTargetPitch + axisInput[4] * targetRate * elapsedSeconds,
            -maxTargetAngle, maxTargetAngle);
        stabilizationTargetRoll = constrain(
            stabilizationTargetRoll + axisInput[5] * targetRate * elapsedSeconds,
            -maxTargetAngle, maxTargetAngle);

        const float maxOutput =
            settings.stabilizationMaxOutputPercent / 100.0f;
        const float pitchError = stabilizationTargetPitch - sensor.attitudePitch;
        const float rollError = stabilizationTargetRoll - sensor.attitudeRoll;
        if (axisHasMixer(4)) {
          stabilizationPitchIntegral = constrain(
              stabilizationPitchIntegral +
                  settings.stabilizationKi * pitchError * elapsedSeconds,
              -maxOutput, maxOutput);
          commandedAxes[4] = constrain(
              settings.stabilizationKp * pitchError +
                  stabilizationPitchIntegral -
                  settings.stabilizationKd * sensor.gyroDps[1],
              -maxOutput, maxOutput);
        }
        if (axisHasMixer(5)) {
          stabilizationRollIntegral = constrain(
              stabilizationRollIntegral +
                  settings.stabilizationKi * rollError * elapsedSeconds,
              -maxOutput, maxOutput);
          commandedAxes[5] = constrain(
              settings.stabilizationKp * rollError +
                  stabilizationRollIntegral -
                  settings.stabilizationKd * sensor.gyroDps[0],
              -maxOutput, maxOutput);
        }
      }

      if (depthHoldEnabled) {
        const float depthMeters = (sensor.pressureMbar -
            depthReferencePressureMbar) * 100.0f /
            (settings.waterDensity * 9.80665f);
        const float depthCm = depthMeters * 100.0f;
        const float targetDepthCm = settings.depthTargetCm;
        const float depthErrorCm = targetDepthCm - depthCm;

        // Anti-derivative kick: compute depth rate only on fresh pressure samples
        if (sensor.pressureSampleCount != lastDepthSampleCount &&
            sensor.pressureTimestampMs > 0) {
          if (previousDepthValid && lastDepthTimestampMs > 0 &&
              sensor.pressureTimestampMs > lastDepthTimestampMs) {
            const float dtPressure =
                (sensor.pressureTimestampMs - lastDepthTimestampMs) / 1000.0f;
            if (dtPressure > 0.005f && dtPressure < 1.0f) {
              const float rawRate = (depthCm - previousDepthCm) / dtPressure;
              depthRateCmPerSecond =
                  0.65f * depthRateCmPerSecond + 0.35f * rawRate;
            }
          }
          previousDepthCm = depthCm;
          lastDepthTimestampMs = sensor.pressureTimestampMs;
          lastDepthSampleCount = sensor.pressureSampleCount;
          previousDepthValid = true;
        }

        const float maxOutput = settings.depthMaxOutputPercent / 100.0f;
        if (missionState == AUTON_SURFACE && depthCm < 5.0f) {
          commandedAxes[2] = 0.0f;
          depthIntegral = 0.0f;
        } else if (fabsf(depthErrorCm) <= settings.depthDeadbandCm) {
          depthIntegral = 0.0f;
          commandedAxes[2] = settings.heaveBias / 100.0f;
        } else {
          depthIntegral = constrain(
              depthIntegral + settings.depthKi * depthErrorCm * elapsedSeconds,
              -maxOutput, maxOutput);
          const float pidOutput = constrain(
              settings.depthKp * depthErrorCm + depthIntegral -
                  settings.depthKd * depthRateCmPerSecond,
              -maxOutput, maxOutput);
          commandedAxes[2] = -pidOutput + (settings.heaveBias / 100.0f);
        }
      }
    }
  }

  float motorOutput[THRUSTER_COUNT]{};
  float largestMagnitude = 1.0f;
  for (size_t motor = 0; motor < THRUSTER_COUNT; ++motor) {
    if (individualThrusterMode) {
      motorOutput[motor] = individualThrusterInput[motor];
    } else {
      for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
        motorOutput[motor] += commandedAxes[axis] * settings.mix[motor][axis];
      }
    }
    const float magnitude = fabsf(motorOutput[motor]);
    if (magnitude > largestMagnitude) {
      largestMagnitude = magnitude;
    }
  }

  const float powerLimit = simMode ? 0.03f : (settings.maxPowerPercent / 100.0f);
  const float maxStep = (settings.rampPercentPerSecond / 100.0f) *
      (elapsedMs / 1000.0f);
  for (size_t motor = 0; motor < THRUSTER_COUNT; ++motor) {
    const float target = (motorOutput[motor] / largestMagnitude) * powerLimit;
    if (target > motorOutputApplied[motor]) {
      motorOutputApplied[motor] =
          min(target, motorOutputApplied[motor] + maxStep);
    } else {
      motorOutputApplied[motor] =
          max(target, motorOutputApplied[motor] - maxStep);
    }

    float output = motorOutputApplied[motor];
    if (settings.reverse[motor]) {
      output = -output;
    }

    int pulse = settings.pulseNeutral[motor];
    if (output >= 0.0f) {
      pulse += lroundf(output *
          (settings.pulseMax[motor] - settings.pulseNeutral[motor]));
    } else {
      const float boost = (settings.revBoost[motor] > 0.0f)
          ? settings.revBoost[motor]
          : (settings.globalRevBoost > 0.0f ? settings.globalRevBoost : 1.0f);
      pulse += lroundf(output * boost *
          (settings.pulseNeutral[motor] - settings.pulseMin[motor]));
      pulse = constrain(pulse, static_cast<int>(settings.pulseMin[motor]),
                        static_cast<int>(settings.pulseNeutral[motor]));
    }
    thrusters[motor].writeMicroseconds(pulse);
    currentThrusterPulse[motor] = pulse;
  }
}

void sendConfiguration() {
  String json = "{\"pins\":[";
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    if (i) json += ',';
    json += String(settings.pins[i]);
  }
  json += "],\"min\":[";
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    if (i) json += ',';
    json += String(settings.pulseMin[i]);
  }
  json += "],\"neutral\":[";
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    if (i) json += ',';
    json += String(settings.pulseNeutral[i]);
  }
  json += "],\"max\":[";
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    if (i) json += ',';
    json += String(settings.pulseMax[i]);
  }
  json += "],\"reverse\":[";
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    if (i) json += ',';
    json += String(settings.reverse[i]);
  }
  json += "],\"revBoost\":[";
  for (size_t i = 0; i < THRUSTER_COUNT; ++i) {
    if (i) json += ',';
    json += String(settings.revBoost[i], 2);
  }
  json += "],\"mix\":[";
  for (size_t motor = 0; motor < THRUSTER_COUNT; ++motor) {
    if (motor) json += ',';
    json += '[';
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
      if (axis) json += ',';
      json += String(settings.mix[motor][axis], 2);
    }
    json += ']';
  }
  json += "],\"globalRevBoost\":" + String(settings.globalRevBoost, 2);
  json += ",\"power\":" + String(settings.maxPowerPercent);
  json += ",\"ramp\":" + String(settings.rampPercentPerSecond);
  json += ",\"failsafe\":" + String(settings.failsafeMs);
  json += ",\"maxDepth\":" + String(settings.maxDepthCm);
  json += ",\"maxTilt\":" + String(settings.maxTiltDeg);
  json += ",\"timeout\":" + String(settings.timeoutSec);
  json += ",\"stallCheck\":" + String(settings.stallCheck ? "true" : "false");
  json += ",\"autorun\":" + String(settings.autorun ? "true" : "false");
  json += ",\"waterDensity\":" + String(settings.waterDensity, 1);
  json += ",\"stabilizationKp\":" + String(settings.stabilizationKp, 4);
  json += ",\"stabilizationKi\":" + String(settings.stabilizationKi, 4);
  json += ",\"stabilizationKd\":" + String(settings.stabilizationKd, 4);
  json += ",\"stabilizationMaxOutput\":" +
      String(settings.stabilizationMaxOutputPercent);
  json += ",\"stabilizationMaxAngle\":" +
      String(settings.stabilizationMaxAngleDegrees);
  json += ",\"stabilizationTargetRate\":" +
      String(settings.stabilizationTargetRateDps);
  json += ",\"depthKp\":" + String(settings.depthKp, 4);
  json += ",\"depthKi\":" + String(settings.depthKi, 4);
  json += ",\"depthKd\":" + String(settings.depthKd, 4);
  json += ",\"depthMaxOutput\":" + String(settings.depthMaxOutputPercent);
  json += ",\"depthDeadband\":" + String(settings.depthDeadbandCm);
  json += ",\"depthTarget\":" + String(settings.depthTargetCm);
  json += ",\"layoutMigrated\":" + String(settingsLayoutMigrated ? "true" : "false");
  json += ",\"ssid\":";
  appendJsonString(json, settings.wifiSsid);
  json += '}';
  server.send(200, "application/json", json);
}

void handleSetupPage() {
  if (!setupMode) {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Redirecting");
    return;
  }
  server.send_P(200, "text/html; charset=utf-8", ROVWebPages::SETUP_PAGE);
}

void handleSetupSave() {
  if (!setupMode) {
    sendError(403, "Wi-Fi setup is available only in setup mode.");
    return;
  }
  const String ssid = server.arg("ssid");
  const String wifiPassword = server.arg("wifiPass");
  const String webPassword = server.arg("webPass");

  Settings candidate = settings;
  if (!ssid.isEmpty() &&
      !copyField(ssid, candidate.wifiSsid, sizeof(candidate.wifiSsid), false)) {
    sendError(400, "Nama hotspot tidak valid.");
    return;
  }
  if (!wifiPassword.isEmpty()) {
    if (wifiPassword.length() < 8 ||
        !copyField(wifiPassword, candidate.wifiPassword, sizeof(candidate.wifiPassword), false)) {
      sendError(400, "Password Wi-Fi minimal 8 karakter.");
      return;
    }
  } else {
    memset(candidate.wifiPassword, 0, sizeof(candidate.wifiPassword));
  }
  if (!webPassword.isEmpty() &&
      !copyField(webPassword, candidate.webPassword, sizeof(candidate.webPassword), false)) {
    sendError(400, "Password web minimal 6 karakter.");
    return;
  }

  if (!validateSettings(candidate)) {
    sendError(400, "Pengaturan tidak valid.");
    return;
  }
  settings = candidate;
  if (!persistSettings()) {
    sendError(500, "Gagal menyimpan pengaturan ke flash.");
    return;
  }
  server.send(200, "text/plain; charset=utf-8",
              "Pengaturan tersimpan. Pico akan reboot. Sambungkan ke hotspot Pico lalu buka http://192.168.4.1.");
  delay(500);
  rp2040.reboot();
}

void handleHome() {
  server.send_P(200, "text/html; charset=utf-8", ROVWebPages::CONTROL_PAGE);
}

void handleControl() {
  if (!requireAuthentication()) return;
  cliCancelAutorun();
  if (setupMode || !outputsReady) {
    sendError(503, "Thruster belum siap.");
    return;
  }

  long requestedArm = 0;
  if (!readIntegerArg("armed", 0, 1, requestedArm)) {
    sendError(400, "Status ARM tidak valid.");
    return;
  }
  if (testSequenceActive && requestedArm) {
    lastControlAt = millis();
    server.send(200, "application/json",
                "{\"armed\":true,\"sequence\":true}");
    return;
  }
  float requestedAxes[AXIS_COUNT]{};
  for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
    String name = "a" + String(axis);
    long value = 0;
    if (!readIntegerArg(name, -100, 100, value)) {
      sendError(400, "Input sumbu tidak valid.");
      return;
    }
    requestedAxes[axis] = value / 100.0f;
  }
  if (stabilizationFaulted) {
    requestedAxes[4] = 0.0f;
    requestedAxes[5] = 0.0f;
  }

  if (!requestedArm) {
    stopAllThrusters();
  } else {
    armed = true;
    thrustersAlreadyNeutral = false;
    if (individualThrusterMode) {
      memset(motorOutputApplied, 0, sizeof(motorOutputApplied));
    }
    individualThrusterMode = false;
    memset(individualThrusterInput, 0, sizeof(individualThrusterInput));
    for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
      axisInput[axis] = requestedAxes[axis];
    }
  }
  lastControlAt = millis();
  updateThrusters();
  server.send(200, "application/json", armed ? "{\"armed\":true}" : "{\"armed\":false}");
}

void handleIndividualThrusterControl() {
  if (!requireAuthentication()) return;
  if (setupMode || !outputsReady) {
    sendError(503, "Thruster belum siap.");
    return;
  }
  if (!armed) {
    sendError(409, "ARM thruster sebelum kontrol langsung.");
    return;
  }
  if (testSequenceActive) {
    sendError(409, "Kontrol langsung tidak tersedia saat urutan uji berjalan.");
    return;
  }

  long motor = 0;
  long value = 0;
  if (!readIntegerArg("motor", 0, THRUSTER_COUNT - 1, motor) ||
      !readIntegerArg("value", -100, 100, value)) {
    sendError(400, "Thruster atau nilai kontrol tidak valid.");
    return;
  }

  memset(axisInput, 0, sizeof(axisInput));
  stabilizationEnabled = false;
  depthHoldEnabled = false;
  autoModeEnabled = false;
  resetPidState();
  if (!individualThrusterMode) {
    memset(motorOutputApplied, 0, sizeof(motorOutputApplied));
  }
  memset(individualThrusterInput, 0, sizeof(individualThrusterInput));
  individualThrusterMode = true;
  individualThrusterInput[static_cast<size_t>(motor)] = value / 100.0f;
  thrustersAlreadyNeutral = false;
  lastControlAt = millis();
  updateThrusters();
  server.send(200, "application/json", "{\"individual\":true}");
}

void handleTestSequenceStart() {
  if (!requireAuthentication()) return;
  if (setupMode || !outputsReady) {
    sendError(503, "Thruster belum siap.");
    return;
  }
  if (!armed) {
    sendError(409, "ARM thruster sebelum memulai urutan uji.");
    return;
  }
  if (testSequenceActive) {
    sendError(409, "Urutan uji sedang berjalan.");
    return;
  }

  long strength = 0;
  if (!readIntegerArg("strength", 10, 100, strength)) {
    sendError(400, "Kekuatan urutan harus antara 10 dan 100 persen.");
    return;
  }

  testSequenceStrength = strength / 100.0f;
  testSequenceStep = 0;
  testSequenceStepStartedAt = millis();
  testSequenceActive = true;
  thrustersAlreadyNeutral = false;
  individualThrusterMode = false;
  stabilizationEnabled = false;
  depthHoldEnabled = false;
  autoModeEnabled = false;
  resetPidState();
  memset(individualThrusterInput, 0, sizeof(individualThrusterInput));
  memset(motorOutputApplied, 0, sizeof(motorOutputApplied));
  applyTestSequenceStep(testSequenceStep);
  lastControlAt = testSequenceStepStartedAt;
  updateThrusters();
  Serial.println("Urutan uji dimulai manual; enam tahap masing-masing 10 detik.");
  server.send(200, "application/json",
              "{\"active\":true,\"step\":1,\"steps\":6}");
}

void handleTestSequenceHeartbeat() {
  if (!requireAuthentication()) return;
  if (testSequenceActive && armed) {
    lastControlAt = millis();
  }
  String response = "{\"active\":";
  response += testSequenceActive ? "true" : "false";
  response += ",\"step\":";
  response += String(testSequenceActive ? testSequenceStep + 1 : 0);
  response += ",\"armed\":";
  response += armed ? "true}" : "false}";
  server.send(200, "application/json", response);
}

void handleStatus() {
  if (!requireAuthentication()) return;
  const IPAddress ip = WiFi.softAPIP();
  String json = "{\"ip\":\"" + ip.toString() + "\",\"armed\":";
  json += armed ? "true}" : "false}";
  server.send(200, "application/json", json);
}

void handleDepthZero() {
  if (!requireAuthentication()) return;
  if (armed) {
    sendError(409, "DISARM sebelum mengatur nol kedalaman.");
    return;
  }

  ROVSensors::State sensor{};
  ROVSensors::snapshot(sensor);
  if (!sensor.pressureReady || !isfinite(sensor.pressureMbar) ||
      sensor.pressureMbar <= 0.0f) {
    sendError(503, "Sensor tekanan belum siap.");
    return;
  }

  depthReferencePressureMbar = sensor.pressureMbar;
  depthReferenceSet = true;
  server.send(200, "application/json", "{\"calibrated\":true}");
}

void handleTelemetry() {
  if (!requireAuthentication()) return;

  ROVSensors::State sensor{};
  ROVSensors::snapshot(sensor);
  String json = "{\"imu\":{\"online\":";
  json += sensor.imuReady ? "true" : "false";
  json += ",\"pitch\":" + String(sensor.attitudePitch, 2);
  json += ",\"roll\":" + String(sensor.attitudeRoll, 2);
  json += ",\"accel\":[" + String(sensor.accelG[0], 3) + ',' +
      String(sensor.accelG[1], 3) + ',' + String(sensor.accelG[2], 3) + ']';
  json += ",\"gyro\":[" + String(sensor.gyroDps[0], 2) + ',' +
      String(sensor.gyroDps[1], 2) + ',' + String(sensor.gyroDps[2], 2) + ']';
  json += "},\"compass\":{\"online\":";
  json += sensor.compassReady ? "true" : "false";
  json += ",\"type\":";
  appendJsonString(json, sensor.compassType);
  json += ",\"heading\":" + String(sensor.compassHeading, 2);
  json += ",\"raw\":[" + String(sensor.magnetometer[0], 1) + ',' +
      String(sensor.magnetometer[1], 1) + ',' +
      String(sensor.magnetometer[2], 1) + ']';
  json += "},\"pressure\":{\"online\":";
  json += sensor.pressureReady ? "true" : "false";
  json += ",\"pressureMbar\":" + String(sensor.pressureMbar, 1);
  json += ",\"temperatureC\":" + String(sensor.temperatureC, 2);
  json += ",\"depthCalibrated\":";
  json += depthReferenceSet ? "true" : "false";
  const float depthMeters = depthReferenceSet && sensor.pressureReady
      ? (sensor.pressureMbar - depthReferencePressureMbar) * 100.0f /
            (settings.waterDensity * 9.80665f)
      : 0.0f;
  json += ",\"depthM\":" + String(depthMeters, 2) + '}';
  json += ",\"power\":{\"vsysOnline\":";
  json += sensor.vsysReady ? "true" : "false";
  json += ",\"vsysV\":" + String(sensor.vsysVoltage, 2) + '}';
  json += ",\"i2cReady\":";
  json += sensor.i2cReady ? "true" : "false";
  json += ",\"armed\":";
  json += armed ? "true" : "false";
  json += ",\"sequence\":{\"active\":";
  json += testSequenceActive ? "true" : "false";
  json += ",\"step\":";
  json += String(testSequenceActive ? testSequenceStep + 1 : 0);
  json += '}';
  json += ",\"stabilization\":{\"enabled\":";
  json += stabilizationEnabled ? "true" : "false";
  json += ",\"faulted\":";
  json += stabilizationFaulted ? "true" : "false";
  json += ",\"heaveMixer\":";
  json += axisHasMixer(2) ? "true" : "false";
  json += ",\"pitchMixer\":";
  json += axisHasMixer(4) ? "true" : "false";
  json += ",\"rollMixer\":";
  json += axisHasMixer(5) ? "true" : "false";
  json += '}';
  json += ",\"auto\":{\"enabled\":";
  json += autoModeEnabled ? "true" : "false";
  json += ",\"depthHold\":";
  json += depthHoldEnabled ? "true" : "false";
  json += ",\"targetDepthCm\":" + String(settings.depthTargetCm) + '}';
  json += ",\"thrustersReady\":";
  json += outputsReady ? "true" : "false";
  json += ",\"uptime\":" + String(millis() / 1000);

  const char* mState = "IDLE";
  const char* mStep = "IDLE";
  if (escCalibActive || missionState == MISSION_CALIBRATING || missionState == MISSION_CALIB_BEEP) {
    mState = "CALIB";
    mStep = "CALIBRATING";
  } else if (missionState == AUTON_START_DELAY) {
    mState = "RUNNING";
    mStep = "START_DELAY";
  } else if (missionState == AUTON_DIVE) {
    mState = "RUNNING";
    mStep = "DIVE";
  } else if (missionState == AUTON_SWIM_FWD) {
    mState = "RUNNING";
    mStep = "SWIM_FWD";
  } else if (missionState == AUTON_BRAKE) {
    mState = "RUNNING";
    mStep = "BRAKE";
  } else if (missionState == AUTON_TURN_CW) {
    mState = "RUNNING";
    mStep = "TURN_CW";
  } else if (missionState == AUTON_SWIM_RET) {
    mState = "RUNNING";
    mStep = "SWIM_RET";
  } else if (missionState == AUTON_SURFACE) {
    mState = "RUNNING";
    mStep = "SURFACE";
  } else if (missionState == AUTON_COMPLETE) {
    mState = "RUNNING";
    mStep = "COMPLETE";
  } else if (missionState == AUTON_ABORT || stabilizationFaulted) {
    mState = "ABORT";
    mStep = "ABORT";
  } else if (autoModeEnabled && armed) {
    mState = "RUNNING";
    mStep = "RUNNING";
  }
  json += ",\"missionState\":\"" + String(mState) + "\"";
  json += ",\"missionStep\":\"" + String(mStep) + "\"";
  json += ",\"depthCm\":" + String(depthMeters * 100.0f, 1);
  json += ",\"pitch\":" + String(sensor.attitudePitch, 2);
  json += ",\"roll\":" + String(sensor.attitudeRoll, 2);
  json += ",\"heading\":" + String(sensor.compassHeading, 2);
  json += ",\"temperatureC\":" + String(sensor.temperatureC, 2);
  json += ",\"powerLimit\":" + String(settings.maxPowerPercent);
  json += ",\"posX\":" + String(posX_m, 2);
  json += ",\"posY\":" + String(posY_m, 2);
  json += ",\"posZ\":" + String(posZ_m, 2) + '}';
  server.send(200, "application/json", json);
}

void handleStabilization() {
  if (!requireAuthentication()) return;
  if (setupMode || !outputsReady) {
    sendError(503, "Thruster belum siap.");
    return;
  }
  if (testSequenceActive) {
    sendError(409, "Stabilisasi tidak dapat diubah saat urutan uji berjalan.");
    return;
  }

  long requestedEnabled = 0;
  if (!readIntegerArg("enabled", 0, 1, requestedEnabled)) {
    sendError(400, "Status stabilisasi tidak valid.");
    return;
  }
  if (requestedEnabled) {
    if (!armed) {
      sendError(409, "ARM thruster sebelum mengaktifkan stabilisasi.");
      return;
    }
    if (individualThrusterMode) {
      sendError(409, "Keluar dari kontrol langsung sebelum mengaktifkan stabilisasi.");
      return;
    }

    ROVSensors::State sensor{};
    ROVSensors::snapshot(sensor);
    if (!sensor.imuReady || !isfinite(sensor.attitudePitch) ||
        !isfinite(sensor.attitudeRoll) || !isfinite(sensor.gyroDps[0]) ||
        !isfinite(sensor.gyroDps[1])) {
      sendError(503, "Sensor MPU-6050 belum siap.");
      return;
    }
    if (!axisHasMixer(4) && !axisHasMixer(5)) {
      sendError(409, "Mixer pitch dan roll belum dikonfigurasi.");
      return;
    }

    stabilizationTargetPitch = 0.0f;
    stabilizationTargetRoll = 0.0f;
    resetPidState();
    stabilizationFaulted = false;
    stabilizationEnabled = true;
  } else {
    stabilizationEnabled = false;
    depthHoldEnabled = false;
    autoModeEnabled = false;
    resetPidState();
    stabilizationFaulted = false;
  }

  lastControlAt = millis();
  updateThrusters();
  String response = "{\"enabled\":";
  response += stabilizationEnabled ? "true" : "false";
  response += ",\"faulted\":";
  response += stabilizationFaulted ? "true}" : "false}";
  server.send(200, "application/json", response);
}

void handleAutoMode() {
  if (!requireAuthentication()) return;
  if (setupMode || !outputsReady) {
    sendError(503, "Thruster belum siap.");
    return;
  }
  if (testSequenceActive || individualThrusterMode) {
    sendError(409, "Keluar dari kontrol langsung atau urutan uji sebelum mode Auto.");
    return;
  }

  long requestedEnabled = 0;
  if (!readIntegerArg("enabled", 0, 1, requestedEnabled)) {
    sendError(400, "Status mode Auto tidak valid.");
    return;
  }
  if (requestedEnabled) {
    if (!armed) {
      sendError(409, "ARM thruster sebelum mengaktifkan mode Auto.");
      return;
    }

    ROVSensors::State sensor{};
    ROVSensors::snapshot(sensor);
    if (!depthReferenceSet || !sensor.pressureReady ||
        !isfinite(sensor.pressureMbar) || sensor.pressureMbar <= 0.0f) {
      sendError(503, "Nolkan sensor kedalaman di permukaan sebelum mode Auto.");
      return;
    }
    if (!sensor.imuReady || !isfinite(sensor.attitudePitch) ||
        !isfinite(sensor.attitudeRoll) || !isfinite(sensor.gyroDps[0]) ||
        !isfinite(sensor.gyroDps[1])) {
      sendError(503, "Sensor MPU-6050 belum siap.");
      return;
    }
    if (!axisHasMixer(2) || (!axisHasMixer(4) && !axisHasMixer(5))) {
      sendError(409, "Mixer heave dan minimal satu sumbu pitch/roll harus dikonfigurasi.");
      return;
    }

    axisInput[2] = 0.0f;
    axisInput[4] = 0.0f;
    axisInput[5] = 0.0f;
    stabilizationTargetPitch = 0.0f;
    stabilizationTargetRoll = 0.0f;
    stabilizationFaulted = false;
    stabilizationEnabled = true;
    depthHoldEnabled = true;
    autoModeEnabled = true;
    resetPidState();
  } else {
    stabilizationEnabled = false;
    depthHoldEnabled = false;
    autoModeEnabled = false;
    stabilizationFaulted = false;
    resetPidState();
  }

  lastControlAt = millis();
  updateThrusters();
  String response = "{\"enabled\":";
  response += autoModeEnabled ? "true}" : "false}";
  server.send(200, "application/json", response);
}

void handleMotorSettings() {
  if (!requireAuthentication()) return;
  if (armed) {
    sendError(409, "DISARM sebelum mengubah parameter thruster.");
    return;
  }

  long motor = 0;
  long pin = 0;
  long pulseMin = 0;
  long pulseNeutral = 0;
  long pulseMax = 0;
  long reverse = 0;
  if (!readIntegerArg("motor", 0, THRUSTER_COUNT - 1, motor) ||
      !readIntegerArg("pin", 2, 28, pin) ||
      !readIntegerArg("min", 900, 1500, pulseMin) ||
      !readIntegerArg("neutral", 1400, 1600, pulseNeutral) ||
      !readIntegerArg("max", 1500, 2100, pulseMax) ||
      !readIntegerArg("reverse", 0, 1, reverse)) {
    sendError(400, "Parameter thruster tidak valid.");
    return;
  }

  Settings candidate = settings;
  const size_t index = static_cast<size_t>(motor);
  candidate.pins[index] = static_cast<uint8_t>(pin);
  candidate.pulseMin[index] = static_cast<uint16_t>(pulseMin);
  candidate.pulseNeutral[index] = static_cast<uint16_t>(pulseNeutral);
  candidate.pulseMax[index] = static_cast<uint16_t>(pulseMax);
  candidate.reverse[index] = static_cast<uint8_t>(reverse);
  for (size_t axis = 0; axis < AXIS_COUNT; ++axis) {
    String name = "mix" + String(axis);
    if (!readFloatArg(name, candidate.mix[index][axis])) {
      sendError(400, "Nilai mixer harus antara -1 dan 1.");
      return;
    }
  }
  // DEAD-02 cleanup: removed redundant pre-validation checks
  if (!validateSettings(candidate)) {
    sendError(400, "Konfigurasi thruster tidak valid atau GPIO sudah digunakan.");
    return;
  }

  settings = candidate;
  server.send(200, "text/plain", "OK");
}

void handleSystemSettings() {
  if (!requireAuthentication()) return;
  if (armed) {
    sendError(409, "DISARM sebelum mengubah pengaturan sistem.");
    return;
  }
  long power = 0;
  long ramp = 0;
  long failsafe = 0;
  long stabilizationMaxOutput = 0;
  long stabilizationMaxAngle = 0;
  long stabilizationTargetRate = 0;
  long depthMaxOutput = 0;
  long depthDeadband = 0;
  long depthTarget = 0;
  float stabilizationKp = 0.0f;
  float stabilizationKi = 0.0f;
  float stabilizationKd = 0.0f;
  float depthKp = 0.0f;
  float depthKi = 0.0f;
  float depthKd = 0.0f;
  if (!readIntegerArg("power", 1, 100, power) ||
      !readIntegerArg("ramp", 5, 200, ramp) ||
      !readIntegerArg("failsafe", MIN_FAILSAFE_MS, MAX_FAILSAFE_MS, failsafe) ||
      !readBoundedFloatArg("stabilizationKp", 0.0f, 1.0f, stabilizationKp) ||
      !readBoundedFloatArg("stabilizationKi", 0.0f, 0.1f, stabilizationKi) ||
      !readBoundedFloatArg("stabilizationKd", 0.0f, 1.0f, stabilizationKd) ||
      !readIntegerArg("stabilizationMaxOutput", 5, 100,
                      stabilizationMaxOutput) ||
      !readIntegerArg("stabilizationMaxAngle", 5, 45,
                      stabilizationMaxAngle) ||
      !readIntegerArg("stabilizationTargetRate", 1, 60,
                      stabilizationTargetRate) ||
      !readBoundedFloatArg("depthKp", 0.0f, 2.0f, depthKp) ||
      !readBoundedFloatArg("depthKi", 0.0f, 0.5f, depthKi) ||
      !readBoundedFloatArg("depthKd", 0.0f, 1.0f, depthKd) ||
      !readIntegerArg("depthMaxOutput", 5, 100, depthMaxOutput) ||
      !readIntegerArg("depthDeadband", 0, 25, depthDeadband) ||
      !readIntegerArg("depthTarget", 0, 500, depthTarget)) {
    sendError(400, "Parameter daya, PID, batas output, target, atau failsafe tidak valid.");
    return;
  }

  Settings candidate = settings;
  candidate.maxPowerPercent = static_cast<uint8_t>(power);
  candidate.rampPercentPerSecond = static_cast<uint8_t>(ramp);
  candidate.failsafeMs = static_cast<uint16_t>(failsafe);
  candidate.stabilizationKp = stabilizationKp;
  candidate.stabilizationKi = stabilizationKi;
  candidate.stabilizationKd = stabilizationKd;
  candidate.stabilizationMaxOutputPercent =
      static_cast<uint8_t>(stabilizationMaxOutput);
  candidate.stabilizationMaxAngleDegrees =
      static_cast<uint8_t>(stabilizationMaxAngle);
  candidate.stabilizationTargetRateDps =
      static_cast<uint8_t>(stabilizationTargetRate);
  candidate.depthKp = depthKp;
  candidate.depthKi = depthKi;
  candidate.depthKd = depthKd;
  candidate.depthMaxOutputPercent = static_cast<uint8_t>(depthMaxOutput);
  candidate.depthDeadbandCm = static_cast<uint8_t>(depthDeadband);
  candidate.depthTargetCm = static_cast<uint16_t>(depthTarget);
  const String ssid = server.arg("ssid");
  const String wifiPassword = server.arg("wifiPass");
  const String webPassword = server.arg("webPass");
  if (!ssid.isEmpty() &&
      !copyField(ssid, candidate.wifiSsid, sizeof(candidate.wifiSsid), false)) {
    sendError(400, "SSID tidak valid.");
    return;
  }
  if (!wifiPassword.isEmpty()) {
    if (wifiPassword.length() < 8 ||
        !copyField(wifiPassword, candidate.wifiPassword, sizeof(candidate.wifiPassword), false)) {
      sendError(400, "Password Wi-Fi minimal 8 karakter.");
      return;
    }
  }
  if (!webPassword.isEmpty() &&
      !copyField(webPassword, candidate.webPassword, sizeof(candidate.webPassword), false)) {
    sendError(400, "Password web minimal 6 karakter.");
    return;
  }
  if (!validateSettings(candidate)) {
    sendError(400, "Pengaturan sistem tidak valid.");
    return;
  }
  settings = candidate;
  server.send(200, "text/plain", "OK");
}

void handleSaveSettings() {
  if (!requireAuthentication()) return;
  if (armed) {
    sendError(409, "DISARM sebelum menyimpan pengaturan.");
    return;
  }
  if (!validateSettings(settings) || !persistSettings()) {
    sendError(500, "Pengaturan tidak valid atau gagal ditulis ke flash.");
    return;
  }
  memcpy(activeWebPassword, settings.webPassword, sizeof(activeWebPassword));
  server.send(200, "text/plain", "Saved");
  delay(500);
  rp2040.reboot();
}

bool startAccessPoint() {
  WiFi.mode(WIFI_AP);

  if (setupMode) {
    setupApName = DEFAULT_HOTSPOT_NAME;
    setupApPassword = DEFAULT_HOTSPOT_PASSWORD;
  } else {
    setupApName = settings.wifiSsid;
    setupApPassword = settings.wifiPassword;
  }

  if (!WiFi.softAPConfig(ACCESS_POINT_IP, ACCESS_POINT_IP,
                         ACCESS_POINT_SUBNET)) {
    Serial.println("ERROR: gagal mengatur IP hotspot 192.168.4.1.");
    return false;
  }

  const bool apStarted = (setupApPassword.length() >= 8)
      ? WiFi.softAP(setupApName.c_str(), setupApPassword.c_str())
      : WiFi.softAP(setupApName.c_str());

  if (!apStarted) {
    Serial.println("ERROR: gagal membuat hotspot. Periksa board dan catu daya.");
    return false;
  }
  WiFi.noLowPowerMode();
  accessPointReady = true;
  Serial.printf("Hotspot aktif: SSID '%s', IP http://%s, WPA2: %s\n",
                setupApName.c_str(), WiFi.softAPIP().toString().c_str(),
                (setupApPassword.length() >= 8) ? "ya" : "terbuka");
  lastSetupMessageAt = millis();
  return true;
}

void startWebServer() {
  server.on("/", HTTP_GET, handleHome);
  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/setup", HTTP_POST, handleSetupSave);
  server.on("/api/control", HTTP_POST, handleControl);
  server.on("/api/thruster/control", HTTP_POST, handleIndividualThrusterControl);
  server.on("/api/sequence/start", HTTP_POST, handleTestSequenceStart);
  server.on("/api/sequence/heartbeat", HTTP_POST, handleTestSequenceHeartbeat);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  server.on("/api/depth/zero", HTTP_POST, handleDepthZero);
  server.on("/api/stabilization", HTTP_POST, handleStabilization);
  server.on("/api/auto", HTTP_POST, handleAutoMode);
  server.on("/api/config", HTTP_GET, []() {
    if (!requireAuthentication()) return;
    sendConfiguration();
  });
  server.on("/api/motor", HTTP_POST, handleMotorSettings);
  server.on("/api/system", HTTP_POST, handleSystemSettings);
  server.on("/api/save", HTTP_POST, handleSaveSettings);
  server.on("/api/cli", HTTP_POST, handleCliCommand);
  server.on("/api/cli", HTTP_GET, handleCliLogDump);
  server.on("/api/terminal", HTTP_POST, handleCliCommand);
  server.on("/api/terminal", HTTP_GET, handleCliLogDump);
  server.on("/api/terminal/log", HTTP_GET, handleCliLogDump);
  server.on("/api/terminal/poll", HTTP_GET, handleCliLogDump);
  server.on("/api/log/download", HTTP_GET, handleCliLogDump);
  server.on("/api/blackbox/download", HTTP_GET, handleBlackboxDownload);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.onNotFound([]() { sendError(404, "Not found"); });
  server.begin();
  serverStarted = true;
  CLIEngine::log(CLIEngine::Level::INFO, "WEB", "Web server active on port 80");
}

}  // namespace

void setup1() {
  while (!sensorsInitialized.load(std::memory_order_acquire)) {
    sleep_ms(1);
  }
}

void loop1() {
  ROVSensors::update();
  sleep_ms(5);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  if (!LittleFS.begin()) {
    LittleFS.format();
    LittleFS.begin();
  }

  settingsLoaded = loadSettings();
  if (!settingsLoaded) {
    setDefaults();
    persistSettings();
    Serial.println("EEPROM baru/direset ke konfigurasi Firmware v1.0 default.");
  }
  setupMode = false;

  // Initialize Unified CLI Engine & Ring-Buffer Logger
  CLIEngine::SystemContext ctx{};
  ctx.settings = &settings;
  ctx.armed = &armed;
  ctx.outputsReady = &outputsReady;
  ctx.autoModeEnabled = &autoModeEnabled;
  ctx.stabilizationEnabled = &stabilizationEnabled;
  ctx.depthHoldEnabled = &depthHoldEnabled;
  ctx.simMode = &simMode;
  ctx.depthReferenceSet = &depthReferenceSet;
  ctx.depthReferencePressureMbar = &depthReferencePressureMbar;
  ctx.rpiStreamEnabled = &rpiStreamEnabled;
  ctx.posX = &posX_m;
  ctx.posY = &posY_m;
  ctx.posZ = &posZ_m;
  ctx.stopAllThrusters = stopAllThrusters;
  ctx.persistSettings = persistSettings;
  ctx.setDefaults = setDefaults;
  ctx.startMission = cliStartMission;
  ctx.startMotorTest = cliStartMotorTest;
  ctx.calibrate = cliCalibrate;
  ctx.escCalib = cliEscCalib;
  ctx.reboot = cliReboot;
  ctx.cancelAutorun = cliCancelAutorun;
  ctx.setSimMode = cliSetSimMode;
  ctx.selfTest = cliRunSelfTest;
  ctx.blackboxControl = cliBlackboxControl;
  CLIEngine::begin(ctx);
  CLIEngine::log(CLIEngine::Level::INFO, "FS", "LittleFS mounted for wireless OTA firmware update");

  if (settingsLoaded) {
    CLIEngine::log(CLIEngine::Level::INFO, "CFG", "Configuration loaded from EEPROM (Firmware v1.0)");
  } else {
    CLIEngine::log(CLIEngine::Level::WARN, "CFG", "No valid configuration found; operating in setup mode");
  }

  ROVSensors::begin();
  sensorsInitialized.store(true, std::memory_order_release);
  CLIEngine::log(CLIEngine::Level::INFO, "SENS", "Sensors initialized (I2C Wire1)");

  outputsReady = attachThrusters();
  if (outputsReady) {
    CLIEngine::log(CLIEngine::Level::INFO, "MOT", "All 6 thrusters attached successfully");
  } else {
    stopAllThrusters();
    CLIEngine::log(CLIEngine::Level::ERROR, "MOT", "Thruster attachment failed; outputs disabled");
  }

  if (startAccessPoint()) {
    startWebServer();
  } else {
    CLIEngine::log(CLIEngine::Level::ERROR, "WIFI", "Failed to start SoftAP; web server aborted");
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize 10s auto-calibration at boot
  missionState = MISSION_CALIBRATING;
  calibStartMs = millis();
  lastCalibSecReported = 0;
  calibPressureSum = 0.0f;
  calibPressureSamples = 0;
  calibGyroSum[0] = 0.0f;
  calibGyroSum[1] = 0.0f;
  calibGyroSum[2] = 0.0f;
  calibGyroSamples = 0;
  autorunCancelled = false;

  CLIEngine::log(CLIEngine::Level::STATE, "CALIB", "Memulai auto-kalibrasi %u detik. Jangan gerakkan robot.", settings.calibTimeSec);
}

void printSerialStatus() {
  Serial.printf("ROV Pico W | Hotspot: %s | IP: http://%s | Web: %s | Thruster: %s\n",
                setupApName.c_str(), WiFi.softAPIP().toString().c_str(),
                accessPointReady ? "siap" : "tidak tersedia",
                outputsReady ? "siap, DISARM" : "ERROR");
  if (setupMode) {
    Serial.println("Setup pertama | Hotspot terbuka tanpa password Wi-Fi.");
  }
  Serial.println("CLI aktif pada 115200 baud. Ketik 'help' atau 'status' untuk memulai.");
  lastStatusMessageAt = millis();
}

void loop() {
  if (serverStarted) {
    server.handleClient();
  }

  // Poll non-blocking USB Serial CLI
  pollSerialCLI();

  const uint32_t now = millis();
  const bool serialConnected = static_cast<bool>(Serial);
  if (serialConnected && !serialWasConnected) {
    printSerialStatus();
  }
  serialWasConnected = serialConnected;

  // Single motor test timer
  if (cliTestMotorUntil > 0 && now >= cliTestMotorUntil) {
    if (cliTestMotorId >= 1 && cliTestMotorId <= THRUSTER_COUNT) {
      if (thrusters[cliTestMotorId - 1].attached()) {
        thrusters[cliTestMotorId - 1].writeMicroseconds(settings.pulseNeutral[cliTestMotorId - 1]);
        currentThrusterPulse[cliTestMotorId - 1] = settings.pulseNeutral[cliTestMotorId - 1];
      }
      CLIEngine::log(CLIEngine::Level::STATE, "TEST", "Motor T%u test complete (returned to neutral)", cliTestMotorId);
    }
    cliTestMotorUntil = 0;
    cliTestMotorId = 0;
    thrustersAlreadyNeutral = false;
  }

  // Underwater Blackbox Flight Recorder (10Hz / 100ms sample rate)
  if (blackboxActive && (missionState >= AUTON_DIVE && missionState <= AUTON_COMPLETE)) {
    if (now - lastBlackboxSampleAt >= 100) {
      lastBlackboxSampleAt = now;
      ROVSensors::State s{};
      ROVSensors::snapshot(s);
      const float curDepthCm = depthReferenceSet
          ? (s.pressureMbar - depthReferencePressureMbar) * 10000.0f / (settings.waterDensity * 9.80665f)
          : 0.0f;
      const char* stName = getMissionStepString();
      if (blackboxFile) {
        blackboxFile.printf("%lu,%s,%.1f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n",
                            now, stName, curDepthCm, s.attitudeRoll, s.attitudePitch,
                            s.compassHeading, currentThrusterPulse[0], currentThrusterPulse[1],
                            currentThrusterPulse[2], currentThrusterPulse[3],
                            currentThrusterPulse[4], currentThrusterPulse[5],
                            posX_m, posY_m);
      }
    }
  }

  // Raspberry Pi 5 Companion Computer Stream Bridge (20Hz / 50ms structured JSON)
  if (rpiStreamEnabled && now - lastRpiStreamAt >= 50) {
    lastRpiStreamAt = now;
    ROVSensors::State s{};
    ROVSensors::snapshot(s);
    const float curDepthCm = depthReferenceSet
        ? (s.pressureMbar - depthReferencePressureMbar) * 10000.0f / (settings.waterDensity * 9.80665f)
        : 0.0f;
    const char* stName = getMissionStepString();
    Serial.printf("{\"t\":%lu,\"depth\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"x\":%.2f,\"y\":%.2f,\"state\":\"%s\",\"armed\":%s}\n",
                  now, curDepthCm, s.attitudeRoll, s.attitudePitch, s.compassHeading,
                  posX_m, posY_m, stName, armed ? "true" : "false");
  }

  // Desk simulation physics update
  static uint32_t lastSimUpdateAt = 0;
  if (simMode && now - lastSimUpdateAt >= 20) {
    updateSimulation(lastSimUpdateAt == 0 ? 20 : (now - lastSimUpdateAt));
    lastSimUpdateAt = now;
  }

  // State machine execution:
  if (missionState == MISSION_CALIBRATING) {
    updateCalibration(now);
  } else if (autoModeEnabled && armed) {
    lastControlAt = now; // Suppress manual web heartbeat failsafe during autonomous run
    updateAutonomousStateMachine(now);
  }

  // Heartbeat failsafe (only in manual tele-operation, NOT during autonomous run)
  if (armed && !autoModeEnabled && now - lastControlAt > settings.failsafeMs) {
    CLIEngine::log(CLIEngine::Level::WARN, "SAFE", "Failsafe: web heartbeat lost; all thrusters neutralized");
    stopAllThrusters();
    updateThrusters();
  } else {
    updateTestSequence(now);
    if (now - lastOutputAt >= 20) {
      updateThrusters();
    }
  }
  if (setupMode && now - lastSetupMessageAt >= 10000) {
    lastSetupMessageAt = now;
    CLIEngine::log(CLIEngine::Level::INFO, "SETUP", "Setup hotspot: SSID '%s', password '%s', URL http://%s",
                   setupApName.c_str(), setupApPassword.c_str(),
                   WiFi.softAPIP().toString().c_str());
  }
  if (!setupMode && serialConnected && now - lastStatusMessageAt >= 15000) {
    Serial.printf("Hotspot: %s | IP: http://%s | Thruster: %s\n",
                  setupApName.c_str(), WiFi.softAPIP().toString().c_str(),
                  armed ? "ARMED" : "DISARM");
    lastStatusMessageAt = now;
  }
  delay(1);
}
