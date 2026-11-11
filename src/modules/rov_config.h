#pragma once

#include <stddef.h>
#include <stdint.h>

namespace ROVConfig {

constexpr uint32_t CONFIG_MAGIC = 0x524F5657;
constexpr uint8_t CONFIG_VERSION = 8;
constexpr char DEFAULT_HOTSPOT_NAME[] = "AUV_UBB_JUARA";
constexpr char DEFAULT_HOTSPOT_PASSWORD[] = "ubbjuara2027";
constexpr char DEFAULT_WEB_USERNAME[] = "menang";
constexpr size_t MIN_WEB_PASSWORD_LENGTH = 6;
constexpr size_t THRUSTER_COUNT = 6;
constexpr size_t AXIS_COUNT = 6;

// Reverse Boost & Deadband
constexpr float DEFAULT_REV_BOOST = 1.3f;
constexpr uint16_t DEFAULT_DEADBAND_US = 25;
constexpr uint16_t DEFAULT_PULSE_MIN_US = 1000;
constexpr uint16_t DEFAULT_PULSE_NEUTRAL_US = 1500;
constexpr uint16_t DEFAULT_PULSE_MAX_US = 2000;

// Safety & Limits
constexpr uint8_t DEFAULT_MAX_POWER_PERCENT = 25;
constexpr uint8_t DEFAULT_RAMP_PERCENT_PER_SECOND = 40;
constexpr uint16_t DEFAULT_FAILSAFE_MS = 600;
constexpr uint16_t MIN_FAILSAFE_MS = 250;
constexpr uint16_t MAX_FAILSAFE_MS = 2000;
constexpr uint16_t DEFAULT_MAX_DEPTH_CM = 120;
constexpr uint8_t DEFAULT_MAX_TILT_DEG = 50;
constexpr uint16_t DEFAULT_TIMEOUT_SEC = 60;
constexpr uint8_t DEFAULT_STALL_CHECK = 1;
constexpr uint16_t DEFAULT_STEP_TIMEOUT_SEC = 15;
constexpr uint8_t DEFAULT_FAILSAFE_MODE = 0; // 0 = float/neutral, 1 = surface

// Autonomous Parameters
constexpr uint8_t DEFAULT_AUTORUN = 1;
constexpr uint16_t DEFAULT_START_DELAY_SEC = 3;
constexpr uint16_t DEFAULT_SETTLE_TIME_SEC = 2;
constexpr uint16_t DEFAULT_SWIM_SEC = 10;
constexpr uint8_t DEFAULT_SWIM_POWER_PERCENT = 25;
constexpr uint16_t DEFAULT_TURN_ANGLE_DEG = 180;
constexpr uint8_t DEFAULT_TURN_DIR = 0; // 0 = CW, 1 = CCW
constexpr uint8_t DEFAULT_TURN_POWER_PERCENT = 20;
constexpr uint8_t DEFAULT_DEPTH_TOL_CM = 2;
constexpr uint8_t DEFAULT_YAW_TOL_DEG = 3;
constexpr uint8_t DEFAULT_MISSION_PROFILE = 0; // 0 = full, 1 = gate, 2 = dive_only, 3 = turn_only
constexpr uint16_t DEFAULT_DEPTH_TARGET_CM = 50;

// Trim & Ballast
constexpr float DEFAULT_HEAVE_BIAS = 0.0f;
constexpr float DEFAULT_PITCH_BIAS = 0.0f;
constexpr float DEFAULT_TRIM_ROLL = 0.0f;
constexpr float DEFAULT_TRIM_YAW = 0.0f;
constexpr float DEFAULT_BRAKE_TIME_SEC = 0.5f;

// Environment & Audio
constexpr float DEFAULT_WATER_DENSITY = 1000.0f; // Freshwater (pool)
constexpr uint16_t DEFAULT_CALIB_TIME_SEC = 10;
constexpr uint8_t DEFAULT_WAVE_FILTER_SAMPLES = 5;
constexpr float DEFAULT_MAG_DECLINATION_DEG = 0.0f;
constexpr uint8_t DEFAULT_WIFI_SLEEP = 1;
constexpr uint8_t DEFAULT_BEEP_CALIB = 1;
constexpr uint8_t DEFAULT_BEEP_MISSION = 0;
constexpr uint8_t DEFAULT_BEEP_PWR_PERCENT = 10;

// PID Stabilization & Heading
constexpr float DEFAULT_STABILIZATION_KP = 0.02f;
constexpr float DEFAULT_STABILIZATION_KI = 0.0f;
constexpr float DEFAULT_STABILIZATION_KD = 0.008f;
constexpr uint8_t DEFAULT_STABILIZATION_MAX_OUTPUT_PERCENT = 25;
constexpr uint8_t DEFAULT_STABILIZATION_MAX_ANGLE_DEGREES = 35;
constexpr uint8_t DEFAULT_STABILIZATION_TARGET_RATE_DPS = 20;

constexpr float DEFAULT_DEPTH_KP = 0.04f;
constexpr float DEFAULT_DEPTH_KI = 0.0f;
constexpr float DEFAULT_DEPTH_KD = 0.015f;
constexpr uint8_t DEFAULT_DEPTH_MAX_OUTPUT_PERCENT = 40;
constexpr uint8_t DEFAULT_DEPTH_DEADBAND_CM = 2;

constexpr float DEFAULT_YAW_KP = 0.03f;
constexpr float DEFAULT_YAW_KI = 0.0f;
constexpr float DEFAULT_YAW_KD = 0.01f;
constexpr uint8_t DEFAULT_PID_LIMIT_PERCENT = 30;
constexpr uint8_t DEFAULT_RL_ENABLED = 0;
constexpr float DEFAULT_RL_RATE = 0.01f;
constexpr float DEFAULT_RL_MAX_KP = 0.5f;

// Hardware Bus & Sensor Definitions
constexpr uint8_t I2C_SDA_PIN = 26;
constexpr uint8_t I2C_SCL_PIN = 27;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint16_t SENSOR_UPDATE_INTERVAL_MS = 50;
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t MPU6050_ALT_ADDRESS = 0x69;
constexpr uint8_t HMC5883L_ADDRESS = 0x1E;
constexpr uint8_t QMC5883L_ADDRESS = 0x0D;
constexpr uint8_t MS5803_ADDRESS_HIGH = 0x76;
constexpr uint8_t MS5803_ADDRESS_LOW = 0x77;
constexpr uint8_t MS5803_PRESSURE_4096_COMMAND = 0x48;
constexpr uint8_t MS5803_TEMPERATURE_4096_COMMAND = 0x58;
constexpr uint8_t MS5803_CONVERSION_TIME_MS = 10;
constexpr uint8_t VSYS_ADC_PIN = 29;
constexpr uint8_t VSYS_ADC_SAMPLE_COUNT = 8;
constexpr uint16_t VSYS_SAMPLE_INTERVAL_MS = 250;
constexpr float VSYS_ADC_REFERENCE_VOLTS = 3.3f;
constexpr float VSYS_DIVIDER_MULTIPLIER = 3.0f;

// GPIO output per motor, in T1-T6 order.
constexpr uint8_t DEFAULT_THRUSTER_PINS[THRUSTER_COUNT] = {2, 3, 4, 5, 6, 7};

// Default direction inversion (0: Normal, 1: Reverse) and reverse boost compensation per motor (T1..T6)
constexpr uint8_t DEFAULT_THRUSTER_REVERSE[THRUSTER_COUNT] = {0, 0, 0, 0, 1, 1};
constexpr float DEFAULT_THRUSTER_REV_BOOST[THRUSTER_COUNT] = {1.30f, 1.05f, 1.20f, 1.40f, 1.25f, 1.35f};

// Mixer row order: T1 rear-left (CW), T2 front-right (CW), T3 front-left (CCW),
// T4 rear-right (CCW), T5 middle-right vertical (CCW), T6 middle-left vertical (CW).
// Column order: surge, sway, heave, yaw, pitch, roll.
// Positive roll raises the right side: T5 is right and T6 is left.
constexpr float DEFAULT_MIX[THRUSTER_COUNT][AXIS_COUNT] = {
    {1.0f, -1.0f, 0.0f, -1.0f, 0.0f,  0.0f},
    {1.0f, -1.0f, 0.0f,  1.0f, 0.0f,  0.0f},
    {1.0f,  1.0f, 0.0f, -1.0f, 0.0f,  0.0f},
    {1.0f,  1.0f, 0.0f,  1.0f, 0.0f,  0.0f},
    {0.0f,  0.0f, 1.0f,  0.0f, 0.0f,  1.0f},
    {0.0f,  0.0f, 1.0f,  0.0f, 0.0f, -1.0f},
};

inline bool isThrusterPinAllowed(uint8_t pin) {
  return (pin >= 2 && pin <= 22) || pin == 28;
}

constexpr char FIRMWARE_VERSION[] = "Firmware v1.0 Autonomous";

// Master Configuration Structure (Firmware v1.0)
struct Settings {
  uint32_t magic;
  uint8_t version;

  // Hardware / Thruster configuration (T1..T6)
  uint8_t pins[THRUSTER_COUNT];
  uint16_t pulseMin[THRUSTER_COUNT];
  uint16_t pulseNeutral[THRUSTER_COUNT];
  uint16_t pulseMax[THRUSTER_COUNT];
  uint8_t reverse[THRUSTER_COUNT];
  float revBoost[THRUSTER_COUNT];
  float mix[THRUSTER_COUNT][AXIS_COUNT];
  float globalRevBoost;
  uint16_t deadbandUs;

  // Safety & Limits
  uint8_t maxPowerPercent;
  uint8_t rampPercentPerSecond;
  uint16_t failsafeMs;
  uint16_t maxDepthCm;
  uint8_t maxTiltDeg;
  uint16_t timeoutSec;
  uint8_t stallCheck;
  uint16_t stepTimeoutSec;
  uint8_t failsafeMode;

  // Wi-Fi & Web Credentials
  char wifiSsid[33];
  char wifiPassword[64];
  char webPassword[33];

  // PID Stabilization (Roll / Pitch)
  float stabilizationKp;
  float stabilizationKi;
  float stabilizationKd;
  uint8_t stabilizationMaxOutputPercent;
  uint8_t stabilizationMaxAngleDegrees;
  uint8_t stabilizationTargetRateDps;

  // PID Depth Hold
  float depthKp;
  float depthKi;
  float depthKd;
  uint8_t depthMaxOutputPercent;
  uint8_t depthDeadbandCm;
  uint16_t depthTargetCm;

  // PID Yaw / Heading Hold
  float yawKp;
  float yawKi;
  float yawKd;
  uint8_t pidLimitPercent;
  uint8_t rlEnabled;
  float rlRate;
  float rlMaxKp;

  // Autonomous Mission Parameters
  uint8_t autorun;
  uint16_t startDelaySec;
  uint16_t settleTimeSec;
  uint16_t swimSec;
  uint8_t swimPower;
  uint16_t turnAngleDeg;
  uint8_t turnDir;
  uint8_t turnPower;
  uint8_t depthTolCm;
  uint8_t yawTolDeg;
  uint8_t missionProfile;

  // Trim, Ballast & Physical Biases
  float heaveBias;
  float pitchBias;
  float trimRoll;
  float trimYaw;
  float brakeTimeSec;

  // Environment & Calibration
  float waterDensity;
  uint16_t calibTimeSec;
  uint8_t waveFilterSamples;
  float magDeclinationDeg;
  uint8_t wifiSleep;

  // ESC Audio / Buzzer Chimes
  uint8_t beepCalib;
  uint8_t beepMission;
  uint8_t beepPwrPercent;

  uint32_t checksum;
};

}  // namespace ROVConfig
