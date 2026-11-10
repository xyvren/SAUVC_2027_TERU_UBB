#include "cli_engine.h"
#include <Arduino.h>
#include <pico/mutex.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

namespace CLIEngine {

namespace {

constexpr size_t LOG_MAX_LINES = 80;
constexpr size_t LOG_LINE_CAPACITY = 112;

char logBuffer[LOG_MAX_LINES][LOG_LINE_CAPACITY];
size_t logHead = 0;
size_t logCount = 0;
mutex_t logMutex;
bool logMutexInitialized = false;

SystemContext sysCtx{};
bool contextInitialized = false;

void ensureMutexInit() {
  if (!logMutexInitialized) {
    mutex_init(&logMutex);
    logMutexInitialized = true;
  }
}

const char* levelToString(Level level) {
  switch (level) {
    case Level::INFO:  return "INFO";
    case Level::WARN:  return "WARN";
    case Level::ERROR: return "ERROR";
    case Level::STATE: return "STATE";
    case Level::CMD:   return "CMD";
    default:           return "INFO";
  }
}

const char* getMissionProfileName(uint8_t profile) {
  switch (profile) {
    case 0: return "full";
    case 1: return "gate";
    case 2: return "dive_only";
    case 3: return "turn_only";
    default: return "unknown";
  }
}

float calculateDepthCm(float pressureMbar, float zeroRefMbar, float waterDensity) {
  if (waterDensity <= 0.0f) waterDensity = 1000.0f;
  const float deltaMbar = pressureMbar - zeroRefMbar;
  // deltaP (Pa) = deltaMbar * 100
  // depth (m) = deltaP / (rho * 9.80665)
  // depth (cm) = depth (m) * 100
  return (deltaMbar * 10000.0f) / (waterDensity * 9.80665f);
}

size_t tokenize(const String& cmd, String tokens[], size_t maxTokens) {
  size_t count = 0;
  const int len = cmd.length();
  int i = 0;
  while (i < len && count < maxTokens) {
    while (i < len && (cmd.charAt(i) == ' ' || cmd.charAt(i) == '\t')) {
      i++;
    }
    if (i >= len) break;
    int start = i;
    while (i < len && cmd.charAt(i) != ' ' && cmd.charAt(i) != '\t') {
      i++;
    }
    tokens[count++] = cmd.substring(start, i);
  }
  return count;
}

bool parseBool(const String& str, bool& result) {
  String s = str;
  s.toLowerCase();
  if (s == "1" || s == "on" || s == "true" || s == "yes" || s == "enable" || s == "enabled") {
    result = true;
    return true;
  }
  if (s == "0" || s == "off" || s == "false" || s == "no" || s == "disable" || s == "disabled") {
    result = false;
    return true;
  }
  return false;
}

bool isKnownParam(const String& p) {
  return (p == "depth" || p == "swim" || p == "swim_power" || p == "turn_angle" ||
          p == "turn_dir" || p == "turn_power" || p == "start_delay" || p == "settle_time" ||
          p == "depth_tol" || p == "yaw_tol" || p == "mission" || p == "autorun" ||
          p == "power" || p == "ramp" || p == "max_depth" || p == "max_tilt" ||
          p == "timeout" || p == "stall_check" || p == "step_timeout" || p == "failsafe" ||
          p == "heave_bias" || p == "pitch_bias" || p == "trim_roll" || p == "trim_yaw" ||
          p == "brake_time" || p == "rev_boost" || p == "deadband" || p == "water_density" ||
          p == "calib_time" || p == "wave_filter" || p == "mag_decl" || p == "wifi_sleep" ||
          p == "beep_calib" || p == "beep_mission" || p == "beep_pwr" || p == "pid" ||
          p == "pid_limit" || p == "rl" || p == "rl_rate" || p == "rl_max_kp" ||
          p == "ssid" || p == "wifi_ssid" || p == "wifi_pass" || p == "wifi_password" ||
          p == "pass" || p == "password");
}

}  // namespace

void begin(const SystemContext& context) {
  ensureMutexInit();
  sysCtx = context;
  contextInitialized = true;
  log(Level::INFO, "CLI", "Unified CLI Engine & Ring-Buffer Logger initialized (Firmware v1.0)");
}

void log(Level level, const char* tag, const char* fmt, ...) {
  ensureMutexInit();

  const uint32_t ms = millis();
  const uint32_t mmm = ms % 1000;
  const uint32_t totalSec = ms / 1000;
  const uint32_t ss = totalSec % 60;
  const uint32_t mm = (totalSec / 60) % 60;
  const uint32_t hh = totalSec / 3600;

  char msgBuf[72];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msgBuf, sizeof(msgBuf), fmt, args);
  va_end(args);

  char line[LOG_LINE_CAPACITY];
  snprintf(line, sizeof(line), "[%02lu:%02lu:%02lu.%03lu] [%s] [%s] %s",
           static_cast<unsigned long>(hh),
           static_cast<unsigned long>(mm),
           static_cast<unsigned long>(ss),
           static_cast<unsigned long>(mmm),
           levelToString(level),
           tag ? tag : "SYS",
           msgBuf);
  line[sizeof(line) - 1] = '\0';

  mutex_enter_blocking(&logMutex);
  strncpy(logBuffer[logHead], line, LOG_LINE_CAPACITY - 1);
  logBuffer[logHead][LOG_LINE_CAPACITY - 1] = '\0';
  logHead = (logHead + 1) % LOG_MAX_LINES;
  if (logCount < LOG_MAX_LINES) {
    logCount++;
  }
  mutex_exit(&logMutex);

  Serial.println(line);
}

void dumpLog(String& out) {
  ensureMutexInit();
  mutex_enter_blocking(&logMutex);
  const size_t start = (logHead + LOG_MAX_LINES - logCount) % LOG_MAX_LINES;
  for (size_t i = 0; i < logCount; ++i) {
    const size_t idx = (start + i) % LOG_MAX_LINES;
    out += logBuffer[idx];
    out += "\r\n";
  }
  mutex_exit(&logMutex);
}

String dumpLog() {
  String out;
  out.reserve(logCount * 75);
  dumpLog(out);
  return out;
}

void clearLog() {
  ensureMutexInit();
  mutex_enter_blocking(&logMutex);
  logHead = 0;
  logCount = 0;
  memset(logBuffer, 0, sizeof(logBuffer));
  mutex_exit(&logMutex);
}

size_t getLogCount() {
  return logCount;
}

namespace {

String getHelpText() {
  return String(
    "=== AUV SAUVAC COMMAND HELP ===\r\n"
    "SYSTEM & MISSION:\r\n"
    "  help / ?                   - Show this command reference\r\n"
    "  status                     - Show sensors, telemetry, and system status\r\n"
    "  run                        - Trigger autonomous mission run\r\n"
    "  stop                       - Immediate emergency abort (neutralize thrusters)\r\n"
    "  autorun <on/off>           - Toggle autonomous run on boot\r\n"
    "  reboot                     - Reboot the Raspberry Pi Pico W\r\n\r\n"
    "PARAMETERS:\r\n"
    "  get all                    - Display all active configuration parameters\r\n"
    "  get <param>                - Display value of a specific parameter\r\n"
    "  set <param> <val>          - Set parameter value (see 'get all' for list)\r\n"
    "  set ssid <name>            - Set Wi-Fi Access Point SSID (1-32 chars)\r\n"
    "  set wifi_pass <password>   - Set Wi-Fi Access Point password (8-63 chars)\r\n"
    "  set pid <depth/roll/yaw> <kp> <ki> <kd> - Tune PID gains\r\n\r\n"
    "THRUSTERS:\r\n"
    "  thrusters                  - Display 6-thruster configuration table\r\n"
    "  thruster <id> <prop> <val> - Configure thruster (min/max/neutral/reverse/rev_boost)\r\n"
    "  test motor <id> <pwm/%>    - Test single motor for 2s (e.g. 'test motor 1 20%')\r\n"
    "  esc_calib <start/cancel>   - Calibrate ESC throttle range (1000-2000 us)\r\n\r\n"
    "CALIBRATION & SIMULATION:\r\n"
    "  calib zero                 - Zero depth sensor at current surface pressure\r\n"
    "  calib ballast              - Measure ballast buoyancy & heave bias\r\n"
    "  calib compass              - Calibrate magnetic compass\r\n"
    "  calib level                - Calibrate IMU zero-level offsets\r\n"
    "  sim <on/off>               - Desk simulation mode (virtual depth, 3% pwr)\r\n\r\n"
    "ADVANCED & BRIDGE:\r\n"
    "  selftest                   - Run pre-flight hardware self-test (sensors, storage, thrusters)\r\n"
    "  stream <on/off>            - High-speed 20Hz JSON telemetry stream for RPi 5 bridge\r\n"
    "  blackbox <dump/clear>      - Manage underwater LittleFS flight recorder (/blackbox.csv)\r\n\r\n"
    "CONFIGURATION & LOGS:\r\n"
    "  save                       - Save current RAM configuration to EEPROM\r\n"
    "  config export              - Export all configuration settings as JSON\r\n"
    "  config default             - Reset all settings to factory defaults\r\n"
    "  log dump                   - Dump ring-buffer log history\r\n"
    "  log clear                  - Clear ring-buffer log history\r\n"
  );
}

String getStatusReport() {
  if (!contextInitialized || !sysCtx.settings) {
    return "ERROR: SystemContext not initialized.";
  }

  ROVSensors::State sensor{};
  ROVSensors::snapshot(sensor);

  String out;
  out.reserve(750);
  out += "=== AUV SAUVAC STATUS ===\r\n";
  out += "Armed: " + String(*sysCtx.armed ? "ARMED" : "DISARMED");
  out += " | Sim Mode: " + String(*sysCtx.simMode ? "ON (Virtual)" : "OFF");
  out += " | Auto-Run: " + String(sysCtx.settings->autorun ? "ON" : "OFF") + "\r\n";
  out += "Hardware Outputs: " + String(*sysCtx.outputsReady ? "READY" : "ERROR / NOT ATTACHED") + "\r\n";
  out += "Mission Profile: " + String(getMissionProfileName(sysCtx.settings->missionProfile));
  out += " | Auto Mode: " + String(*sysCtx.autoModeEnabled ? "ACTIVE" : "STANDBY") + "\r\n";
  out += "-------------------------\r\n";
  out += "[Sensors]\r\n";
  out += "  MS5803 Depth  : ";
  if (sensor.pressureReady) {
    out += "ONLINE | Pres: " + String(sensor.pressureMbar, 1) + " mbar | Temp: " + String(sensor.temperatureC, 1) + " C\r\n";
    if (*sysCtx.depthReferenceSet) {
      const float depthCm = calculateDepthCm(sensor.pressureMbar, *sysCtx.depthReferencePressureMbar, sysCtx.settings->waterDensity);
      out += "  Depth Calc    : " + String(depthCm, 1) + " cm (Zero Ref: " + String(*sysCtx.depthReferencePressureMbar, 1) + " mbar)\r\n";
    } else {
      out += "  Depth Calc    : NOT ZEROED (Run 'calib zero' at surface)\r\n";
    }
  } else {
    out += "OFFLINE / NOT DETECTED\r\n";
  }

  out += "  MPU-6050 IMU  : ";
  if (sensor.imuReady) {
    out += "ONLINE | Pitch: " + String(sensor.attitudePitch, 1) + " deg | Roll: " + String(sensor.attitudeRoll, 1) + " deg\r\n";
    out += "  Gyro Rates    : X=" + String(sensor.gyroDps[0], 1) + " Y=" + String(sensor.gyroDps[1], 1) + " Z=" + String(sensor.gyroDps[2], 1) + " dps\r\n";
  } else {
    out += "OFFLINE / NOT DETECTED\r\n";
  }

  out += "  Compass       : ";
  if (sensor.compassReady) {
    out += "ONLINE (" + String(sensor.compassType) + ") | Heading: " + String(sensor.compassHeading, 1) + " deg\r\n";
  } else {
    out += "OFFLINE / NOT DETECTED\r\n";
  }

  out += "-------------------------\r\n";
  out += "[Power & Safety]\r\n";
  out += "  VSYS Voltage  : " + String(sensor.vsysVoltage, 2) + " V\r\n";
  if (sysCtx.posX && sysCtx.posY && sysCtx.posZ) {
    out += "  Odometry (XYZ): X=" + String(*sysCtx.posX, 2) + " m | Y=" + String(*sysCtx.posY, 2) + " m | Z=" + String(*sysCtx.posZ, 2) + " m\r\n";
  }
  out += "  Max Power     : " + String(sysCtx.settings->maxPowerPercent) + "% | Ramp: " + String(sysCtx.settings->rampPercentPerSecond) + "%/s\r\n";
  out += "  Safety Limits : Max Depth: " + String(sysCtx.settings->maxDepthCm) + " cm | Max Tilt: " + String(sysCtx.settings->maxTiltDeg) + " deg\r\n";
  out += "  Timeouts      : Mission: " + String(sysCtx.settings->timeoutSec) + " s | Step: " + String(sysCtx.settings->stepTimeoutSec) + " s\r\n";
  out += "=========================";
  return out;
}

String getConfigurationSummary() {
  if (!contextInitialized || !sysCtx.settings) {
    return "ERROR: SystemContext not initialized.";
  }

  const ROVConfig::Settings* s = sysCtx.settings;
  String out;
  out.reserve(1300);
  out += "=== AUV SAUVAC ACTIVE CONFIGURATION (Firmware v1.0) ===\r\n";
  out += "[Mission Parameters]\r\n";
  out += "  depth          : " + String(s->depthTargetCm) + " cm\r\n";
  out += "  swim           : " + String(s->swimSec) + " s\r\n";
  out += "  swim_power     : " + String(s->swimPower) + " %\r\n";
  out += "  turn_angle     : " + String(s->turnAngleDeg) + " deg\r\n";
  out += "  turn_dir       : " + String(s->turnDir == 0 ? "CW" : "CCW") + "\r\n";
  out += "  turn_power     : " + String(s->turnPower) + " %\r\n";
  out += "  start_delay    : " + String(s->startDelaySec) + " s\r\n";
  out += "  settle_time    : " + String(s->settleTimeSec) + " s\r\n";
  out += "  depth_tol      : " + String(s->depthTolCm) + " cm\r\n";
  out += "  yaw_tol        : " + String(s->yawTolDeg) + " deg\r\n";
  out += "  mission        : " + String(getMissionProfileName(s->missionProfile)) + "\r\n";
  out += "  autorun        : " + String(s->autorun ? "ON" : "OFF") + "\r\n";
  out += "[Safety & Limits]\r\n";
  out += "  power          : " + String(s->maxPowerPercent) + " %\r\n";
  out += "  ramp           : " + String(s->rampPercentPerSecond) + " %/s\r\n";
  out += "  max_depth      : " + String(s->maxDepthCm) + " cm\r\n";
  out += "  max_tilt       : " + String(s->maxTiltDeg) + " deg\r\n";
  out += "  timeout        : " + String(s->timeoutSec) + " s\r\n";
  out += "  stall_check    : " + String(s->stallCheck ? "ON" : "OFF") + "\r\n";
  out += "  step_timeout   : " + String(s->stepTimeoutSec) + " s\r\n";
  out += "  failsafe       : " + String(s->failsafeMode == 0 ? "float" : "surface") + "\r\n";
  out += "[Trim & Ballast]\r\n";
  out += "  heave_bias     : " + String(s->heaveBias, 2) + " %\r\n";
  out += "  pitch_bias     : " + String(s->pitchBias, 2) + " %\r\n";
  out += "  trim_roll      : " + String(s->trimRoll, 2) + " %\r\n";
  out += "  trim_yaw       : " + String(s->trimYaw, 2) + " %\r\n";
  out += "  brake_time     : " + String(s->brakeTimeSec, 2) + " s\r\n";
  out += "[PID Controller]\r\n";
  out += "  pid depth      : Kp=" + String(s->depthKp, 4) + " Ki=" + String(s->depthKi, 4) + " Kd=" + String(s->depthKd, 4) + "\r\n";
  out += "  pid roll       : Kp=" + String(s->stabilizationKp, 4) + " Ki=" + String(s->stabilizationKi, 4) + " Kd=" + String(s->stabilizationKd, 4) + "\r\n";
  out += "  pid yaw        : Kp=" + String(s->yawKp, 4) + " Ki=" + String(s->yawKi, 4) + " Kd=" + String(s->yawKd, 4) + "\r\n";
  out += "  pid_limit      : " + String(s->pidLimitPercent) + " %\r\n";
  out += "  rl             : " + String(s->rlEnabled ? "ON" : "OFF") + "\r\n";
  out += "  rl_rate        : " + String(s->rlRate, 4) + "\r\n";
  out += "  rl_max_kp      : " + String(s->rlMaxKp, 4) + "\r\n";
  out += "[Thruster Global]\r\n";
  out += "  rev_boost      : " + String(s->globalRevBoost, 2) + "\r\n";
  out += "  deadband       : " + String(s->deadbandUs) + " us\r\n";
  out += "[Environment & Audio]\r\n";
  out += "  water_density  : " + String(s->waterDensity, 1) + " kg/m3\r\n";
  out += "  calib_time     : " + String(s->calibTimeSec) + " s\r\n";
  out += "  wave_filter    : " + String(s->waveFilterSamples) + " samples\r\n";
  out += "  mag_decl       : " + String(s->magDeclinationDeg, 2) + " deg\r\n";
  out += "  wifi_sleep     : " + String(s->wifiSleep ? "ON" : "OFF") + "\r\n";
  out += "  beep_calib     : " + String(s->beepCalib) + "\r\n";
  out += "  beep_mission   : " + String(s->beepMission) + "\r\n";
  out += "  beep_pwr       : " + String(s->beepPwrPercent) + " %\r\n";
  out += "NETWORK:\r\n";
  out += "  ssid = " + String(s->wifiSsid) + " | wifi_pass = " + String(s->wifiPassword) + "\r\n";
  out += "===========================================";
  return out;
}

String getThrusterTable() {
  if (!contextInitialized || !sysCtx.settings) {
    return "ERROR: SystemContext not initialized.";
  }

  const ROVConfig::Settings* s = sysCtx.settings;
  String out;
  out.reserve(550);
  out += "=== THRUSTER CONFIGURATION ===\r\n";
  out += "ID | Pin  | Min (us) | Neutral (us) | Max (us) | Reverse | Rev Boost\r\n";
  out += "---+------+----------+--------------+----------+---------+----------\r\n";
  for (size_t i = 0; i < ROVConfig::THRUSTER_COUNT; ++i) {
    char row[80];
    snprintf(row, sizeof(row), " %u | GP%-2u |     %4u |         %4u |     %4u |      %s |      %4.2f\r\n",
             static_cast<unsigned>(i + 1),
             static_cast<unsigned>(s->pins[i]),
             static_cast<unsigned>(s->pulseMin[i]),
             static_cast<unsigned>(s->pulseNeutral[i]),
             static_cast<unsigned>(s->pulseMax[i]),
             s->reverse[i] ? "YES" : " NO",
             s->revBoost[i]);
    out += row;
  }
  out += "Global Rev Boost: " + String(s->globalRevBoost, 2) +
         " | Deadband: " + String(s->deadbandUs) + " us\r\n";
  return out;
}

String exportConfigJson() {
  if (!contextInitialized || !sysCtx.settings) {
    return "ERROR: SystemContext not initialized.";
  }

  const ROVConfig::Settings* s = sysCtx.settings;
  String json = "{\r\n";
  json += "  \"version\": " + String(s->version) + ",\r\n";
  json += "  \"autorun\": " + String(s->autorun ? "true" : "false") + ",\r\n";
  json += "  \"power\": " + String(s->maxPowerPercent) + ",\r\n";
  json += "  \"ramp\": " + String(s->rampPercentPerSecond) + ",\r\n";
  json += "  \"depthTarget\": " + String(s->depthTargetCm) + ",\r\n";
  json += "  \"swimSec\": " + String(s->swimSec) + ",\r\n";
  json += "  \"swimPower\": " + String(s->swimPower) + ",\r\n";
  json += "  \"turnAngle\": " + String(s->turnAngleDeg) + ",\r\n";
  json += "  \"turnDir\": \"" + String(s->turnDir == 0 ? "cw" : "ccw") + "\",\r\n";
  json += "  \"turnPower\": " + String(s->turnPower) + ",\r\n";
  json += "  \"startDelay\": " + String(s->startDelaySec) + ",\r\n";
  json += "  \"settleTime\": " + String(s->settleTimeSec) + ",\r\n";
  json += "  \"depthTol\": " + String(s->depthTolCm) + ",\r\n";
  json += "  \"yawTol\": " + String(s->yawTolDeg) + ",\r\n";
  json += "  \"missionProfile\": " + String(s->missionProfile) + ",\r\n";
  json += "  \"maxDepth\": " + String(s->maxDepthCm) + ",\r\n";
  json += "  \"maxTilt\": " + String(s->maxTiltDeg) + ",\r\n";
  json += "  \"timeout\": " + String(s->timeoutSec) + ",\r\n";
  json += "  \"stepTimeout\": " + String(s->stepTimeoutSec) + ",\r\n";
  json += "  \"stallCheck\": " + String(s->stallCheck ? "true" : "false") + ",\r\n";
  json += "  \"failsafeMode\": " + String(s->failsafeMode) + ",\r\n";
  json += "  \"globalRevBoost\": " + String(s->globalRevBoost, 2) + ",\r\n";
  json += "  \"deadbandUs\": " + String(s->deadbandUs) + ",\r\n";
  json += "  \"waterDensity\": " + String(s->waterDensity, 1) + ",\r\n";
  json += "  \"heaveBias\": " + String(s->heaveBias, 2) + ",\r\n";
  json += "  \"pitchBias\": " + String(s->pitchBias, 2) + ",\r\n";
  json += "  \"trimRoll\": " + String(s->trimRoll, 2) + ",\r\n";
  json += "  \"trimYaw\": " + String(s->trimYaw, 2) + ",\r\n";
  json += "  \"brakeTime\": " + String(s->brakeTimeSec, 2) + ",\r\n";
  json += "  \"depthKp\": " + String(s->depthKp, 4) + ",\r\n";
  json += "  \"depthKi\": " + String(s->depthKi, 4) + ",\r\n";
  json += "  \"depthKd\": " + String(s->depthKd, 4) + ",\r\n";
  json += "  \"stabilizationKp\": " + String(s->stabilizationKp, 4) + ",\r\n";
  json += "  \"stabilizationKi\": " + String(s->stabilizationKi, 4) + ",\r\n";
  json += "  \"stabilizationKd\": " + String(s->stabilizationKd, 4) + ",\r\n";
  json += "  \"yawKp\": " + String(s->yawKp, 4) + ",\r\n";
  json += "  \"yawKi\": " + String(s->yawKi, 4) + ",\r\n";
  json += "  \"yawKd\": " + String(s->yawKd, 4) + ",\r\n";
  json += "  \"pidLimit\": " + String(s->pidLimitPercent) + "\r\n";
  json += "}";
  return json;
}

String handleGetCommand(const String& param) {
  if (!contextInitialized || !sysCtx.settings) {
    return "ERROR: SystemContext not initialized.";
  }

  const ROVConfig::Settings* s = sysCtx.settings;
  String p = param;
  p.toLowerCase();

  if (p.isEmpty() || p == "all") {
    return getConfigurationSummary();
  }
  if (p == "ssid" || p == "wifi_ssid") return "ssid = " + String(s->wifiSsid);
  if (p == "wifi_pass" || p == "wifi_password" || p == "pass" || p == "password") return "wifi_pass = " + String(s->wifiPassword);
  if (p == "depth") return "depth = " + String(s->depthTargetCm) + " cm";
  if (p == "swim") return "swim = " + String(s->swimSec) + " s";
  if (p == "swim_power") return "swim_power = " + String(s->swimPower) + " %";
  if (p == "turn_angle") return "turn_angle = " + String(s->turnAngleDeg) + " deg";
  if (p == "turn_dir") return "turn_dir = " + String(s->turnDir == 0 ? "cw" : "ccw");
  if (p == "turn_power") return "turn_power = " + String(s->turnPower) + " %";
  if (p == "start_delay") return "start_delay = " + String(s->startDelaySec) + " s";
  if (p == "settle_time") return "settle_time = " + String(s->settleTimeSec) + " s";
  if (p == "depth_tol") return "depth_tol = " + String(s->depthTolCm) + " cm";
  if (p == "yaw_tol") return "yaw_tol = " + String(s->yawTolDeg) + " deg";
  if (p == "mission") return "mission = " + String(getMissionProfileName(s->missionProfile));
  if (p == "autorun") return "autorun = " + String(s->autorun ? "ON" : "OFF");
  if (p == "power") return "power = " + String(s->maxPowerPercent) + " %";
  if (p == "ramp") return "ramp = " + String(s->rampPercentPerSecond) + " %/s";
  if (p == "max_depth") return "max_depth = " + String(s->maxDepthCm) + " cm";
  if (p == "max_tilt") return "max_tilt = " + String(s->maxTiltDeg) + " deg";
  if (p == "timeout") return "timeout = " + String(s->timeoutSec) + " s";
  if (p == "stall_check") return "stall_check = " + String(s->stallCheck ? "ON" : "OFF");
  if (p == "step_timeout") return "step_timeout = " + String(s->stepTimeoutSec) + " s";
  if (p == "failsafe") return "failsafe = " + String(s->failsafeMode == 0 ? "float" : "surface");
  if (p == "heave_bias") return "heave_bias = " + String(s->heaveBias, 2) + " %";
  if (p == "pitch_bias") return "pitch_bias = " + String(s->pitchBias, 2) + " %";
  if (p == "trim_roll") return "trim_roll = " + String(s->trimRoll, 2) + " %";
  if (p == "trim_yaw") return "trim_yaw = " + String(s->trimYaw, 2) + " %";
  if (p == "brake_time") return "brake_time = " + String(s->brakeTimeSec, 2) + " s";
  if (p == "rev_boost") return "rev_boost = " + String(s->globalRevBoost, 2);
  if (p == "deadband") return "deadband = " + String(s->deadbandUs) + " us";
  if (p == "water_density") return "water_density = " + String(s->waterDensity, 1) + " kg/m3";
  if (p == "calib_time") return "calib_time = " + String(s->calibTimeSec) + " s";
  if (p == "wave_filter") return "wave_filter = " + String(s->waveFilterSamples) + " samples";
  if (p == "mag_decl") return "mag_decl = " + String(s->magDeclinationDeg, 2) + " deg";
  if (p == "wifi_sleep") return "wifi_sleep = " + String(s->wifiSleep ? "ON" : "OFF");
  if (p == "beep_calib") return "beep_calib = " + String(s->beepCalib);
  if (p == "beep_mission") return "beep_mission = " + String(s->beepMission);
  if (p == "beep_pwr") return "beep_pwr = " + String(s->beepPwrPercent) + " %";
  if (p == "pid") {
    return "pid depth: Kp=" + String(s->depthKp, 4) + " Ki=" + String(s->depthKi, 4) + " Kd=" + String(s->depthKd, 4) + "\r\n" +
           "pid roll : Kp=" + String(s->stabilizationKp, 4) + " Ki=" + String(s->stabilizationKi, 4) + " Kd=" + String(s->stabilizationKd, 4) + "\r\n" +
           "pid yaw  : Kp=" + String(s->yawKp, 4) + " Ki=" + String(s->yawKi, 4) + " Kd=" + String(s->yawKd, 4);
  }
  if (p == "pid_limit") return "pid_limit = " + String(s->pidLimitPercent) + " %";
  if (p == "rl") return "rl = " + String(s->rlEnabled ? "ON" : "OFF");
  if (p == "rl_rate") return "rl_rate = " + String(s->rlRate, 4);
  if (p == "rl_max_kp") return "rl_max_kp = " + String(s->rlMaxKp, 4);

  return "ERROR: Unknown parameter '" + param + "'. Type 'get all' to see all available parameters.";
}

String handleSetCommand(const String tokens[], size_t tokenCount) {
  if (!contextInitialized || !sysCtx.settings) {
    return "ERROR: SystemContext not initialized.";
  }

  if (tokenCount < 3) {
    return "ERROR: Usage: set <param> <value> or set pid <depth/roll/yaw> <kp> <ki> <kd>";
  }

  ROVConfig::Settings* s = sysCtx.settings;
  String param = tokens[1];
  param.toLowerCase();
  String valStr = tokens[2];

  // Special case: set pid <subsystem> <kp> <ki> <kd>
  if (param == "pid") {
    if (tokenCount < 6) {
      return "ERROR: Usage: set pid <depth/roll/yaw> <kp> <ki> <kd>";
    }
    String sub = tokens[2];
    sub.toLowerCase();
    const float kp = tokens[3].toFloat();
    const float ki = tokens[4].toFloat();
    const float kd = tokens[5].toFloat();
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f || kp > 2.0f || ki > 2.0f || kd > 2.0f) {
      return "ERROR: PID values must be positive and within reasonable limits (0.0 to 2.0).";
    }

    if (sub == "depth") {
      s->depthKp = kp;
      s->depthKi = ki;
      s->depthKd = kd;
      log(Level::INFO, "CLI", "PID depth gains set to Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
      return "OK: PID depth updated to Kp=" + String(kp, 4) + " Ki=" + String(ki, 4) + " Kd=" + String(kd, 4) + " (type 'save' to commit)";
    } else if (sub == "roll") {
      s->stabilizationKp = kp;
      s->stabilizationKi = ki;
      s->stabilizationKd = kd;
      log(Level::INFO, "CLI", "PID roll gains set to Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
      return "OK: PID roll updated to Kp=" + String(kp, 4) + " Ki=" + String(ki, 4) + " Kd=" + String(kd, 4) + " (type 'save' to commit)";
    } else if (sub == "yaw") {
      s->yawKp = kp;
      s->yawKi = ki;
      s->yawKd = kd;
      log(Level::INFO, "CLI", "PID yaw gains set to Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
      return "OK: PID yaw updated to Kp=" + String(kp, 4) + " Ki=" + String(ki, 4) + " Kd=" + String(kd, 4) + " (type 'save' to commit)";
    } else {
      return "ERROR: Unknown PID subsystem '" + tokens[2] + "'. Must be depth, roll, or yaw.";
    }
  }

  if (param == "ssid" || param == "wifi_ssid") {
    String val = tokens[2];
    for (size_t i = 3; i < tokenCount; ++i) {
      val += " " + tokens[i];
    }
    if (val.length() < 1 || val.length() > 32) {
      return "ERROR: SSID must be between 1 and 32 characters.";
    }
    memset(s->wifiSsid, 0, sizeof(s->wifiSsid));
    strncpy(s->wifiSsid, val.c_str(), sizeof(s->wifiSsid) - 1);
    log(Level::INFO, "CLI", "Wi-Fi SSID set to '%s'", s->wifiSsid);
    return "OK: Wi-Fi SSID set to '" + String(s->wifiSsid) + "'. Type 'save' and then 'reboot' to apply.";
  }

  if (param == "wifi_pass" || param == "wifi_password" || param == "pass" || param == "password") {
    String val = tokens[2];
    for (size_t i = 3; i < tokenCount; ++i) {
      val += " " + tokens[i];
    }
    if (val.length() < 8 || val.length() > 63) {
      return "ERROR: Wi-Fi password must be between 8 and 63 characters (WPA2 requirement).";
    }
    memset(s->wifiPassword, 0, sizeof(s->wifiPassword));
    strncpy(s->wifiPassword, val.c_str(), sizeof(s->wifiPassword) - 1);
    log(Level::INFO, "CLI", "Wi-Fi password updated to '%s'", s->wifiPassword);
    return "OK: Wi-Fi password updated to '" + String(s->wifiPassword) + "'. Type 'save' and then 'reboot' to apply.";
  }

  if (param == "depth") {
    long val = valStr.toInt();
    if (val < 0 || val > 500) return "ERROR: depth must be between 0 and 500 cm.";
    s->depthTargetCm = static_cast<uint16_t>(val);
  } else if (param == "swim") {
    long val = valStr.toInt();
    if (val < 1 || val > 120) return "ERROR: swim must be between 1 and 120 seconds.";
    s->swimSec = static_cast<uint16_t>(val);
  } else if (param == "swim_power") {
    long val = valStr.toInt();
    if (val < 5 || val > 100) return "ERROR: swim_power must be between 5 and 100%.";
    s->swimPower = static_cast<uint8_t>(val);
  } else if (param == "turn_angle") {
    long val = valStr.toInt();
    if (val < 0 || val > 360) return "ERROR: turn_angle must be between 0 and 360 degrees.";
    s->turnAngleDeg = static_cast<uint16_t>(val);
  } else if (param == "turn_dir") {
    String d = valStr;
    d.toLowerCase();
    if (d == "cw" || d == "0") s->turnDir = 0;
    else if (d == "ccw" || d == "1") s->turnDir = 1;
    else return "ERROR: turn_dir must be 'cw' or 'ccw'.";
  } else if (param == "turn_power") {
    long val = valStr.toInt();
    if (val < 5 || val > 100) return "ERROR: turn_power must be between 5 and 100%.";
    s->turnPower = static_cast<uint8_t>(val);
  } else if (param == "start_delay") {
    long val = valStr.toInt();
    if (val < 0 || val > 30) return "ERROR: start_delay must be between 0 and 30 seconds.";
    s->startDelaySec = static_cast<uint16_t>(val);
  } else if (param == "settle_time") {
    long val = valStr.toInt();
    if (val < 0 || val > 30) return "ERROR: settle_time must be between 0 and 30 seconds.";
    s->settleTimeSec = static_cast<uint16_t>(val);
  } else if (param == "depth_tol") {
    long val = valStr.toInt();
    if (val < 1 || val > 20) return "ERROR: depth_tol must be between 1 and 20 cm.";
    s->depthTolCm = static_cast<uint8_t>(val);
  } else if (param == "yaw_tol") {
    long val = valStr.toInt();
    if (val < 1 || val > 30) return "ERROR: yaw_tol must be between 1 and 30 degrees.";
    s->yawTolDeg = static_cast<uint8_t>(val);
  } else if (param == "mission") {
    String m = valStr;
    m.toLowerCase();
    if (m == "full" || m == "0") s->missionProfile = 0;
    else if (m == "gate" || m == "1") s->missionProfile = 1;
    else if (m == "dive_only" || m == "dive" || m == "2") s->missionProfile = 2;
    else if (m == "turn_only" || m == "turn" || m == "3") s->missionProfile = 3;
    else return "ERROR: mission profile must be full, gate, dive_only, or turn_only.";
  } else if (param == "autorun") {
    bool b = false;
    if (!parseBool(valStr, b)) return "ERROR: autorun must be on/off or 1/0.";
    s->autorun = b ? 1 : 0;
  } else if (param == "power") {
    long val = valStr.toInt();
    if (val < 5 || val > 100) return "ERROR: power must be between 5 and 100%.";
    s->maxPowerPercent = static_cast<uint8_t>(val);
  } else if (param == "ramp") {
    long val = valStr.toInt();
    if (val < 5 || val > 100) return "ERROR: ramp must be between 5 and 100%/s.";
    s->rampPercentPerSecond = static_cast<uint8_t>(val);
  } else if (param == "max_depth") {
    long val = valStr.toInt();
    if (val < 10 || val > 500) return "ERROR: max_depth must be between 10 and 500 cm.";
    s->maxDepthCm = static_cast<uint16_t>(val);
  } else if (param == "max_tilt") {
    long val = valStr.toInt();
    if (val < 10 || val > 85) return "ERROR: max_tilt must be between 10 and 85 degrees.";
    s->maxTiltDeg = static_cast<uint8_t>(val);
  } else if (param == "timeout") {
    long val = valStr.toInt();
    if (val < 5 || val > 300) return "ERROR: timeout must be between 5 and 300 seconds.";
    s->timeoutSec = static_cast<uint16_t>(val);
  } else if (param == "stall_check") {
    bool b = false;
    if (!parseBool(valStr, b)) return "ERROR: stall_check must be on/off or 1/0.";
    s->stallCheck = b ? 1 : 0;
  } else if (param == "step_timeout") {
    long val = valStr.toInt();
    if (val < 2 || val > 60) return "ERROR: step_timeout must be between 2 and 60 seconds.";
    s->stepTimeoutSec = static_cast<uint16_t>(val);
  } else if (param == "failsafe") {
    String f = valStr;
    f.toLowerCase();
    if (f == "float" || f == "0") s->failsafeMode = 0;
    else if (f == "surface" || f == "1") s->failsafeMode = 1;
    else return "ERROR: failsafe must be 'float' or 'surface'.";
  } else if (param == "heave_bias") {
    float val = valStr.toFloat();
    if (val < -50.0f || val > 50.0f) return "ERROR: heave_bias must be between -50% and +50%.";
    s->heaveBias = val;
  } else if (param == "pitch_bias") {
    float val = valStr.toFloat();
    if (val < -50.0f || val > 50.0f) return "ERROR: pitch_bias must be between -50% and +50%.";
    s->pitchBias = val;
  } else if (param == "trim_roll") {
    float val = valStr.toFloat();
    if (val < -50.0f || val > 50.0f) return "ERROR: trim_roll must be between -50% and +50%.";
    s->trimRoll = val;
  } else if (param == "trim_yaw") {
    float val = valStr.toFloat();
    if (val < -50.0f || val > 50.0f) return "ERROR: trim_yaw must be between -50% and +50%.";
    s->trimYaw = val;
  } else if (param == "brake_time") {
    float val = valStr.toFloat();
    if (val < 0.0f || val > 5.0f) return "ERROR: brake_time must be between 0.0 and 5.0 seconds.";
    s->brakeTimeSec = val;
  } else if (param == "rev_boost") {
    float val = valStr.toFloat();
    if (val < 1.0f || val > 2.5f) return "ERROR: rev_boost must be between 1.00 and 2.50.";
    s->globalRevBoost = val;
  } else if (param == "deadband") {
    long val = valStr.toInt();
    if (val < 0 || val > 100) return "ERROR: deadband must be between 0 and 100 us.";
    s->deadbandUs = static_cast<uint16_t>(val);
  } else if (param == "water_density") {
    float val = valStr.toFloat();
    if (val < 900.0f || val > 1100.0f) return "ERROR: water_density must be between 900.0 and 1100.0 kg/m3.";
    s->waterDensity = val;
  } else if (param == "calib_time") {
    long val = valStr.toInt();
    if (val < 3 || val > 60) return "ERROR: calib_time must be between 3 and 60 seconds.";
    s->calibTimeSec = static_cast<uint16_t>(val);
  } else if (param == "wave_filter") {
    long val = valStr.toInt();
    if (val < 1 || val > 20) return "ERROR: wave_filter must be between 1 and 20 samples.";
    s->waveFilterSamples = static_cast<uint8_t>(val);
  } else if (param == "mag_decl") {
    float val = valStr.toFloat();
    if (val < -180.0f || val > 180.0f) return "ERROR: mag_decl must be between -180.0 and +180.0 degrees.";
    s->magDeclinationDeg = val;
  } else if (param == "wifi_sleep") {
    bool b = false;
    if (!parseBool(valStr, b)) return "ERROR: wifi_sleep must be on/off or 1/0.";
    s->wifiSleep = b ? 1 : 0;
  } else if (param == "beep_calib") {
    long val = valStr.toInt();
    if (val != 0 && val != 1) return "ERROR: beep_calib must be 0 or 1.";
    s->beepCalib = static_cast<uint8_t>(val);
  } else if (param == "beep_mission") {
    long val = valStr.toInt();
    if (val != 0 && val != 1) return "ERROR: beep_mission must be 0 or 1.";
    s->beepMission = static_cast<uint8_t>(val);
  } else if (param == "beep_pwr") {
    long val = valStr.toInt();
    if (val < 1 || val > 50) return "ERROR: beep_pwr must be between 1 and 50%.";
    s->beepPwrPercent = static_cast<uint8_t>(val);
  } else if (param == "pid_limit") {
    long val = valStr.toInt();
    if (val < 5 || val > 100) return "ERROR: pid_limit must be between 5 and 100%.";
    s->pidLimitPercent = static_cast<uint8_t>(val);
  } else if (param == "rl") {
    bool b = false;
    if (!parseBool(valStr, b)) return "ERROR: rl must be on/off or 1/0.";
    s->rlEnabled = b ? 1 : 0;
  } else if (param == "rl_rate") {
    float val = valStr.toFloat();
    if (val < 0.0001f || val > 0.1f) return "ERROR: rl_rate must be between 0.0001 and 0.1.";
    s->rlRate = val;
  } else if (param == "rl_max_kp") {
    float val = valStr.toFloat();
    if (val < 0.01f || val > 2.0f) return "ERROR: rl_max_kp must be between 0.01 and 2.0.";
    s->rlMaxKp = val;
  } else {
    return "ERROR: Unknown parameter '" + tokens[1] + "'. Type 'get all' to see all available parameters.";
  }

  log(Level::INFO, "CLI", "Parameter '%s' set to '%s'", param.c_str(), valStr.c_str());
  return "OK: " + param + " set to " + valStr + " (type 'save' to commit to EEPROM)";
}

String handleThrusterCommand(const String tokens[], size_t tokenCount) {
  if (!contextInitialized || !sysCtx.settings) {
    return "ERROR: SystemContext not initialized.";
  }

  if (tokenCount < 4) {
    return "ERROR: Usage: thruster <id 1-6> <min/max/neutral/reverse/rev_boost> <val>";
  }

  const int id = tokens[1].toInt();
  if (id < 1 || id > static_cast<int>(ROVConfig::THRUSTER_COUNT)) {
    return "ERROR: Thruster ID must be between 1 and 6.";
  }

  const size_t idx = id - 1;
  String prop = tokens[2];
  prop.toLowerCase();
  const String valStr = tokens[3];
  ROVConfig::Settings* s = sysCtx.settings;

  if (prop == "min") {
    long val = valStr.toInt();
    if (val < 800 || val > 1400) return "ERROR: min pulse must be between 800 and 1400 us.";
    s->pulseMin[idx] = static_cast<uint16_t>(val);
  } else if (prop == "max") {
    long val = valStr.toInt();
    if (val < 1600 || val > 2200) return "ERROR: max pulse must be between 1600 and 2200 us.";
    s->pulseMax[idx] = static_cast<uint16_t>(val);
  } else if (prop == "neutral") {
    long val = valStr.toInt();
    if (val < 1400 || val > 1600) return "ERROR: neutral pulse must be between 1400 and 1600 us.";
    s->pulseNeutral[idx] = static_cast<uint16_t>(val);
  } else if (prop == "reverse") {
    bool b = false;
    if (!parseBool(valStr, b)) return "ERROR: reverse must be 0/1 or on/off.";
    s->reverse[idx] = b ? 1 : 0;
  } else if (prop == "rev_boost") {
    float val = valStr.toFloat();
    if (val < 1.0f || val > 2.5f) return "ERROR: rev_boost must be between 1.00 and 2.50.";
    s->revBoost[idx] = val;
  } else {
    return "ERROR: Unknown property '" + tokens[2] + "'. Must be min, max, neutral, reverse, or rev_boost.";
  }

  log(Level::INFO, "CLI", "Thruster T%d %s set to %s", id, prop.c_str(), valStr.c_str());
  return "OK: Thruster T" + String(id) + " " + prop + " set to " + valStr + " (type 'save' to commit to EEPROM)";
}

}  // namespace

String executeCommand(const String& cmdLine, bool isWeb) {
  String cmd = cmdLine;
  cmd.trim();
  if (cmd.startsWith("/")) {
    cmd = cmd.substring(1);
    cmd.trim();
  }
  if (cmd.isEmpty()) {
    return "";
  }

  // Ring-buffer & Serial broadcast of executed command
  log(Level::CMD, "CLI", "%s%s", isWeb ? "[WEB] " : "[SER] ", cmd.c_str());

  String tokens[10];
  const size_t tokenCount = tokenize(cmd, tokens, 10);
  if (tokenCount == 0) return "";

  String root = tokens[0];
  root.toLowerCase();

  // Smart Auto-Run cancellation: Only explicit control/stop/override commands cancel autorun countdown.
  // Passive informational commands (status, help, ?, get, thrusters, log, blackbox) do NOT cancel autorun.
  const bool isControlCmd = (root == "stop" || root == "disarm" || root == "cancel" ||
                             root == "test" || root == "set" || root == "run" ||
                             root == "thruster" || root == "calib" || root == "esc_calib" ||
                             root == "sim" || root == "reboot" || root == "selftest" ||
                             (root == "autorun" && tokenCount > 1) ||
                             (isKnownParam(root) && tokenCount > 1));
  if (isControlCmd && sysCtx.cancelAutorun) {
    sysCtx.cancelAutorun();
  }

  // 1. HELP / ?
  if (root == "help" || root == "?") {
    return getHelpText();
  }

  // 2. STATUS
  if (root == "status") {
    return getStatusReport();
  }

  // 3. RUN (Mission start)
  if (root == "run") {
    if (!contextInitialized || !sysCtx.startMission) {
      return "ERROR: Mission control interface not initialized.";
    }
    if (sysCtx.outputsReady && !(*sysCtx.outputsReady)) {
      return "ERROR: Thruster hardware outputs not ready or failed to attach.";
    }
    if (sysCtx.startMission()) {
      return "OK: Autonomous mission started.";
    } else {
      return "ERROR: Failed to start autonomous mission (check sensors/safety).";
    }
  }

  // 4. STOP / DISARM / CANCEL (Emergency stop)
  if (root == "stop" || root == "disarm" || root == "cancel") {
    if (sysCtx.stopAllThrusters) {
      sysCtx.stopAllThrusters();
    }
    log(Level::STATE, "SAFE", "Emergency stop triggered via CLI. Thrusters neutralized.");
    return "OK: Emergency stop executed. All thrusters neutralized and disarmed.";
  }

  // 5. AUTORUN <on/off>
  if (root == "autorun") {
    if (tokenCount < 2) {
      return "autorun = " + String(sysCtx.settings && sysCtx.settings->autorun ? "ON" : "OFF");
    }
    bool b = false;
    if (!parseBool(tokens[1], b)) {
      return "ERROR: Usage: autorun <on/off>";
    }
    if (sysCtx.settings) {
      sysCtx.settings->autorun = b ? 1 : 0;
      log(Level::INFO, "CLI", "autorun set to %s", b ? "ON" : "OFF");
      return "OK: autorun is now " + String(b ? "ON" : "OFF") + " (type 'save' to commit to EEPROM)";
    }
    return "ERROR: SystemContext not initialized.";
  }

  // 6. SET <param> <val>
  if (root == "set") {
    return handleSetCommand(tokens, tokenCount);
  }

  // 7. GET <param> / GET ALL
  if (root == "get") {
    return handleGetCommand(tokenCount > 1 ? tokens[1] : "all");
  }

  // 8. THRUSTERS (Table)
  if (root == "thrusters") {
    return getThrusterTable();
  }

  // 9. THRUSTER <id> <prop> <val>
  if (root == "thruster") {
    return handleThrusterCommand(tokens, tokenCount);
  }

  // 10. CALIB <zero/ballast/compass/level>
  if (root == "calib") {
    if (tokenCount < 2) {
      return "ERROR: Usage: calib <zero/ballast/compass/level>";
    }
    if (!sysCtx.calibrate) {
      return "ERROR: Calibration callback not available.";
    }
    String response;
    String type = tokens[1];
    type.toLowerCase();
    sysCtx.calibrate(type, response);
    return response;
  }

  // 11. ESC_CALIB <start/cancel>
  if (root == "esc_calib") {
    if (tokenCount < 2) {
      return "ERROR: Usage: esc_calib <start/cancel>";
    }
    if (!sysCtx.escCalib) {
      return "ERROR: ESC calibration callback not available.";
    }
    String response;
    String action = tokens[1];
    action.toLowerCase();
    sysCtx.escCalib(action, response);
    return response;
  }

  // 12. TEST MOTOR <id> <pwm/percent>
  if (root == "test") {
    if (tokenCount < 4 || tokens[1].equalsIgnoreCase("motor") == false) {
      return "ERROR: Usage: test motor <id 1-6> <pwm or percent>";
    }
    const int id = tokens[2].toInt();
    if (id < 1 || id > static_cast<int>(ROVConfig::THRUSTER_COUNT)) {
      return "ERROR: Thruster ID must be 1 to 6.";
    }
    String valStr = tokens[3];
    int16_t pulseOrPercent = 0;
    if (valStr.endsWith("%")) {
      pulseOrPercent = valStr.substring(0, valStr.length() - 1).toInt();
    } else {
      pulseOrPercent = valStr.toInt();
    }

    if (!sysCtx.startMotorTest) {
      return "ERROR: Motor test callback not available.";
    }
    String errorMsg;
    if (sysCtx.startMotorTest(static_cast<uint8_t>(id), pulseOrPercent, errorMsg)) {
      return "OK: Motor T" + String(id) + " running test for 2.0s.";
    } else {
      return "ERROR: " + errorMsg;
    }
  }

  // 13. SIM <on/off>
  if (root == "sim") {
    if (tokenCount < 2) {
      return "Simulation mode: " + String(sysCtx.simMode && *sysCtx.simMode ? "ON" : "OFF");
    }
    bool b = false;
    if (!parseBool(tokens[1], b)) {
      return "ERROR: Usage: sim <on/off>";
    }
    if (sysCtx.setSimMode) {
      sysCtx.setSimMode(b);
      return "OK: Simulation mode is now " + String(b ? "ON" : "OFF");
    } else if (sysCtx.simMode) {
      *sysCtx.simMode = b;
      log(Level::STATE, "SIM", "Simulation mode %s (desk testing with 3%% power limit)", b ? "ENABLED" : "DISABLED");
      return "OK: Simulation mode is now " + String(b ? "ON" : "OFF");
    }
    return "ERROR: SystemContext not initialized.";
  }

  // 14. SAVE
  if (root == "save") {
    if (!sysCtx.persistSettings) {
      return "ERROR: EEPROM persist function not available.";
    }
    if (sysCtx.persistSettings()) {
      log(Level::INFO, "CFG", "Configuration successfully saved to EEPROM (Firmware v1.0)");
      return "OK: Configuration successfully saved to EEPROM.";
    } else {
      log(Level::ERROR, "CFG", "Failed to save configuration to EEPROM");
      return "ERROR: Failed to save configuration to EEPROM.";
    }
  }

  // 15. CONFIG EXPORT / DEFAULT
  if (root == "config") {
    if (tokenCount < 2) {
      return "ERROR: Usage: config <export/default>";
    }
    String sub = tokens[1];
    sub.toLowerCase();
    if (sub == "export") {
      return exportConfigJson();
    } else if (sub == "default") {
      if (sysCtx.setDefaults) {
        sysCtx.setDefaults();
        log(Level::WARN, "CFG", "Settings reset to factory defaults in RAM");
        return "OK: Settings reset to factory defaults in RAM. Type 'save' to commit to EEPROM.";
      }
      return "ERROR: setDefaults not available.";
    } else {
      return "ERROR: Unknown config action '" + tokens[1] + "'. Must be 'export' or 'default'.";
    }
  }

  // 16. LOG <dump/clear>
  if (root == "log") {
    if (tokenCount < 2) {
      return "ERROR: Usage: log <dump/clear>";
    }
    String sub = tokens[1];
    sub.toLowerCase();
    if (sub == "dump") {
      String out;
      dumpLog(out);
      if (out.isEmpty()) return "(Log buffer is empty)";
      return out;
    } else if (sub == "clear") {
      clearLog();
      return "OK: Log ring-buffer cleared.";
    } else {
      return "ERROR: Unknown log action '" + tokens[1] + "'. Must be 'dump' or 'clear'.";
    }
  }

  // 17. REBOOT
  if (root == "reboot") {
    log(Level::STATE, "SYS", "System reboot initiated via CLI");
    if (sysCtx.reboot) {
      sysCtx.reboot();
    }
    return "OK: Rebooting Raspberry Pi Pico W...";
  }

  // 18. SELFTEST (Pre-Flight Hardware & Thruster Self-Test)
  if (root == "selftest") {
    if (!sysCtx.selfTest) {
      return "ERROR: Pre-flight self-test callback not available.";
    }
    String report;
    sysCtx.selfTest(report);
    return report;
  }

  // 19. STREAM <on/off> (RPi 5 Companion Stream Bridge)
  if (root == "stream") {
    if (!sysCtx.rpiStreamEnabled) {
      return "ERROR: RPi 5 companion stream bridge not available.";
    }
    if (tokenCount < 2) {
      return "Companion Stream Bridge (RPi 5 20Hz JSON): " + String(*sysCtx.rpiStreamEnabled ? "ON" : "OFF");
    }
    bool b = false;
    if (!parseBool(tokens[1], b)) {
      return "ERROR: Usage: stream <on/off>";
    }
    *sysCtx.rpiStreamEnabled = b;
    log(Level::STATE, "RPI", "RPi 5 Companion stream bridge %s (USB Serial 20Hz JSON)", b ? "ENABLED" : "DISABLED");
    return "OK: Companion stream bridge is now " + String(b ? "ON" : "OFF");
  }

  // 20. BLACKBOX <dump/clear> (Underwater LittleFS Flight Recorder)
  if (root == "blackbox") {
    if (!sysCtx.blackboxControl) {
      return "ERROR: Blackbox flight recorder not available.";
    }
    if (tokenCount < 2) {
      String status;
      sysCtx.blackboxControl("status", status);
      return status;
    }
    String sub = tokens[1];
    sub.toLowerCase();
    String resp;
    sysCtx.blackboxControl(sub, resp);
    return resp;
  }

  // 21. DIRECT PARAMETER SHORTCUTS (e.g. 'depth 50', 'depth', 'pid depth 0.8 0.05 0.2')
  if (isKnownParam(root)) {
    if (tokenCount == 1) {
      return handleGetCommand(root);
    }
    String synthTokens[10];
    synthTokens[0] = "set";
    for (size_t i = 0; i < tokenCount && (i + 1) < 10; ++i) {
      synthTokens[i + 1] = tokens[i];
    }
    return handleSetCommand(synthTokens, tokenCount + 1);
  }

  return "ERROR: Unknown command '" + tokens[0] + "'. Type 'help' or '?' for available commands.";
}

}  // namespace CLIEngine
