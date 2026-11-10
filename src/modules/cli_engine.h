#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>
#include "rov_config.h"
#include "rov_sensors.h"

namespace CLIEngine {

enum class Level {
  INFO,
  WARN,
  ERROR,
  STATE,
  CMD
};

struct SystemContext {
  ROVConfig::Settings* settings;
  bool* armed;
  bool* outputsReady;
  bool* autoModeEnabled;
  bool* stabilizationEnabled;
  bool* depthHoldEnabled;
  bool* simMode;
  bool* depthReferenceSet;
  float* depthReferencePressureMbar;
  bool* rpiStreamEnabled;
  float* posX;
  float* posY;
  float* posZ;

  // System Action Callbacks
  void (*stopAllThrusters)();
  bool (*persistSettings)();
  void (*setDefaults)();
  bool (*startMission)();
  bool (*startMotorTest)(uint8_t id, int16_t pulseOrPercent, String& errorMsg);
  bool (*calibrate)(const String& type, String& response);
  bool (*escCalib)(const String& action, String& response);
  void (*reboot)();
  void (*cancelAutorun)();
  void (*setSimMode)(bool enabled);
  bool (*selfTest)(String& response);
  bool (*blackboxControl)(const String& action, String& response);
};

void begin(const SystemContext& context);
void log(Level level, const char* tag, const char* fmt, ...);
String executeCommand(const String& cmdLine, bool isWeb = false);
void dumpLog(String& out);
String dumpLog();
void clearLog();
size_t getLogCount();

}  // namespace CLIEngine
