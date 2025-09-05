#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <Wire.h>
#include <OneWire.h>
#include <algorithm>
#include "esp_task_wdt.h"
#include "SHTManager_RS485.h"
#include "TimeClient.h"
#include "S3Log.h"
#include "OTAManager.h"
#include "esp_ota_ops.h"
#include "TimeBudgetManager.h"
#include "RTClib.h"
#include <ArduinoJson.h>
#include "ExperimentManager.h"
#include "ScheduleManager.h"
#include "SystemTypes.h"

// ===========================
// Wifi credentials
// ===========================
const char* WIFI_SSID = "ThermoTera";
//const char* WIFI_SSID = "BokishHome";
const char* WIFI_PASSWORD = "Thermo2007";
//const char* WIFI_PASSWORD = "ShalomShalom";

// Timezone offset for GMT+3
const long gmtOffset_sec = 3 * 3600; // 3 hours in seconds
const int daylightOffset_sec = 0;    // No daylight saving time adjustment

const String SITE_NAME = "Series1";
const String BUILDING_NAME = "Mavnad2.0";
const String CONTROLLER_TYPE = "Mavnad2.0.Flat";
const String CONTROLLER_LOCATION = "mavnad";
const float FLOAT_NAN = -127;
const String CURRENT_FIRMWARE_VERSION = "1.0.2.170";
const String TOKEN = "pm8z4oxs7awwcx68gwov"; // ein shemer
//const String TOKEN = "8sqfmy0fdvacex3ef0mo"; // asaf

// OTA Health Check Constants
const unsigned int REQUIRED_HEALTH_CHECKS = 15;  // 15 successful loops required
const unsigned long MAX_VALIDATION_TIME = 5 * 60 * 1000;  // 5 minutes timeout
const unsigned long HEARTBEAT_INTERVAL = 30 * 1000;  // 30 seconds heartbeat

// Setup ThingsBoard client
WiFiClient wiFiClient;
PubSubClient mqttClient(wiFiClient);
OTAManager otaManager(mqttClient, CURRENT_FIRMWARE_VERSION, TOKEN);

const int PIN_FAN_RIGHT1 = 21;
const int PIN_FAN_RIGHT2 = 47;
const int PIN_FAN_RIGHT3 = 45;
const int PIN_FAN_RIGHT4 = 35;
const int PIN_FAN_LEFT1 = 37;
const int PIN_FAN_LEFT2 = 38;
const int PIN_FAN_LEFT3 = 39;
const int PIN_FAN_LEFT4 = 40;

const int FAN_CHANNEL = 0;
const int FAN_FREQUENCY = 20000; // PWM frequency in Hz
const int FAN_RESOLUTION = 8;  // 8-bit resolution (0-255)

const int PIN_PUMP_DRIPPERS = 13;
const int PIN_DAMPER = 14; // 12;  <============================= Changed to unused pin for safety
const int PIN_DAMPER_POWER = 11;
const int PIN_PUMP_SPRINKLERS = 10; 

const int PUMP_PWM_CHANNEL = 1;
const int PUMP_PWM_FREQ = 1000;     // 1 kHz (safe value within 100–2000 Hz)
const int PUMP_PWM_RESOLUTION = 8;  // 8-bit (0–255)

const int PIN_FAN_INNER1 = 8;
const int PIN_FAN_INNER_ADC = 18;
//const int PIN_FAN_INNER2 = 7;
const int FAN_INNER_CHANNEL = 2;
const int FAN_INNER_FREQUENCY = 20000; // PWM frequency in Hz
const int FAN_INNER_RESOLUTION = 8;  // 8-bit resolution (0-255)

const int fanPins[] = { 
  PIN_FAN_RIGHT1, 
  PIN_FAN_RIGHT2, 
  PIN_FAN_RIGHT3, 
  PIN_FAN_RIGHT4,
  PIN_FAN_LEFT1, 
  PIN_FAN_LEFT2, 
  PIN_FAN_LEFT3, 
  PIN_FAN_LEFT4
};

float START_COOLING_DEG = 25.2;
float START_HEATING_DEG = 18.0;

// Define the relay pins for the air valves
const int airValvesRelayPins[] = { PIN_DAMPER };

HardwareSerial RS485Serial(2); // UART2
SHTManager_RS485 shtRS485Manager(RS485Serial); // For RS485 sensors
S3Log* dataLog;
TimeClient* timeClient;
bool isDataSent = false;

// OTA Health Check Variables
unsigned long firmwareStartTime = 0;
unsigned int healthCheckCounter = 0;
bool firmwareValidated = false;
unsigned long lastHeartbeatTime = 0;
bool isFirstBootAfterOTA = false;
unsigned long lastLoopTime = 0;
const unsigned long MAX_LOOP_TIME = 10 * 1000;  // 10 seconds max loop time

enum class DamperState {
  Idle,
  Opening,
  Closing
};


std::vector<ScheduleManager::ScheduleEntry> modeSchedule = {
  // startHour, min, endHour, min, mode, startFan%, endFan%, innerFan%, drippersBudgetSeconds
  // Sunday
  { 0,  0,  0,  6,  0, SystemMode::Regenerate, 35, 25, 20, 30 },      // 00:00-06:00: Fan 35-25%, Inner 30%, Drippers 0s
  { 0,  6,  0,  6, 20, SystemMode::Regenerate, 25, 25, 0, 30 },      // 06:00-06:20: Fan 25%, Inner 30%, Drippers 0s
  { 0,  6, 20,  7, 30, SystemMode::Stop, 0, 0, 0, 0 },               // 06:20-07:30: Stop mode
  { 0,  7, 30, 16,  0, SystemMode::Cool, 20, 40, 30, 25 },           // 07:30-16:00: Fan 20-40%, Inner 20%, Drippers 25s
  { 0, 16,  0, 20,  0, SystemMode::Cool, 40, 20, 30, 25 },           // 16:00-20:00: Fan 40-20%, Inner 20%, Drippers 25s
  { 0, 20,  0, 21,  0, SystemMode::Regenerate, 50, 50, 20, 30 },      // 20:00-21:00: Fan 50%, Inner 30%, Drippers 0s
  { 0, 21,  0, 23, 59, SystemMode::Regenerate, 50, 35, 20, 30 },     // 21:00-23:59: Fan 50-35%, Inner 30%, Drippers 0s
  // Monday 
  { 1,  0,  0,  6,  0, SystemMode::Regenerate, 35, 25, 20, 30 },      // 00:00-06:00: Fan 35-25%, Inner 30%, Drippers 0s
  { 1,  6,  0,  6, 20, SystemMode::Regenerate, 25, 25, 0, 30 },      // 06:00-06:20: Fan 25%, Inner 30%, Drippers 0s
  { 1,  6, 20,  7, 30, SystemMode::Stop, 0, 0, 0, 0 },               // 06:20-07:30: Stop mode
  { 1,  7, 30, 16,  0, SystemMode::Cool, 20, 40, 30, 25 },           // 07:30-16:00: Fan 20-40%, Inner 20%, Drippers 25s
  { 1, 16,  0, 20,  0, SystemMode::Cool, 40, 20, 30, 25 },           // 16:00-20:00: Fan 40-20%, Inner 20%, Drippers 25s
  { 1, 20,  0, 21,  0, SystemMode::Regenerate, 50, 50, 20, 30 },      // 20:00-21:00: Fan 50%, Inner 30%, Drippers 0s
  { 1, 21,  0, 23, 59, SystemMode::Regenerate, 50, 35, 20, 30 },     // 21:00-23:59: Fan 50-35%, Inner 30%, Drippers 0s
  // Tuesday 
  { 2,  0,  0,  6,  0, SystemMode::Regenerate, 35, 25, 20, 30 },      // 00:00-06:00: Fan 35-25%, Inner 30%, Drippers 0s
  { 2,  6,  0,  6, 20, SystemMode::Regenerate, 25, 25, 0, 30 },      // 06:00-06:20: Fan 25%, Inner 30%, Drippers 0s
  { 2,  6, 20,  7, 30, SystemMode::Stop, 0, 0, 0, 0 },               // 06:20-07:30: Stop mode
  { 2,  7, 30, 16,  0, SystemMode::Cool, 20, 40, 30, 25 },           // 07:30-16:00: Fan 20-40%, Inner 20%, Drippers 25s
  { 2, 16,  0, 20,  0, SystemMode::Cool, 40, 20, 30, 25 },           // 16:00-20:00: Fan 40-20%, Inner 20%, Drippers 25s
  { 2, 20,  0, 21,  0, SystemMode::Regenerate, 50, 50, 20, 30 },      // 20:00-21:00: Fan 50%, Inner 30%, Drippers 0s
  { 2, 21,  0, 23, 59, SystemMode::Regenerate, 50, 35, 20, 30 },     // 21:00-23:59: Fan 50-35%, Inner 30%, Drippers 0s
  // Wednesday
  { 3,  0,  0,  6,  0, SystemMode::Regenerate, 35, 25, 20, 30 },      // 00:00-06:00: Fan 35-25%, Inner 30%, Drippers 0s
  { 3,  6,  0,  6, 20, SystemMode::Regenerate, 25, 25, 0, 30 },      // 06:00-06:20: Fan 25%, Inner 30%, Drippers 0s
  { 3,  6, 20,  7, 30, SystemMode::Stop, 0, 0, 0, 0 },               // 06:20-07:30: Stop mode
  { 3,  7, 30, 16,  0, SystemMode::Cool, 20, 40, 30, 25 },           // 07:30-16:00: Fan 20-40%, Inner 20%, Drippers 25s
  { 3, 16,  0, 20,  0, SystemMode::Cool, 40, 20, 30, 25 },           // 16:00-20:00: Fan 40-20%, Inner 20%, Drippers 25s
  { 3, 20,  0, 21,  0, SystemMode::Regenerate, 50, 50, 20, 30 },      // 20:00-21:00: Fan 50%, Inner 30%, Drippers 0s
  { 3, 21,  0, 23, 59, SystemMode::Regenerate, 50, 35, 20, 30 },     // 21:00-23:59: Fan 50-35%, Inner 30%, Drippers 0s
  // Thursday
  { 4,  0,  0,  6,  0, SystemMode::Regenerate, 35, 25, 20, 30 },      // 00:00-06:00: Fan 35-25%, Inner 30%, Drippers 0s
  { 4,  6,  0,  6, 20, SystemMode::Regenerate, 25, 25, 0, 30 },      // 06:00-06:20: Fan 25%, Inner 30%, Drippers 0s
  { 4,  6, 20,  7, 30, SystemMode::Stop, 0, 0, 0, 0 },               // 06:20-07:30: Stop mode
  { 4,  7, 30, 16,  0, SystemMode::Cool, 20, 40, 30, 25 },           // 07:30-16:00: Fan 20-40%, Inner 20%, Drippers 25s
  { 4, 16,  0, 20,  0, SystemMode::Cool, 40, 20, 30, 25 },           // 16:00-20:00: Fan 40-20%, Inner 20%, Drippers 25s
  { 4, 20,  0, 21,  0, SystemMode::Regenerate, 50, 50, 20, 30 },      // 20:00-21:00: Fan 50%, Inner 30%, Drippers 0s
  { 4, 21,  0, 23, 59, SystemMode::Regenerate, 50, 35, 20, 30 },     // 21:00-23:59: Fan 50-35%, Inner 30%, Drippers 0s
  // Friday
  { 5,  0,  0,  6,  0, SystemMode::Regenerate, 35, 25, 20, 30 },      // 00:00-06:00: Fan 35-25%, Inner 30%, Drippers 0s
  { 5,  6,  0,  6, 20, SystemMode::Regenerate, 25, 25, 0, 30 },      // 06:00-06:20: Fan 25%, Inner 30%, Drippers 0s
  { 5,  6, 20,  7, 30, SystemMode::Stop, 0, 0, 0, 0 },               // 06:20-07:30: Stop mode
  { 5,  7, 30, 16,  0, SystemMode::Cool, 20, 40, 30, 25 },           // 07:30-16:00: Fan 20-40%, Inner 20%, Drippers 25s
  { 5, 16,  0, 20,  0, SystemMode::Cool, 40, 20, 30, 25 },           // 16:00-20:00: Fan 40-20%, Inner 20%, Drippers 25s
  { 5, 20,  0, 21,  0, SystemMode::Regenerate, 50, 50, 20, 30 },      // 20:00-21:00: Fan 50%, Inner 30%, Drippers 0s
  { 5, 21,  0, 23, 59, SystemMode::Regenerate, 50, 35, 20, 30 },     // 21:00-23:59: Fan 50-35%, Inner 30%, Drippers 0s
  // Saturday
  { 6,  0,  0,  6,  0, SystemMode::Regenerate, 35, 25, 20, 30 },      // 00:00-06:00: Fan 35-25%, Inner 30%, Drippers 0s
  { 6,  6,  0,  6, 20, SystemMode::Regenerate, 25, 25, 0, 30 },      // 06:00-06:20: Fan 25%, Inner 30%, Drippers 0s
  { 6,  6, 20,  7, 30, SystemMode::Stop, 0, 0, 0, 0 },               // 06:20-07:30: Stop mode
  { 6,  7, 30, 16,  0, SystemMode::Cool, 20, 40, 30, 25 },           // 07:30-16:00: Fan 20-40%, Inner 20%, Drippers 25s
  { 6, 16,  0, 20,  0, SystemMode::Cool, 40, 20, 30, 25 },           // 16:00-20:00: Fan 40-20%, Inner 20%, Drippers 25s
  { 6, 20,  0, 21,  0, SystemMode::Regenerate, 50, 50, 20, 30 },      // 20:00-21:00: Fan 50%, Inner 30%, Drippers 0s
  { 6, 21,  0, 23, 59, SystemMode::Regenerate, 50, 35, 20, 30 }      // 21:00-23:59: Fan 50-35%, Inner 30%, Drippers 0s
};

// Global ScheduleManager instance
ScheduleManager* scheduleManager = nullptr;

struct SystemModeHelper {
  static String toString(SystemMode mode) {
    switch (mode) {
      case SystemMode::Cool:      return "Cool";
      case SystemMode::Heat:      return "Heat";
      case SystemMode::Regenerate: return "Regenerate";
      case SystemMode::Stop:      return "Stop";
      case SystemMode::Manual:      return "Manual";
      case SystemMode::Experiment:  return "Experiment";
      default:                       return "Unknown";
    }
  }

  static SystemMode fromString(const String& str) {
    if (str.equalsIgnoreCase("Cool"))      return SystemMode::Cool;
    if (str.equalsIgnoreCase("Heat"))      return SystemMode::Heat;
    if (str.equalsIgnoreCase("Regenerate")) return SystemMode::Regenerate;
    if (str.equalsIgnoreCase("Stop"))      return SystemMode::Stop;
    if (str.equalsIgnoreCase("Experiment")) return SystemMode::Experiment;

    Serial.printf("[SystemModeHelper] Unknown mode string: %s\n", str.c_str());
    return SystemMode::Stop;  // fallback
  }
};

// Global variable to hold the current system status
DamperState damperState = DamperState::Idle;
SystemMode currentSystemMode = SystemMode::Stop;
AirValveMode currentAirMode = AirValveMode::Close;
AirValveMode desiredDamperState = AirValveMode::Close;
WateringMode currentWaterMode = WateringMode::Off;
WateringMode currentDrippersMode = WateringMode::Off;
WateringMode desiredSprinklersMode = WateringMode::Off;
WateringMode currentSprinklersMode = WateringMode::Off;
WateringMode manualWaterMode = WateringMode::Off;
int currentPWMSpeed = 0; // in percentage (0 - 100)
int currentInnerPWMSpeed = 0; // in percentage (0 - 100)
float lastAfterHumidity = NAN; // Last valid humidity reading
bool debugMode = false;
unsigned long damperActionStartTime = 0;
const unsigned long DAMPER_ACTION_DURATION_MS = 15 * 1000;
RTC_DS3231 rtc;

// Forward declarations
void offDrippers();
void offSprinklers();
void updateRegenerationBudget();
bool isTopOfHour();
bool isSystemHealthy();
void onFans(int percentage);
void onInnerFans(int speedPercentage);
void offFans();
void setWater(WateringMode mode);
void setDampers(AirValveMode mode);
void restartDevice();
void onHeat();
void setSystemMode(AirValveMode airMode, int fanPercentage, WateringMode wateringMode, int innerFanSpeed, int drippersBudgetSeconds);
void updateSystemMode();

// Define TimeBudgetManager instances for watering and sprinklers
TimeBudgetManager wateringBudget(1L * 60 * 1000, 0.25L * 60 * 1000, offDrippers);
TimeBudgetManager sprinklersBudget(10L * 60 * 1000, 0.166L * 60 * 1000, offSprinklers);

// Define a struct for relay configuration
struct Relay {
  int pin;            // Pin number
  bool isHighTriggered;  // True if the relay is high-triggered, false if low-triggered
  const char* name;   // Name or description of the relay
};

// ==============================================================================
// ==============================================================================

void debugMessage(String message) {
  if(debugMode) Serial.println(message);
}

float FloatValidityCheck(float value) {
  if(isnan(value)) return FLOAT_NAN;
  else return value;
}

/**
 * @brief Write a log entry to S3 file for long-term local logging.
 */
void logToS3(const String& sensorName, const String& sensorType, const String& unit, float value) {
  if (!dataLog) return;

  dataLog->appendToLogFormatted(
    timeClient->getFormattedTime(),
    SITE_NAME,
    BUILDING_NAME,
    CONTROLLER_TYPE,
    CONTROLLER_LOCATION,
    sensorName,
    sensorType,
    unit,
    value
  );
}

void logToS3(const String& sensorName, const String& sensorType, const String& sensorLocation, const String& unit, float value) {
  if (!dataLog) return;

  dataLog->appendToLogFormatted(
    timeClient->getFormattedTime(),
    SITE_NAME,
    BUILDING_NAME,
    CONTROLLER_TYPE,
    sensorLocation,
    sensorName,
    sensorType,
    unit,
    value
  );
}

int getSystemStatusCode() {
  // System state value has 4 digits:
  // First digit is the system mode: 1 = cooling; 2 = heating; 3 = regenerating; 4 = manual; 5 = experiment; 0 = off
  // Second digit is fans speed: 1 = low; 2 = medium; 3 = high speed; 0 = fans are closed
  // Third digit is the water status: 1 = open; 0 = close
  // Fourth digit is the dampers status: 1 = open; 0 = close
  // The sign (negative or positive number) is: > = outtake; < 0 = intake (reverse)
  int status = 0; // first digit - operating mode
  if(currentSystemMode == SystemMode::Cool) status = 1;
  else if(currentSystemMode == SystemMode::Heat) status = 2;
  else if(currentSystemMode == SystemMode::Regenerate) status = 3;
  else if(currentSystemMode == SystemMode::Manual) status = 4;
  else if(currentSystemMode == SystemMode::Experiment) status = 5;

  status *= 10; // second digit - speed (convert precenteg to a number between 0 to 3)
  status += map(currentPWMSpeed, 0, 100, 0, 3);

  status *= 10; // third digit - water status
  if(currentWaterMode == WateringMode::On) status += 1;

  status *= 10; // forth digit - dampers status
  if(currentAirMode == AirValveMode::Open) status += 1;

  // in this system there is no way to turn fans outtake (reverse), so no need to check - value always possitive

  return status;
}

void logMessage(const String& msg) {
  Serial.println(msg);
  otaManager.sendTelemetry("log", msg);
}

void logMode() {
  logToS3("System", "system", "cooler", "mode", getSystemStatusCode()); // "mode" must be in location "cooler"

  otaManager.sendTelemetry("System_Status_Code", String(getSystemStatusCode()));
  otaManager.sendTelemetry("System_Mode", SystemModeHelper::toString(currentSystemMode));
  otaManager.sendTelemetry("Fans_Speed", currentPWMSpeed);
  otaManager.sendTelemetry("Water_Mode", currentWaterMode == WateringMode::On ? "Open" : "Close");
  otaManager.sendTelemetry("Dampers_Status", currentAirMode == AirValveMode::Open ? "Open" : "Close");
  otaManager.sendTelemetry("Drippers_Budget", (int)(wateringBudget.getBudgetDurationMs() / 1000));
  otaManager.sendTelemetry("Sprinklers_Budget", (int)(sprinklersBudget.getBudgetDurationMs() / 1000));
}

void onFans(int percentage) {
  if(currentPWMSpeed == percentage) return;

  String message = "[Fans] current pwm = " + (String)currentPWMSpeed + "; new = " + (String)percentage + "\n";
  logMessage(message);
  int duty = map(percentage, 0, 100, 0, 255);
  ledcWrite(FAN_CHANNEL, duty);
  currentPWMSpeed = percentage;
  logMode();
}

void offFans() {
  debugMessage("off fans");
  if(currentPWMSpeed == 0) return;
  logMessage("[Fans] off");
  ledcWrite(FAN_CHANNEL, 0);
  currentPWMSpeed = 0;
  logMode();
}

void onDampers() {
  debugMessage("on dampers");
  return; // disable dampers for safety
  if(currentAirMode == AirValveMode::Open) return;
  desiredDamperState = AirValveMode::Open; // set the desired state to open
  if (damperState  != DamperState::Idle) return; // Already busy

  logMessage("[Dampers] open start");

  digitalWrite(PIN_DAMPER, HIGH);
  delay(100);
  digitalWrite(PIN_DAMPER_POWER, HIGH);

  damperActionStartTime = millis();
  damperState = DamperState::Opening;
}

void offDampers(bool forceClose = false) {
  debugMessage("off dampers");
  if(!forceClose && currentAirMode == AirValveMode::Close) return;
  desiredDamperState = AirValveMode::Close; // set the desired state to close
  if (damperState  != DamperState::Idle) return; // busy

  logMessage("[Dampers] close start");

  digitalWrite(PIN_DAMPER, LOW);
  delay(100);
  digitalWrite(PIN_DAMPER_POWER, HIGH);
  damperActionStartTime = millis();
  damperState  = DamperState::Closing;
}

void tickDampers() {
  if (damperState == DamperState::Idle) return;

  if (millis() - damperActionStartTime >= DAMPER_ACTION_DURATION_MS) {
    digitalWrite(PIN_DAMPER_POWER, LOW);
    delay(10);

    if (damperState == DamperState::Opening) {
      digitalWrite(PIN_DAMPER, LOW);
      delay(10);
      currentAirMode = AirValveMode::Open;
      logMessage("[Dampers] open complete");
    } else if (damperState == DamperState::Closing) {
      currentAirMode = AirValveMode::Close;
      logMessage("[Dampers] close complete");
    }

    damperState = DamperState::Idle;
    logMode();

    // the action has finished, check if the desired state is different from the actual state
    if(desiredDamperState != currentAirMode) {
      logMessage("[Dampers] desired state is different from the actual state");
      if(desiredDamperState == AirValveMode::Open) {
        onDampers();
      } else {
        offDampers();
      }
    }
  }
}

void offSprinklers() {
  debugMessage("off sprinklers");
  if(currentSprinklersMode != WateringMode::Off) 
    logMessage("[Sprinklers] off"); // printing only once, when changing state from on to off
  else return; // already off
  
  digitalWrite(PIN_PUMP_SPRINKLERS, LOW);
  
  currentSprinklersMode = WateringMode::Off;
  logMode();
}

void onSprinklers() {
  debugMessage("on sprinklers");
  if (!sprinklersBudget.isAllowed()) {
    offSprinklers();
    return;
  }

  if(currentSprinklersMode == WateringMode::On) return;
  logMessage("[Sprinklers] on");
  
  digitalWrite(PIN_PUMP_SPRINKLERS, HIGH);
  
  currentSprinklersMode = WateringMode::On;
  logMode();
}

void setDrippersPumpSpeed(int percent) {
  if (percent < 20) {
    percent = 0;  // stop
  } else if (percent > 90) {
    percent = 100;  // full speed
  }
  int duty = map(percent, 0, 100, 0, 255);
  ledcWrite(PUMP_PWM_CHANNEL, duty);
}

void offDrippers() {
  if(currentDrippersMode == WateringMode::Off) return;
  logMessage("[Drippers] off");

  //digitalWrite(PIN_PUMP_DRIPPERS, LOW);
  setDrippersPumpSpeed(0);

  currentDrippersMode = WateringMode::Off;
  logMode();
}

void onDrippers(int pumpPowerPercentage = 30) {
  debugMessage("on drippers");
  if (!wateringBudget.isAllowed()) {
    offDrippers();
    return;
  }
  if(currentDrippersMode == WateringMode::On) return;

  logMessage("[Drippers] on");

  setDrippersPumpSpeed(pumpPowerPercentage);

  currentDrippersMode = WateringMode::On;
  logMode();
}

void off() {
  debugMessage("off");
  if(currentSystemMode == SystemMode::Stop) return;

  offSprinklers();
  offDrippers();
  offFans();
  offDampers();

  currentWaterMode = WateringMode::Off;
  currentSystemMode = SystemMode::Stop;
}

void setDampers(AirValveMode mode) {
  debugMessage("set dampers");
  switch (mode) {
    case AirValveMode::Close:
      offDampers();
      break;
    case AirValveMode::Open:
      logMessage("[Dampers] set to open");
      onDampers();
      break;
    default:
      offDampers();
      break;
  }
}

void setWater(WateringMode mode) {
  debugMessage("set water");
  switch(mode) {
    case WateringMode::Off:
      offDrippers();
      break;
    case WateringMode::On:
      onDrippers();
      break;
    default:
      offDrippers();
      break;
  }
}

AirValveMode getAirModeByRoom() {
  debugMessage("get air mode by room");
  AirValveMode airMode = currentAirMode;
  float roomRH = shtRS485Manager.getRoomRH();
  if(isnan(roomRH)) {
    // Use fallback when humidity sensor is not available
    logMessage("[getAirModeByRoom] Room RH not available, using fallback value: 50%");
    roomRH = 50.0; // Use 50% as fallback - safe middle ground
  }

  if(currentAirMode == AirValveMode::Open && roomRH > 70) airMode = AirValveMode::Close;
  if(currentAirMode == AirValveMode::Close && roomRH <= 60) airMode = AirValveMode::Open;
  return airMode;
}

void setSystemMode(AirValveMode airMode, int fanPercentage, WateringMode wateringMode, int innerFanSpeed, int drippersBudgetSeconds) {
  debugMessage("set system mode");
  currentWaterMode = wateringMode;
  setWater(wateringMode);
  onFans(fanPercentage);
  setDampers(airMode);

  // Handle inner fan speed if specified (>= 0)
  if (innerFanSpeed >= 0) {
    onInnerFans(innerFanSpeed);
  }
  
  // Set water budget for drippers
  wateringBudget.setBudgetDurationMs(drippersBudgetSeconds * 1000);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Helper function to find current regeneration schedule entry and calculate fan speed
int getCurrentRegenerationFanSpeed() {
  if (!scheduleManager) {
    logMessage("[getCurrentRegenerationFanSpeed] ScheduleManager not available, using default fan speed");
    return 60; // Default fallback speed
  }
  
  // Get current schedule entry
  const ScheduleManager::ScheduleEntry* entry = scheduleManager->getCurrentEntry();
  if (!entry) {
    logMessage("[getCurrentRegenerationFanSpeed] No schedule entry found, using default fan speed");
    return 60; // Default fallback speed
  }
  
  // Check if current entry is regeneration mode
  if (entry->mode != SystemMode::Regenerate) {
    logMessage("[getCurrentRegenerationFanSpeed] Current mode is not regeneration, using default fan speed");
    return 60; // Default fallback speed
  }
  
  // Calculate fan speed for this entry
  return scheduleManager->calculateFanSpeedForEntry(*entry);
}


// Helper function to get current schedule information for debugging
String getCurrentScheduleInfo() {
  if (!scheduleManager) {
    return "ScheduleManager not available";
  }
  
  // Get current time for display
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "Failed to get local time";
  }
  
  String info = "Current time: " + String(timeinfo.tm_hour) + ":" + 
                String(timeinfo.tm_min < 10 ? "0" : "") + String(timeinfo.tm_min) + 
                " (Day " + String(timeinfo.tm_wday) + ")\n";
  
  // Use ScheduleManager to get current schedule info
  info += scheduleManager->getCurrentScheduleInfo();
  
  return info;
}

void setWaterBudgetFromPwm(int pwm) {
  // Set the watering budget based on the fan speed
  pwm = constrain(pwm, 0, 100); // Ensure pwm is within 0-100%
  int budgetTime = map(pwm, 20, 60, 15, 35);     // seconds 0..15
  logMessage("[setWaterBudgetFromPwm] Setting watering budget to " + String(budgetTime) + " seconds");
  wateringBudget.setBudgetDurationMs((unsigned long)budgetTime * 1000UL);
}




// Helper function to check if it's the top of the hour (minute = 0)
bool isTopOfHour() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    logMessage("[Regenerate] Failed to get local time, cannot determine if it's top of hour");
    return false;
  }
  return timeinfo.tm_min == 0;
}


// Helper function to update regeneration budget based on beforeRH
void updateRegenerationBudget() {
  float beforeRH = shtRS485Manager.getBeforeRH();
  
  // Fallback value when sensor is not available
  if(isnan(beforeRH)) {
    logMessage("[Regenerate] Before RH is not available, using fallback value: 50%");
    beforeRH = 50.0; // Use 50% as fallback - safe middle ground
  }
  
  // Safety check for valid humidity values
  if (beforeRH < 0 || beforeRH > 100) {
    logMessage("[Regenerate] Invalid beforeRH value: " + String(beforeRH) + "%, using fallback value: 50%");
    beforeRH = 50.0; // Use 50% as fallback
  }
  
  unsigned long newBudgetMs = 0;
  String budgetReason = "";
  
  if (beforeRH < 90.0) {
    // Below 90%: constant 15 seconds
    newBudgetMs = 25 * 1000;
    budgetReason = "Below 90% RH - constant 25s budget";
  } else if (beforeRH <= 95.0) {
    // 90% to 95%: map from 15 seconds to 6 seconds
    // Map 90% -> 15s, 95% -> 6s
    float mappedSeconds = mapFloat(beforeRH, 90.0, 95.0, 20.0, 10.0);
    newBudgetMs = (unsigned long)(mappedSeconds * 1000);
    budgetReason = "90-95% RH - mapped budget: " + String(mappedSeconds, 1) + "s";
  } else {
    // Above 95%: stop drippers (0 budget)
    newBudgetMs = 0;
    budgetReason = "Above 95% RH - stopping drippers";
  }
  
  // Only update if the budget has changed
  if (wateringBudget.getBudgetDurationMs() != newBudgetMs) {
    wateringBudget.setBudgetDurationMs(newBudgetMs);
    
    if (newBudgetMs > 0) {
      logMessage("[Regenerate] Budget updated: " + String(newBudgetMs / 1000) + "s - " + budgetReason);
    } else {
      logMessage("[Regenerate] Drippers stopped - " + budgetReason);
    }
  }
}



void onHeat() {
  logMessage("[onHeat] on");
  Serial.println("Heat mode is not supported yet");
}

float getInnerFansSpeed() { 
  return currentInnerPWMSpeed;
}

// Helper function to check if sensors are available for cool mode
bool areCoolSensorsAvailable() {
  float roomTemp = shtRS485Manager.getRoomTemp();
  float beforeRH = shtRS485Manager.getBeforeRH();
  return !isnan(roomTemp) && !isnan(beforeRH);
}

// Helper function to check if sensors are available for regenerate mode
bool areRegenerateSensorsAvailable() {
  float beforeRH = shtRS485Manager.getBeforeRH();
  float afterRH = shtRS485Manager.getAfterRH();
  return !isnan(beforeRH) && !isnan(afterRH);
}

// Helper function to calculate inner fan speed based on room temperature
int calcInnerFansSpeedByRoomTemp(float roomTemp) {
  if(isnan(roomTemp)) {
    logMessage("[calcInnerFansSpeedByRoomTemp] Room temp not available, using fallback value: 25°C");
    roomTemp = 25.0; // Use 25°C as fallback - middle of the range
  }

  // Calculate PWM percentage based on room temperature
  int percentage;
  if(roomTemp >= 27.0) percentage = 30;
  else if(roomTemp <= 23.0) percentage = 0;
  else percentage = (int)mapFloat(roomTemp, 23.0, 27.0, 20.0, 30.0);

  // In any case, set max fans speed to 30%, for safety reasons (electrical and noise)
  percentage = constrain(percentage, 0, 30);

  logMessage("[calcInnerFansSpeedByRoomTemp] Room temp: " + String(roomTemp, 1) + "°C → Fan speed: " + String(percentage) + "%");
  return percentage;
}

// Turn on inner fans at specified speed percentage
void onInnerFans(int speedPercentage) {
  // Convert to duty cycle (0–255 for 8-bit resolution)
  speedPercentage = constrain(speedPercentage, 0, 100);
  int duty = map(speedPercentage, 0, 100, 0, 255);
  ledcWrite(FAN_INNER_CHANNEL, duty);
  currentInnerPWMSpeed = speedPercentage;
  
  logMessage("[onInnerFans] Inner fans: " + String(speedPercentage) + "% (duty " + String(duty) + ")");
}

// ========== update mode =============
SystemMode lastSelectedMode = SystemMode::Stop;
void updateSystemMode() {
  if(currentSystemMode == SystemMode::Manual) return; // Manual mode is not controlled by schedule

  // Buffer for log messages
  char logBuffer[200];

  // Step 1: Get system mode from ScheduleManager
  SystemMode newMode = SystemMode::Stop;
  if (scheduleManager) {
    newMode = scheduleManager->getCurrentSystemMode();
    logMessage("[UpdateSystemMode] ScheduleManager mode: " + SystemModeHelper::toString(newMode) + "; " + getCurrentScheduleInfo());
  } else {
    logMessage("[UpdateSystemMode] ScheduleManager not available, using default mode");
  }

  // Step 2: Check if experiment should override (highest priority)
  if (experimentManager.isExperimentTime()) {
    newMode = SystemMode::Experiment;
    logMessage("[UpdateSystemMode] Experiment active, overriding mode to Experiment");
  }

  // Step 3: Get sensor data
  float roomTemp = shtRS485Manager.getRoomTemp();
  float beforeRH = shtRS485Manager.getBeforeRH();
  float afterRH = shtRS485Manager.getAfterRH();

  // Step 4: Validate mode based on sensor data (if available)
  if (newMode == SystemMode::Cool) {
    if (!isnan(roomTemp) && roomTemp < START_COOLING_DEG) {
      newMode = SystemMode::Stop;
      logMessage("[UpdateSystemMode] Room temperature too low for cooling: " + String(roomTemp) + "°C");
    }
  } else if (newMode == SystemMode::Heat) {
    if (!isnan(roomTemp) && roomTemp > START_HEATING_DEG) {
      newMode = SystemMode::Stop;
      logMessage("[UpdateSystemMode] Room temperature too high for heating: " + String(roomTemp) + "°C");
    }
  }

  // Step 5: Log mode change
  if (newMode != currentSystemMode && lastSelectedMode != newMode) {
    lastSelectedMode = newMode;
    logMessage("[System] Mode changed to " + SystemModeHelper::toString(newMode));
  }

  // Step 6: Calculate system parameters and call setSystemMode
  AirValveMode airMode;
  int fanSpeed;
  WateringMode waterMode;
  int innerFanSpeed;
  int drippersBudgetSeconds;

  switch(newMode) {
    case SystemMode::Experiment: {
      // Get parameters from experiment manager
      fanSpeed = experimentManager.getCurrentFanSpeed();
      innerFanSpeed = experimentManager.getCurrentInnerFanSpeed();
      drippersBudgetSeconds = experimentManager.getCurrentWaterBudget();
      airMode = experimentManager.getCurrentDampersState() ? AirValveMode::Open : AirValveMode::Close;
      waterMode = WateringMode::On; // Always enable watering in experiments
      logMessage("[UpdateSystemMode] Experiment mode - EXPERIMENT-BASED: Fan=" + String(fanSpeed) + 
                 "%, Inner=" + String(innerFanSpeed) + "%, Air=" + String(airMode == AirValveMode::Open ? "Open" : "Close") + 
                 ", Budget=" + String(drippersBudgetSeconds) + "s");
      break;
    }
    
    case SystemMode::Cool: {
      if (areCoolSensorsAvailable()) {
        // Sensor-based control - we have room temp and before RH
        airMode = getAirModeByRoom();
        fanSpeed = (int)mapFloat(roomTemp, 25.0, 29.0, 20.0, 40.0);
        fanSpeed = constrain(fanSpeed, 20, 40);
        waterMode = (beforeRH < 95) ? WateringMode::On : WateringMode::Off;
        innerFanSpeed = calcInnerFansSpeedByRoomTemp(roomTemp); // Calculate based on room temp
        drippersBudgetSeconds = map(fanSpeed, 20, 40, 10, 35);
        sprintf(logBuffer, "[UpdateSystemMode] Cool mode - SENSOR-BASED: Temp=%.1f°C, BeforeRH=%.1f%%, Fan=%d%%, Inner=%d%%", 
                roomTemp, beforeRH, fanSpeed, innerFanSpeed);
        logMessage(String(logBuffer));
      } else {
        // Schedule-based control - missing room temp or before RH
        airMode = scheduleManager->getCurrentAirMode();
        fanSpeed = scheduleManager->getCurrentFanSpeed();
        waterMode = WateringMode::On; // Default to water on for cooling
        innerFanSpeed = scheduleManager->getCurrentInnerFanSpeed();
        drippersBudgetSeconds = scheduleManager->getCurrentDrippersBudgetSeconds();
        sprintf(logBuffer, "[UpdateSystemMode] Cool mode - SCHEDULE-BASED (missing sensors): %s", 
                scheduleManager->getCurrentScheduleInfo().c_str());
        logMessage(String(logBuffer));
      }
      break;
    }
    
    case SystemMode::Regenerate: {
      if (areRegenerateSensorsAvailable()) {
        // Sensor-based control - we have before RH and after RH
        airMode = AirValveMode::Open;
        fanSpeed = getCurrentRegenerationFanSpeed();
        waterMode = WateringMode::On;
        innerFanSpeed = calcInnerFansSpeedByRoomTemp(roomTemp); // Calculate based on room temp
        updateRegenerationBudget(); // Set budget based on beforeRH
        drippersBudgetSeconds = wateringBudget.getBudgetDurationMs() / 1000;
        sprintf(logBuffer, "[UpdateSystemMode] Regenerate mode - SENSOR-BASED: BeforeRH=%.1f%%, AfterRH=%.1f%%, Fan=%d%%, Inner=%d%%", 
                beforeRH, afterRH, fanSpeed, innerFanSpeed);
        logMessage(String(logBuffer));
      } else {
        // Schedule-based control - missing before RH or after RH
        airMode = scheduleManager->getCurrentAirMode();
        fanSpeed = scheduleManager->getCurrentFanSpeed();
        waterMode = WateringMode::On;
        innerFanSpeed = scheduleManager->getCurrentInnerFanSpeed();
        drippersBudgetSeconds = scheduleManager->getCurrentDrippersBudgetSeconds();
        sprintf(logBuffer, "[UpdateSystemMode] Regenerate mode - SCHEDULE-BASED (missing sensors): %s", 
                scheduleManager->getCurrentScheduleInfo().c_str());
        logMessage(String(logBuffer));
      }
      break;
    }
    
    case SystemMode::Heat: {
      // Heat mode - use schedule parameters
      airMode = AirValveMode::Close; // Heat mode typically uses closed dampers
      fanSpeed = scheduleManager ? scheduleManager->getCurrentFanSpeed() : 50;
      waterMode = WateringMode::Off; // No water for heating
      if (!isnan(roomTemp)) {
        innerFanSpeed = calcInnerFansSpeedByRoomTemp(roomTemp); // Calculate based on room temp
        sprintf(logBuffer, "[UpdateSystemMode] Heat mode - SENSOR-BASED: Temp=%.1f°C, Inner=%d%%", 
                roomTemp, innerFanSpeed);
        logMessage(String(logBuffer));
      } else {
        innerFanSpeed = scheduleManager ? scheduleManager->getCurrentInnerFanSpeed() : 0;
        logMessage("[UpdateSystemMode] Heat mode - SCHEDULE-BASED (no room temp)");
      }
      drippersBudgetSeconds = 0;
      break;
    }
    
    case SystemMode::Stop:
    default: {
      airMode = AirValveMode::Close;
      fanSpeed = 0;
      waterMode = WateringMode::Off;
      innerFanSpeed = 0;
      drippersBudgetSeconds = 0;
      logMessage("[UpdateSystemMode] Stop mode - SYSTEM-OFF: All systems disabled");
      break;
    }
  }

  // Step 7: Handle special cases
  if (currentSystemMode != SystemMode::Stop && newMode == SystemMode::Stop) {
    lastAfterHumidity = shtRS485Manager.getAfterRH();
  }

  // Step 8: Call setSystemMode with all parameters
  if(newMode != SystemMode::Stop) {
    setSystemMode(airMode, fanSpeed, waterMode, innerFanSpeed, drippersBudgetSeconds);
  } else {
    off();
  }

  // Step 9: Update the current system mode
  currentSystemMode = newMode;
}

void tickDrippers() {
  wateringBudget.tick(currentDrippersMode == WateringMode::On); // tick the budget only if water is on

  if(currentWaterMode == WateringMode::On) {
    onDrippers();
  } else {
    offDrippers();
  }
}

void tickSprinklers() {
  sprinklersBudget.tick(currentSprinklersMode == WateringMode::On); // tick the budget only if water is on

  if(desiredSprinklersMode == WateringMode::On) {
    onSprinklers();
  }
}

unsigned long lastSystemUpdate = 0;

void tick() {
  tickDampers();
  tickDrippers();
  tickSprinklers();

  // Update the system mode every 5 minutes
  unsigned long now = millis();
  if (now - lastSystemUpdate >= 5 * 60 * 1000) { // 5 minutes in ms
    updateSystemMode();
    lastSystemUpdate = now;
  }
}

String timeToString(struct tm timeInfo) {
  char buffer[32];
  sprintf(buffer, "%02d.%02d.%d %02d:%02d:%02d", 
        timeInfo.tm_mday, 
        timeInfo.tm_mon + 1, 
        timeInfo.tm_year + 1900, 
        timeInfo.tm_hour, 
        timeInfo.tm_min,
        timeInfo.tm_sec);
  return String(buffer);
}

void PrintSensors(){
  Serial.println("--------------------------------------------");
  logMessage("Ver: " + CURRENT_FIRMWARE_VERSION);
  struct tm localTime;
  if(getLocalTime(&localTime)) Serial.println(timeToString(localTime));
  String message = "System mode code " + (String)getSystemStatusCode() +
                   "; Mode = " + SystemModeHelper::toString(currentSystemMode) +
                    "; PWM = " + (String)currentPWMSpeed +
                    "; Drippers Mode = " + (currentWaterMode == WateringMode::On ? "Open" : "Close") +
                    "; Drippers Actual = " + (currentDrippersMode == WateringMode::On ? "Open" : "Close") +
                    "; Drippers Budget = " + (String(wateringBudget.getBudgetDurationMs() / 1000)) +
                    "; Sprinklers Mode = " + (desiredSprinklersMode == WateringMode::On ? "Open" : "Close") +
                    "; Sprinklers Actual = " + (currentSprinklersMode == WateringMode::On ? "Open" : "Close") +
                    "; Dampers = " + (currentAirMode == AirValveMode::Open ? "Open" : "Close");
  logMessage(message);
  wateringBudget.printStatus("Drippers");
  sprinklersBudget.printStatus("Sprinklers");

  // RS485 SHT31 sensors
  Serial.println("RS485 SHT31 sensors:");
  float ambiantTemp = shtRS485Manager.getAmbiantTemp();
  float ambiantRH = shtRS485Manager.getAmbiantRH();
  float beforeTemp = shtRS485Manager.getBeforeTemp();
  float beforeRH = shtRS485Manager.getBeforeRH();
  float afterTemp = shtRS485Manager.getAfterTemp();
  float afterRH = shtRS485Manager.getAfterRH();
  float roomTemp = shtRS485Manager.getRoomTemp();
  float roomRH = shtRS485Manager.getRoomRH();
  
  Serial.print("Ambiant: Temp = " + (isnan(ambiantTemp) ? "N/A" : String(ambiantTemp)));
  Serial.println("; RH = " + (isnan(ambiantRH) ? "N/A" : String(ambiantRH)));
  Serial.print("Before: Temp = " + (isnan(beforeTemp) ? "N/A" : String(beforeTemp)));
  Serial.println("; RH = " + (isnan(beforeRH) ? "N/A" : String(beforeRH)));
  Serial.print("After: Temp= " + (isnan(afterTemp) ? "N/A" : String(afterTemp)));
  Serial.println("; RH = " + (isnan(afterRH) ? "N/A" : String(afterRH)));
  Serial.print("Room: Temp = " + (isnan(roomTemp) ? "N/A" : String(roomTemp)));
  Serial.println("; RH = " + (isnan(roomRH) ? "N/A" : String(roomRH)));
  
     // Show regeneration status if applicable
   if (currentSystemMode == SystemMode::Regenerate) {
     Serial.println("Regeneration Status: Active");
     Serial.println("Regeneration Budget: " + String(wateringBudget.getBudgetDurationMs() / 1000) + "s");
   }
  
  // Show OTA status if applicable
  if (isFirstBootAfterOTA && !firmwareValidated) {
    Serial.println("OTA Status: VALIDATING NEW FIRMWARE");
    Serial.printf("Health check progress: %d/%d\n", healthCheckCounter, REQUIRED_HEALTH_CHECKS);
    unsigned long elapsed = (millis() - firmwareStartTime) / 1000;
    unsigned long remaining = (MAX_VALIDATION_TIME - (millis() - firmwareStartTime)) / 1000;
    Serial.printf("Time elapsed: %ds, Time remaining: %ds\n", elapsed, remaining);
    Serial.printf("System healthy: %s\n", isSystemHealthy() ? "YES" : "NO");
    Serial.printf("Last loop time: %dms\n", lastLoopTime);
  } else if (firmwareValidated) {
    Serial.println("OTA Status: FIRMWARE VALIDATED");
  }
  
  Serial.println("--------------------------------------------");
}

void setupWiFi(unsigned long timeoutMs = 10000) {
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeoutMs) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        logMessage("[WiFi] Connected.");
    } else {
        logMessage("[WiFi] Failed to connect. Continuing in offline mode.");
        // Optionally disable WiFi to save power
        WiFi.mode(WIFI_OFF);
    }
}

// The method init NTP server and wait until the builtin 
// RTC will get the current time from the net
struct tm SetupTime() {
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
    logMessage("NTP configured");

    struct tm timeinfo;
    const int maxRetries = 10;  // Try for 10 seconds max
    int retries = 0;

    while (!getLocalTime(&timeinfo) && retries < maxRetries) {
        Serial.println("Failed to obtain time, retrying...");
        retries++;
        delay(1000);
    }

    if (retries >= maxRetries) {
        logMessage("ERROR: Unable to set time from NTP. Using default time or RTC fallback.");
        // Optional: you could set default values in timeinfo or handle it gracefully.
    } else {
        Serial.println("Time successfully set!");
    }

    return timeinfo;
}

void PrintPartitions() {
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
    const esp_partition_t* p = esp_partition_get(it);
    Serial.printf("Partition: %s, Offset: 0x%06x, Size: 0x%06x\n",
                  p->label, p->address, p->size);
    it = esp_partition_next(it);
  }
}

void sendTelemetry() {
  // ThingsBoard server
  // RS485 SHT31 sensors
  float val;
  val = shtRS485Manager.getAmbiantTemp(); if (!isnan(val)) {
    otaManager.sendTelemetry("RS485_Ambiant_Temp", val);
    logToS3("RS485_Ambiant_Temp", "SHT31", "deg_c", val);
  }
  val = shtRS485Manager.getAmbiantRH();   if (!isnan(val)) {
    otaManager.sendTelemetry("RS485_Ambiant_RH", val);
    logToS3("RS485_Ambiant_RH", "SHT31", "rh", val);
  }
  val = shtRS485Manager.getBeforeTemp();  if (!isnan(val)) {
    otaManager.sendTelemetry("RS485_Before_Temp", val);
    logToS3("RS485_Before_Temp", "SHT31", "deg_c", val);
  }
  val = shtRS485Manager.getBeforeRH();    if (!isnan(val)) {
    otaManager.sendTelemetry("RS485_Before_RH", val);
    logToS3("RS485_Before_RH", "SHT31", "rh", val);
  }
  val = shtRS485Manager.getAfterTemp();   if (!isnan(val)) {
    otaManager.sendTelemetry("RS485_After_Temp", val);
    logToS3("RS485_After_Temp", "SHT31", "deg_c", val);
  }
  val = shtRS485Manager.getAfterRH();     if (!isnan(val)) {
    otaManager.sendTelemetry("RS485_After_RH", val);
    logToS3("RS485_After_RH", "SHT31", "rh", val);
  }

  // Room temperature and humidity
  val = shtRS485Manager.getRoomTemp();    if (!isnan(val)) {
    otaManager.sendTelemetry("RS485_Room_Temp", val);
    logToS3("RS485_Room_Temp", "SHT31", "deg_c", val);
  }
  val = shtRS485Manager.getRoomRH();      if (!isnan(val)) {
    otaManager.sendTelemetry("RS485_Room_RH", val);
    logToS3("RS485_Room_RH", "SHT31", "rh", val);
  }

  // Send the system status code
  // The code is a 4 digit number:
  // First digit is the system mode: 1 = cooling; 2 = heating; 3 = regenerating; 0 = off
  // Second digit is fans speed: 1 = low; 2 = medium; 3 = high speed; 0 = fans are closed
  // Third digit is the water status: 1 = open; 0 = close
  // Fourth digit is the dampers status: 1 = open; 0 = close
  // The sign (negative or positive number) is: > = outtake; < 0 = intake (reverse)
  // Example: 1234 means: cooling mode, fans at medium speed, water is open, dampers are open, and the fans are outtake (positive number).
  // Example: -1234 means: cooling mode, fans at medium speed, water is open, dampers are open, and the fans are intake (reverse).
  // The code is sent as a string, so it can be easily parsed by the server.
  // The code is also logged to S3 for long-term storage.
  logMode();
  otaManager.sendTelemetry("Drippers_Actual", currentDrippersMode == WateringMode::On ? "Open" : "Close");
  otaManager.sendTelemetry("Drippers_Slot", (int)(wateringBudget.getSlotDurationMs() / 1000));
  otaManager.sendTelemetry("Dampers_Actual", currentAirMode == AirValveMode::Open ? "Open" : "Close");
  otaManager.sendTelemetry("Sprinklers_Mode", desiredSprinklersMode == WateringMode::On ? "Open" : "Close");
  otaManager.sendTelemetry("Sprinklers_Actual", currentSprinklersMode == WateringMode::On ? "Open" : "Close");
  otaManager.sendTelemetry("Sprinklers_Slot", (int)(sprinklersBudget.getSlotDurationMs() / 1000));

  // Regeneration status telemetry
  if (currentSystemMode == SystemMode::Regenerate) {
    otaManager.sendTelemetry("Regeneration_Status", "Active");
  } else {
    otaManager.sendTelemetry("Regeneration_Status", "Inactive");
  }

  int fanAdcRaw = analogRead(PIN_FAN_INNER_ADC);
  float fanVoltage = fanAdcRaw * (3.3 / 4095.0);  // Adjust if voltage divider exists
  otaManager.sendTelemetry("InnerFan_Voltage", fanVoltage);

  int fanDuty = ledcRead(FAN_INNER_CHANNEL);
  float fanPercent = (fanDuty / 255.0f) * 100.0f;
  otaManager.sendTelemetry("InnerFan_PWM", fanPercent);

  
  // Send the current time
  struct tm localTime;
  if(getLocalTime(&localTime)) 
    otaManager.sendTelemetry("Current_Time", timeToString(localTime));
  
  // OTA Status Telemetry
  if (isFirstBootAfterOTA && !firmwareValidated) {
    otaManager.sendTelemetry("OTA_Status", "VALIDATING");
    otaManager.sendTelemetry("OTA_Health_Progress", healthCheckCounter);
    otaManager.sendTelemetry("OTA_Health_Remaining", REQUIRED_HEALTH_CHECKS - healthCheckCounter);
    otaManager.sendTelemetry("OTA_Validation_Time_Elapsed", (int)((millis() - firmwareStartTime) / 1000));
    otaManager.sendTelemetry("OTA_Validation_Time_Remaining", (int)((MAX_VALIDATION_TIME - (millis() - firmwareStartTime)) / 1000));
    otaManager.sendTelemetry("OTA_System_Healthy", isSystemHealthy());
    otaManager.sendTelemetry("OTA_Last_Loop_Time", lastLoopTime);
  } else if (firmwareValidated) {
    otaManager.sendTelemetry("OTA_Status", "VALIDATED");
    otaManager.sendTelemetry("OTA_Firmware_Version", CURRENT_FIRMWARE_VERSION);
  } else {
    otaManager.sendTelemetry("OTA_Status", "NORMAL");
  }
  
  // S3 server
  delay(300);
  dataLog->uploadDataFile(SITE_NAME, BUILDING_NAME, "Mavnad1");
}

int getFanSpeed() {
  return currentPWMSpeed;
}

void setFanSpeed(int pwm) {
  currentSystemMode = SystemMode::Manual;
  onFans(pwm);
  if(manualWaterMode == WateringMode::On) {
    // If manual water mode is on, set the watering budget based on the fan speed
    setWaterBudgetFromPwm(pwm);
  }
}

bool getDampersStatus() {
  return currentAirMode == AirValveMode::Open;
}

void setDampersStatus(bool isOpen) {
  currentSystemMode = SystemMode::Manual;
  setDampers(isOpen ? AirValveMode::Open : AirValveMode::Close);
}

bool getSolenoidStatus() {
  return (currentWaterMode == WateringMode::On);
}

void setSolenoidStatus(bool status) {
  currentSystemMode = SystemMode::Manual;
  logMessage("[setSolenoidStatus] Setting solenoid status to " + String(status ? "On" : "Off"));
  currentWaterMode = status ? WateringMode::On : WateringMode::Off;
  setWater(currentWaterMode);
}

void restartDevice() {
  logMessage("[restartDevice] Restarting ESP32...");
  delay(1000); // Give time for log message to be sent
  ESP.restart();
}

int getWaterSlot() { // minutes
  return (int)(wateringBudget.getSlotDurationMs() / 1000 / 60);
}

void setWaterSlot(int duration) { // minutes
  currentSystemMode = SystemMode::Manual;
  manualWaterMode = WateringMode::On;
  wateringBudget.setSlotDurationMs(duration * 60 * 1000);
}

int getWaterBudget() { // seconds
  return (int)(wateringBudget.getBudgetDurationMs() / 1000);
}

void setWaterBudget(int duration) {
  currentSystemMode = SystemMode::Manual;
  manualWaterMode = WateringMode::On;
  wateringBudget.setBudgetDurationMs(duration * 1000);
}

bool getSprinklersMode() {
  return desiredSprinklersMode == WateringMode::On;
}

void setSprinklersMode(bool mode) {
  currentSystemMode = SystemMode::Manual;
  desiredSprinklersMode = mode ? WateringMode::On : WateringMode::Off;
}

int getSprinklersSlot() { // minutes
  return (int)(sprinklersBudget.getSlotDurationMs() / 1000 / 60);
}

void setSprinklersSlot(int duration) { // minutes
  currentSystemMode = SystemMode::Manual;
  sprinklersBudget.setSlotDurationMs(duration * 1000 * 60);
}

int getSprinklersBudget() { // seconds
  return (int)(sprinklersBudget.getBudgetDurationMs() / 1000);
}

void setSprinklersBudget(int duration) {
  currentSystemMode = SystemMode::Manual;
  sprinklersBudget.setBudgetDurationMs(duration * 1000);
}

bool getSystemAutoMode() {
  // any mode that is not Manual is auto
  return currentSystemMode != SystemMode::Manual;
}

void setSystemAutoMode(bool autoMode) {
  if(autoMode) {
    // turn off manual mode
    manualWaterMode = WateringMode::Off; 
    off(); // turn off all devices
    updateSystemMode(); // update the system mode based on the current environment parameters
  } else {
    // turn on manual mode
    currentSystemMode = SystemMode::Manual; // turn on system mode
  }
}

bool getDrippersAutoMode() {
  // any mode that is not Manual is auto
  return manualWaterMode == WateringMode::Off; // if manual mode is off, then it is auto mode
}

void setDrippersAutoMode(bool autoMode) {
  if(autoMode) {
    manualWaterMode = WateringMode::Off; // turn off manual mode
  } else {
    manualWaterMode = WateringMode::On; // turn on manual mode
    currentSystemMode = SystemMode::Manual; // turn on system mode
  }
} 

void HandleManualControl();

// ==============================================================================
// OTA HEALTH CHECK FUNCTIONS
// ==============================================================================

// Check if this is the first boot after an OTA update
bool isFirstBootAfterOTAUpdate() {
  esp_ota_img_states_t ota_state;
  if (esp_ota_get_state_partition(NULL, &ota_state) == ESP_OK) {
    return (ota_state == ESP_OTA_IMG_PENDING_VERIFY);
  }
  return false;
}

// Load health check state from NVS
void loadHealthCheckState() {
  // For now, we'll use simple variables. In a more robust implementation,
  // you could use NVS to persist these values across reboots
  if (isFirstBootAfterOTAUpdate()) {
    Serial.println("[OTA-Health] First boot after OTA update detected");
    isFirstBootAfterOTA = true;
    firmwareStartTime = millis();
    healthCheckCounter = 0;
    firmwareValidated = false;
    lastHeartbeatTime = 0;
  } else {
    Serial.println("[OTA-Health] Normal boot - firmware already validated");
    isFirstBootAfterOTA = false;
    firmwareValidated = true;
  }
}

// Save health check state to NVS (placeholder for future implementation)
void saveHealthCheckState() {
  // In a more robust implementation, save to NVS
  // For now, we'll just use the variables
}

// Check if WiFi and sensors are healthy
bool isSystemHealthy() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA-Health] WiFi not connected - system unhealthy");
    return false;
  }
  
  // Check if sensors are responding (basic check)
  if (isnan(shtRS485Manager.getAmbiantTemp())) {
    Serial.println("[OTA-Health] Sensors not responding - system unhealthy");
    return false;
  }
  
  return true;
}

// Mark firmware as valid and send confirmation
void markFirmwareValid() {
  if (firmwareValidated) {
    return; // Already validated
  }
  
  Serial.println("[OTA-Health] Firmware validation successful! Marking as valid...");
  
  // Mark the OTA partition as valid
  esp_ota_img_states_t ota_state;
  if (esp_ota_get_state_partition(NULL, &ota_state) == ESP_OK) {
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
      if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        Serial.println("[OTA-Health] Firmware marked as valid. Rollback cancelled.");
        firmwareValidated = true;
        
        // Send validation confirmation to ThingsBoard
        otaManager.sendTelemetry("fw_state", "VALIDATED");
        otaManager.sendTelemetry("fw_version_validated", CURRENT_FIRMWARE_VERSION);
        otaManager.sendTelemetry("fw_validation_time", (int)((millis() - firmwareStartTime) / 1000));
        
        Serial.printf("[OTA-Health] Firmware %s validated in %d seconds\n", 
                     CURRENT_FIRMWARE_VERSION.c_str(), 
                     (int)((millis() - firmwareStartTime) / 1000));
      } else {
        Serial.println("[OTA-Health] Failed to mark firmware as valid!");
      }
    }
  }
}

// Trigger rollback to previous firmware
void triggerRollback(const String& reason) {
  Serial.printf("[OTA-Health] Triggering rollback: %s\n", reason.c_str());
  
  // Send rollback notification to ThingsBoard
  otaManager.sendTelemetry("fw_state", "ROLLBACK");
  otaManager.sendTelemetry("fw_rollback_reason", reason);
  
  // Mark current firmware as invalid to trigger rollback
  if (esp_ota_mark_app_invalid_rollback_and_reboot() == ESP_OK) {
    Serial.println("[OTA-Health] Rollback initiated. Rebooting...");
    delay(1000);
    ESP.restart();
  } else {
    Serial.println("[OTA-Health] Failed to trigger rollback!");
    // Force restart as fallback
    ESP.restart();
  }
}

// Send heartbeat during validation period
void sendValidationHeartbeat() {
  if (firmwareValidated || !isFirstBootAfterOTA) {
    return; // No need for heartbeat
  }
  
  unsigned long now = millis();
  if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    // Send heartbeat with validation progress
    otaManager.sendTelemetry("fw_validation_progress", healthCheckCounter);
    otaManager.sendTelemetry("fw_validation_remaining", REQUIRED_HEALTH_CHECKS - healthCheckCounter);
    otaManager.sendTelemetry("fw_validation_time_elapsed", (int)((now - firmwareStartTime) / 1000));
    otaManager.sendTelemetry("fw_validation_time_remaining", (int)((MAX_VALIDATION_TIME - (now - firmwareStartTime)) / 1000));
    
    lastHeartbeatTime = now;
    Serial.printf("[OTA-Health] Heartbeat sent - Progress: %d/%d, Time: %ds\n", 
                 healthCheckCounter, REQUIRED_HEALTH_CHECKS, (int)((now - firmwareStartTime) / 1000));
  }
}

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);

  // register to the esp whatchdog, so the esp will reset if get stuck
  Serial.println("[Setup] initializing Watchdog");
  esp_task_wdt_init(90, true); // 10 second timeout, panic/restart
  esp_task_wdt_add(NULL);      // Add current thread to WDT

  Serial.println("----------------------------------------------------------");
  PrintPartitions();
  Serial.println("----------------------------------------------------------");
  Serial.println("[Setup] Welcome to the ThermoTerra whether Controller!");
  Serial.printf("[Setup] Firmware version: %s\n", CURRENT_FIRMWARE_VERSION);

  // Relays
  logMessage("[setup] set relays");
  pinMode(PIN_DAMPER, OUTPUT);
  pinMode(PIN_DAMPER_POWER, OUTPUT);
  pinMode(PIN_PUMP_SPRINKLERS, OUTPUT);
  pinMode(PIN_PUMP_DRIPPERS, OUTPUT);

  digitalWrite(PIN_DAMPER_POWER, LOW);
  digitalWrite(PIN_DAMPER, LOW);
  digitalWrite(PIN_PUMP_SPRINKLERS, LOW);
  digitalWrite(PIN_PUMP_DRIPPERS, LOW);

  ledcSetup(PUMP_PWM_CHANNEL, PUMP_PWM_FREQ, PUMP_PWM_RESOLUTION);
  ledcAttachPin(PIN_PUMP_DRIPPERS, PUMP_PWM_CHANNEL);
  setDrippersPumpSpeed(0); // start off

  // close dampers forcely, to make sure the currentDampersState and the reality are one
  offDampers(true);

  // Mosfet
  logMessage("[setup] set mosfets");
  ledcSetup(FAN_CHANNEL, FAN_FREQUENCY, FAN_RESOLUTION);
  for(const auto& fan : fanPins) {
    pinMode(fan, OUTPUT);
    ledcAttachPin(fan, FAN_CHANNEL);
  }
  ledcWrite(FAN_CHANNEL, 0);

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  // Inner Fans
  logMessage("[setup] set inner fans");
  pinMode(PIN_FAN_INNER1, OUTPUT);
  pinMode(PIN_FAN_INNER_ADC, INPUT);
  //pinMode(PIN_FAN_INNER2, OUTPUT);
  digitalWrite(PIN_FAN_INNER1, LOW);
  //digitalWrite(PIN_FAN_INNER2, LOW);
  ledcSetup(FAN_INNER_CHANNEL, FAN_INNER_FREQUENCY, FAN_INNER_RESOLUTION);
  ledcAttachPin(PIN_FAN_INNER1, FAN_INNER_CHANNEL);
  //ledcAttachPin(PIN_FAN_INNER2, FAN_INNER_CHANNEL);
  ledcWrite(FAN_INNER_CHANNEL, 0);

  // Setup SHT sensors manager
  logMessage("[setup] SHT RS485 sensors manager:");
  shtRS485Manager.setSensorAddr(SHTManager_RS485::AMBIANT, 4);
  shtRS485Manager.setSensorAddr(SHTManager_RS485::BEFORE, 5);
  shtRS485Manager.setSensorAddr(SHTManager_RS485::AFTER, 3);
  shtRS485Manager.setSensorAddr(SHTManager_RS485::ROOM, 1);
  shtRS485Manager.setSensorAddr(SHTManager_RS485::ROOF, 2);
  shtRS485Manager.begin(4800);

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  Serial.println("[Setup] Connecting to WiFi");
  setupWiFi();
  if(WiFi.status() == WL_CONNECTED) {
    int32_t rssi = WiFi.RSSI();
    String message = "[WiFi] Current signal strength (RSSI): " + (String)rssi + "Bm";
    if (rssi < -70) {
      logMessage("[WiFI] Warning: Weak WiFi signal detected. Consider moving closer to the router.");
    } else {
      Serial.println("[WiFi] WiFi signal strength is good.");
    }
  }
  

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  // Step 1: Configure NTP and attempt to get current time
  Serial.println("[Setup] Initializing TimeClient (NTP)");
  struct tm timeInfo = SetupTime();  // This sets the internal time using NTP servers
  timeClient = new TimeClient();
  
  // Initialize ScheduleManager
  scheduleManager = new ScheduleManager(modeSchedule, timeClient);
  logMessage("[Setup] ScheduleManager initialized");

  // Step 2: Initialize RTC over I2C on custom pins (SDA = 7, SCL = 6)
  Serial.println("[Setup] Initializing RTC");
  Wire1.begin(7, 6);  // Use secondary I2C bus
  bool rtcReady = rtc.begin(&Wire1);
  if (!rtcReady) {
    logMessage("[RTC] Failed to initialize RTC — will use NTP only if available.");
  } else {
    logMessage("[RTC] RTC initialized successfully.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Optional: Set compile-time as fallback base
  }

  // Step 3: If NTP time was successfully obtained → set RTC from NTP
  if (getLocalTime(&timeInfo)) {
    Serial.println("[Time] Time retrieved from NTP.");
    Serial.println(timeToString(timeInfo));

    if (rtcReady) {
      rtc.adjust(DateTime(timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
                          timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec));
      Serial.println("[RTC] RTC updated from NTP.");
    }
  } else {
    logMessage("[Time] NTP unavailable. Trying to get time from RTC...");

    // Step 4: Fallback to RTC if NTP fails
    if (rtcReady) {
      DateTime rtcNow = rtc.now();

      // Fill timeInfo struct from RTC
      timeInfo.tm_year = rtcNow.year() - 1900;
      timeInfo.tm_mon  = rtcNow.month() - 1;
      timeInfo.tm_mday = rtcNow.day();
      timeInfo.tm_hour = rtcNow.hour();
      timeInfo.tm_min  = rtcNow.minute();
      timeInfo.tm_sec  = rtcNow.second();

      // Set system time manually (if needed by your TimeClient or system)
      struct timeval tv = {
        .tv_sec = mktime(&timeInfo),
        .tv_usec = 0
      };
      settimeofday(&tv, nullptr);
      logMessage("[Time] System time restored from RTC.");
      Serial.println(timeToString(timeInfo));
    } else {
      logMessage("[RTC] Failed to read time from RTC as well. Using default or previous time.");
    }
  }

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  Serial.print("[Setup] Initializing S3Log ");
  dataLog = new S3Log("/log.txt", timeClient);

  logMessage("[Setup] Initializing ThingsBoard");
  otaManager.setBeforeFirmwareUpdateCallback(off);
  otaManager.getFanSpeedFunc = getFanSpeed;
  otaManager.setFanSpeedFunc = setFanSpeed;
  otaManager.getDampersStatusFunc = getDampersStatus;
  otaManager.setDampersStatusFunc = setDampersStatus;
  otaManager.getSolenoidStatusFunc = getSolenoidStatus;
  otaManager.setSolenoidStatusFunc = setSolenoidStatus;
  otaManager.restartDeviceFunc = restartDevice;
  otaManager.getWaterSlotFunc = getWaterSlot;
  otaManager.setWaterSlotFunc = setWaterSlot;
  otaManager.getWaterBudgetFunc = getWaterBudget;
  otaManager.setWaterBudgetFunc = setWaterBudget;
  otaManager.getSprinklersSlotFunc = getSprinklersSlot;
  otaManager.setSprinklersSlotFunc = setSprinklersSlot;
  otaManager.getSprinklersBudgetFunc = getSprinklersBudget;
  otaManager.setSprinklersBudgetFunc = setSprinklersBudget; 
  otaManager.getSprinklersModeFunc = getSprinklersMode;
  otaManager.setSprinklersModeFunc = setSprinklersMode;
  otaManager.getSystemAutoModeFunc = getSystemAutoMode;
  otaManager.setSystemAutoModeFunc = setSystemAutoMode;
  otaManager.getDrippersAutoModeFunc = getDrippersAutoMode;
  otaManager.setDrippersAutoModeFunc = setDrippersAutoMode;
  otaManager.setInnerFansSpeedFunc = onInnerFans;
  otaManager.getInnerFansSpeedFunc = getInnerFansSpeed;
  otaManager.begin();
  otaManager.sendAttribute("fw_version_actual", CURRENT_FIRMWARE_VERSION);

  // Initialize OTA Health Check System
  logMessage("[Setup] Initializing OTA Health Check System");
  loadHealthCheckState();
  
  if (isFirstBootAfterOTA) {
    Serial.println("[OTA-Health] Starting firmware validation process...");
    otaManager.sendTelemetry("fw_state", "VALIDATING");
    otaManager.sendTelemetry("fw_version_validating", CURRENT_FIRMWARE_VERSION);
  }

  logMessage("[setup] setup done successfuly!");

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();


  updateSystemMode();
  PrintSensors();
  sendTelemetry();

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();
}

// ==============================================================================
// LOOP
// ==============================================================================
bool is2Minute = false;
void loop() {
  unsigned long loopStartTime = millis();
  
  otaManager.tick();
  shtRS485Manager.tick();
  experimentManager.tick();
  tick();

  // OTA Health Check Logic
  if (!firmwareValidated && isFirstBootAfterOTA) {
    // Increment health check counter
    healthCheckCounter++;
    
    // Check if system is healthy
    if (!isSystemHealthy()) {
      Serial.println("[OTA-Health] System unhealthy - triggering rollback");
      triggerRollback("System unhealthy");
      return; // Exit loop to prevent further execution
    }
    
    // Check if we have enough successful loops
    if (healthCheckCounter >= REQUIRED_HEALTH_CHECKS) {
      markFirmwareValid();
    }
    
    // Check timeout
    if (millis() - firmwareStartTime > MAX_VALIDATION_TIME) {
      Serial.printf("[OTA-Health] Validation timeout after %d seconds - triggering rollback\n", 
                   (int)(MAX_VALIDATION_TIME / 1000));
      triggerRollback("Validation timeout");
      return; // Exit loop to prevent further execution
    }
    
    // Send heartbeat during validation
    sendValidationHeartbeat();
    
    // Save progress
    saveHealthCheckState();
  }

  // Every 2 minutes
  if((timeClient->getMinute() % 2) == 0) {
    if(!is2Minute) {
      is2Minute = true;
      esp_task_wdt_reset(); // I'm still alive - Reset watchdog to prevent timeout
      PrintSensors();
      esp_task_wdt_reset(); // I'm still alive - Reset watchdog to timeout
    }
  } else {
    is2Minute = false;
  }

  // Every 5 minutes
  if((timeClient->getMinute() % 5) == 0) {
    // check wifi connection
    if(WiFi.status() != WL_CONNECTED) {
      setupWiFi();
    }
  }

  // Every 10 minutes
  if((timeClient->getMinute() % 10) == 0) {
    if(!isDataSent) {
      isDataSent = true;
      sendTelemetry();
    }
  }
  else {
    isDataSent = false;
  }

  HandleManualControl();

  // Check if loop is taking too long (endless loop detection)
  unsigned long loopTime = millis() - loopStartTime;
  if (loopTime > MAX_LOOP_TIME) {
    Serial.printf("[OTA-Health] Loop took too long: %dms - possible endless loop\n", loopTime);
    if (!firmwareValidated && isFirstBootAfterOTA) {
      triggerRollback("Endless loop detected");
      return;
    }
  }
  
  // Update last loop time for health monitoring
  lastLoopTime = loopTime;

  esp_task_wdt_reset(); // I'm still alive - Reset watchdog to prevent timeout
}

// ==============================================================================
// ==============================================================================
void HandleManualControl(){
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read input until newline
    input.trim(); // Remove leading/trailing whitespace

    if (input.equalsIgnoreCase("print")) {  // PRINT ==============================
      Serial.println("Print");
      PrintSensors();
    } 
    else if (input.equalsIgnoreCase("stop")) {  // STOP ==============================
      Serial.println("Stop");
      off();
    }
    else if (input.equalsIgnoreCase("fans")) {  // FANS ==============================
      Serial.println("Enter speed percentage for fans (0 to 100): ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String speed = Serial.readStringUntil('\n');
        speed.trim();
        onFans(speed.toInt());
    }
    else if (input.equalsIgnoreCase("dampers")) {  // DAMPERS ==============================
      Serial.println("Enter state (1 = Open, 0 = Close): ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String state = Serial.readStringUntil('\n');
        state.trim();
        state.toInt() ? onDampers() : offDampers();
    }
    else if (input.equalsIgnoreCase("sprink")) {  // SPRINKLERS ==============================
      Serial.println("Enter state (1 = Open, 0 = Close): ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String state = Serial.readStringUntil('\n');
        state.trim();
        state.toInt() ? onSprinklers() : offSprinklers();
    }
    else if (input.equalsIgnoreCase("drip")) {  // DRIPPERS ==============================
      Serial.println("Enter state (1 = Open, 0 = Close): ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String state = Serial.readStringUntil('\n');
        state.trim();
        state.toInt() ? onDrippers() : offDrippers();
    }
    else if (input.equalsIgnoreCase("relay")) {  // Relay ==============================
      Serial.println("Enter relay pin number: ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String pinStr = Serial.readStringUntil('\n');
        pinStr.trim();
        int pin = pinStr.toInt();
        Serial.println("Enter state (1 = Open, 0 = Close): ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String stateStr = Serial.readStringUntil('\n');
        stateStr.trim();
        int state = stateStr.toInt();
        state ? digitalWrite(pin, HIGH) : digitalWrite(pin, LOW);
    }
    else if(input.equalsIgnoreCase("heat")) { // HEAT ==================================
      Serial.println("Set system in HEATING mode");
      onHeat();
    }
    else if(input.equalsIgnoreCase("upload")) { // UPLOAD ==============================
      sendTelemetry();  
    }
    else if(input.equalsIgnoreCase("restart")) { // RESTART =============================
      Serial.println("Rebooting...");
      ESP.restart();
    }
    else if(input.equalsIgnoreCase("manual")) { // Manual =============================
      Serial.println("Manual Mode");
      currentSystemMode = SystemMode::Manual;
    }
    else if(input.equalsIgnoreCase("mWater")) { // Manual Watring =============================
      Serial.println("Enter manual watering state (1 = Open, 0 = Close): ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String state = Serial.readStringUntil('\n');
        state.trim();
        if(state.toInt()) {
          Serial.println("manual watering on");
          currentSystemMode = SystemMode::Manual;
          manualWaterMode = WateringMode::On;
        } else {
          Serial.println("manual watering off");
          currentSystemMode = SystemMode::Stop;
          manualWaterMode = WateringMode::Off;
          updateSystemMode();
        }
    }
    else if (input.equalsIgnoreCase("shtcfg")) {  // SHT31 RS485 CONFIG ==============================
      Serial.println("Enter current sensor address (1-247): ");
      while (!Serial.available()) delay(10);
      int oldAddr = Serial.readStringUntil('\n').toInt();

      Serial.println("Enter new sensor address (1-247): ");
      while (!Serial.available()) delay(10);
      int newAddr = Serial.readStringUntil('\n').toInt();

      Serial.println("Enter baud rate (0=2400, 1=4800, 2=9600): ");
      while (!Serial.available()) delay(10);
      int baudCode = Serial.readStringUntil('\n').toInt();

      // Set new address and baud
      bool ok = shtRS485Manager.setSensorAddressAndBaud(oldAddr, newAddr, baudCode);
      if (ok) {
        Serial.println("Sensor address/baud updated successfully.");
      } else {
        Serial.println("Failed to update sensor address/baud.");
      }

      delay(200); // Give sensor time to switch
    }
    else if (input.equalsIgnoreCase("shtbaud")) {  // SHT31 RS485 SERIAL BAUD CHANGE ==============================
      Serial.println("Enter new baud rate (e.g. 2400, 4800, 9600): ");
      while (!Serial.available()) delay(10);
      int newBaud = Serial.readStringUntil('\n').toInt();
      if (newBaud == 2400 || newBaud == 4800 || newBaud == 9600) {
        shtRS485Manager.begin(newBaud);
        Serial.printf("RS485 serial baud rate changed to %d.\n", newBaud);
      } else {
        Serial.println("Invalid baud rate. Allowed: 2400, 4800, 9600.");
      }
    }
    else if (input.equalsIgnoreCase("shtscan")) {  // SHT31 RS485 SCAN ==============================
      Serial.println("Scanning RS485 bus at all common baud rates (2400, 4800, 9600)...");
      shtRS485Manager.scanRS485AllBauds(RS485Serial);
    }
    else if(input.equalsIgnoreCase("debug")) { // Debug Mode =============================
      Serial.println("Debug Mode");
      debugMode = true;
    }
    else if(input.equalsIgnoreCase("nodebug")) { // No Debug Mode =============================
      Serial.println("No Debug Mode");
      debugMode = false;
    }
    else if(input.equalsIgnoreCase("auto")) { // Auto =============================
      Serial.println("Auto Mode");
      currentSystemMode = SystemMode::Stop;
      updateSystemMode();
    }
    else if(input.equalsIgnoreCase("otastatus")) { // OTA STATUS =============================
      Serial.println("=== OTA Health Status ===");
      Serial.printf("First boot after OTA: %s\n", isFirstBootAfterOTA ? "YES" : "NO");
      Serial.printf("Firmware validated: %s\n", firmwareValidated ? "YES" : "NO");
      if (isFirstBootAfterOTA && !firmwareValidated) {
        Serial.printf("Health check progress: %d/%d\n", healthCheckCounter, REQUIRED_HEALTH_CHECKS);
        unsigned long elapsed = (millis() - firmwareStartTime) / 1000;
        unsigned long remaining = (MAX_VALIDATION_TIME - (millis() - firmwareStartTime)) / 1000;
        Serial.printf("Time elapsed: %ds, Time remaining: %ds\n", elapsed, remaining);
        Serial.printf("System healthy: %s\n", isSystemHealthy() ? "YES" : "NO");
        Serial.printf("Last loop time: %dms\n", lastLoopTime);
      }
      Serial.printf("Current firmware version: %s\n", CURRENT_FIRMWARE_VERSION.c_str());
      Serial.println("========================");
    }
    else if(input.equalsIgnoreCase("otarollback")) { // OTA ROLLBACK =============================
      Serial.println("=== OTA Manual Rollback ===");
      Serial.println("WARNING: This will rollback to the previous firmware version!");
      Serial.println("Are you sure? Type 'YES' to confirm:");
      while (!Serial.available()) delay(10);
      String confirmation = Serial.readStringUntil('\n');
      confirmation.trim();
      if (confirmation.equalsIgnoreCase("YES")) {
        Serial.println("Manual rollback confirmed. Rolling back...");
        triggerRollback("Manual rollback requested");
      } else {
        Serial.println("Rollback cancelled.");
      }
      Serial.println("==========================");
    }
         else if(input.equalsIgnoreCase("otavalidate")) { // OTA VALIDATE =============================
       Serial.println("=== OTA Manual Validation ===");
       if (isFirstBootAfterOTA && !firmwareValidated) {
         Serial.println("Manually validating firmware...");
         markFirmwareValid();
       } else if (firmwareValidated) {
         Serial.println("Firmware is already validated.");
       } else {
         Serial.println("No pending firmware to validate.");
       }
       Serial.println("============================");
     }
     else if(input.equalsIgnoreCase("schedule")) { // SCHEDULE INFO =============================
       Serial.println("=== Current Schedule Information ===");
       if (scheduleManager) {
         Serial.println(scheduleManager->getCurrentScheduleInfo());
       } else {
         Serial.println("ScheduleManager not available");
       }
       Serial.println("=====================================");
     }
    else if (input.startsWith("exp")) {  // EXPERIMENT COMMANDS =============================
      if (input.equalsIgnoreCase("exp")) {
        Serial.println("Experiment commands:");
        Serial.println("  exp start <experiment_name> - Start specific experiment");
        Serial.println("  exp stop                    - Stop current experiment");
        Serial.println("  exp pause                   - Pause current experiment");
        Serial.println("  exp resume                  - Resume current experiment");
        Serial.println("  exp status                  - Show experiment status");
        Serial.println("  exp list                    - List available experiments");
      }
      else if (input.startsWith("exp start")) {
        String expName = input.substring(9);
        expName.trim();
        if (expName.isEmpty()) {
          Serial.println("Usage: exp start <experiment_name>");
          Serial.println("Use 'exp list' to see available experiments");
        } else if (experimentManager.startExperiment(expName)) {
          Serial.printf("Started experiment: %s\n", expName.c_str());
        } else {
          Serial.printf("Failed to start experiment: %s\n", expName.c_str());
        }
      }
      else if (input.equalsIgnoreCase("exp stop")) {
        if (experimentManager.isExperimentRunning()) {
          experimentManager.stopExperiment();
          Serial.println("Experiment stopped");
        } else {
          Serial.println("No experiment is currently running");
        }
      }
      else if (input.equalsIgnoreCase("exp pause")) {
        if (experimentManager.isExperimentRunning()) {
          experimentManager.pauseExperiment();
          Serial.println("Experiment paused");
        } else {
          Serial.println("No experiment is currently running");
        }
      }
      else if (input.equalsIgnoreCase("exp resume")) {
        experimentManager.resumeExperiment();
        Serial.println("Experiment resumed");
      }
      else if (input.equalsIgnoreCase("exp status")) {
        String status = experimentManager.getStatus();
        Serial.println("=== Experiment Status ===");
        Serial.println(status);
        Serial.println("========================");
      }
      else if (input.equalsIgnoreCase("exp list")) {
        Serial.println("=== Available Experiments ===");
        experimentManager.listExperiments();
        Serial.println("=============================");
      }
      else {
        Serial.println("Unknown experiment command. Use 'exp' for help.");
      }
    }
         else {
          Serial.println("No such command. use: print, stop, dampers, drip, sprink, reg, regeff, exp, otastatus, otarollback, otavalidate, regslot, schedule");
     }
  }
}