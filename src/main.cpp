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
const String CURRENT_FIRMWARE_VERSION = "1.0.2.136";
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
const int PIN_DAMPER = 12;
const int PIN_DAMPER_POWER = 11;
const int PIN_PUMP_SPRINKLERS = 3; //10;

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

// Define the AirValveMode enum
enum AirValveMode {
  Close,
  Open
};
enum class DamperState {
  Idle,
  Opening,
  Closing
};

// Define the possible statuses of the system
enum SystemMode {
  Regenerate,
  Heat,
  Cool,
  Stop,
  Manual
};

enum WateringMode {
  Off,
  On
};

struct ScheduleEntry {
  uint8_t dayOfWeek;  // 0 = Sunday, 1 = Monday, ... 6 = Saturday
  uint8_t startHour;  // 24h format
  uint8_t startMin;
  uint8_t endHour;
  uint8_t endMin;
  SystemMode mode;
};

std::vector<ScheduleEntry> modeSchedule = {
  // Sunday
  { 0,  0,  0,  6,  0, SystemMode::Regenerate }, 
  { 0,  6,  0, 19,  0, SystemMode::Cool },      
  { 0, 22,  0, 23, 59, SystemMode::Regenerate },
  // Monday 
  { 1,  0,  0,  6,  0, SystemMode::Regenerate }, 
  { 1,  6,  0, 19,  0, SystemMode::Cool },      
  { 1, 22,  0, 23, 59, SystemMode::Regenerate },
  // Tuesday 
  { 2,  0,  0,  6,  0, SystemMode::Regenerate }, 
  { 2,  6,  0, 19,  0, SystemMode::Cool },      
  { 2, 22,  0, 23, 59, SystemMode::Regenerate }, 
  // Wednesday
  { 3,  0,  0,  6,  0, SystemMode::Regenerate }, 
  { 3,  6,  0, 19,  0, SystemMode::Cool },      
  { 3, 22,  0, 23, 59, SystemMode::Regenerate }, 
  // Thursday
  { 4,  0,  0,  6,  0, SystemMode::Regenerate }, 
  { 4,  6,  0, 19,  0, SystemMode::Cool },      
  { 4, 22,  0, 23, 59, SystemMode::Regenerate }, 
  // Friday
  { 5,  0,  0,  6,  0, SystemMode::Regenerate }, 
  { 5,  6,  0, 19,  0, SystemMode::Cool },      
  { 5, 22,  0, 23, 59, SystemMode::Regenerate }, 
  // Saturday
  { 6,  0,  0,  6,  0, SystemMode::Regenerate },
  { 6,  6,  0, 19,  0, SystemMode::Cool },
  { 6, 22,  0, 23, 59, SystemMode::Regenerate } 
  // Add more days as needed
};

struct SystemModeHelper {
  static String toString(SystemMode mode) {
    switch (mode) {
      case SystemMode::Cool:      return "Cool";
      case SystemMode::Heat:      return "Heat";
      case SystemMode::Regenerate: return "Regenerate";
      case SystemMode::Stop:      return "Stop";
      case SystemMode::Manual:      return "Manual";
      default:                       return "Unknown";
    }
  }

  static SystemMode fromString(const String& str) {
    if (str.equalsIgnoreCase("Cool"))      return SystemMode::Cool;
    if (str.equalsIgnoreCase("Heat"))      return SystemMode::Heat;
    if (str.equalsIgnoreCase("Regenerate")) return SystemMode::Regenerate;
    if (str.equalsIgnoreCase("Stop"))      return SystemMode::Stop;

    Serial.printf("[SystemModeHelper] Unknown mode string: %s\n", str.c_str());
    return SystemMode::Stop;  // fallback
  }
};

// Global variable to hold the current system status
DamperState damperState = DamperState::Idle;
SystemMode currentSystemMode = SystemMode::Stop;
AirValveMode currentAirMode = AirValveMode::Close;
WateringMode currentWaterMode = WateringMode::Off;
WateringMode currentDrippersMode = WateringMode::Off;
WateringMode desiredSprinklersMode = WateringMode::Off;
WateringMode currentSprinklersMode = WateringMode::Off;
WateringMode manualWaterMode = WateringMode::Off;
int currentPWMSpeed = 0; // in percentage (0 - 100)
float lastAfterHumidity = NAN; // Last valid humidity reading
bool debugMode = false;
unsigned long damperActionStartTime = 0;
const unsigned long DAMPER_ACTION_DURATION_MS = 15 * 1000;
unsigned long regenerationStartTime = 0; // When the current regeneration cycle started
bool canStartRegeneration = true; // Track if we can start regeneration in this slot - true for first time in slot
RTC_DS3231 rtc;

void offDrippers();
void offSprinklers();
void startRegeneration();
bool isRegenerationEffective();
bool isTopOfHour();
bool isSystemHealthy();

// Define TimeBudgetManager instances for watering and sprinklers
TimeBudgetManager wateringBudget(1L * 60 * 1000, 0.25L * 60 * 1000, offDrippers);
TimeBudgetManager sprinklersBudget(4L * 60 * 1000, 0.166L * 60 * 1000, offSprinklers);

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
  // System state value has 3 digits:
  // First one is the system mode: 1 = cooling; 2 = heating; 3 = regenerating; 0 = off
  // Second digit is fans speed: 1 = low; 2 = medium; 3 = high speed; 0 = fans are closed
  // Third digit is the water status: 1 = open; 0 = close
  // Fourth digit is the dampers status: 1 = open; 0 = close
  // The sing (negative or possitive number) is: > = outtake; < 0 = intake (reverse)
  int status = 0; // first digit - operating mode
  if(currentSystemMode == SystemMode::Cool) status = 1;
  else if(currentSystemMode == SystemMode::Heat) status = 2;
  else if(currentSystemMode == SystemMode::Regenerate) status = 3;
  else if(currentSystemMode == SystemMode::Manual) status = 4;

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

void logModeToS3(float value) {
    logToS3("System", "system", "cooler", "mode", value); // "mode" must be in location "cooler"

    String message = "System mode code " + (String)getSystemStatusCode() +
                   "; Mode = " + SystemModeHelper::toString(currentSystemMode) +
                    "; PWM = " + (String)currentPWMSpeed +
                    "; Drippers Mode = " + (currentWaterMode == WateringMode::On ? "Open" : "Close") +
                    "; Drippers Actual = " + (currentDrippersMode == WateringMode::On ? "Open" : "Close") +
                    "; Sprinklers Mode = " + (desiredSprinklersMode == WateringMode::On ? "Open" : "Close") +
                    "; Sprinklers Actual = " + (currentSprinklersMode == WateringMode::On ? "Open" : "Close") +
                    "; Dampers = " + (currentAirMode == AirValveMode::Open ? "Open" : "Close");
  logMessage(message);
}

void onFans(int percentage = 70) {
  if(currentPWMSpeed == percentage) return;

  String message = "[Fans] current pwm = " + (String)currentPWMSpeed + "; new = " + (String)percentage + "\n";
  logMessage(message);
  int duty = map(percentage, 0, 100, 0, 255);
  ledcWrite(FAN_CHANNEL, duty);
  currentPWMSpeed = percentage;
  logModeToS3(getSystemStatusCode());
}

void offFans() {
  debugMessage("off fans");
  if(currentPWMSpeed == 0) return;
  logMessage("[Fans] off");
  ledcWrite(FAN_CHANNEL, 0);
  currentPWMSpeed = 0;
  logModeToS3(getSystemStatusCode());
}

void onDampers() {
  debugMessage("on dampers");
  if(currentAirMode == AirValveMode::Open) return;
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
    logModeToS3(getSystemStatusCode());
  }
}

void offSprinklers() {
  debugMessage("off sprinklers");
  if(currentSprinklersMode != WateringMode::Off) 
    logMessage("[Sprinklers] off"); // printing only once, when changing state from on to off
  
  digitalWrite(PIN_PUMP_SPRINKLERS, LOW);
  
  currentSprinklersMode = WateringMode::Off;
  logModeToS3(getSystemStatusCode());
}

void onSprinklers() {
  // debugMessage("on sprinklers");
  // if (!sprinklersBudget.isAllowed()) {
  //   offSprinklers();
  //   return;
  // }

  // if(currentSprinklersMode == WateringMode::On) return;
  // logMessage("[Sprinklers] on");
  
  // digitalWrite(PIN_PUMP_SPRINKLERS, HIGH);
  
  // currentSprinklersMode = WateringMode::On;
  // logModeToS3(getSystemStatusCode());
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
  logModeToS3(getSystemStatusCode());
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
  logModeToS3(getSystemStatusCode());
}

void off() {
  debugMessage("off");
  if(currentSystemMode == SystemMode::Stop) return;

  offSprinklers();
  offDrippers();
  offFans();
  offDampers();

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
  if(isnan(roomRH)) return airMode;

  if(currentAirMode == AirValveMode::Open && roomRH > 70) airMode = AirValveMode::Close;
  if(currentAirMode == AirValveMode::Close && roomRH <= 60) airMode = AirValveMode::Open;
  return airMode;
}

void setSystemMode(AirValveMode airMode, int fanPercentage = 70, WateringMode wateringMode = WateringMode::Off) {
  debugMessage("set system mode");
  setWater(wateringMode);
  onFans(fanPercentage);
  setDampers(airMode);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setWaterBudgetFromPwm(int pwm) {
  // Set the watering budget based on the fan speed
  pwm = constrain(pwm, 0, 100); // Ensure pwm is within 0-100%
  int budgetTime = map(pwm, 0, 100, 0, 15);     // seconds 0..15
  logMessage("[setWaterBudgetFromPwm] Setting watering budget to " + String(budgetTime) + " seconds");
  wateringBudget.setBudgetDurationMs((unsigned long)budgetTime * 1000UL);
}

void onCool() {
  debugMessage("on cool");
  float roomTemp = shtRS485Manager.getRoomTemp();
  if(isnan(roomTemp)) roomTemp = START_COOLING_DEG; // If room temperature is not available, use the default value
  if(roomTemp < START_COOLING_DEG) {off(); return;} // It's too cold to start cooling

  // Select DAMPERS mode based on room temperature
  AirValveMode airMode = getAirModeByRoom();

  // Select FANS speed based on room temperature
  float pwmPercentage = mapFloat(roomTemp, 25.0, 29.0, 20.0, 70.0);
  pwmPercentage = max(20.0f, min(70.0f, pwmPercentage)); // clamp the value

  // As long as the before RH is under 95%, we can add water
  float beforeRH = shtRS485Manager.getBeforeRH();
  if(!isnan(beforeRH)) {
    if(beforeRH < 95) {
      currentWaterMode = WateringMode::On; 
    } else {
      currentWaterMode = WateringMode::Off;
    }
  }

  if(currentWaterMode == WateringMode::On) {
    // Set the water budget - 10-40 seconds depending on fan speed 
    setWaterBudgetFromPwm((int)pwmPercentage);
  } 
  else {
    wateringBudget.setBudgetDurationMs(0); // No budget if water is off
  }

  setSystemMode(airMode, (int)pwmPercentage, currentWaterMode);

  currentSystemMode = SystemMode::Cool;
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

// Helper function to check if regeneration is effective
bool isRegenerationEffective() {
  float beforeRH = shtRS485Manager.getBeforeRH();
  float afterRH = shtRS485Manager.getAfterRH();
  
  if(isnan(beforeRH)) {
    logMessage("[Regenerate] Before RH is not available, cannot validate effectiveness");
    return false;
  }
  
  if(isnan(afterRH)) {
    logMessage("[Regenerate] After RH is not available, cannot validate effectiveness");
    return false;
  }
  
  // Additional safety checks
  if (beforeRH < 0 || afterRH < 0 || beforeRH > 100 || afterRH > 100) {
    logMessage("[Regenerate] Invalid humidity values - Before RH: " + String(beforeRH) + 
               "%, After RH: " + String(afterRH) + "%");
    return false;
  }
  
  // Check if after RH is at least 10% lower than before RH
  // Example: if beforeRH = 60%, then threshold = 54% (60 * 0.9)
  // Regeneration is effective if afterRH <= 54% (meaning humidity was added)
  float threshold = beforeRH * 0.95f; // 10% relative threshold
  bool isEffective = threshold >= afterRH;
  
  logMessage("[Regenerate] Effectiveness check - Before RH: " + String(beforeRH, 1) + 
             "%, After RH: " + String(afterRH, 1) + "%, Threshold: " + String(threshold, 1) + 
             "%, Effective: " + String(isEffective ? "YES" : "NO"));
  
  return isEffective;
}

// Helper function to start fresh regeneration
void startRegeneration() {
  logMessage("[Regenerate] Starting fresh regeneration cycle");
  
  // Log current humidity values for reference
  float beforeRH = shtRS485Manager.getBeforeRH();
  float afterRH = shtRS485Manager.getAfterRH();
  float ambiantRH = shtRS485Manager.getAmbiantRH();
  
  if (!isnan(beforeRH) && !isnan(afterRH) && !isnan(ambiantRH)) {
    logMessage("[Regenerate] Starting with - Before RH: " + String(beforeRH, 1) + 
               "%, After RH: " + String(afterRH, 1) + "%, Ambiant RH: " + String(ambiantRH, 1) + "%");
  } else {
    logMessage("[Regenerate] Warning: Some humidity sensors not available");
  }
  
  // Set drippers budget to 15 seconds
  wateringBudget.setBudgetDurationMs(15 * 1000);

  // Set regeneration parameters: 80% fan speed, 15 seconds drippers budget, open dampers
  currentWaterMode = WateringMode::On;
  setSystemMode(AirValveMode::Open, 80, currentWaterMode);
  
  // Store the start time
  regenerationStartTime = millis();
  
  // Set system mode
  currentSystemMode = SystemMode::Regenerate;
  
  logMessage("[Regenerate] Regeneration started - Fans: 80%, Drippers: 15s, Dampers: Open");
}

void onRegenerate() {
  debugMessage("on regenerate");

  // If we're not currently regenerating, check if it's time to start
  if (currentSystemMode != SystemMode::Regenerate) {
    // Priority 1: First time entering regeneration slot - start immediately
    if (canStartRegeneration) {
      logMessage("[Regenerate] First time in regeneration slot - starting immediately");
      startRegeneration();
      canStartRegeneration = false;  // Clear the flag after using it
      return;
    }
    
    // Priority 2: Check if it's top of hour for subsequent attempts
    if (isTopOfHour()) {
      logMessage("[Regenerate] Top of hour - starting fresh regeneration");
      startRegeneration();
      return;
    } else {
      logMessage("[Regenerate] Not time to start regeneration yet (not top of hour)");
      return; // Not time yet
    }
  }
  
  // We're already regenerating, check if it's time to validate effectiveness
  if (regenerationStartTime > 0 && millis() - regenerationStartTime >= 10 * 60 * 1000) { // 10 minutes
    logMessage("[Regenerate] 10+ minutes passed, checking regeneration effectiveness");
    
    if (!isRegenerationEffective()) {
      logMessage("[Regenerate] Regeneration is not effective - stopping and waiting for next hour");
      off(); // Stop regeneration and wait for next hour
      regenerationStartTime = 0; // Reset start time
      return;
    } else {
      logMessage("[Regenerate] Regeneration is effective - continuing");
      // Continue regenerating with current parameters
    }
  } else if (regenerationStartTime > 0) {
    // Less than 10 minutes, continue with current parameters
    unsigned long remainingMinutes = (10 * 60 * 1000 - (millis() - regenerationStartTime)) / 60000;
    logMessage("[Regenerate] Continuing regeneration - " + String(remainingMinutes) + " minutes until effectiveness check");
  }
}

void onHeat() {
  logMessage("[onHeat] on");
  Serial.println("Heat mode is not supported yet");
}

void setInnerFansSpeed(int percentage = -1) {
  String message = "[setInnerFansSpeed] ";

  if(percentage == -1) { // Auto mode
    float roomTemp = shtRS485Manager.getRoomTemp();
    if(isnan(roomTemp)) {
      ledcWrite(FAN_INNER_CHANNEL, 0); // turn off inner fans
      logMessage("[setInnerFansSpeed] Room temp not available. Fans turned off.");
      return;
    }

    // Calculate PWM percentage based on room temperature
    if(roomTemp >= 27.0) percentage = 30;
    else if(roomTemp <= 23.0) percentage = 0;
    else percentage = (int)mapFloat(roomTemp, 23.0, 27.0, 20.0, 30.0);

    message += "Room temp: " + String(roomTemp, 1) + "°C → ";
  }

  // Convert to duty cycle (0–255 for 8-bit resolution)
  percentage = constrain(percentage, 0, 100); // just to make sure
  int duty = map(percentage, 0, 100, 0, 255);
  ledcWrite(FAN_INNER_CHANNEL, duty);

  message += "Fan speed: " + String(percentage) + "% (duty " + String(duty) + ")";
  logMessage(message);
}

// ========== update mode =============
SystemMode lastSelectedMode = SystemMode::Stop;
void updateSystemMode() {
  if(currentSystemMode == SystemMode::Manual) return; // Manual mode is not controlled by schedule

  SystemMode newMode = SystemMode::Stop;
  
  // Gets the room temperature
  float roomTemp = shtRS485Manager.getRoomTemp();
  if(isnan(roomTemp)) roomTemp = shtRS485Manager.getAfterTemp(); // room and after are very similar

  //if(isnan(roomTemp)) { // if no sensor is available, check for time entry
  // gets current time
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    logMessage("[UpdateSystemMode] Failed to get local time");
    return; // <<<<<<<<<<<<<<<<============================= TBD: handle this case better
  }

  uint8_t dow = timeinfo.tm_wday; // 0 = Sunday
  uint16_t currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  // looking for current time schedualing entry
  for (const auto& entry : modeSchedule) {
    if (entry.dayOfWeek != dow) continue;

    uint16_t start = entry.startHour * 60 + entry.startMin;
    uint16_t end   = entry.endHour   * 60 + entry.endMin;

    if (currentMinutes >= start && currentMinutes < end) {
      newMode = entry.mode;
      logMessage("[UpdateSystemMode] Schedule entry found: " + SystemModeHelper::toString(newMode));
      break; // Stop at first match
    }
  }

  if(newMode == SystemMode::Cool) { // If we got the cool mode, we can decide based on room temperature
    if(roomTemp >= START_COOLING_DEG) {
      newMode = SystemMode::Cool; // If room temperature is high enough, start cooling
    } else {
      newMode = SystemMode::Stop; // If temperature is low, stop the system
      logMessage("[UpdateSystemMode] Room temperature is too low for cooling: " + String(roomTemp) + "°C");
    }
  } else if(newMode == SystemMode::Heat) { // If we got the heat mode, we can decide based on room temperature
    if(roomTemp <= START_HEATING_DEG) {
      newMode = SystemMode::Heat; // If room temperature is low enough, start heating
    } else {
      newMode = SystemMode::Stop; // If temperature is high, stop the system
      logMessage("[UpdateSystemMode] Room temperature is too high for heating: " + String(roomTemp) + "°C");
    }
  } else if(newMode == SystemMode::Regenerate) { // If we got the regenerate mode, check if we can start regeneration
    // Regeneration mode is scheduled - the onRegenerate() function will handle the actual start logic
    // based on whether it's the top of the hour and other conditions
  }

  // Will be true only if the mode has changed
  if (newMode != currentSystemMode && lastSelectedMode != newMode) {
    // lastSelectedMode is just to make sure i print this only once
    lastSelectedMode = newMode;
    String message = "[System] Mode changed to " + SystemModeHelper::toString(newMode) + "\n";
    logMessage(message);
  }

  // float beforeRH = NAN;
  // float afterRH = NAN;

  // // Check the sprinklers mode
  // if (newMode == SystemMode::Cool) {
  //   // get before humidity and after humidity for sprinklers decision
  //   beforeRH = shtRS485Manager.getBeforeRH();
  //   afterRH = shtRS485Manager.getAfterRH();

  //   if (!isnan(beforeRH) && !isnan(afterRH) && beforeRH <= 92 && afterRH <= 92) {
  //     desiredSprinklersMode = WateringMode::On; // Turn on sprinklers if before RH is low
  //   } else {
  //     desiredSprinklersMode = WateringMode::Off; // Turn off sprinklers if before RH is high
  //   }
  // } 

  // If the system is stopping, save the last after humidity
  if(currentSystemMode != SystemMode::Stop && newMode == SystemMode::Stop) {
    lastAfterHumidity = shtRS485Manager.getAfterRH();
  }
  // If the system was already in Stop mode, check if ambiant RH is high enough to regenerate the mattress
  // if(currentSystemMode == SystemMode::Stop) {
  //   float ambiantRH = shtRS485Manager.getAmbiantRH();
  //   if(!isnan(ambiantRH) && !isnan(lastAfterHumidity) && ambiantRH > lastAfterHumidity + 5) {
  //     newMode = SystemMode::Regenerate; // If ambiant RH is high enough, regenerate the mattress
  //   }
  // }

  // Reset regeneration flag when exiting regeneration time slot
  if (currentSystemMode == SystemMode::Regenerate && newMode != SystemMode::Regenerate) {
    canStartRegeneration = true;  // Reset to true for next regeneration slot
    logMessage("[UpdateSystemMode] Exiting regeneration time slot - resetting regeneration flag");
  }
    
  switch(newMode) {
    case SystemMode::Regenerate:
      onRegenerate();
      break;
    case SystemMode::Cool:
      onCool();
      break;
    case SystemMode::Heat:
      onHeat();
      break;
    case SystemMode::Stop:
      off();
      break;
    default:
      off();
      break;
  }

  // Update inner fans speed
  setInnerFansSpeed();
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
                    "; Sprinklers Mode = " + (desiredSprinklersMode == WateringMode::On ? "Open" : "Close") +
                    "; Sprinklers Actual = " + (currentSprinklersMode == WateringMode::On ? "Open" : "Close") +
                    "; Dampers = " + (currentAirMode == AirValveMode::Open ? "Open" : "Close");
  logMessage(message);
  wateringBudget.printStatus("Drippers");
  sprinklersBudget.printStatus("Sprinklers");

  // RS485 SHT31 sensors
  Serial.println("RS485 SHT31 sensors:");
  Serial.print("Ambiant: Temp = " + (String)shtRS485Manager.getAmbiantTemp());
  Serial.println("; RH = " + (String)shtRS485Manager.getAmbiantRH());
  Serial.print("Before: Temp = " + (String)shtRS485Manager.getBeforeTemp());
  Serial.println("; RH = " + (String)shtRS485Manager.getBeforeRH());
  Serial.print("After: Temp= " + (String)shtRS485Manager.getAfterTemp());
  Serial.println("; RH = " + (String)shtRS485Manager.getAfterRH());
  Serial.print("Room: Temp = " + (String)shtRS485Manager.getRoomTemp());
  Serial.println("; RH = " + (String)shtRS485Manager.getRoomRH());
  
     // Show regeneration status if applicable
   if (currentSystemMode == SystemMode::Regenerate && regenerationStartTime > 0) {
     unsigned long regenerationTime = (millis() - regenerationStartTime) / 1000; // in seconds
     Serial.println("Regeneration Status: Active for " + String(regenerationTime) + " seconds");
     if (regenerationTime >= 600) { // 10 minutes
       Serial.println("Regeneration: Ready for effectiveness check");
     } else {
       unsigned long remainingTime = 600 - regenerationTime;
       Serial.println("Regeneration: " + String(remainingTime) + " seconds until effectiveness check");
     }
   } else if (currentSystemMode == SystemMode::Regenerate) {
     Serial.println("Regeneration Status: Starting up...");
   }
   
   // Show regeneration slot status
   if (canStartRegeneration) {
    Serial.println("Regeneration Slot: Can start regeneration - will start immediately");
  } else {
    Serial.println("Regeneration Slot: Regeneration already attempted - waiting for top of hour");
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
  otaManager.sendTelemetry("System_Status_Code", String(getSystemStatusCode()));
  logToS3("System", "system", "cooler", "mode", getSystemStatusCode()); // mode data must be in location "cooler"

  otaManager.sendTelemetry("System_Mode", SystemModeHelper::toString(currentSystemMode));
  otaManager.sendTelemetry("Fans_Speed", currentPWMSpeed);
  otaManager.sendTelemetry("Water_Mode", currentWaterMode == WateringMode::On ? "On" : "Off");
  otaManager.sendTelemetry("Drippers_Actual", currentDrippersMode == WateringMode::On ? "Open" : "Close");
  otaManager.sendTelemetry("Drippers_Slot", (int)(wateringBudget.getSlotDurationMs() / 1000));
  otaManager.sendTelemetry("Drippers_Budget", (int)(wateringBudget.getBudgetDurationMs() / 1000));
  otaManager.sendTelemetry("Dampers_Status", currentAirMode == AirValveMode::Open ? "true" : "false");
  otaManager.sendTelemetry("Dampers_Actual", currentAirMode == AirValveMode::Open ? "Open" : "Close");
  otaManager.sendTelemetry("Sprinklers_Mode", desiredSprinklersMode == WateringMode::On ? "Open" : "Close");
  otaManager.sendTelemetry("Sprinklers_Actual", currentSprinklersMode == WateringMode::On ? "Open" : "Close");
  otaManager.sendTelemetry("Sprinklers_Slot", (int)(sprinklersBudget.getSlotDurationMs() / 1000));
  otaManager.sendTelemetry("Sprinklers_Budget", (int)(sprinklersBudget.getBudgetDurationMs() / 1000));

  // Regeneration status telemetry
  if (currentSystemMode == SystemMode::Regenerate && regenerationStartTime > 0) {
    unsigned long regenerationTime = (millis() - regenerationStartTime) / 1000; // in seconds
    otaManager.sendTelemetry("Regeneration_Active_Time", (int)regenerationTime);
    otaManager.sendTelemetry("Regeneration_Ready_For_Check", regenerationTime >= 600 ? "Yes" : "No");
    if (regenerationTime < 600) {
      unsigned long remainingTime = 600 - regenerationTime;
      otaManager.sendTelemetry("Regeneration_Time_Until_Check", (int)remainingTime);
    }
  } else if (currentSystemMode == SystemMode::Regenerate) {
    otaManager.sendTelemetry("Regeneration_Active_Time", 0);
    otaManager.sendTelemetry("Regeneration_Ready_For_Check", "Starting");
  } else {
    otaManager.sendTelemetry("Regeneration_Active_Time", 0);
    otaManager.sendTelemetry("Regeneration_Ready_For_Check", "N/A");
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

int getWaterSlot() { // minutes
  return (int)(wateringBudget.getSlotDurationMs() / 1000 / 60);
}

void setWaterSlot(int duration) {
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
  if (shtRS485Manager.getAmbiantTemp() == FLOAT_NAN) {
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
  otaManager.getWaterSlotFunc = getWaterSlot;
  otaManager.setWaterSlotFunc = setWaterSlot;
  otaManager.getWaterBudgetFunc = getWaterBudget;
  otaManager.setWaterBudgetFunc = setWaterBudget;
  otaManager.getSystemAutoModeFunc = getSystemAutoMode;
  otaManager.setSystemAutoModeFunc = setSystemAutoMode;
  otaManager.getDrippersAutoModeFunc = getDrippersAutoMode;
  otaManager.setDrippersAutoModeFunc = setDrippersAutoMode;
  otaManager.setInnerFansSpeedFunc = setInnerFansSpeed;
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

  // Initialize regeneration start time to 0 (no regeneration active)
  regenerationStartTime = 0;

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
    else if (input.equalsIgnoreCase("reg")) { // REG (REGENERATION) ===========================
      Serial.println("Set system in REGENERATING mode");
      onRegenerate();
    }
    else if (input.equalsIgnoreCase("regeff")) { // REGENERATION EFFECTIVENESS CHECK ===========================
      Serial.println("Testing regeneration effectiveness check...");
      if (currentSystemMode == SystemMode::Regenerate) {
        bool isEffective = isRegenerationEffective();
        Serial.println("Regeneration effectiveness: " + String(isEffective ? "YES" : "NO"));
      } else {
        Serial.println("System is not in regeneration mode");
      }
    }
    else if(input.equalsIgnoreCase("cool")) { // COOL ==============================
      Serial.println("Set system in COOLING mode");
      onCool();
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
     else if(input.equalsIgnoreCase("regslot")) { // REGENERATION SLOT STATUS =============================
       Serial.println("=== Regeneration Slot Status ===");
       Serial.printf("First time in regeneration slot: %s\n", canStartRegeneration ? "YES" : "NO");
       Serial.printf("Current system mode: %s\n", SystemModeHelper::toString(currentSystemMode).c_str());
       Serial.printf("Regeneration start time: %lu\n", regenerationStartTime);
       Serial.println("================================");
     }
         else {
          Serial.println("No such command. use: print, stop, dampers, drip, sprink, reg, regeff, otastatus, otarollback, otavalidate, regslot");
     }
  }
}