#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <algorithm>
#include "esp_task_wdt.h"
#include "SHTManager_RS485.h"
#include "TimeClient.h"
#include "S3Log.h"
#include "DallasManager.h"
#include "OTAManager.h"
#include "esp_ota_ops.h"
#include "TimeBudgetManager.h"

// ===========================
// Wifi credentials
// ===========================
const char* WIFI_SSID = "ThermoTera";
// const char* WIFI_SSID = "The Goldenberg’s";
const char* WIFI_PASSWORD = "Thermo2007";
// const char* WIFI_PASSWORD = "0523089060";

// Timezone offset for GMT+3
const long gmtOffset_sec = 3 * 3600; // 3 hours in seconds
const int daylightOffset_sec = 0;    // No daylight saving time adjustment

const String SITE_NAME = "Series1";
const String BUILDING_NAME = "Mavnad2.0";
const String CONTROLLER_TYPE = "Mavnad2.0.Flat";
const String CONTROLLER_LOCATION = "mavnad";
const float FLOAT_NAN = -127;
const String CURRENT_FIRMWARE_VERSION = "1.0.2.61";
const String TOKEN = "pm8z4oxs7awwcx68gwov"; // ein shemer
//const String TOKEN = "8sqfmy0fdvacex3ef0mo"; // asaf

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
const int PIN_PUMP_SPRINKLERS = 10;

const int PUMP_PWM_CHANNEL = 1;
const int PUMP_PWM_FREQ = 1000;     // 1 kHz (safe value within 100–2000 Hz)
const int PUMP_PWM_RESOLUTION = 8;  // 8-bit (0–255)

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

// OneWire setup
DallasManager dallas(15);
// in wall - 28E47446D426677A
// roof top - 28FF641F69E928E9

// Define the relay pins for the air valves
const int airValvesRelayPins[] = { PIN_DAMPER };
HardwareSerial RS485Serial(2); // UART2
SHTManager_RS485 shtSensorsManager(RS485Serial);
S3Log* dataLog;
TimeClient* timeClient;
bool isDataSent = false;

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
WateringMode currentWatereMode = WateringMode::Off;
WateringMode manualWaterMode = WateringMode::Off;
int currentPWMSpeed = 0; // in percentage (0 - 100)
bool debugMode = false;
unsigned long damperActionStartTime = 0;
const unsigned long DAMPER_ACTION_DURATION_MS = 30 * 1000;

void offDrippers();
TimeBudgetManager wateringBudget(2L * 60 * 1000, 0.25L * 60 * 1000, offDrippers);

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
  if(currentWatereMode == WateringMode::On) status += 1;

  status *= 10; // forth digit - dampers status
  if(currentAirMode == AirValveMode::Open) status += 1;

  // in this system there is no way to turn fans outtake (reverse), so no need to check - value always possitive

  return status;
}

void logModeToS3(float value) {
    logToS3("System", "system", "cooler", "mode", value); // "mode" must be in location "cooler"
}

void onFans(int percentage = 70) {
  if(currentPWMSpeed == percentage) return;

  Serial.printf("[Fans] current pwm = %i; new = %i\n", currentPWMSpeed, percentage);
  int duty = map(percentage, 0, 100, 0, 255);
  ledcWrite(FAN_CHANNEL, duty);
  currentPWMSpeed = percentage;
  logModeToS3(getSystemStatusCode());
}

void offFans() {
  debugMessage("off fans");
  if(currentPWMSpeed == 0) return;
  Serial.println("[Fans] off");
  ledcWrite(FAN_CHANNEL, 0);
  currentPWMSpeed = 0;
  logModeToS3(getSystemStatusCode());
}

void onDampers() {
  debugMessage("on dampers");
  if(currentAirMode == AirValveMode::Open) return;
  if (damperState  != DamperState::Idle) return; // Already busy

  Serial.println("[Dampers] open start");

  digitalWrite(PIN_DAMPER_POWER, HIGH);
  delay(100);
  digitalWrite(PIN_DAMPER, HIGH);

  damperActionStartTime = millis();
  damperState = DamperState::Opening;
}

void offDampers(bool forceClose = false) {
  debugMessage("off dampers");
  if(!forceClose && currentAirMode == AirValveMode::Close) return;
  if (damperState  != DamperState::Idle) return; // busy

  Serial.println("[Dampers] close start");

  digitalWrite(PIN_DAMPER_POWER, HIGH);
  damperActionStartTime = millis();
  damperState  = DamperState::Closing;

  // delay(1000 * 12); // 12 seconds until dampers get closed
  // digitalWrite(PIN_DAMPER_POWER, LOW);

  // currentAirMode = AirValveMode::Close;
  // logModeToS3(getSystemStatusCode());
}

void tickDampers() {
  if (damperState == DamperState::Idle) return;

  if (millis() - damperActionStartTime >= DAMPER_ACTION_DURATION_MS) {
    digitalWrite(PIN_DAMPER_POWER, LOW);

    if (damperState == DamperState::Opening) {
      delay(10);
      digitalWrite(PIN_DAMPER, LOW);
      currentAirMode = AirValveMode::Open;
      Serial.println("[Dampers] open complete");
    } else if (damperState == DamperState::Closing) {
      currentAirMode = AirValveMode::Close;
      Serial.println("[Dampers] close complete");
    }

    damperState = DamperState::Idle;
    logModeToS3(getSystemStatusCode());
  }
}


void onSprinklers() {
  debugMessage("on sprinklers");
  if(currentWatereMode == WateringMode::On) return;
  Serial.println("[Sprinklers] on");
  
  digitalWrite(PIN_PUMP_SPRINKLERS, HIGH);
  
  currentWatereMode = WateringMode::On;
  logModeToS3(getSystemStatusCode());
}

void offSprinklers() {
  debugMessage("off sprinklers");
  if(currentWatereMode == WateringMode::Off) return;
  Serial.println("[Sprinklers] off");
  
  digitalWrite(PIN_PUMP_SPRINKLERS, LOW);
  
  currentWatereMode = WateringMode::Off;
  logModeToS3(getSystemStatusCode());
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
  if(currentWatereMode == WateringMode::Off) return;
  Serial.println("[Drippers] off");

  //digitalWrite(PIN_PUMP_DRIPPERS, LOW);
  setDrippersPumpSpeed(0);

  currentWatereMode = WateringMode::Off;
  logModeToS3(getSystemStatusCode());
}

void onDrippers(int percentage = 40) {
  debugMessage("on drippers");
  if (!wateringBudget.isAllowed()) {
    // prints the message only once - the first time the water are on but not allowed. 
    // later the current mode will be already off
    if(currentWatereMode == WateringMode::On) Serial.println("[Drippers] Budget used up → cannot open");
    offDrippers();
    return;
  }
  if(currentWatereMode == WateringMode::On) return;

  Serial.println("[Drippers] on");

  //digitalWrite(PIN_PUMP_DRIPPERS, HIGH);
  setDrippersPumpSpeed(percentage);

  currentWatereMode = WateringMode::On;
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
  float roomRH = shtSensorsManager.getRoomRH();
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


void onCool() {
  debugMessage("on cool");
  float roomTemp = shtSensorsManager.getRoomTemp();
  if(isnan(roomTemp) || roomTemp < START_COOLING_DEG) return;

  AirValveMode airMode = getAirModeByRoom();
  float pwmPercentage = mapFloat(roomTemp, 25.0, 29.0, 20.0, 70.0);
  pwmPercentage = max(20.0f, min(70.0f, pwmPercentage)); // clamp the value

  WateringMode wateringMode = currentWatereMode;
  float beforeRH = shtSensorsManager.getBeforeRH();
  if(!isnan(beforeRH))
    if(beforeRH <= 80) wateringMode = WateringMode::On; 
    else wateringMode = WateringMode::Off;
  
  setSystemMode(airMode, (int)pwmPercentage, wateringMode);

  currentSystemMode = SystemMode::Cool;
}

void onRegenerate() {
  debugMessage("on regenerate");

  // Gets water mode
  WateringMode waterMode = currentWatereMode;
  float beforeRh = shtSensorsManager.getBeforeRH();
  //if(currentWatereMode == WateringMode::Off && shtSensorsManager.getBeforeRH() <= 90) waterMode = WateringMode::On;
  //if(currentWatereMode == WateringMode::On && shtSensorsManager.getBeforeRH() > 95) waterMode = WateringMode::Off;
  if(!isnan(beforeRh))
    if(beforeRh <= 95) waterMode = WateringMode::On;
    else waterMode = WateringMode::Off;

  // Gets air mode
  AirValveMode airMode = getAirModeByRoom();
  
  setSystemMode(airMode, 80, waterMode);

  currentSystemMode = SystemMode::Regenerate;
}

void onHeat() {
  Serial.println("on heat");
  Serial.println("Heat mode is not supported yet");
}

// ========== update mode =============
SystemMode lastSelectedMode = SystemMode::Stop;
void updateSystemMode() {
  if(currentSystemMode == SystemMode::Manual) {
    if(manualWaterMode == WateringMode::On) onDrippers();
    return;
  }

  // gets current time
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[Time] Failed to get local time");
    return;
  }

  uint8_t dow = timeinfo.tm_wday; // 0 = Sunday
  uint16_t currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  SystemMode newMode = SystemMode::Stop;

  // looking for current time schedualing entry
  for (const auto& entry : modeSchedule) {
    if (entry.dayOfWeek != dow) continue;

    uint16_t start = entry.startHour * 60 + entry.startMin;
    uint16_t end   = entry.endHour   * 60 + entry.endMin;

    if (currentMinutes >= start && currentMinutes < end) {
      newMode = entry.mode;
      break; // Stop at first match
    }
  }

  if (newMode != currentSystemMode && lastSelectedMode != newMode) {
    // lastSelectedMode is just to make sure i print this only once
    lastSelectedMode = newMode;
    Serial.printf("[System] Mode changed to %s\n", SystemModeHelper::toString(newMode).c_str());
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
}

bool systemUpdated = false;
void tick() {
  wateringBudget.tick(currentWatereMode == WateringMode::On); // (digitalRead(PIN_PUMP_DRIPPERS));
  tickDampers();

  //int currMin = timeClient->getMinute();
  //if(currMin % 5 == 0 && systemUpdated == false) {
    updateSystemMode();
  //  systemUpdated = true;
  //}
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
  Serial.println("Ver: " + CURRENT_FIRMWARE_VERSION);
  struct tm localTime;
  if(getLocalTime(&localTime)) Serial.println(timeToString(localTime));
  Serial.printf("System mode code %i: Mode = % s; PWM = %i; Water = %s; Dampers = %s\n", 
              getSystemStatusCode(),
              SystemModeHelper::toString(currentSystemMode).c_str(),
              currentPWMSpeed,
              currentWatereMode == WateringMode::On ? "Open" : "Close",
              currentAirMode == AirValveMode::Open ? "Open" : "Close");
  wateringBudget.printStatus("Watering");

  Serial.print("Ambiant: Temp = " + (String)shtSensorsManager.getAmbiantTemp());
  Serial.println("; RH = " + (String)shtSensorsManager.getAmbiantRH());
  Serial.print("Before: Temp = " + (String)shtSensorsManager.getBeforeTemp());
  Serial.println("; RH = " + (String)shtSensorsManager.getBeforeRH());
  Serial.print("After: Temp= " + (String)shtSensorsManager.getAfterTemp());
  Serial.println("; RH = " + (String)shtSensorsManager.getAfterRH());
  Serial.print("Room: Temp = " + (String)shtSensorsManager.getRoomTemp());
  Serial.println("; RH = " + (String)shtSensorsManager.getRoomRH());
  Serial.println("In Wall: Temp = " + (String)dallas.getTemperature("in_wall"));
  Serial.println("Roof Top: Temp = " + (String)dallas.getTemperature("roof_top"));
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
        Serial.println("\n[WiFi] Connected.");
    } else {
        Serial.println("\n[WiFi] Failed to connect. Continuing in offline mode.");
        // Optionally disable WiFi to save power
        WiFi.mode(WIFI_OFF);
    }
}

// The method init NTP server and wait until the builtin 
// RTC will get the current time from the net
struct tm SetupTime() {
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
    Serial.println("NTP configured");

    struct tm timeinfo;
    const int maxRetries = 10;  // Try for 10 seconds max
    int retries = 0;

    while (!getLocalTime(&timeinfo) && retries < maxRetries) {
        Serial.println("Failed to obtain time, retrying...");
        retries++;
        delay(1000);
    }

    if (retries >= maxRetries) {
        Serial.println("ERROR: Unable to set time from NTP. Using default time or RTC fallback.");
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
  otaManager.sendTelemetry("Before_Temp", FloatValidityCheck(shtSensorsManager.getBeforeTemp()));
  otaManager.sendTelemetry("Before_RH", FloatValidityCheck(shtSensorsManager.getBeforeRH()));
  otaManager.sendTelemetry("After_Temp", FloatValidityCheck(shtSensorsManager.getAfterTemp()));
  otaManager.sendTelemetry("After_RH", FloatValidityCheck(shtSensorsManager.getAfterRH()));
  otaManager.sendTelemetry("Ambiant_Temp", FloatValidityCheck(shtSensorsManager.getAmbiantTemp()));
  otaManager.sendTelemetry("Ambiant_RH", FloatValidityCheck(shtSensorsManager.getAmbiantRH()));
  otaManager.sendTelemetry("Room_Temp", FloatValidityCheck(shtSensorsManager.getRoomTemp()));
  otaManager.sendTelemetry("Room_RH", FloatValidityCheck(shtSensorsManager.getRoomRH()));
  otaManager.sendTelemetry("In_Wall_Temp", FloatValidityCheck(dallas.getTemperature("in_wall")));
  otaManager.sendTelemetry("Roof_Top_Temp", FloatValidityCheck(dallas.getTemperature("roof_top")));
  otaManager.sendTelemetry("System_Status", getSystemStatusCode());
  delay(300);

  // S3 server
  logToS3("After", "SHT31", "deg_c", FloatValidityCheck(shtSensorsManager.getAfterTemp())); 
  logToS3("After", "SHT31", "rh", FloatValidityCheck(shtSensorsManager.getAfterRH()));
  logToS3("Before", "SHT31", "deg_c", FloatValidityCheck(shtSensorsManager.getBeforeTemp())); 
  logToS3("Before", "SHT31", "rh", FloatValidityCheck(shtSensorsManager.getBeforeRH()));
  logToS3("Room", "SHT31", "deg_c", FloatValidityCheck(shtSensorsManager.getRoomTemp())); 
  logToS3("Room", "SHT31", "rh", FloatValidityCheck(shtSensorsManager.getRoomRH()));
  logToS3("Ambiant", "SHT31", "deg_c", FloatValidityCheck(shtSensorsManager.getAmbiantTemp())); 
  logToS3("Ambiant", "SHT31", "rh", FloatValidityCheck(shtSensorsManager.getAmbiantRH()));
  logToS3("System", "system", "cooler", "mode", getSystemStatusCode()); // mode data must be in location "cooler"
  logToS3("In_Wall", "DS180B20", "deg_c", FloatValidityCheck(dallas.getTemperature("in_wall"))); 
  logToS3("Roof_Top", "DS180B20", "deg_c", FloatValidityCheck(dallas.getTemperature("roof_top"))); 
  delay(300);
  dataLog->uploadDataFile(SITE_NAME, BUILDING_NAME, "Mavnad1");
}

int getFanSpeed() {
  return currentPWMSpeed;
}

void setFanSpeed(int pwm) {
  currentSystemMode = SystemMode::Manual;
  onFans(pwm);
}

bool getDampersStatus() {
  return currentAirMode == AirValveMode::Open;
}

void setDampersStatus(bool isOpen) {
  currentSystemMode = SystemMode::Manual;
  setDampers(isOpen ? AirValveMode::Open : AirValveMode::Close);
}

bool getSolenoidStatus() {
  return currentWatereMode == WateringMode::On;
}

void setSolenoidStatus(bool status) {
  currentSystemMode = SystemMode::Manual;
  setWater(status ? WateringMode::On : WateringMode::Off);
}

int getWaterSlot() { // minutes
  return wateringBudget.getSlotDurationMs() / 1000 / 60;
}

void setWaterSlot(int duration) {
  currentSystemMode = SystemMode::Manual;
  wateringBudget.setSlotDurationMs(duration * 60 * 1000);
  manualWaterMode = WateringMode::On;
}

int getWaterBudget() { // seconds
  return wateringBudget.getBudgetDurationMs() / 1000;
}

void setWaterBudget(int duration) {
  currentSystemMode = SystemMode::Manual;
  wateringBudget.setBudgetDurationMs(duration * 1000);
  manualWaterMode = WateringMode::On;
}

bool getSystemAutoMode() {
  // any mode that is not Manual is auto
  return currentSystemMode != SystemMode::Manual;
}

void setSystemAutoMode(bool autoMode) {
  currentSystemMode = autoMode ? SystemMode::Stop : SystemMode::Manual;
}

bool getDrippersAutoMode(bool autoMode) {
  // any mode that is not Manual is auto
  return manualWaterMode;
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
  Serial.println("[setup] set relays");
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
  Serial.println("[setup] set mosfets");
  ledcSetup(FAN_CHANNEL, FAN_FREQUENCY, FAN_RESOLUTION);
  for(const auto& fan : fanPins) {
    pinMode(fan, OUTPUT);
    ledcAttachPin(fan, FAN_CHANNEL);
  }
  ledcWrite(FAN_CHANNEL, 0);

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  // Setup SHT sensors manager
  Serial.println("[setup] SHT RS485 sensors manager:");
  shtSensorsManager.setSensorAddr(SHTManager_RS485::AMBIANT, 1);
  shtSensorsManager.setSensorAddr(SHTManager_RS485::BEFORE, 2);
  shtSensorsManager.setSensorAddr(SHTManager_RS485::AFTER, 3);
  shtSensorsManager.setSensorAddr(SHTManager_RS485::ROOM, 4);
  shtSensorsManager.begin(9600); // or 4800 if not yet reconfigured

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  Serial.println("[setup] DS180B20 sensors manager:");
  dallas.begin();
  dallas.scanAndPrint();
  dallas.addSensor("in_wall",  {0x28, 0xE4, 0x74, 0x46, 0xD4, 0x26, 0x67, 0x7A});
  dallas.addSensor("roof_top", {0x28, 0xFF, 0x64, 0x1F, 0x69, 0xE9, 0x28, 0xE9});

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  Serial.println("[Setup] Connecting to WiFi");
  setupWiFi();

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  Serial.println("[Setup] Initializing TimeClient");
  struct tm timeInfo = SetupTime();
  timeClient = new TimeClient();
  if (!getLocalTime(&timeInfo)) Serial.println("[Time] Failed to get local time");
  else timeToString(timeInfo); // print time

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  Serial.print("[Setup] Initializing S3Log ");
  dataLog = new S3Log("/log.txt", timeClient);

  Serial.println("[Setup] Initializing ThingsBoard");
  otaManager.begin();
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

  Serial.println("[setup] setup done successfuly!");

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();

  PrintSensors();
  updateSystemMode();

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();
}

// ==============================================================================
// LOOP
// ==============================================================================
bool is2Minute = false;
void loop() {
  otaManager.tick();
  tick();

  // Every 2 minutes
  if((timeClient->getMinute() % 2) == 0) {
    if(!is2Minute) {
      is2Minute = true;
      PrintSensors();
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

  // I'm still alive - Reset watchdog to prevent timeout
  esp_task_wdt_reset();
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

      // Step 1: Begin at 4800 (default)
      shtSensorsManager.begin(4800);

      // Step 2: Set new address and baud
      bool ok = shtSensorsManager.setSensorAddressAndBaud(oldAddr, newAddr, baudCode);
      if (ok) {
        Serial.println("Sensor address/baud updated successfully.");
      } else {
        Serial.println("Failed to update sensor address/baud.");
      }

      delay(200); // Give sensor time to switch

      // Step 3: Return to 9600 for normal operation
      shtSensorsManager.begin(9600);
    }
    else if (input.equalsIgnoreCase("shtscan")) {  // SHT31 RS485 SCAN ==============================
      Serial.println("Scanning RS485 bus at all common baud rates (2400, 4800, 9600)...");
      shtSensorsManager.scanRS485AllBauds(RS485Serial);
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
    else {
      Serial.println("No such command. use: print, stop, dampers, drip, sprink");
    }
  }
}