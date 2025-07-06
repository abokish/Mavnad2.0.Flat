#include <Arduino.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "SHTManager.h"
#include "TimeClient.h"
#include "S3Log.h"
#include "DallasManager.h"

// ===========================
// Wifi credentials
// ===========================
const char* WIFI_SSID = "ThermoTera";
const char* WIFI_PASSWORD = "Thermo2007";

// Timezone offset for GMT+3
const long gmtOffset_sec = 3 * 3600; // 3 hours in seconds
const int daylightOffset_sec = 0;    // No daylight saving time adjustment

const String SITE_NAME = "Series1";
const String BUILDING_NAME = "001";
const String CONTROLLER_TYPE = "Mavnad2.0.Flat";
const String CONTROLLER_LOCATION = "mavnad";
const float FLOAT_NAN = -127;

const int PIN_FAN_RIGHT1 = 21;
const int PIN_FAN_RIGHT2 = 47;
const int PIN_FAN_RIGHT3 = 45;
const int PIN_FAN_RIGHT4 = 35;
const int PIN_FAN_LEFT1 = 37;
const int PIN_FAN_LEFT2 = 38;
const int PIN_FAN_LEFT3 = 39;
const int PIN_FAN_LEFT4 = 40;

const int PIN_PUMP = 13;
const int PIN_DAMPER = 12;
const int PIN_DAMPER_POWER = 11;
const int PIN_SELENOID = 10;

const int FAN_CHANNEL = 0;
const int FAN_FREQUENCY = 20000; // PWM frequency in Hz
const int FAN_RESOLUTION = 8;  // 8-bit resolution (0-255)

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

// OneWire setup
DallasManager dallas(15);
// in wall - 28E47446D426677A
// roof top - 28FF641F69E928E9

// Define the relay pins for the air valves
const int airValvesRelayPins[] = { PIN_DAMPER };
SHTManager shtSensorsManager;
S3Log* dataLog;
TimeClient* timeClient;
bool isDataSent = false;

// Define the AirValveMode enum
enum AirValveMode {
  Close,
  Open
};

// Define the possible statuses of the system
enum SystemMode {
  Regenerate,
  Heat,
  Cool,
  Stop
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
  { 5, 10,  0, 16,  00, SystemMode::Cool },      // Friday 10:00–16:00
  { 6, 10,  0, 16,  0, SystemMode::Cool },      // Saturday 10:00–16:00
  { 0,  0,  0,  7,  0, SystemMode::Regenerate }, // Sunday 00:00–07:00
  { 0, 10,  0, 16,  0, SystemMode::Cool },      // Sunday 10:00–16:00
  { 1, 10,  0, 16,  0, SystemMode::Cool },      // Monday 10:00–16:00
  // Add more days as needed
};

struct SystemModeHelper {
  static String toString(SystemMode mode) {
    switch (mode) {
      case SystemMode::Cool:      return "Cool";
      case SystemMode::Heat:      return "Heat";
      case SystemMode::Regenerate: return "Regenerate";
      case SystemMode::Stop:      return "Stop";
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
SystemMode currentSystemMode = SystemMode::Stop;
AirValveMode currentAirMode = AirValveMode::Close;
WateringMode currentWatereMode = WateringMode::Off;
int currentPWMSpeed = 0;
const bool debugMode = false;

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

void onFans(int percentage = 100) {
  debugMessage("on fans");
  if(currentPWMSpeed == percentage) return;
  Serial.println("[Fans] on " + percentage);
  int duty = map(percentage, 0, 100, 0, 255);
  ledcWrite(FAN_CHANNEL, duty);
  currentPWMSpeed = percentage;
}

void offFans() {
  debugMessage("off fans");
  if(currentPWMSpeed == 0) return;
  Serial.println("[Fans] off");
  ledcWrite(FAN_CHANNEL, 0);
  currentPWMSpeed = 0;
}

void onDampers() {
  debugMessage("on dampers");
  if(currentAirMode == AirValveMode::Open) return;
  Serial.println("[Dampers] on");

  digitalWrite(PIN_DAMPER_POWER, HIGH);
  delay(100);
  digitalWrite(PIN_DAMPER, HIGH);

  // Wait until the dampers get closed, and then turn off electricity
  delay(1000 * 12); // 12 seconds
  digitalWrite(PIN_DAMPER_POWER, LOW);
  digitalWrite(PIN_DAMPER, LOW);

  currentAirMode = AirValveMode::Open;
}

void offDampers() {
  debugMessage("off dampers");
  if(currentAirMode == AirValveMode::Close) return;
  Serial.println("[Dampers] off");

  digitalWrite(PIN_DAMPER, LOW);
  delay(1000 * 12); // 12 seconds until dampers get closed
  digitalWrite(PIN_DAMPER_POWER, LOW);

  currentAirMode = AirValveMode::Close;
}

void onPump() {
  debugMessage("on pump");
  if(currentWatereMode == WateringMode::On) return;
  Serial.println("[Pump] on");
  
  digitalWrite(PIN_PUMP, HIGH);
  
  currentWatereMode = WateringMode::On;
}

void offPump() {
  debugMessage("off pump");
  if(currentWatereMode == WateringMode::Off) return;
  Serial.println("[Pump] off");
  
  digitalWrite(PIN_PUMP, LOW);
  
  currentWatereMode = WateringMode::Off;
}

void onSelenoid() {
  debugMessage("on selenoid");
  if(currentWatereMode == WateringMode::On) return;
  Serial.println("[Selenoid] on");
  digitalWrite(PIN_SELENOID, HIGH);
  currentWatereMode = WateringMode::On;
}

void offSelenoid() {
  if(currentWatereMode == WateringMode::Off) return;
  Serial.println("[Selenoid] off");
  digitalWrite(PIN_SELENOID, LOW);
  currentWatereMode = WateringMode::Off;
}

void off() {
  debugMessage("off");
  if(currentSystemMode == SystemMode::Stop) return;

  offFans();
  offSelenoid();
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
      offSelenoid();
      break;
    case WateringMode::On:
      onSelenoid();
      break;
    default:
      offSelenoid();
      break;
  }
}

AirValveMode getAirModeByRoom() {
  debugMessage("get air mode by room");
  AirValveMode airMode = currentAirMode;
  if(currentAirMode == AirValveMode::Open && shtSensorsManager.getRoomRH() > 75) airMode = AirValveMode::Close;
  if(currentAirMode == AirValveMode::Close && shtSensorsManager.getRoomRH() <= 60) airMode = AirValveMode::Open;
  return airMode;
}

void setSystemMode(AirValveMode airMode, int fanPercentage = 100, WateringMode wateringMode = WateringMode::Off) {
  debugMessage("set system mode");
  setWater(wateringMode);
  onFans(fanPercentage);
  setDampers(airMode);
}

void onCool() {
  debugMessage("on cool");
  AirValveMode airMode = getAirModeByRoom();
  setSystemMode(airMode, 80, WateringMode::Off);

  currentSystemMode = SystemMode::Cool;
}

void onRegenerate() {
  debugMessage("on regenerate");

  WateringMode waterMode = currentWatereMode;
  if(currentWatereMode == WateringMode::Off && shtSensorsManager.getBeforeRH() <= 85) waterMode = WateringMode::On;
  if(currentWatereMode == WateringMode::On && shtSensorsManager.getBeforeRH() > 90) waterMode = WateringMode::Off;

  AirValveMode airMode = getAirModeByRoom();
  
  setSystemMode(airMode, 80, waterMode);

  currentSystemMode = SystemMode::Regenerate;
}

void onHeat() {
  Serial.println("on heat");
  Serial.println("Heat mode is not supported yet");
}

// ========== update mode =============
void updateSystemMode() {
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

  if (newMode != currentSystemMode) {
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

void tick() {
  updateSystemMode();
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

  status *= 10; // second digit - speed (convert precenteg to a number between 0 to 3)
  status += map(currentPWMSpeed, 0, 100, 0, 3);

  status *= 10; // third digit - water status
  if(currentWatereMode == WateringMode::On) status += 1;

  status *= 10; // forth digit - dampers status
  if(currentAirMode == AirValveMode::Open) status += 1;

  // in this system there is no way to turn fans outtake (reverse), so no need to check - value always possitive

  return status;
}

void PrintSensors(){
  Serial.println("--------------------------------------------");
  Serial.print(shtSensorsManager.getShtName(shtSensorsManager.LEFT, 1) + ": Temp = " + (String)shtSensorsManager.getTempSht31(shtSensorsManager.LEFT, 1));
  Serial.println("; RH = " + (String)shtSensorsManager.getRHSht31(shtSensorsManager.LEFT, 1));
  Serial.print(shtSensorsManager.getShtName(shtSensorsManager.LEFT, 2) + ": Temp= " + (String)shtSensorsManager.getTempSht31(shtSensorsManager.LEFT, 2));
  Serial.println("; RH = " + (String)shtSensorsManager.getRHSht31(shtSensorsManager.LEFT, 2));
  Serial.print(shtSensorsManager.getShtName(shtSensorsManager.RIGHT, 1) + ": Temp = " + (String)shtSensorsManager.getTempSht31(shtSensorsManager.RIGHT, 1));
  Serial.println("; RH = " + (String)shtSensorsManager.getRHSht31(shtSensorsManager.RIGHT, 1));
  Serial.print(shtSensorsManager.getShtName(shtSensorsManager.RIGHT, 2) + ": Temp = " + (String)shtSensorsManager.getTempSht31(shtSensorsManager.RIGHT, 2));
  Serial.println("; RH = " + (String)shtSensorsManager.getRHSht31(shtSensorsManager.RIGHT, 2));
  Serial.println("In Wall: Temp = " + (String)dallas.getTemperature("in_wall"));
  Serial.println("Roof Top: Temp = " + (String)dallas.getTemperature("roof_top"));
  Serial.println("--------------------------------------------");
}

void setupWiFi(unsigned long timeoutMs) {
    Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
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
struct tm SetupTime(){
    // Initialize NTP
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
    Serial.println("NTP configured");

    // Wait until the time is set
    struct tm timeinfo;
    while (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        delay(1000);
    }
    Serial.println("Time successfully set!");
    return timeinfo;
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

SystemMode determineSystemMode(const std::vector<ScheduleEntry>& schedule, const tm& timeinfo) {
  uint8_t currentDOW = timeinfo.tm_wday; // 0=Sunday, 1=Monday, etc.
  uint16_t currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  for (const auto& entry : schedule) {
    if (entry.dayOfWeek != currentDOW) continue;

    uint16_t start = entry.startHour * 60 + entry.startMin;
    uint16_t end = entry.endHour * 60 + entry.endMin;

    if (currentMinutes >= start && currentMinutes < end) {
      return entry.mode;
    }
  }

  return SystemMode::Stop; // fallback if no match
}




void HandleManualControl();

// ==============================================================================
// SETUP
// ==============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[setup] Welcome to the ThermoTerra whether Controller!");

  // Relays
  Serial.println("[setup] set relays");
  pinMode(PIN_DAMPER, OUTPUT);
  pinMode(PIN_DAMPER_POWER, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_SELENOID, OUTPUT);
  digitalWrite(PIN_DAMPER_POWER, LOW);
  digitalWrite(PIN_DAMPER, LOW);
  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_SELENOID, LOW);

  // Mosfet
  Serial.println("[setup] set mosfets");
  ledcSetup(FAN_CHANNEL, FAN_FREQUENCY, FAN_RESOLUTION);
  for(const auto& fan : fanPins) {
    pinMode(fan, OUTPUT);
    ledcAttachPin(fan, FAN_CHANNEL);
  }
  ledcWrite(FAN_CHANNEL, 0);

  // Setup SHT sensors manager
  Serial.println("[setup] SHT sensors manager:");
  shtSensorsManager.setupShtSensors();

  Serial.println("[setup] DS180B20 sensors manager:");
  dallas.begin();
  dallas.scanAndPrint();

  dallas.addSensor("in_wall",  {0x28, 0xE4, 0x74, 0x46, 0xD4, 0x26, 0x67, 0x7A});
  dallas.addSensor("roof_top", {0x28, 0xFF, 0x64, 0x1F, 0x69, 0xE9, 0x28, 0xE9});


  Serial.println("[Setup] Connecting to WiFi");
  setupWiFi(10000);

  Serial.println("[Setup] Initializing TimeClient");
  struct tm timeInfo = SetupTime();
  timeClient = new TimeClient();
  if (!getLocalTime(&timeInfo)) Serial.println("[Time] Failed to get local time");
  else Serial.printf("[Time] %02d.%02d.%d %02d:%02d:%02d\n", 
        timeInfo.tm_mday, 
        timeInfo.tm_mon + 1, 
        timeInfo.tm_year + 1900, 
        timeInfo.tm_hour, 
        timeInfo.tm_min,
        timeInfo.tm_sec);

  Serial.print("[Setup] Initializing S3Log ");
  dataLog = new S3Log("/log.txt", timeClient);

  Serial.println("[setup] setup done successfuly!");

  PrintSensors();
}

void sendTelemetry() {
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

// ==============================================================================
// LOOP
// ==============================================================================
void loop() {
  tick();

  // Every 10 minutes
  if((timeClient->getMinute() % 10) == 0) {
    if(!isDataSent) {
      isDataSent = true;
      PrintSensors();
      sendTelemetry();
    }
  }
  else {
    isDataSent = false;
  }

  HandleManualControl();
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
    else if (input.equalsIgnoreCase("pump")) {  // PUMP ==============================
      Serial.println("Enter state (1 = Open, 0 = Close): ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String state = Serial.readStringUntil('\n');
        state.trim();
        state.toInt() ? onPump() : offPump();
    }
    else if (input.equalsIgnoreCase("selenoid")) {  // SELENOID ==============================
      Serial.println("Enter state (1 = Open, 0 = Close): ");
        while (!Serial.available()) {
            delay(10); // Wait for input
        }
        String state = Serial.readStringUntil('\n');
        state.trim();
        state.toInt() ? onSelenoid() : offSelenoid();
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
    else {
      Serial.println("No such command. use: print, stop, dampers, pump");
    }
  }
}