#ifndef EXPERIMENT_MANAGER_H
#define EXPERIMENT_MANAGER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

enum ExperimentState {
  IDLE,
  RUNNING,
  PAUSED,
  COMPLETED,
  ERROR
};

struct ExperimentStep {
  String time;           // HH:MM format
  int fanSpeed;          // 0-100%
  int innerFanSpeed;     // 0-100%
  int waterBudget;       // seconds
  bool dampersOpen;      // true = open, false = closed
  String description;    // Optional description of this step
};

struct Experiment {
  String name;
  String description;
  String startDate;      // YYYY-MM-DD format
  String startTime;      // HH:MM format
  int durationHours;
  int logIntervalSeconds;
  ExperimentStep steps[20];  // Fixed size array for steps
  int stepCount;
  ExperimentState state;
  int currentStepIndex;
  unsigned long experimentStartTime;  // Unix timestamp when experiment actually started
  unsigned long lastLogTime;          // Last time we logged sensor data
  String logFilename;                 // CSV file for logging
};

// Forward declaration for external objects and functions
extern SHTManager_RS485 shtRS485Manager;
extern float FloatValidityCheck(float value);

class ExperimentManager {
private:
    static const int MAX_EXPERIMENTS = 10;
    Experiment experiments[MAX_EXPERIMENTS];
    int experimentCount = 0;
    Experiment* currentExperiment = nullptr;
    bool experimentMode = false;

    // Helper functions
    bool parseExperimentsJson(const String& jsonContent) {
      DynamicJsonDocument doc(8192);
      DeserializationError error = deserializeJson(doc, jsonContent);
      
      if (error) {
        Serial.printf("[ExperimentManager] JSON parse failed: %s\n", error.c_str());
        return false;
      }

      JsonArray experimentsArray = doc["experiments"];
      for (JsonObject expObj : experimentsArray) {
        Experiment exp;
        exp.name = expObj["name"] | "";
        exp.description = expObj["description"] | "";
        exp.startDate = expObj["start_date"] | "";
        exp.startTime = expObj["start_time"] | "";
        exp.durationHours = expObj["duration_hours"] | 1;
        exp.logIntervalSeconds = expObj["log_interval_seconds"] | 120;
        exp.state = IDLE;
        exp.currentStepIndex = 0;
        exp.experimentStartTime = 0;
        exp.lastLogTime = 0;
        exp.stepCount = 0;

        JsonArray stepsArray = expObj["steps"];
        for (JsonObject stepObj : stepsArray) {
          if (exp.stepCount >= 20) break;  // Max 20 steps per experiment

          ExperimentStep step;
          step.time = stepObj["time"] | "";
          step.fanSpeed = stepObj["fan_speed"] | 0;
          step.innerFanSpeed = stepObj["inner_fan_speed"] | 0;
          step.waterBudget = stepObj["water_budget"] | 15;
          step.dampersOpen = stepObj["dampers_open"] | true;
          step.description = stepObj["description"] | "";
          exp.steps[exp.stepCount++] = step;
        }

        if (validateExperiment(exp)) {
          if (experimentCount < MAX_EXPERIMENTS) {
            experiments[experimentCount++] = exp;
          } else {
            Serial.println("[ExperimentManager] Maximum number of experiments reached");
          }
        } else {
          Serial.printf("[ExperimentManager] Experiment '%s' failed validation\n", exp.name.c_str());
        }
      }

      return true;
    }

    bool validateExperiment(const Experiment& exp) {
      if (exp.name.isEmpty() || exp.stepCount == 0) {
        return false;
      }

      // Validate date format (YYYY-MM-DD)
      if (exp.startDate.length() != 10 ||
          exp.startDate.charAt(4) != '-' ||
          exp.startDate.charAt(7) != '-') {
        return false;
      }

      // Validate time format (HH:MM)
      if (exp.startTime.length() != 5 ||
          exp.startTime.charAt(2) != ':') {
        return false;
      }

      return true;
    }

    unsigned long dateTimeToTimestamp(const String& date, const String& time) {
      // Parse YYYY-MM-DD HH:MM
      int year = date.substring(0, 4).toInt();
      int month = date.substring(5, 7).toInt();
      int day = date.substring(8, 10).toInt();
      int hour = time.substring(0, 2).toInt();
      int minute = time.substring(3, 5).toInt();

      struct tm timeinfo = {0};
      timeinfo.tm_year = year - 1900;
      timeinfo.tm_mon = month - 1;
      timeinfo.tm_mday = day;
      timeinfo.tm_hour = hour;
      timeinfo.tm_min = minute;
      timeinfo.tm_sec = 0;

      return mktime(&timeinfo);
    }

    unsigned long timeStringToSeconds(const String& timeStr) {
      int colonIndex = timeStr.indexOf(':');
      if (colonIndex == -1) return 0;
      
      int hours = timeStr.substring(0, colonIndex).toInt();
      int minutes = timeStr.substring(colonIndex + 1).toInt();
      
      return hours * 3600 + minutes * 60;
    }

    void createLogFile() {
      if (currentExperiment == nullptr) return;
      
      String filename = "/experiments/" + currentExperiment->name + "_" + 
                       String(currentExperiment->experimentStartTime) + ".csv";
      currentExperiment->logFilename = filename;
      
      File file = LittleFS.open(filename, "w");
      if (file) {
        file.println("timestamp,experiment_name,step_index,event_type,fan_speed,inner_fan_speed,water_budget,dampers_open,room_rh,room_temp,before_rh,before_temp,after_rh,after_temp,ambiant_rh,ambiant_temp,roof_rh,roof_temp");
        file.close();
        Serial.printf("[ExperimentManager] Created log file: %s\n", filename.c_str());
      } else {
        Serial.printf("[ExperimentManager] Failed to create log file: %s\n", filename.c_str());
      }
    }

    void logParameterChange(const String& parameter, int oldValue, int newValue) {
      if (currentExperiment == nullptr || currentExperiment->logFilename.isEmpty()) return;
      
      File file = LittleFS.open(currentExperiment->logFilename, "a");
      if (file) {
        String line = formatTimestamp(getCurrentTimestamp()) + "," +
                     currentExperiment->name + "," +
                     String(currentExperiment->currentStepIndex) + "," +
                     parameter + "," +
                     String(getCurrentFanSpeed()) + "," +
                     String(getCurrentInnerFanSpeed()) + "," +
                     String(getCurrentWaterBudget()) + "," +
                     String(getCurrentDampersState() ? "true" : "false") + "," +
                     "0,0,0,0,0,0,0,0"; // Placeholder for sensor data
        file.println(line);
        file.close();
      }
    }

    String formatSensorDataLine() {
      if (currentExperiment == nullptr) return "";
      
      String line = formatTimestamp(getCurrentTimestamp()) + ",";
      line += currentExperiment->name + ",";
      line += String(currentExperiment->currentStepIndex) + ",";
      line += "sensor_data,";
      line += String(getCurrentFanSpeed()) + ",";
      line += String(getCurrentInnerFanSpeed()) + ",";
      line += String(getCurrentWaterBudget()) + ",";
      line += String(getCurrentDampersState() ? "true" : "false") + ",";
      
      // Add sensor readings
      line += String(FloatValidityCheck(shtRS485Manager.getRoomRH())) + ",";
      line += String(FloatValidityCheck(shtRS485Manager.getRoomTemp())) + ",";
      line += String(FloatValidityCheck(shtRS485Manager.getBeforeRH())) + ",";
      line += String(FloatValidityCheck(shtRS485Manager.getBeforeTemp())) + ",";
      line += String(FloatValidityCheck(shtRS485Manager.getAfterRH())) + ",";
      line += String(FloatValidityCheck(shtRS485Manager.getAfterTemp())) + ",";
      line += String(FloatValidityCheck(shtRS485Manager.getAmbiantRH())) + ",";
      line += String(FloatValidityCheck(shtRS485Manager.getAmbiantTemp())) + ",";
      line += String(FloatValidityCheck(shtRS485Manager.getRoomRH())) + ","; // Using room as roof placeholder
      line += String(FloatValidityCheck(shtRS485Manager.getRoomTemp())); // Using room as roof placeholder
      
      return line;
    }

    unsigned long getCurrentTimestamp() {
      struct tm timeinfo;
      if (!getLocalTime(&timeinfo)) {
        return 0;
      }
      return mktime(&timeinfo);
    }

    String formatTimestamp(unsigned long timestamp) {
      struct tm* timeinfo = localtime((time_t*)&timestamp);
      char buffer[20];
      strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
      return String(buffer);
    }

public:
    // Constructor - loads experiments from LittleFS
    ExperimentManager() {
      // Create experiments directory if it doesn't exist
      if (!LittleFS.exists("/experiments")) {
        LittleFS.mkdir("/experiments");
      }

      // Load experiments from JSON file
      if (LittleFS.exists("/experiments/experiments.json")) {
        File file = LittleFS.open("/experiments/experiments.json", "r");
        if (file) {
          String jsonContent = file.readString();
          file.close();

          if (!parseExperimentsJson(jsonContent)) {
            Serial.println("[ExperimentManager] Failed to parse experiments JSON");
          } else {
            Serial.printf("[ExperimentManager] Loaded %d experiments\n", experimentCount);
          }
        } else {
          Serial.println("[ExperimentManager] Failed to open experiments.json");
        }
      } else {
        Serial.println("[ExperimentManager] experiments.json not found, creating empty manager");
      }
    }

    // Experiment management
    bool startExperiment(const String& experimentName) {
      if (currentExperiment != nullptr) {
        Serial.println("[ExperimentManager] Another experiment is already running");
        return false;
      }

      for (int i = 0; i < experimentCount; i++) {
        if (experiments[i].name == experimentName) {
          currentExperiment = &experiments[i];
          experiments[i].state = RUNNING;
          experiments[i].currentStepIndex = 0;
          experiments[i].experimentStartTime = getCurrentTimestamp();
          experiments[i].lastLogTime = experiments[i].experimentStartTime;

          createLogFile();

          Serial.printf("[ExperimentManager] Started experiment: %s\n", experiments[i].name.c_str());
          logParameterChange("experiment_start", 0, 1);

          experimentMode = true;
          return true;
        }
      }

      Serial.printf("[ExperimentManager] Experiment '%s' not found\n", experimentName.c_str());
      return false;
    }

    void stopExperiment() {
      if (currentExperiment == nullptr) return;

      logParameterChange("experiment_stop", 1, 0);
      Serial.printf("[ExperimentManager] Stopped experiment: %s\n", currentExperiment->name.c_str());

      currentExperiment->state = COMPLETED;
      currentExperiment = nullptr;
      experimentMode = false;
    }

    void pauseExperiment() {
      if (currentExperiment != nullptr && currentExperiment->state == RUNNING) {
        currentExperiment->state = PAUSED;
        Serial.println("[ExperimentManager] Experiment paused");
      }
    }

    void resumeExperiment() {
      if (currentExperiment != nullptr && currentExperiment->state == PAUSED) {
        currentExperiment->state = RUNNING;
        Serial.println("[ExperimentManager] Experiment resumed");
      }
    }

    // Parameter getters
    int getCurrentFanSpeed() {
      if (currentExperiment == nullptr || currentExperiment->state != RUNNING) {
        return 0;
      }

      if (currentExperiment->currentStepIndex >= currentExperiment->stepCount) {
        return currentExperiment->steps[currentExperiment->stepCount - 1].fanSpeed;
      }

      return currentExperiment->steps[currentExperiment->currentStepIndex].fanSpeed;
    }

    int getCurrentInnerFanSpeed() {
      if (currentExperiment == nullptr || currentExperiment->state != RUNNING) {
        return 0;
      }

      if (currentExperiment->currentStepIndex >= currentExperiment->stepCount) {
        return currentExperiment->steps[currentExperiment->stepCount - 1].innerFanSpeed;
      }

      return currentExperiment->steps[currentExperiment->currentStepIndex].innerFanSpeed;
    }

    int getCurrentWaterBudget() {
      if (currentExperiment == nullptr || currentExperiment->state != RUNNING) {
        return 15; // Default
      }

      if (currentExperiment->currentStepIndex >= currentExperiment->stepCount) {
        return currentExperiment->steps[currentExperiment->stepCount - 1].waterBudget;
      }

      return currentExperiment->steps[currentExperiment->currentStepIndex].waterBudget;
    }

    bool getCurrentDampersState() {
      if (currentExperiment == nullptr || currentExperiment->state != RUNNING) {
        return false;
      }

      if (currentExperiment->currentStepIndex >= currentExperiment->stepCount) {
        return currentExperiment->steps[currentExperiment->stepCount - 1].dampersOpen;
      }

      return currentExperiment->steps[currentExperiment->currentStepIndex].dampersOpen;
    }

    bool isExperimentTime() {
      if (currentExperiment != nullptr) {
        return true; // Already running
      }

      unsigned long currentTime = getCurrentTimestamp();
      if (currentTime == 0) return false;

      for (int i = 0; i < experimentCount; i++) {
        if (experiments[i].state == IDLE) {
          unsigned long experimentTime = dateTimeToTimestamp(experiments[i].startDate, experiments[i].startTime);
          if (experimentTime > 0 && currentTime >= experimentTime &&
              currentTime < experimentTime + (experiments[i].durationHours * 3600UL)) {
            // Start this experiment
            startExperiment(experiments[i].name);
            return true;
          }
        }
      }

      return false;
    }

    bool isExperimentRunning() {
      return currentExperiment != nullptr && currentExperiment->state == RUNNING;
    }

    String getStatus() {
      if (currentExperiment == nullptr) {
        return "No experiment running";
      }

      String status = "Experiment: " + currentExperiment->name + "\n";
      status += "State: ";
      switch (currentExperiment->state) {
        case IDLE: status += "Idle"; break;
        case RUNNING: status += "Running"; break;
        case PAUSED: status += "Paused"; break;
        case COMPLETED: status += "Completed"; break;
        case ERROR: status += "Error"; break;
      }
      status += "\n";

      if (currentExperiment->state == RUNNING) {
        status += "Current step: " + String(currentExperiment->currentStepIndex + 1) + "/" +
                  String(currentExperiment->stepCount) + "\n";
        if (currentExperiment->currentStepIndex < currentExperiment->stepCount) {
          status += "Step description: " + currentExperiment->steps[currentExperiment->currentStepIndex].description + "\n";
        }
      }

      return status;
    }

    void listExperiments() {
      Serial.println("Available experiments:");
      for (int i = 0; i < experimentCount; i++) {
        Serial.printf("  %s: %s (%s %s, %dh)\n",
                      experiments[i].name.c_str(),
                      experiments[i].description.c_str(),
                      experiments[i].startDate.c_str(),
                      experiments[i].startTime.c_str(),
                      experiments[i].durationHours);
      }
    }

    void tick() {
      if (currentExperiment == nullptr || currentExperiment->state != RUNNING) {
        return;
      }

      unsigned long currentTime = getCurrentTimestamp();
      if (currentTime == 0) return;

      // Check if experiment should end
      unsigned long experimentDuration = currentTime - currentExperiment->experimentStartTime;
      if (experimentDuration >= (currentExperiment->durationHours * 3600UL)) {
        Serial.println("[ExperimentManager] Experiment completed (duration reached)");
        stopExperiment();
        return;
      }

      // Update current step based on elapsed time
      unsigned long elapsedSeconds = currentTime - currentExperiment->experimentStartTime;

      for (int i = 0; i < currentExperiment->stepCount; i++) {
        unsigned long stepTimeSeconds = timeStringToSeconds(currentExperiment->steps[i].time);
        if (elapsedSeconds >= stepTimeSeconds) {
          if (i != currentExperiment->currentStepIndex) {
            currentExperiment->currentStepIndex = i;
            Serial.printf("[ExperimentManager] Step changed to: %s\n",
                          currentExperiment->steps[i].description.c_str());
            logParameterChange("step_change", currentExperiment->currentStepIndex - 1, currentExperiment->currentStepIndex);
          }
        } else {
          break;
        }
      }

      // Log sensor data at regular intervals
      if (currentTime - currentExperiment->lastLogTime >= currentExperiment->logIntervalSeconds) {
        logExperimentData();
        currentExperiment->lastLogTime = currentTime;
      }
    }

    // Data logging
    void logExperimentData() {
      if (currentExperiment == nullptr || currentExperiment->logFilename.isEmpty()) return;

      String line = formatSensorDataLine();
      if (line.isEmpty()) return;

      File file = LittleFS.open(currentExperiment->logFilename, "a");
      if (file) {
        file.println(line);
        file.close();
      }
    }
};

// Create global instance
ExperimentManager experimentManager;

#endif // EXPERIMENT_MANAGER_H