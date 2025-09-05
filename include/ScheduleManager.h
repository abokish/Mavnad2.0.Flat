#pragma once

#include <vector>
#include <Arduino.h>
#include "TimeClient.h"
#include "SystemTypes.h"

// Forward declarations
class TimeClient;

// ScheduleManager class to handle schedule operations and fallback when sensors are unavailable
class ScheduleManager {
public:
  struct ScheduleEntry {
    uint8_t dayOfWeek;  // 0 = Sunday, 1 = Monday, ... 6 = Saturday
    uint8_t startHour;  // 24h format
    uint8_t startMin;
    uint8_t endHour;
    uint8_t endMin;
    SystemMode mode;
    int startFanSpeed;      // Fan speed at start of period (0-100)
    int endFanSpeed;        // Fan speed at end of period (0-100), -1 for constant speed
    int innerFanSpeed;      // Inner fan speed (0-100), -1 for auto/disabled
    int drippersBudgetSeconds; // Drippers budget in seconds, -1 for default
  };

private:
  std::vector<ScheduleEntry> schedule;
  TimeClient* timeClient;
  
public:
  ScheduleManager(const std::vector<ScheduleEntry>& sched, TimeClient* tc) : schedule(sched), timeClient(tc) {}
  
  // Get current schedule entry based on time
  const ScheduleEntry* getCurrentEntry() {
    if (!timeClient) return nullptr;
    
    uint8_t currentDay = timeClient->getDayOfWeek();
    uint8_t currentHour = timeClient->getHour();
    uint8_t currentMin = timeClient->getMinute();
    
    // Convert current time to minutes for easier comparison
    int currentTimeMinutes = currentHour * 60 + currentMin;
    
    for (const auto& entry : schedule) {
      if (entry.dayOfWeek != currentDay) continue;
      
      int startTimeMinutes = entry.startHour * 60 + entry.startMin;
      int endTimeMinutes = entry.endHour * 60 + entry.endMin;
      
      // Handle overnight periods (e.g., 23:00 to 07:00)
      if (startTimeMinutes > endTimeMinutes) {
        if (currentTimeMinutes >= startTimeMinutes || currentTimeMinutes <= endTimeMinutes) {
          return &entry;
        }
      } else {
        // Normal period (e.g., 07:00 to 19:00)
        if (currentTimeMinutes >= startTimeMinutes && currentTimeMinutes <= endTimeMinutes) {
          return &entry;
        }
      }
    }
    
    return nullptr; // No matching entry found
  }
  
  // Get fan speed for current time (with interpolation)
  int getCurrentFanSpeed() {
    const ScheduleEntry* entry = getCurrentEntry();
    if (!entry) return 70; // Default fallback
    
    if (entry->startFanSpeed == -1 || entry->endFanSpeed == -1) {
      return 70; // Default for Cool mode or invalid entries
    }
    
    if (entry->startFanSpeed == entry->endFanSpeed) {
      return entry->startFanSpeed; // Constant speed
    }
    
    // Calculate interpolated fan speed
    return calculateFanSpeedForScheduleEntry(*entry);
  }
  
  // Get inner fan speed for current time
  int getCurrentInnerFanSpeed() {
    const ScheduleEntry* entry = getCurrentEntry();
    if (!entry) return -1; // Auto/disabled
    
    return entry->innerFanSpeed;
  }
  
  // Get drippers budget for current time
  int getCurrentDrippersBudgetSeconds() {
    const ScheduleEntry* entry = getCurrentEntry();
    if (!entry) return 3600; // Default 3600 seconds (1 hour)
    
    return entry->drippersBudgetSeconds;
  }
  
  // Get air mode for current time
  AirValveMode getCurrentAirMode() {
    const ScheduleEntry* entry = getCurrentEntry();
    if (!entry) return AirValveMode::Close; // Default
    
    // Cool mode typically uses open dampers, Regenerate uses open dampers
    if (entry->mode == SystemMode::Cool || entry->mode == SystemMode::Regenerate) {
      return AirValveMode::Open;
    }
    
    return AirValveMode::Close;
  }
  
  // Get system mode for current time
  SystemMode getCurrentSystemMode() {
    const ScheduleEntry* entry = getCurrentEntry();
    if (!entry) return SystemMode::Stop; // Default
    
    return entry->mode;
  }
  
  // Check if sensors are needed for current mode
  bool areSensorsRequired() {
    const ScheduleEntry* entry = getCurrentEntry();
    if (!entry) return true; // Default to requiring sensors
    
    // Cool and Regenerate modes can work with schedule fallback
    return (entry->mode != SystemMode::Cool && entry->mode != SystemMode::Regenerate);
  }
  
  // Get debug info about current schedule
  String getCurrentScheduleInfo() {
    const ScheduleEntry* entry = getCurrentEntry();
    if (!entry) return "No schedule entry found";
    
    String info = "Schedule: ";
    info += "Day=" + String(entry->dayOfWeek);
    info += ", Time=" + String(entry->startHour) + ":" + String(entry->startMin) + "-" + 
            String(entry->endHour) + ":" + String(entry->endMin);
    info += ", Mode=" + String((int)entry->mode);
    info += ", Fan=" + String(entry->startFanSpeed) + "-" + String(entry->endFanSpeed);
    info += ", Inner=" + String(entry->innerFanSpeed);
    info += ", Drippers=" + String(entry->drippersBudgetSeconds) + "s";
    
    return info;
  }
  
  // Public method to calculate fan speed for a given entry (for external use)
  int calculateFanSpeedForEntry(const ScheduleEntry& entry) {
    return calculateFanSpeedForScheduleEntry(entry);
  }

private:
  // Calculate fan speed based on current time within a schedule entry
  int calculateFanSpeedForScheduleEntry(const ScheduleEntry& entry) {
    // If both fan speeds are -1, return -1 (no fan speed control for this mode)
    if (entry.startFanSpeed == -1 && entry.endFanSpeed == -1) {
      return -1;
    }
    
    // If endFanSpeed is -1, use constant startFanSpeed
    if (entry.endFanSpeed == -1) {
      return entry.startFanSpeed;
    }
    
    // Get current time
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      return entry.startFanSpeed; // Fallback to start speed
    }
    
    // Calculate current time in minutes since midnight
    uint16_t currentMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
    
    // Calculate start and end times in minutes since midnight
    uint16_t startMinutes = entry.startHour * 60 + entry.startMin;
    uint16_t endMinutes = entry.endHour * 60 + entry.endMin;
    
    // Handle overnight periods (e.g., 23:00 to 06:00)
    if (endMinutes < startMinutes) {
      endMinutes += 24 * 60; // Add 24 hours worth of minutes
      if (currentMinutes < startMinutes) {
        currentMinutes += 24 * 60; // Add 24 hours worth of minutes
      }
    }
    
    // Calculate progress through the time period (0.0 to 1.0)
    float progress = (float)(currentMinutes - startMinutes) / (float)(endMinutes - startMinutes);
    progress = max(0.0f, min(1.0f, progress)); // Clamp between 0 and 1
    
    // Interpolate between start and end fan speeds
    int fanSpeed = entry.startFanSpeed + (int)((entry.endFanSpeed - entry.startFanSpeed) * progress);
    
    return fanSpeed;
  }
};
