#pragma once

#include <OneWire.h>
#include <DallasTemperature.h>
#include <map>
#include <array>

class DallasManager {
public:
  using DeviceAddress = std::array<uint8_t, 8>;

  DallasManager(uint8_t pin) : m_oneWire(pin), m_sensors(&m_oneWire) {}

  void begin() {
    Serial.println("[DallasManager] Initializing sensors...");
    m_sensors.begin();
    Serial.printf("[DallasManager] Found %d sensors on bus.\n", m_sensors.getDeviceCount());
  }

  void addSensor(const String& name, const DeviceAddress& addr) {
    m_sensorsMap[name] = addr;
  }

  float getTemperature(const String& name) {
    auto it = m_sensorsMap.find(name);
    if (it == m_sensorsMap.end()) {
      Serial.printf("[DallasManager] Sensor '%s' not found!\n", name.c_str());
      return NAN;
    }
    const DeviceAddress& addr = it->second;
    m_sensors.requestTemperaturesByAddress(addr.data());
    float temp = m_sensors.getTempC(addr.data());
    if (isnan(temp)) {
      Serial.printf("[DallasManager] Failed to read '%s'\n", name.c_str());
    }
    return temp;
  }

  void scanAndPrint() {
    Serial.println("[DallasManager] Scanning OneWire bus...");
    DeviceAddress addr;
    int count = 0;

    m_oneWire.reset_search();
    while (m_oneWire.search(addr.data())) {
      Serial.printf("Sensor %d Address: ", count++);
      printAddress(addr);
      Serial.println();
    }
    if (count == 0) {
      Serial.println("No DS18B20 devices found.");
    }
  }

private:
  void printAddress(const DeviceAddress& addr) {
    for (uint8_t i = 0; i < 8; i++) {
      if (addr[i] < 16) Serial.print("0");
      Serial.print(addr[i], HEX);
    }
  }

  OneWire m_oneWire;
  DallasTemperature m_sensors;
  std::map<String, DeviceAddress> m_sensorsMap;
};
