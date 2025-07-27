#pragma once
#include <Arduino.h>
#include <ModbusMaster.h>

// Pin definitions for RS485
#define RS485_RX_PIN    16 // RO
#define RS485_TX_PIN    17 // DI
#define RS485_DE_RE_PIN 5  // DE + RE pin for RS485 direction control
// A -> Yellow, B -> Blue
#define RS485_BAUD_RATE 4800 // Default baud rate for RS485 communication

class SHTManager_RS485 {
public:
    enum SensorIndex { AMBIANT = 0, BEFORE = 1, AFTER = 2, ROOM = 3, ROOF = 4 };

    SHTManager_RS485(HardwareSerial& serialPort)
        : _serial(serialPort) {
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            _modbus[i] = new ModbusMaster();
            _sensorAddr[i] = i + 1; // Default: 1=Ambiant, 2=Before, 3=After, 4=Room
            _lastTemp[i] = NAN;
            _lastRH[i] = NAN;
            _lastReadMs[i] = 0;
        }
    }

    void begin(long baud = RS485_BAUD_RATE) {
        _serial.begin(baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
        pinMode(RS485_DE_RE_PIN, OUTPUT);
        digitalWrite(RS485_DE_RE_PIN, LOW);
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            _modbus[i]->begin(_sensorAddr[i], _serial);
            _modbus[i]->preTransmission(preTransmission);
            _modbus[i]->postTransmission(postTransmission);
        }
        fillSensorData(); // Initial read to populate data
    }

    void tick() {
        unsigned long now = millis();

        switch (_status) {
            case ReadStatus::IDLE:
                if (now - _lastReadMs[_currentSensorIndex] >= _sensorReadIntervalMs[_currentSensorIndex]) {
                    // It's time to update this sensor
                    _opTimestamp = now;
                    _currentRead = ReadType::TEMP;
                    _status = ReadStatus::WAITING;
                } else {
                    // Not time yet → check next sensor
                    _currentSensorIndex = (_currentSensorIndex + 1) % SENSOR_COUNT;
                }
                break;

            case ReadStatus::WAITING:
                if (now - _opTimestamp >= SENSOR_WAIT_MS) {
                    if (_currentRead == ReadType::TEMP) {
                        float value = readTemperature(static_cast<SensorIndex>(_currentSensorIndex));
                        if (value == NAN) {
                            // If read failed, retry up to MAX_RETRIES
                            if (++_retryCount < MAX_RETRIES) {
                                _opTimestamp = now; // Reset timer for retry
                                return; // Wait for next tick
                            } else {
                                Serial.printf("Failed to read temperature from sensor %d after %d retries\n", _currentSensorIndex, MAX_RETRIES);
                                _retryCount = 0; // Reset retry count
                            }
                        } else {
                            _lastTemp[_currentSensorIndex] = value;
                            _retryCount = 0; // Reset retry count on success
                        }
                        
                        _currentRead = ReadType::HUMID;
                        _opTimestamp = now;  // restart timer before next read
                    }
                    else if (_currentRead == ReadType::HUMID) {
                        float value = readHumidity(static_cast<SensorIndex>(_currentSensorIndex));
                        if(value == NAN) {
                            // If read failed, retry up to MAX_RETRIES
                            if (++_retryCount < MAX_RETRIES) {
                                _opTimestamp = now; // Reset timer for retry
                                return; // Wait for next tick
                            } else {
                                Serial.printf("Failed to read humidity from sensor %d after %d retries\n", _currentSensorIndex, MAX_RETRIES);
                                _retryCount = 0; // Reset retry count
                            }
                        } else {
                            _lastRH[_currentSensorIndex] = value;
                            _retryCount = 0; // Reset retry count on success
                        }
                        // Update last read time
                        _lastReadMs[_currentSensorIndex] = now;

                        _currentSensorIndex = (_currentSensorIndex + 1) % SENSOR_COUNT;
                        _status = ReadStatus::IDLE;
                    }
                }
                break;
        }
    }

    // Fill the sensor data (usealy for first time setup)
    void fillSensorData() {
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            _lastTemp[i] = readTemperature(static_cast<SensorIndex>(i));
            delay(30); // Small delay to avoid flooding the bus
            _lastRH[i] = readHumidity(static_cast<SensorIndex>(i));
            delay(30); // Small delay to avoid flooding the bus
            _lastReadMs[i] = millis();
        }
    }

    // Getters for cached data
    float getAmbiantTemp() { return _lastTemp[AMBIANT]; }
    float getBeforeTemp()  { return _lastTemp[BEFORE]; }
    float getAfterTemp()   { return _lastTemp[AFTER]; }
    float getRoomTemp()    { return _lastTemp[ROOM]; }

    float getAmbiantRH()   { return _lastRH[AMBIANT]; }
    float getBeforeRH()    { return _lastRH[BEFORE]; }
    float getAfterRH()     { return _lastRH[AFTER]; }
    float getRoomRH()      { return _lastRH[ROOM]; }

    // Set Modbus address for each sensor (if you want to change from default)
    void setSensorAddr(SensorIndex idx, uint8_t addr) { _sensorAddr[idx] = addr; }

    // Scan for sensors at a specific baud rate
    void scanRS485(uint32_t baud, HardwareSerial& serial) {
        Serial.printf("Scanning RS485 bus at %lu baud...\n", baud);
        serial.begin(baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
        pinMode(RS485_DE_RE_PIN, OUTPUT);
        digitalWrite(RS485_DE_RE_PIN, LOW);

        ModbusMaster scanner;
        for (uint8_t addr = 1; addr <= 247; ++addr) {
            scanner.begin(addr, serial);
            scanner.preTransmission([](){ digitalWrite(RS485_DE_RE_PIN, HIGH); });
            scanner.postTransmission([](){ digitalWrite(RS485_DE_RE_PIN, LOW); });

            uint8_t result = scanner.readInputRegisters(0x0001, 1); // Try reading temperature
            if (result == ModbusMaster::ku8MBSuccess) {
                int16_t raw = scanner.getResponseBuffer(0);
                float temp = raw / 10.0f;
                Serial.printf("Found sensor at address %d, temp: %.1f°C (baud %lu)\n", addr, temp, baud);
            }
            delay(20);
        }
        Serial.println("Scan complete for this baud.\n");
    }

    // Scan for sensors at all common baud rates
    void scanRS485AllBauds(HardwareSerial& serial) {
        const uint32_t baudRates[] = {2400, 4800, 9600};
        for (uint8_t b = 0; b < sizeof(baudRates)/sizeof(baudRates[0]); ++b) {
            scanRS485(baudRates[b], serial);
        }
    }

    // Set sensor address and baud rate (call for each sensor individually)
    // oldAddr: current address, newAddr: desired address (1-247), baud: 0=2400, 1=4800, 2=9600
    bool setSensorAddressAndBaud(uint8_t oldAddr, uint8_t newAddr, uint16_t baud = 1) {
        ModbusMaster modbus;
        modbus.begin(oldAddr, _serial);
        modbus.preTransmission(preTransmission);
        modbus.postTransmission(postTransmission);

        // Set baud rate
        uint8_t result1 = modbus.writeSingleRegister(0x07D1, baud);
        delay(100);
        // Set new address
        uint8_t result2 = modbus.writeSingleRegister(0x07D0, newAddr);
        delay(100);

        return (result1 == ModbusMaster::ku8MBSuccess) && (result2 == ModbusMaster::ku8MBSuccess);
    }

    void printConfig() {
        for (int i = 0; i < 4; ++i) {
            Serial.printf("Sensor %d address: %d\n", i, _sensorAddr[i]);
        }
    } 

    void printReadings() {
        for (int i = 0; i < 4; ++i) {
            Serial.printf("Sensor %d - Temp: %.1f°C, RH: %.1f%%\n", i, _lastTemp[i], _lastRH[i]);
        }
    }

private:
    static constexpr int SENSOR_COUNT = 5;
    static constexpr int MAX_RETRIES = 3;
    static constexpr unsigned long SENSOR_WAIT_MS = 200; // delay between read attempts
    
    // ==== RS485 and Modbus ====
    HardwareSerial& _serial;
    ModbusMaster* _modbus[SENSOR_COUNT];
    uint8_t _sensorAddr[SENSOR_COUNT];

    // ==== Sensor data ====
    float _lastTemp[SENSOR_COUNT];
    float _lastRH[SENSOR_COUNT];

    // ==== Read timing and scheduling ====
    unsigned long _lastReadMs[SENSOR_COUNT];
    unsigned long _sensorReadIntervalMs[SENSOR_COUNT] = {
        60000, 60000, 60000, 60000, 60000  // Default: 60s per sensor
    };

    // ==== Read state machine ====
    enum class ReadType { TEMP, HUMID };
    enum class ReadStatus { IDLE, WAITING };

    int _currentSensorIndex = 0;
    int _retryCount = 0;
    ReadType _currentRead = ReadType::TEMP;
    ReadStatus _status = ReadStatus::IDLE;
    unsigned long _opTimestamp = 0;

    // ==== Modbus direction control ====
    static void preTransmission() { digitalWrite(RS485_DE_RE_PIN, HIGH); }
    static void postTransmission() { digitalWrite(RS485_DE_RE_PIN, LOW); }

    // ==== Read functions ====
    // Read humidity (returns %RH, float)
    float readHumidity(SensorIndex idx) {
        if (idx < 0 || idx > SENSOR_COUNT) return NAN;
        uint8_t result = _modbus[idx]->readInputRegisters(0x0000, 1);
        if (result == ModbusMaster::ku8MBSuccess) {
            int16_t raw = _modbus[idx]->getResponseBuffer(0);
            return raw / 10.0f;
        }
        return NAN;
    }

    // Read temperature (returns °C, float)
    float readTemperature(SensorIndex idx) {
        if (idx < 0 || idx > SENSOR_COUNT) return NAN;
        uint8_t result = _modbus[idx]->readInputRegisters(0x0001, 1);
        if (result == ModbusMaster::ku8MBSuccess) {
            int16_t raw = _modbus[idx]->getResponseBuffer(0);
            return raw / 10.0f;
        }
        return NAN;
    }
};