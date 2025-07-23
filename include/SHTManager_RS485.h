#pragma once
#include <Arduino.h>
#include <ModbusMaster.h>

// Pin definitions for RS485 (adjust as needed)
#define RS485_RX_PIN 16
#define RS485_TX_PIN 17
#define RS485_DE_RE_PIN 4

class SHTManager_RS485 {
public:
    enum SensorIndex { AMBIANT = 0, BEFORE = 1, AFTER = 2, ROOM = 3 };

    SHTManager_RS485(HardwareSerial& serialPort)
        : _serial(serialPort) {
        for (int i = 0; i < 4; ++i) {
            _modbus[i] = new ModbusMaster();
            _sensorAddr[i] = i + 1; // Default: 1=Ambiant, 2=Before, 3=After, 4=Room
        }
    }

    void begin(long baud = 9600) {
        _serial.begin(baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
        pinMode(RS485_DE_RE_PIN, OUTPUT);
        digitalWrite(RS485_DE_RE_PIN, LOW);
        for (int i = 0; i < 4; ++i) {
            _modbus[i]->begin(_sensorAddr[i], _serial);
            _modbus[i]->preTransmission(preTransmission);
            _modbus[i]->postTransmission(postTransmission);
        }
    }

    // Set sensor address and baud rate (call for each sensor individually)
    // oldAddr: current address, newAddr: desired address (1-247), baud: 0=2400, 1=4800, 2=9600
    bool setSensorAddressAndBaud(uint8_t oldAddr, uint8_t newAddr, uint16_t baud = 2) {
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

    // Read humidity (returns %RH, float)
    float readHumidity(SensorIndex idx) {
        if (idx < 0 || idx > 3) return NAN;
        uint8_t result = _modbus[idx]->readInputRegisters(0x0000, 1);
        if (result == ModbusMaster::ku8MBSuccess) {
            int16_t raw = _modbus[idx]->getResponseBuffer(0);
            return raw / 10.0f;
        }
        return NAN;
    }

    // Read temperature (returns °C, float)
    float readTemperature(SensorIndex idx) {
        if (idx < 0 || idx > 3) return NAN;
        uint8_t result = _modbus[idx]->readInputRegisters(0x0001, 1);
        if (result == ModbusMaster::ku8MBSuccess) {
            int16_t raw = _modbus[idx]->getResponseBuffer(0);
            return raw / 10.0f;
        }
        return NAN;
    }

    // Named accessors
    float getAmbiantTemp() { return readTemperature(AMBIANT); }
    float getBeforeTemp()  { return readTemperature(BEFORE); }
    float getAfterTemp()   { return readTemperature(AFTER); }
    float getRoomTemp()    { return readTemperature(ROOM); }

    float getAmbiantRH()   { return readHumidity(AMBIANT); }
    float getBeforeRH()    { return readHumidity(BEFORE); }
    float getAfterRH()     { return readHumidity(AFTER); }
    float getRoomRH()      { return readHumidity(ROOM); }

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

private:
    HardwareSerial& _serial;
    ModbusMaster* _modbus[4];
    uint8_t _sensorAddr[4];

    static void preTransmission() { digitalWrite(RS485_DE_RE_PIN, HIGH); }
    static void postTransmission() { digitalWrite(RS485_DE_RE_PIN, LOW); }
};