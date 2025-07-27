#pragma once
#include <Arduino.h>
#include <Adafruit_SHT31.h>
#include <Wire.h>

// I2C pin definitions
#define SCL_RIGHT 4
#define SDA_RIGHT 5
#define SCL_LEFT 6
#define SDA_LEFT 7

class SHTManager_I2C {
public:
    // Enum to reference each sensor by logical role
    enum SensorIndex { AMBIANT = 0, BEFORE = 1, AFTER = 2, ROOM = 3 };

    SHTManager_I2C() {
        // Create Adafruit_SHT31 instances, assigning them to the correct I2C bus
        m_sensors[BEFORE]  = new Adafruit_SHT31(&I2C_BUS_LEFT);
        m_sensors[AFTER]   = new Adafruit_SHT31(&I2C_BUS_LEFT);
        m_sensors[AMBIANT] = new Adafruit_SHT31(&I2C_BUS_RIGHT);
        m_sensors[ROOM]    = new Adafruit_SHT31(&I2C_BUS_RIGHT);

        // Initialize sensor data and read timestamps
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            m_temp[i] = NAN;
            m_humid[i] = NAN;
            m_lastReadMs[i] = 0;
        }
    }

    // High-level init: setup I2C and sensors, and populate data
    void begin() {
        setup();            // Setup buses and start sensors
        fillSensorData();   // Read initial temperature + humidity values
    }

    // Non-blocking tick function: progresses one step in the reading process
    void tick() {
        unsigned long now = millis();

        switch (m_status) {
            case ReadStatus::IDLE:
                // Check if it's time to read the current sensor
                if (now - m_lastReadMs[m_currentSensor] >= SENSOR_READ_INTERVAL_MS) {
                    // Start reading temperature first
                    m_readType = ReadType::TEMP;
                    m_status = ReadStatus::WAITING;
                    m_retryCount = 0;
                    m_opTimestamp = now;
                } else {
                    // If not ready, move to the next sensor
                    m_currentSensor = static_cast<SensorIndex>((m_currentSensor + 1) % SENSOR_COUNT);
                }
                break;

            case ReadStatus::WAITING:
                // Wait a short time between reads to avoid I2C timing issues
                if (now - m_opTimestamp >= SENSOR_WAIT_MS) {
                    Adafruit_SHT31* sensor = m_sensors[m_currentSensor];
                    float value = NAN;

                    if (m_readType == ReadType::TEMP) {
                        // Read temperature from sensor
                        value = sensor->readTemperature();
                        if (!isnan(value)) {
                            m_temp[m_currentSensor] = value;
                            m_readType = ReadType::HUMID;  // Move to humidity reading
                            m_retryCount = 0;
                        } else if (++m_retryCount < MAX_RETRIES) {
                            // Retry if reading failed
                            m_opTimestamp = now;
                            return;
                        } else {
                            // Failed after retries
                            //Serial.printf("Failed to read temperature from sensor %d\n", m_currentSensor);
                            m_retryCount = 0;
                            m_readType = ReadType::HUMID;  // Still attempt RH
                        }
                        m_opTimestamp = now;
                    } else if (m_readType == ReadType::HUMID) {
                        // Read humidity from sensor
                        value = sensor->readHumidity();
                        if (!isnan(value)) {
                            m_humid[m_currentSensor] = value;
                        } else if (++m_retryCount < MAX_RETRIES) {
                            m_opTimestamp = now;
                            return;
                        } else {
                            //Serial.printf("Failed to read humidity from sensor %d\n", m_currentSensor);
                        }

                        // Mark this sensor as updated
                        m_lastReadMs[m_currentSensor] = now;
                        m_currentSensor = static_cast<SensorIndex>((m_currentSensor + 1) % SENSOR_COUNT);
                        m_status = ReadStatus::IDLE;
                        m_retryCount = 0;
                    }
                }
                break;
        }
    }

    // Fill all sensor readings immediately (blocking)
    void fillSensorData() {
        for (int i = 0; i < SENSOR_COUNT; ++i) {
            Adafruit_SHT31* sensor = m_sensors[i];

            float temp = sensor->readTemperature();
            delay(50);
            float humid = sensor->readHumidity();
            delay(50);

            m_temp[i] = !isnan(temp) ? temp : NAN;
            m_humid[i] = !isnan(humid) ? humid : NAN;
            m_lastReadMs[i] = millis();

            Serial.printf("Sensor %d boot read - Temp: %.1f°C, RH: %.1f%%\n", i, m_temp[i], m_humid[i]);
        }
    }


    // Public getter methods for each reading
    float getBeforeTemp()   { return m_temp[BEFORE]; }
    float getBeforeRH()     { return m_humid[BEFORE]; }
    float getAfterTemp()    { return m_temp[AFTER]; }
    float getAfterRH()      { return m_humid[AFTER]; }
    float getAmbiantTemp()  { return m_temp[AMBIANT]; }
    float getAmbiantRH()    { return m_humid[AMBIANT]; }
    float getRoomTemp()     { return m_temp[ROOM]; }
    float getRoomRH()       { return m_humid[ROOM]; }

private:
    // ==== Constants and types ====
    static constexpr int SENSOR_COUNT = 4;
    static constexpr int MAX_RETRIES = 3;
    static constexpr unsigned long SENSOR_READ_INTERVAL_MS = 60000;  // 1 minute between reads per sensor
    static constexpr unsigned long SENSOR_WAIT_MS = 200;             // Delay between read attempts

    enum class ReadStatus { IDLE, WAITING };
    enum class ReadType { TEMP, HUMID };

    // ==== I2C buses ====
    TwoWire I2C_BUS_LEFT = TwoWire(0);
    TwoWire I2C_BUS_RIGHT = TwoWire(1);

    // ==== Sensor objects ====
    Adafruit_SHT31* m_sensors[SENSOR_COUNT];

    // ==== Cached sensor data ====
    float m_temp[SENSOR_COUNT];
    float m_humid[SENSOR_COUNT];
    unsigned long m_lastReadMs[SENSOR_COUNT];

    // ==== Tick state machine ====
    SensorIndex m_currentSensor = AMBIANT;
    ReadStatus m_status = ReadStatus::IDLE;
    ReadType m_readType = ReadType::TEMP;
    int m_retryCount = 0;
    unsigned long m_opTimestamp = 0;

    // ==== Setup function ====
    // This function initializes the I2C buses and begins communication with each sensor
    void setup() {
        I2C_BUS_LEFT.begin(SDA_LEFT, SCL_LEFT);
        I2C_BUS_RIGHT.begin(SDA_RIGHT, SCL_RIGHT);
        delay(100);

        // Sensor I2C addresses: 0x44 and 0x45 per bus
        m_sensors[BEFORE]->begin(0x44);
        delay(50);
        m_sensors[AFTER]->begin(0x45);
        delay(50);
        m_sensors[AMBIANT]->begin(0x44);
        delay(50);
        m_sensors[ROOM]->begin(0x45);
        delay(50);
    }
};
