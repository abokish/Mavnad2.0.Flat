#pragma once
#include <Arduino.h>
#include <Adafruit_SHT31.h>
#include <Wire.h>
#include <vector>
#include <algorithm> // For std::sort

#define SCL_RIGHT 4
#define SDA_RIGHT 5
#define SCL_LEFT 6
#define SDA_LEFT 7

class SHTManager
{
    TwoWire I2C_BUS_LEFT = TwoWire(0);
    TwoWire I2C_BUS_RIGHT = TwoWire(1);

    // Create SHT31 objects
    Adafruit_SHT31* sht31_Before;
    Adafruit_SHT31* sht31_After;
    Adafruit_SHT31* sht31_Ambiant;
    Adafruit_SHT31* sht31_Room;

    bool sht31_Before_isWorking = false;
    bool sht31_After_isWorking = false;
    bool sht31_Ambiant_isWorking = false;
    bool sht31_Room_isWorking = false;

    bool IsRight(int side) { return side == RIGHT; }
    bool IsLeft(int side) { return side == LEFT; }
    String SideName(int side) { return side == RIGHT ? "RIGHT" : "LEFT"; }

    const int READ_DELAY = 20;
    const int  NUM_READINGS = 7;

public:
    const int RIGHT = 1;
    const int LEFT = 0;

    SHTManager() {
        sht31_Before = new Adafruit_SHT31(&I2C_BUS_LEFT);
        sht31_After = new Adafruit_SHT31(&I2C_BUS_LEFT);
        sht31_Ambiant = new Adafruit_SHT31(&I2C_BUS_RIGHT);
        sht31_Room = new Adafruit_SHT31(&I2C_BUS_RIGHT);
    };    

    void setupShtSensors() {
        // Initialize I2C buses
        I2C_BUS_LEFT.begin(SDA_LEFT, SCL_LEFT);
        I2C_BUS_RIGHT.begin(SDA_RIGHT, SCL_RIGHT);
        delay(100); // give sensors time to boot

        // Initialize sensors:
        sht31_Before_isWorking = sht31_Before->begin(0x44); // Before
        delay(100); // give sensors time to boot
        sht31_After_isWorking = sht31_After->begin(0x45); // After
        delay(100); // give sensors time to boot
        sht31_Ambiant_isWorking = sht31_Ambiant->begin(0x44); // Ambiant
        delay(100); // give sensors time to boot
        sht31_Room_isWorking = sht31_Room->begin(0x45); // Room
        delay(100); // give sensors time to boot

        if (sht31_Before_isWorking) { // Address for first SHT31
            Serial.println("Left SHT31 1 sensor has initialized successfuly");
        }
        else {
            Serial.println("Couldn't find LEFT SHT31 1 sensor on the BUS!");
        }

        if (sht31_After_isWorking) {
            Serial.println("Left SHT231 2 sensor has initialized successfuly");
        }
        else {
            Serial.println("Couldn't find LEFT SHT31 2 sensor on the BUS !");
        }

        if (sht31_Ambiant_isWorking) { // Address for second SHT31
            Serial.println("Right SHT31 1 sensor has initialized successfuly");
        }
        else { 
            Serial.println("Couldn't find RIGHT SHT31 1 sensor on the BUS!");
        }

        if (sht31_Room_isWorking) {
            Serial.println("Right SHT31 2 sensor has initialized successfuly");
        }
        else {
            Serial.println("Couldn't find RIGHT SHT31 2 sensor on the BUS !");
        }
    };

    float cleanReading(std::function<float()> readFunc, uint8_t samples = 5) {
        std::vector<float> validReadings;

        for (uint8_t i = 0; i < samples; ++i) {
            float val = readFunc();
            if (!isnan(val)) {
            validReadings.push_back(val);
            }
            delay(10); // Optional: add small delay if needed for sensor stability
        }

        if (validReadings.size() == 0) {
            return NAN; // Nothing valid
        }

        std::sort(validReadings.begin(), validReadings.end());

        size_t n = validReadings.size();

        if (n >= 5) {
            return (validReadings[1] + validReadings[2] + validReadings[3]) / 3.0;
        } else if (n == 4) {
            return (validReadings[1] + validReadings[2]) / 2.0;
        } else if (n == 3) {
            return validReadings[1];
        } else if (n == 2) {
            return (validReadings[0] + validReadings[1]) / 2.0;
        } else {
            return validReadings[0];
        }
    }

    String getShtName(int side, int index) {
        String name = "";

        if(index == 1) {
            if(side == RIGHT) name = "Ambiant";
            else name = "Before";
        } else { // index == 2
            if(side == RIGHT) name = "Room";
            else name = "After";
        }

        return name;
    }

    Adafruit_SHT31* getSht31(int side, int index) {
        Adafruit_SHT31* sht31 = nullptr;

        if(index == 1) {
            if(side == RIGHT) sht31 = sht31_Ambiant;
            else sht31 = sht31_Before;
        } else { // index == 2
            if(side == RIGHT) sht31 = sht31_Room;
            else sht31 = sht31_After;
        }

        return sht31;
    }

    bool getStatus(int side, int index) {
        if (side == RIGHT) {
            if (index == 1) return sht31_Ambiant_isWorking;
            else return sht31_Room_isWorking;
        } else {
            if (index == 1) return sht31_Before_isWorking;
            else return sht31_After_isWorking;
        }
    }

    float getTempSht31(int side, int index) {
        Adafruit_SHT31* sht31 = getSht31(side, index);
        bool isWorking = getStatus(side, index);
        if (sht31 && isWorking) {
            return sht31->readTemperature();
        }
        return NAN; 
    };

    float getRHSht31(int side, int index)
    {
        Adafruit_SHT31* sht31 = getSht31(side, index);
        bool isWorking = getStatus(side, index);
        if (sht31 && isWorking) {
            return sht31->readHumidity();
        }
        return NAN; 
    };

    float getBeforeTemp() {
        float temp = cleanReading([&]() { return sht31_Before->readTemperature(); });
        return temp;
    }

    float getAfterTemp() {
        float temp = cleanReading([&]() { return sht31_After->readTemperature(); });
        return temp;
    }

    float getAmbiantTemp() {
        float temp = cleanReading([&]() { return sht31_Ambiant->readTemperature(); });
        return temp;
    }

    float getRoomTemp() {
        float temp = cleanReading([&]() { return sht31_Room->readTemperature(); });
        return temp;
    }

    float getBeforeRH() {
        float rh = cleanReading([&]() { return sht31_Before->readHumidity(); });
        return rh;
    }

    float getAfterRH() {
        float rh = cleanReading([&]() { return sht31_After->readHumidity(); });
        return rh;
    }

    float getAmbiantRH() {
        float rh = cleanReading([&]() { return sht31_Ambiant->readHumidity(); });
        return rh;
    }

    float getRoomRH() {
        float rh = cleanReading([&]() { return sht31_Room->readHumidity(); });
        return rh;
    }
};