#pragma once

#include "WiFi.h"


// for NTP client
#include <NTPClient.h>
#include <WiFiUdp.h>

// GMT +3 for the summer for the winter change to GMT +2
#define time_offset 10800 

class TimeClient{

    // real time clock
// NTP server details
// Define NTP Client to get time
    WiFiUDP ntpUDP;
    NTPClient* ntpClient;
    bool realtime_clk_obtained;

/// @brief initialze the NTP realtime Clock
bool init_NTP_realtime_clk(){
    if (WiFi.status() == WL_CONNECTED)
    {
        // Initialize a NTPClient to get time
        ntpClient->begin();
        //ntpClient->setTimeOffset(time_offset); //remarked due to use of UTC time.
        return ntpClient->isTimeSet();
    }
    else
    {
        return false;
    }
}

public:

    TimeClient(){
        ntpClient = new NTPClient(ntpUDP);
        realtime_clk_obtained = init_NTP_realtime_clk();
    }

    /// @brief update the NTP if possible
    /// @return formated time Y-m-d H:M:S
    String getFormattedTime(){
        
        ntpClient->update();
        // Get the current epoch time
        time_t epochTime = ntpClient->getEpochTime();

        // Convert epoch time to tm structure
        struct tm *ptm = gmtime(&epochTime);

        // Format the time into a readable format
        char timeString[30];
        strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", ptm);

        // Return the formatted time string
        return String(timeString);
    }
    int getHour(){
        ntpClient->update();
        // Get the current epoch time
        time_t epochTime = ntpClient->getEpochTime();

        // Convert epoch time to tm structure
        struct tm *ptm = gmtime(&epochTime);
        
        return ptm->tm_hour;
    }
    int getMinute(){
        ntpClient->update();
        // Get the current epoch time
        time_t epochTime = ntpClient->getEpochTime();

        // Convert epoch time to tm structure
        struct tm *ptm = gmtime(&epochTime);
        
        return ptm->tm_min;
    }

    bool getHasClkObtained(){
        return realtime_clk_obtained;
    }

    // Update the NTP client
    void update(){
        
        ntpClient->update();
    }
    
    unsigned long getEpochTime(){
        return ntpClient->getEpochTime();
    }
};