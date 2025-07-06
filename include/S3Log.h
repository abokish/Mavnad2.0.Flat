#pragma once

#include <LittleFS.h>
#include <string.h>
#include <FS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "TimeClient.h"
#include "execute-api.eu-north-1.amazonaws.com.h"

// aws configuration
const String serverName = "4h3tw3mv94.execute-api.eu-north-1.amazonaws.com";
const String bucketPath = "/dev/thermoterra/";
const int PORT = 443;
const int BUFFER_SIZE = 1024;
const int TIMEOUT_TIMER = 10000;
const int WIFI_TIMEOUT_TIMER = 10000;

class S3Log{

    String path;
    TimeClient* timeClient;
    WiFiClientSecure ssll_client;

    void appendToFile(const char *message) {
        Serial.printf("Appending to file: %s\n", path);

        File file = LittleFS.open(path, FILE_APPEND);
        if (!file)
        {
            Serial.println("- failed to open file for appending");
            return;
        }
        if (file.print(message))
        {
            Serial.println("- message appended");
        }
        else
        {
            Serial.println("- append failed");
        }
        file.close();
    }
    void deleteFile(){
        Serial.printf("Deleting file: %s\n", path.c_str());
        if (LittleFS.remove(path.c_str()))
        {
            Serial.printf("file %s deleted.\n", path.c_str());
        }
        else
        {
            Serial.printf("file %s delete failed.\n", path.c_str());
        }
    }

public:

    S3Log(const String filePath,TimeClient* timeClient)
        : path(filePath), timeClient(timeClient)
    {
        ssll_client.setCACert(amazonaws_ca);

        // begin the SPIFF
        if (!LittleFS.begin(true)) { // 'true' formats the LittleFS if it is not formatted
            Serial.println("Failed to mount LittleFS. Formatting...");
        } else {
            Serial.println("LittleFS mounted successfully.");
        }
    }

    void appendToLog(const char *message){
        //Serial.printf("Appending line '%s'to file: %s\r\n", message, path.c_str());
        File file = LittleFS.open(path, FILE_APPEND);
        if (!file)
        {
            Serial.println("- failed to open file for appending");
        } else  {
            if (file.print(message)){
                //Serial.println("- message appended");
            }else {
                Serial.println("- append failed");
            }
        }
        file.close();
    }

    void appendToLogFormatted(
    const String& timestamp,
    const String& site,
    const String& building,
    const String& controllerType,
    const String& controllerLocation,
    const String& sensorName,
    const String& sensorType,
    const String& unit,
    float value
) {
    String message = timestamp + "," + 
                     site + "," + 
                     building + "," + 
                     controllerType + "," + 
                     controllerLocation + "," +
                     sensorName + "," + 
                     sensorType + "," + 
                     unit + "," + 
                     String(value) + "\n";
                     
    appendToLog(message.c_str());
}


    size_t getLogFileSize(){
        size_t size=1;
        File file = LittleFS.open(path, FILE_READ);
        if (!file){
             Serial.println("- failed to open file for appending");
        }else{
            size = file.size();
        }
        file.close();

        return size;
    }

    String getLogFileText(){
        String fileContent;
        File file = LittleFS.open(path, FILE_READ);
        if (!file)
        {
            Serial.println("failed to open log file for reading");
        } else {
            fileContent = file.readString();
        }
        file.close();
        return fileContent;
    }

    void deleteLogFile(){
        Serial.printf("Deleting file: %s\n", path.c_str());
        if (LittleFS.remove(path))
        {
            Serial.printf("File %s deleted\n",path.c_str());
        }
        else
        {
            Serial.printf("file %s Delete failed\n",path.c_str());
        }
    }

    /// @brief Returns an s3 folder to be used as an Athena partition based on config.
    String getPartitionFolderForS3(String dt_string, String site, String building){
        String folder = "logs%2F";
        folder.concat("site%3D" + site + "%2F");
        folder.concat("building%3D" + building + "%2F");
        folder.concat("dt%3D" + dt_string + "%2F");
        return folder;
    }

    void handleFileReadAndUpload(File &file) {
        // set buffer in the RAM
        uint8_t buf[BUFFER_SIZE];
        size_t fbLen = file.size();
        // iterate on the file
        for (size_t n = 0; n < fbLen; n += BUFFER_SIZE)
        {
            size_t currReadSize = BUFFER_SIZE;
            if (n + BUFFER_SIZE >= fbLen)
            {
                currReadSize = fbLen % BUFFER_SIZE;
            }
            file.read(buf, currReadSize);
            ssll_client.write(buf, currReadSize);
        }
    }

    String waitForServerResponse(String &getAll, boolean &state, unsigned long startTimer, int timeoutTimer) {
        String getBody = "";
        while ((startTimer + timeoutTimer) > millis())
        {
            delay(100);
            while (ssll_client.available())
            {
                char c = (char)ssll_client.read();
                if (c == '\n')
                {
                    if (getAll.length() == 0)
                    {
                        state = true;
                    }
                    getAll = "";
                }
                else if (c != '\r')
                {
                    getAll += String(c);
                }
                if (state)
                {
                    getBody += String(c);
                }
                startTimer = millis();
            }
            if (getBody.length() > 0)
            {
                break;
            }
        }
        return getBody;
    }

    /// @brief handles the HTTPS to the AWS S3 bucket
    void uploadDataFile(String site, String building, String espName){  
        String ts_string = timeClient->getFormattedTime();
        String s3_folder = getPartitionFolderForS3(ts_string.substring(0, 10), site, building);
        String esp_name = espName;
        String file_name = s3_folder + "log_" + esp_name + "_" + ts_string + ".csv";
        file_name.replace(' ', '_'); // Replace spaces with underscores

        //Serial.printf("[HTTPS] begin... - %s%s%s", serverName.c_str(), bucketPath.c_str(), file_name.c_str());
        //Serial.printf("Connecting to server: %s", serverName.c_str());

        String getAll;
        String getBody;

        // ssll_client.setInsecure();
        Serial.println("S3 Connection successful!");
        File file = LittleFS.open(path);

        if (!file){
            Serial.print(path);
            Serial.println(" - Failed to open file for reading");
            return;
        }
        uint16_t fileLen = file.size();
        if (fileLen==0) {
            Serial.println("size of file to upload is 0 - ABORTING!!!");
            file.close();
            return;
        }else{
            //Serial.printf("size of file to upload - %d", fileLen);
        }

        bool sendOK = false; //used to decide if file can be deleted
        if (ssll_client.connect(serverName.c_str(), PORT))
        {
            // standard request in HTTP/1.1
            ssll_client.println("PUT " + bucketPath + file_name + " HTTP/1.1");
            ssll_client.println("Host: " + serverName);
            ssll_client.println("Content-Length: " + String(fileLen));
            ssll_client.println("Content-Type: text/plain");
            ssll_client.println();
            //TODO: do not keep file open. give the handleFileReadAndUpload function the file name 
            //      and let it open and close it immediately after send is done. consider renaming 
            //      the file so the other processes could write to the fresh file in the meantime.
            handleFileReadAndUpload(file);
            file.close();
            Serial.println("file uploaded");
            int timeoutTimer = TIMEOUT_TIMER;
            unsigned long startTimer = millis();
            boolean state = false;
            //Serial.printf("waiting for ssl client's response");
            getBody = waitForServerResponse(getAll, state, startTimer, timeoutTimer);
            ssll_client.stop();
            if (getBody.length() > 0){ //something unusual happend.
                Serial.printf("body: %s\n", getBody.c_str());
            }
            Serial.println("finished");
            sendOK=true;

        }else{
            file.close();
            getBody = "Connection to " + serverName + " failed.";
            Serial.printf("\n%s\n", getBody.c_str());

        }

        if (sendOK) {
            deleteLogFile();
        }
    }
};