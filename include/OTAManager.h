#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "mbedtls/sha256.h"
#include "mbedtls/md.h"  // For SHA256 checksum
#include "esp_ota_ops.h"

class OTAManager {
public:
  const String THINGSBOARD_SERVER = "thingsboard.cloud";
  std::function<int()> getFanSpeedFunc;
  std::function<void(int)> setFanSpeedFunc;
  std::function<bool()> getDampersStatusFunc;
  std::function<void(bool)> setDampersStatusFunc;
  std::function<bool()> getSolenoidStatusFunc;
  std::function<void(bool)> setSolenoidStatusFunc;
  std::function<int()> getWaterSlotFunc;
  std::function<void(int)> setWaterSlotFunc;
  std::function<void()> restartDeviceFunc;
  std::function<int()> getWaterBudgetFunc;
  std::function<void(int)> setWaterBudgetFunc;
  std::function<int()> getSprinklersSlotFunc;
  std::function<void(int)> setSprinklersSlotFunc;
  std::function<int()> getSprinklersBudgetFunc;
  std::function<void(int)> setSprinklersBudgetFunc;
  std::function<bool()> getSprinklersModeFunc;
  std::function<void(bool)> setSprinklersModeFunc;
  std::function<bool()> getSystemAutoModeFunc;
  std::function<void(bool)> setSystemAutoModeFunc;
  std::function<bool()> getDrippersAutoModeFunc;
  std::function<void(bool)> setDrippersAutoModeFunc;
  std::function<void(int)> setInnerFansSpeedFunc;
  std::function<int()> getInnerFansSpeedFunc;

  OTAManager(PubSubClient &mqttClient, const String& fwVersion, const String& deviceToken)
    : m_mqttClient(mqttClient), m_fwVersion(fwVersion), m_token(deviceToken) {
      m_mqttClient.setServer(THINGSBOARD_SERVER.c_str(), 1883);
    }

  void begin() {
    Serial.println("[OTA] Initializing LittleFS...");
    if (!LittleFS.begin(true)) {
      Serial.println("[OTA] Failed to mount LittleFS");
    }

    m_mqttClient.setCallback([this](char* topic, byte* payload, unsigned int length) {
      this->handleMqttMessage(topic, payload, length);
    });
    subscribeTopics();
    checkAndConfirmOTA();
  }

  void tick() {
    if (!m_mqttClient.connected()) {
      connectMQTT();
    }
    m_mqttClient.loop();
  }

  void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[MQTT] WiFi is not connecting. aborting");
      return;
    }

    if (!m_mqttClient.connected()) {
      unsigned long now = millis();
      if (now - lastMqttAttempt > mqttReconnectInterval) {
        Serial.print("[MQTT] Connecting...");
        if (m_mqttClient.connect("", m_token.c_str(), "")) {
          Serial.println(" connected!");
          subscribeTopics();
        } else {
          Serial.printf(" failed, rc=%d\n", m_mqttClient.state());
        }
        lastMqttAttempt = now;
      }
    } else {
      Serial.println("[MQTT] mqttClient is not connected");
    }
  }

  void checkAndConfirmOTA() {
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(NULL, &ota_state) == ESP_OK) {
      if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        Serial.println("[OTA] Firmware is pending verify - will be validated by health check system");
        sendTelemetry("fw_state", "PENDING_VALIDATION");
      } else {
        Serial.println("[OTA] Firmware is already valid.");
        sendTelemetry("fw_state", "VALID");
      }
    } else {
      Serial.println("[OTA] Failed to get OTA state!");
    }
  }

  void subscribeTopics() {
    Serial.println("[OTA] Subscribing to OTA topics...");
    m_mqttClient.subscribe("v1/devices/me/attributes/response/+"); // for pull
    m_mqttClient.subscribe("v1/devices/me/attributes");            // for push
    m_mqttClient.subscribe("v1/devices/me/rpc/request/+");         // for RPC

    m_mqttClient.publish("v1/devices/me/attributes/request/1", "{\"sharedKeys\":\"systemConfig,fw_version,fw_checksum,fw_size,fw_title\"}");
  }

  void handleRPC(const String& topic, JsonDocument& doc) {
    // Extract request ID from topic: "v1/devices/me/rpc/request/<id>"
    String requestId = topic.substring(topic.lastIndexOf("/") + 1);

    String method = doc["method"] | "";
    Serial.printf("[RPC] Method: %s\n", method.c_str());

    if (method == "getFanSpeed") {  // =============== getFanSpeed
      int fanSpeed = getFanSpeedFunc();
      String response = String(fanSpeed);
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent fan speed: %d\n", fanSpeed);

    } else if (method == "setFanSpeed") { // ========== setFansSpeed
      int newSpeed = doc["params"] | 0;
      setFanSpeedFunc(newSpeed);
      Serial.printf("[RPC] Set fan speed to: %d\n", newSpeed);

    } else if (method == "getDampersStatus") {  // ====== getDampersStatus
      bool state = getDampersStatusFunc();
      String response = state ? "true" : "false";
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent dampers status: %d\n", state);

    } else if (method == "setDampersStatus") {  // ======= setDampersStatus
      bool state = doc["params"] | false;
      setDampersStatusFunc(state);
      Serial.printf("[RPC] Set dampers status to: %d\n", state);

    } else if (method == "getSolenoidStatus") {  // ====== getSolenoidStatus
      bool state = getSolenoidStatusFunc();
      String response = state ? "true" : "false";
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent solenoid status: %d\n", state);

    } else if (method == "setSolenoidStatus") {  // ======= setSolenoidStatus
      bool state = doc["params"] | false;
      setSolenoidStatusFunc(state);
      Serial.printf("[RPC] Set solenoid status to: %d\n", state);

    } else if (method == "getWaterSlot") {  // ====== getSlot
      int slot = getWaterSlotFunc();
      String response = String(slot);
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent water slot: %d\n", slot);

    } else if (method == "setWaterSlot") {  // ======= setSlot
      int slot = doc["params"] | 0;
      setWaterSlotFunc(slot);
      Serial.printf("[RPC] Set water slot to: %d\n", slot);

    } else if (method == "getWaterBudget") {  // ====== getBudget
      int budget = getWaterBudgetFunc();
      String response = String(budget);
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent water budget: %d\n", budget);

    } else if (method == "setWaterBudget") {  // ======= setBudget
      int budget = doc["params"] | 0;
      setWaterBudgetFunc(budget);
      Serial.printf("[RPC] Set water budget to: %d\n", budget);

    } else if (method == "getSprinklersSlot") {  // ====== getSprinklersSlot
      int slot = getSprinklersSlotFunc();
      String response = String(slot);
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent sprinklers slot: %d\n", slot);

    } else if (method == "setSprinklersSlot") {  // ======= setSprinklersSlot
      int slot = doc["params"] | 0;
      setSprinklersSlotFunc(slot);
      Serial.printf("[RPC] Set sprinklers slot to: %d\n", slot);

    } else if (method == "getSprinklersBudget") {  // ====== getSprinklersBudget
      int budget = getSprinklersBudgetFunc();
      String response = String(budget);
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent sprinklers budget: %d\n", budget);

    } else if (method == "setSprinklersBudget") {  // ======= setSprinklersBudget
      int budget = doc["params"] | 0;
      setSprinklersBudgetFunc(budget);
      Serial.printf("[RPC] Set sprinklers budget to: %d\n", budget);

    } else if (method == "getSystemAutoMode") {  // ====== getSystemAutoMode
      bool mode = getSystemAutoModeFunc();
      String response = mode ? "true" : "false";
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent system auto mode status: %d\n", mode);

    } else if (method == "setSystemAutoMode") {  // ======= setSystemAutoMode
      bool mode = doc["params"] | false;
      setSystemAutoModeFunc(mode);
      Serial.printf("[RPC] Set system auto mode to: %d\n", mode);

    } else if (method == "getDrippersAutoMode") {  // ====== getDrippersAutoMode
      bool mode = getDrippersAutoModeFunc();
      String response = mode ? "true" : "false";
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent drippers auto mode status: %d\n", mode);

    } else if (method == "getSprinklersMode") {  // ====== getSprinklersMode
      bool mode = getSprinklersModeFunc();
      String response = mode ? "true" : "false";
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Sent sprinklers mode: %d\n", mode);

    } else if (method == "setSprinklersMode") {  // ======= setSprinklersMode
      bool mode = doc["params"] | false;
      setSprinklersModeFunc(mode);
      Serial.printf("[RPC] Set sprinklers mode to: %d\n", mode);

    } else if (method == "setDrippersAutoMode") {  // ======= setDrippersAutoMode
      bool mode = doc["params"] | false;
      setDrippersAutoModeFunc(mode);
      Serial.printf("[RPC] Set drippers auto mode to: %d\n", mode);

    } else if (method == "setInnerFanSpeed") {
      int percentage = doc["params"] | 0;
      setInnerFansSpeedFunc(percentage);  
      Serial.printf("[RPC] Set inner fan speed to %i%\n", percentage);

    } else if (method == "getInnerFansSpeed") {
      float speed = getInnerFansSpeedFunc();

      String response = String(speed);
      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      m_mqttClient.publish(responseTopic.c_str(), response.c_str());
      Serial.printf("[RPC] Get inner fan speed: %d\n", speed);

    } else if (method == "restartDevice") {  // ======= restartDevice
      restartDeviceFunc();
      Serial.printf("[RPC] Restarting device...\n");
    
    } else {
      Serial.println("[RPC] Unknown method.");
    }
  }

  void handleMqttMessage(const char* topic, byte* payload, unsigned int length) {
    DynamicJsonDocument doc(1024);
    auto err = deserializeJson(doc, payload, length);
    if (err) {
      Serial.printf("[MQTT] JSON parse failed: %s\n", err.c_str());
      return;
    }

    // Check if this is an RPC topic
    String topicStr(topic);
    if (topicStr.startsWith("v1/devices/me/rpc/request/")) {
      handleRPC(topicStr, doc);
      return; // Done
    }

    // === OTA handling ===
    JsonObject shared;
    if (doc["shared"].is<JsonObject>()) {
      shared = doc["shared"].as<JsonObject>(); // Push
    } else {
      shared = doc.as<JsonObject>();           // Pull
    }

    if (shared.isNull()) {
      Serial.println("[OTA] No shared firmware attributes found.");
      return;
    }

    if (shared["fw_version"].is<String>()) {
      String fwVersion = shared["fw_version"] | "";

      if (fwVersion.length() > 0 && fwVersion != m_fwVersion) {
        String fwTitle = shared["fw_title"] | ""; 
        String fwChecksum = shared["fw_checksum"] | "";
        String fwURL = shared["fw_url"] | "";
        size_t fwSize = shared["fw_size"] | 0;

        triggerBeforeFirmwareUpdate(); // Call the callback before firmware update

        Serial.printf("[OTA] New firmware %s version %s. Downloading...\n", fwTitle.c_str(), fwVersion.c_str());
        downloadFirmware(fwURL, fwSize, fwChecksum, fwTitle, fwVersion);
      } else {
        Serial.printf("[OTA] Firmware version %s is already installed\n", fwVersion);
      }
    } else {
      Serial.println("[OTA] bad fw_version in shared");
    }

    // === SystemConfig handling ===
    if (doc["systemConfig"].is<JsonVariant>()) {
      String configJson;
      serializeJson(doc["systemConfig"], configJson);
      Serial.printf("[Config] Received systemConfig: %s\n", configJson.c_str());
      saveConfig(configJson);
    }
  }

  void saveConfig(const String& configJson) {
    Serial.println("[OTA] Saving config to /config.json...");
    File f = LittleFS.open("/config.json", "w");
    if (!f) {
      Serial.println("[OTA] Failed to open config file for writing");
      return;
    }
    f.print(configJson);
    f.close();
    Serial.println("[OTA] Config saved. Rebooting...");
    delay(1000);
    ESP.restart();
  }

  void downloadFirmware(const String& pushedUrl, size_t fwSize, const String& fwChecksum,
                        const String& fwTitle, const String& fwVersion) {
    String url;

    if ( pushedUrl.length() > 0) {
      url = pushedUrl; // Manual OTA with static fw_url
      Serial.printf("[OTA] Using pushed fw_url: %s\n", url.c_str());
    } else if (fwTitle.length() > 0 && fwVersion.length() > 0) {
      // Build fallback OTA URL with title & version
      url = "https://thingsboard.cloud/api/v1/";
      url += m_token;
      url += "/firmware?title=" + fwTitle + "&version=" + fwVersion;
      Serial.printf("[OTA] Using ThingsBoard OTA API with title & version: %s\n", url.c_str());
    } else {
      // Last resort fallback
      url = "https://thingsboard.cloud/api/v1/" + m_token + "/firmware";
      Serial.printf("[OTA] Using ThingsBoard OTA API: %s\n", url.c_str());
    }

    Serial.printf("[OTA] Expected size: %d bytes\n", fwSize);
    Serial.printf("[OTA] Expected SHA256 checksum: %s\n", fwChecksum.c_str());

    // Use WiFiClientSecure for HTTPS
    WiFiClientSecure client;
    client.setCACert(thingsboard_root_ca_cert);
    //WiFiClient client;
    HTTPClient http;

    Serial.printf("[OTA] Starting download...\n");
    if (!http.begin(client, url)) {
      Serial.println("[OTA] HTTPClient begin() failed!");
      sendTelemetry("fw_state", "FAILED");
      return;
    }

    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
      Serial.printf("[OTA] HTTP GET failed, code: %d\n", httpCode);
      sendTelemetry("fw_state", "FAILED");
      http.end();
      return;
    }

    int contentLength = http.getSize();
    if (contentLength <= 0) {
      Serial.println("[OTA] Invalid content length!");
      sendTelemetry("fw_state", "FAILED");
      http.end();
      return;
    }

    if (contentLength != (int)fwSize) {
      Serial.printf("[OTA] Content length mismatch! Expected: %d, Got: %d\n", fwSize, contentLength);
      sendTelemetry("fw_state", "FAILED");
      http.end();
      return;
    }

    WiFiClient *stream = http.getStreamPtr();
    if (!Update.begin(contentLength)) {
      Serial.println("[OTA] Not enough space to begin OTA");
      sendTelemetry("fw_state", "FAILED");
      http.end();
      return;
    }

    // === SHA256 ===
    mbedtls_sha256_context shaCtx;
    mbedtls_sha256_init(&shaCtx);
    mbedtls_sha256_starts_ret(&shaCtx, 0);  // 0 = SHA256 רגיל

    size_t written = 0;
    uint8_t buff[512];
    unsigned long lastProgress = millis();

    sendTelemetry("fw_state", "DOWNLOADING");

    esp_task_wdt_delete(NULL); // removes current task from the watchdog
    while (http.connected() && written < contentLength) {
      size_t available = stream->available();
      if (available) {
        int readLen = stream->readBytes(buff, min(available, sizeof(buff)));
        if (readLen <= 0) continue;

        // Update SHA256
        mbedtls_sha256_update_ret(&shaCtx, buff, readLen);

        // Write chunk to flash
        if (Update.write(buff, readLen) != readLen) {
          Serial.println("[OTA] Write failed!");
          Update.abort();
          sendTelemetry("fw_state", "FAILED");
          http.end();
          return;
        }

        written += readLen;

        if (millis() - lastProgress > 1000) {
          int percent = (written * 100) / contentLength;
          Serial.printf("[OTA] Progress: %d%% (%d/%d bytes)\n", percent, written, contentLength);
          lastProgress = millis();
        }
      }
    }
    esp_task_wdt_add(NULL); // adds the current thread (loopTask) back to the watchdog

    if (written != contentLength) {
      Serial.printf("[OTA] Mismatch! Written: %d bytes, Expected: %d bytes\n", written, contentLength);
      Update.abort();
      sendTelemetry("fw_state", "FAILED");
      http.end();
      return;
    }
    sendTelemetry("fw_state", "DOWNLOADED");  // download finished

    // === close SHA256 ===
    uint8_t result[32];
    mbedtls_sha256_finish_ret(&shaCtx, result);
    mbedtls_sha256_free(&shaCtx);

    // Convert binary hash to hex string
    char hexResult[65] = {0};
    for (int i = 0; i < 32; ++i) {
      sprintf(hexResult + (i * 2), "%02x", result[i]);
    }

    Serial.printf("[OTA] Calculated SHA256: %s\n", hexResult);

    if (fwChecksum != String(hexResult)) {
      Serial.println("[OTA] Checksum mismatch! Aborting OTA.");
      Update.abort();
      sendTelemetry("fw_state", "FAILED");
      http.end();
      return;
    }
    sendTelemetry("fw_state", "VERIFIED");  // checksum verified

    if (Update.end()) {
      Serial.println("[OTA] OTA update finished!");
      if (Update.isFinished()) {
        Serial.println("[OTA] OTA successful. Rebooting...");
        sendTelemetry("fw_state", "UPDATING");
        ESP.restart();
      } else {
        Serial.println("[OTA] OTA not finished properly.");
        sendTelemetry("fw_state", "FAILED");
      }
    } else {
      Serial.print("[OTA] OTA update failed: ");
      Update.printError(Serial);
      sendTelemetry("fw_state", "FAILED");
    }

    http.end();
  }

  // For floats, ints, or other numeric
  void sendTelemetry(const String& key, float value) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[OTA-Telemetry] WiFi not connected. Skipping telemetry.");
      return;
    }
    if (!m_mqttClient.connected()) {
      Serial.println("[OTA-Telemetry] MQTT not connected. Skipping telemetry.");
      return;
    }

    StaticJsonDocument<256> doc;
    doc[key] = value;
    String payload;
    serializeJson(doc, payload);

    Serial.printf("[OTA-Telemetry] Publishing: %s\n", payload.c_str());
    m_mqttClient.publish("v1/devices/me/telemetry", payload.c_str());
  }

  // Overload for String values (e.g., fw_state, fw_version)
  void sendTelemetry(const String& key, const String& value) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[OTA-Telemetry] WiFi not connected. Skipping telemetry.");
      return;
    }
    if (!m_mqttClient.connected()) {
      Serial.println("[OTA-Telemetry] MQTT not connected. Skipping telemetry.");
      return;
    }

    StaticJsonDocument<256> doc;
    doc[key] = value;
    String payload;
    serializeJson(doc, payload);

    //Serial.printf("[OTA-Telemetry] Publishing: %s\n", payload.c_str());
    m_mqttClient.publish("v1/devices/me/telemetry", payload.c_str());
  }

  void sendTelemetryBatch(const JsonDocument& doc) {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!m_mqttClient.connected()) return;

    String payload;
    serializeJson(doc, payload);
    //Serial.printf("[OTA-Telemetry] Batch: %s\n", payload.c_str());
    m_mqttClient.publish("v1/devices/me/telemetry", payload.c_str());
  }

  void sendAttribute(const String& key, const String& value) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[OTA-Attr] WiFi not connected. Skipping attribute.");
      return;
    }
    if (!m_mqttClient.connected()) {
      Serial.println("[OTA-Attr] MQTT not connected. Skipping attribute.");
      return;
    }

    StaticJsonDocument<256> doc;
    doc[key] = value;
    String payload;
    serializeJson(doc, payload);

    //Serial.printf("[OTA-Attr] Publishing attribute: %s\n", payload.c_str());
    m_mqttClient.publish("v1/devices/me/attributes", payload.c_str());
  }

  // Wrapper for float
  void sendAttribute(const String& key, float value) {
    sendAttribute(key, String(value, 2)); // 2 decimal places
  }

  // Wrapper for int
  void sendAttribute(const String& key, int value) {
    sendAttribute(key, String(value));
  }

  void setBeforeFirmwareUpdateCallback(std::function<void()> callback) {
    onBeforeFirmwareUpdate = callback;
  }

private:
  PubSubClient &m_mqttClient;
  String m_fwVersion;
  String m_token;

  unsigned long lastMqttAttempt = 0;
  const unsigned long mqttReconnectInterval = 5000;

  std::function<void()> onBeforeFirmwareUpdate = nullptr;
  void triggerBeforeFirmwareUpdate() {
    if (onBeforeFirmwareUpdate) {
      Serial.println("[OTA] Triggering safe shutdown before firmware update...");
      onBeforeFirmwareUpdate();
    } else {
      Serial.println("[OTA] No callback set for before firmware update.");
    }
  }

  const char* thingsboard_root_ca_cert = \
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n";
};
