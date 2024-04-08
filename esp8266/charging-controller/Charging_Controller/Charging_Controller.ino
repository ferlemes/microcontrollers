/*********************************************************************************
* MIT License                                                                    *
*                                                                                *
* Copyright (c) 2023 Fernando Lemes da Silva                                     *
*                                                                                *
* Permission is hereby granted, free of charge, to any person obtaining a copy   *
* of this software and associated documentation files (the "Software"), to deal  *
* in the Software without restriction, including without limitation the rights   *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      *
* copies of the Software, and to permit persons to whom the Software is          *
* furnished to do so, subject to the following conditions:                       *
*                                                                                *
* The above copyright notice and this permission notice shall be included in all *
* copies or substantial portions of the Software.                                *
*                                                                                *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  *
* SOFTWARE.                                                                      *
*********************************************************************************/

// Libraries
#include <ESP8266WiFi.h>
#include <WiFiManager.h>         // WiFiManager@2.0.16-rc.2 by tzapu
#include <WiFiUdp.h>             // ESP8266WiFi@3.1.2
#include <NTPClient.h>           // NTPClient@3.2.1 by Fabrice Weinberg version
#include <PubSubClient.h>        // PubSubClient@2.8.0 by Nick O'Leary
#include <ESP8266WebServer.h>    // ESP8266WebServer@3.1.2
#include <ArduinoJson.h>         // ArduinoJson@6.21.4 by Benoit Blanchon
#include <DoubleResetDetector.h> // DoubleResetDetector@1.0.3 by Stephen Denne
#include <Preferences.h>         // Preferences@2.1.0 by Volodymyr Shymanskyy

// IO Pins
#define BATTERY_VOLTAGE_PIN A0
#define ENABLE_INVERSOR_PIN D0
#define ENABLE_CHARGING_PIN D1
#define BUZZER_PIN          D5

// Hardware settings
#define VOLTAGE_DIVIDER_R1                               68000 // 68k ohm (to battery)
#define VOLTAGE_DIVIDER_R2                                6800 // 6k8 ohm (to ground)

// Software settings'
#define DEVICE_NAME                        "ChargingController"
#define DEVICE_TOPIC_PREFIX                     "charging-ctrl"
#define NTP_REFRESH_INTERVAL                             86400 // Sync with NTP servers once a day

// Default values
#define ACTIVATE_CHARGING_VOLTAGE_KEY           "charging-volt"
#define ACTIVATE_CHARGING_VOLTAGE_DEFAULT_VALUE           26.0 // 3.250v per cell
#define ACTIVATE_INVERSOR_VOLTAGE_KEY           "inversor-volt"
#define ACTIVATE_INVERSOR_VOLTAGE_DEFAULT_VALUE           24.4 // 3.050v per cell
#define ACTIVATE_WARNING_VOLTAGE_KEY             "warning-volt"
#define ACTIVATE_WARNING_VOLTAGE_DEFAULT_VALUE            24.2 // 3.025v per cell
#define CHARGING_TIME_SECONDS_KEY               "charging-time"
#define CHARGING_TIME_SECONDS_DEFAULT_VALUE              10800 // 3 hours
#define IDLE_MIN_TIME_SECONDS_KEY               "idle-min-time"
#define IDLE_MIN_TIME_SECONDS_DEFAULT_VALUE               1800 // 30 minutes
#define VOLTAGE_ADJUST_FACTOR_KEY                     "voltAdj"
#define VOLTAGE_ADJUST_FACTOR_DEFAULT_VALUE                1.0 // 1 = No changes
#define FORCE_CHARGE_START_TIME_KEY              "charge-start"
#define FORCE_CHARGE_START_TIME_DEFAULT_VALUE                0
#define FORCE_CHARGE_END_TIME_KEY                  "charge-end"
#define FORCE_CHARGE_END_TIME_DEFAULT_VALUE                  0
#define MQTT_SERVER_HOSTNAME_KEY                  "mqtt-server"
#define MQTT_SERVER_HOSTNAME_DEFAULT_VALUE                  "" // Blank means no message will be sent
#define MQTT_SERVER_PORT_KEY                        "mqtt-port"
#define MQTT_SERVER_PORT_DEFAULT_VALUE                    1883 // Default Mosquitto port
#define MQTT_SERVER_USERNAME_KEY                    "mqtt-user"
#define MQTT_SERVER_USERNAME_DEFAULT_VALUE                  "" // No default username
#define MQTT_SERVER_PASSWORD_KEY                    "mqtt-pass"
#define MQTT_SERVER_PASSWORD_DEFAULT_VALUE                  "" // No default password

// Double Reset to reconfigure WiFi
#define DRD_TIMEOUT 10  // Number of seconds after reset during which a subsequent reset will be considered a double reset.
#define DRD_ADDRESS 0   // RTC Memory Address for the DoubleResetDetector to use

// Objects used to sync time using NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Objects used for MQTT communication
WiFiClient aWiFiClient;
PubSubClient mqttClient(aWiFiClient);

Preferences preferences;

float voltage                           = 0.0;
float voltageMultiplier                 = 0.0;
bool inverterEnabled                    = false;
bool chargerEnabled                     = false;
unsigned long startupTime               = 0;
unsigned long epochTime                 = 0;
unsigned long nextNtpTimeUpdate         = 0;
unsigned long nextMQTTMessage           = 0;
unsigned long nextVoltageUpdateSec      = 0;
unsigned long nextSerialReport          = 0;
unsigned long nextSettingsReport        = 0;
unsigned long chargerStatusBlockedUntil = 0;

// Web server for metrics
ESP8266WebServer server(80);
StaticJsonDocument<255> jsonDocument;

// Double Reset detector
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

float getActivateChargingVoltage() {
    return preferences.getFloat(ACTIVATE_CHARGING_VOLTAGE_KEY, ACTIVATE_CHARGING_VOLTAGE_DEFAULT_VALUE);
}

float getActivateInversorVoltage() {
    return preferences.getFloat(ACTIVATE_INVERSOR_VOLTAGE_KEY, ACTIVATE_INVERSOR_VOLTAGE_DEFAULT_VALUE);
}

float getActivateWarningVoltage() {
    return preferences.getFloat(ACTIVATE_WARNING_VOLTAGE_KEY, ACTIVATE_WARNING_VOLTAGE_DEFAULT_VALUE);
}

unsigned int getChargingTimeSeconds() {
    return preferences.getUInt(CHARGING_TIME_SECONDS_KEY, CHARGING_TIME_SECONDS_DEFAULT_VALUE);
}

unsigned int getIdleMinTimeSeconds() {
    return preferences.getUInt(IDLE_MIN_TIME_SECONDS_KEY, IDLE_MIN_TIME_SECONDS_DEFAULT_VALUE);
}

float getVoltageAdjustFactor() {
    return preferences.getFloat(VOLTAGE_ADJUST_FACTOR_KEY, VOLTAGE_ADJUST_FACTOR_DEFAULT_VALUE);
}

unsigned int getForceChargeStartTime() {
    return preferences.getUInt(FORCE_CHARGE_START_TIME_KEY, FORCE_CHARGE_START_TIME_DEFAULT_VALUE);
}

unsigned int getForceChargeEndTime() {
    return preferences.getUInt(FORCE_CHARGE_END_TIME_KEY, FORCE_CHARGE_END_TIME_DEFAULT_VALUE);
}

String getMQTTHostname() {
    return preferences.getString(MQTT_SERVER_HOSTNAME_KEY, MQTT_SERVER_HOSTNAME_DEFAULT_VALUE);
}

int getMQTTPort() {
    return preferences.getUInt(MQTT_SERVER_PORT_KEY, MQTT_SERVER_PORT_DEFAULT_VALUE);
}

String getMQTTUsername() {
    return preferences.getString(MQTT_SERVER_USERNAME_KEY, MQTT_SERVER_USERNAME_DEFAULT_VALUE);
}

String getMQTTPassword() {
    return preferences.getString(MQTT_SERVER_PASSWORD_KEY, MQTT_SERVER_PASSWORD_DEFAULT_VALUE);
}

float getVoltage() {
    return analogRead(BATTERY_VOLTAGE_PIN) * voltageMultiplier;
}

void updateTime() {
    epochTime = timeClient.getEpochTime();
    if (nextNtpTimeUpdate < epochTime) {
        Serial.println("Updating local time with NTP server...");
        timeClient.update();
        epochTime = timeClient.getEpochTime();
        nextNtpTimeUpdate = epochTime + NTP_REFRESH_INTERVAL;
    }
    if (startupTime == 0 && epochTime > 0) {
        startupTime = epochTime;
    }
}

void updateLedStatus() {
    unsigned long ledMillis = millis() % 2000;
    if (ledMillis < 1000) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
}

void connectToMQTTServer() {
    String mqttServer = getMQTTHostname();
    if (!mqttServer.isEmpty()) {
        int mqttPort = getMQTTPort();
        String mqttUsername = getMQTTUsername();
        String mqttPassword = getMQTTPassword();
        mqttClient.setServer(mqttServer.c_str(), mqttPort);
        Serial.printf("Connecting to MQTT server %s:%d ", mqttServer, mqttPort);
        while (!mqttClient.connected()) {
            Serial.print(".");
            mqttClient.connect(DEVICE_NAME, mqttUsername.c_str(), mqttPassword.c_str());
            delay(500);
        }
        Serial.printf(" connected!\n");
    }
}

void setupMQTT() {
    if (!getMQTTHostname().isEmpty()) {
        mqttClient.setBufferSize(512);
        if (!mqttClient.connected()) {
            connectToMQTTServer();
        }
        char newSensorInfo[512];
        sprintf(newSensorInfo, "{\"unique_id\":\"%s-%08X-voltage\",\"name\":\"%s Voltage (%08X)\",\"device_class\":\"voltage\",\"unit_of_measurement\":\"V\",\"icon\":\"mdi:battery-charging-60\",\"state_topic\":\"%s-voltage/%08X/state\",\"expire_after\":\"300\"}",
                                DEVICE_TOPIC_PREFIX, ESP.getChipId(), DEVICE_NAME, ESP.getChipId(), DEVICE_TOPIC_PREFIX, ESP.getChipId());
        char mqttTopic[128];
        sprintf(mqttTopic, "homeassistant/sensor/%s-voltage/%08X/config", DEVICE_TOPIC_PREFIX, ESP.getChipId());
        Serial.printf("Publishing to topic '%s' message: '%s'\n", mqttTopic, newSensorInfo);
        if (!mqttClient.publish(mqttTopic, newSensorInfo)) {
            Serial.println("ERROR: Could not publish setup message!");
        }
    }
}

void updateMQTT() {
    if (nextMQTTMessage <= epochTime) {
        if (!getMQTTHostname().isEmpty()) {
            if (!mqttClient.connected()) {
                connectToMQTTServer();
            }
            char mqttTopic[128];
            char data[128];
            sprintf(mqttTopic, "%s-voltage/%08X/state", DEVICE_TOPIC_PREFIX, ESP.getChipId());
            sprintf(data, "%.3f", voltage);
            if (!mqttClient.publish(mqttTopic, data)) {
                Serial.println("ERROR: Could not publish update message!");
            }
        }
        nextMQTTMessage = epochTime + 60;
    }
}

void updateVoltage() {
    if (nextVoltageUpdateSec <= epochTime) {
        voltage = (voltage * 9 + getVoltage()) / 10;
        nextVoltageUpdateSec = epochTime + 1;
    } 
}

void updateChargerStatus() {
    if (chargerStatusBlockedUntil <= epochTime) {
        if (voltage <= getActivateChargingVoltage()) {
            chargerStatusBlockedUntil = epochTime + getChargingTimeSeconds();
            digitalWrite(ENABLE_CHARGING_PIN, LOW);
            chargerEnabled = true;
        } else {
            if (getForceChargeStartTime() != 0 && getForceChargeEndTime() != 0) {
                unsigned int currentTime = timeClient.getHours() * 100 + timeClient.getMinutes();;
                if (getForceChargeStartTime() <= currentTime && currentTime <= getForceChargeEndTime()) {
                    digitalWrite(ENABLE_CHARGING_PIN, LOW);
                    chargerEnabled = true;
                } else {
                    digitalWrite(ENABLE_CHARGING_PIN, HIGH);
                    chargerEnabled = false;
                }
            } else {
                chargerStatusBlockedUntil = epochTime + getIdleMinTimeSeconds();
                digitalWrite(ENABLE_CHARGING_PIN, HIGH);
                chargerEnabled = false;
            }
        }
    }
}

void updateInversorStatus() {
    if (voltage >= getActivateInversorVoltage()) {
        digitalWrite(ENABLE_INVERSOR_PIN, LOW);
        inverterEnabled = true;
    } else {
        digitalWrite(ENABLE_INVERSOR_PIN, HIGH);
        inverterEnabled = false;
    }
}

void updateBuzzer() {
    if (voltage < getActivateWarningVoltage()) {
        unsigned long buzzerMillis = millis() % 5300;
        if (buzzerMillis < 100 || (buzzerMillis >= 200 && buzzerMillis < 300)) {
            digitalWrite(BUZZER_PIN, HIGH);
        } else {
            digitalWrite(BUZZER_PIN, LOW);
        }
    } else {
        digitalWrite(BUZZER_PIN, LOW);
    }
}

void printSerialReport() {
    unsigned long currentUptime = epochTime - startupTime;
    int seconds = currentUptime % 60;
    currentUptime = currentUptime / 60;
    int minutes = currentUptime % 60;
    currentUptime = currentUptime / 60;
    int hours = currentUptime % 24;
    int days = (int) currentUptime / 24;
    if (nextSettingsReport <= epochTime) {
      float activateChargingVoltage = getActivateChargingVoltage();
      float activateInversorVoltage = getActivateInversorVoltage();
      float activateWarningVoltage = getActivateWarningVoltage();
      unsigned int chargingTimeSecs = getChargingTimeSeconds();
      unsigned int idleMinTimeSecs = getIdleMinTimeSeconds();
      float voltageAdjustFactor = getVoltageAdjustFactor();
      unsigned int forceChargeStartTime = getForceChargeStartTime();
      unsigned int forceChargeEndTime = getForceChargeEndTime();
      Serial.printf("timestamp=%lu | uptime=%dd%02dh%02dm%02ds | voltageAdjustFactor=%.3f | activateChargingVoltage=%.3f | activateInversorVoltage=%.3f | activateWarningVoltage=%.3f | chargingTimeSecs=%d | idleMinTimeSecs=%d | forceChargeStart=%d | forceChargeEnd=%d\n", epochTime, days, hours, minutes, seconds, voltageAdjustFactor, activateChargingVoltage, activateInversorVoltage, activateWarningVoltage, chargingTimeSecs, idleMinTimeSecs, forceChargeStartTime, forceChargeEndTime);
      nextSettingsReport = epochTime + 60;
    }
    if (nextSerialReport <= epochTime) {
        bool alarm = voltage < getActivateWarningVoltage();
        Serial.printf("timestamp=%lu | uptime=%dd%02dh%02dm%02ds | voltage=%.3f | inverterEnabled=%d | chargerEnabled=%d | alarm=%d\n", epochTime, days, hours, minutes, seconds, voltage, inverterEnabled, chargerEnabled, alarm);
        nextSerialReport = epochTime + 3;
    }
}

void serveStatusPage() {
    String jsonString = "";
    JsonObject object = jsonDocument.to<JsonObject>();
    object["timestamp"] = epochTime;
    object["uptime"] = epochTime - startupTime;
    object["voltage"] = voltage;
    object["inverterEnabled"] = inverterEnabled;
    object["chargerEnabled"] = chargerEnabled;
    object["alarm"] = voltage < getActivateWarningVoltage();
    serializeJson(jsonDocument, jsonString);
    server.send(200, "application/json", jsonString);
}


void serveMqttSettingsPage() {

    int http_status;
    if (server.method() == HTTP_PUT) {
        String data = server.arg("plain");
        Serial.print("Update MQTT settings received with data: ");
        Serial.println(data);
        DeserializationError error = deserializeJson(jsonDocument, data);
        if (!error) {
            String hostname = jsonDocument["hostname"];
            String username = jsonDocument["username"];
            String password = jsonDocument["password"];
            unsigned int port = jsonDocument["port"];
            if (0 <= hostname.length() && hostname.length() < 256 &&
                0 <= username.length() && username.length() < 256 &&
                0 <= password.length() && password.length() < 256 &&
                0 <  port              && port              < 65536) {
                preferences.putString(MQTT_SERVER_HOSTNAME_KEY, hostname);
                preferences.putString(MQTT_SERVER_USERNAME_KEY, username);
                preferences.putString(MQTT_SERVER_PASSWORD_KEY, password);
                preferences.putUInt(MQTT_SERVER_PORT_KEY, port);
                setupMQTT();
                nextMQTTMessage = 0;
                http_status = 200;
            } else {
                http_status = 400;
            }
        } else {
            Serial.printf("JSON deserialization error: %s\n", error.c_str());
            http_status = 400;
        }
        
    } else {
        if (server.method() == HTTP_GET) {
            http_status = 200;
        } else {
            server.send(405, "plain/text", "405 Method Not Allowed");
            return;
        }
    }

    String jsonString = "";
    JsonObject object = jsonDocument.to<JsonObject>();
    object["hostname"] = getMQTTHostname();
    object["username"] = getMQTTUsername();
    if (getMQTTPassword().isEmpty()) {
        object["password"] = "";
    } else {
        object["password"] = "******";
    }
    object["port"]     = getMQTTPort();
    serializeJson(jsonDocument, jsonString);
    server.send(http_status, "application/json", jsonString);
}


void serveSettingsPage() {

    int http_status;
    if (server.method() == HTTP_PUT) {
        String data = server.arg("plain");
        Serial.print("Update settings received with data: ");
        Serial.println(data);
        DeserializationError error = deserializeJson(jsonDocument, data);
        if (!error) {
            float activateChargingVoltage = jsonDocument["activate_charging_voltage"];
            float activateInversorVoltage = jsonDocument["activate_inversor_voltage"];
            float activateWarningVoltage = jsonDocument["activate_warning_voltage"];
            unsigned int chargingTimeSecs = jsonDocument["charging_time_secs"];
            unsigned int idleMinTimeSecs = jsonDocument["idle_min_time_secs"];
            unsigned int forceChargeStartTime = jsonDocument["force_charge_start_time"];
            unsigned int forceChargeEndTime = jsonDocument["force_charge_end_time"];
            if (0 < activateChargingVoltage && activateChargingVoltage <   100 &&
                0 < activateInversorVoltage && activateInversorVoltage <   100 &&
                0 < activateWarningVoltage  && activateWarningVoltage  <   100 &&
                0 < chargingTimeSecs        && chargingTimeSecs        < 86400 &&
                0 < idleMinTimeSecs         && idleMinTimeSecs         < 43200) {
                preferences.putFloat(ACTIVATE_CHARGING_VOLTAGE_KEY, activateChargingVoltage);
                preferences.putFloat(ACTIVATE_INVERSOR_VOLTAGE_KEY, activateInversorVoltage);
                preferences.putFloat(ACTIVATE_WARNING_VOLTAGE_KEY, activateWarningVoltage);
                preferences.putUInt(CHARGING_TIME_SECONDS_KEY, chargingTimeSecs);
                preferences.putUInt(IDLE_MIN_TIME_SECONDS_KEY, idleMinTimeSecs);
                preferences.putUInt(FORCE_CHARGE_START_TIME_KEY, forceChargeStartTime);
                preferences.putUInt(FORCE_CHARGE_END_TIME_KEY, forceChargeEndTime);
                nextSettingsReport = 0;
                http_status = 201;
            } else {
                http_status = 400;
            }
        } else {
            Serial.printf("JSON deserialization error: %s\n", error.c_str());
            http_status = 400;
        }
        
    } else {
        if (server.method() == HTTP_GET) {
            http_status = 200;
        } else {
            server.send(405, "plain/text", "405 Method Not Allowed");
            return;
        }
    }

    String jsonString = "";
    JsonObject object = jsonDocument.to<JsonObject>();
    object["activate_charging_voltage"] = getActivateChargingVoltage();
    object["activate_inversor_voltage"] = getActivateInversorVoltage();
    object["activate_warning_voltage"] = getActivateWarningVoltage();
    object["charging_time_secs"] = getChargingTimeSeconds();
    object["idle_min_time_secs"] = getIdleMinTimeSeconds();
    object["force_charge_start_time"] = getForceChargeStartTime();
    object["force_charge_end_time"] = getForceChargeEndTime();
    serializeJson(jsonDocument, jsonString);
    server.send(http_status, "application/json", jsonString);
}

void serveVoltageAjustPage() {

    int http_status;
    if (server.method() == HTTP_PUT) {
        String data = server.arg("plain");
        Serial.print("Update voltage adjust factor received with data: ");
        Serial.println(data);
        DeserializationError error = deserializeJson(jsonDocument, data);
        if (!error) {
            float voltageAdjustFactor = jsonDocument["voltage_adjust_factor"];
            if (0.0 < voltageAdjustFactor && voltageAdjustFactor < 2.0) {
                preferences.remove(VOLTAGE_ADJUST_FACTOR_KEY);
                preferences.putFloat(VOLTAGE_ADJUST_FACTOR_KEY, voltageAdjustFactor);
                voltageMultiplier = 3.3 * (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / 1023.0 / VOLTAGE_DIVIDER_R2 * voltageAdjustFactor;
                nextSettingsReport = 0;
                http_status = 201;
            } else {
                http_status = 400;
            }
        } else {
            Serial.printf("JSON deserialization error: %s\n", error.c_str());
            http_status = 400;
        }
        
    } else {
        if (server.method() == HTTP_GET) {
            http_status = 200;
        } else {
            server.send(405, "plain/text", "405 Method Not Allowed");
            return;
        }
    }

    String jsonString = "";
    JsonObject object = jsonDocument.to<JsonObject>();
    object["voltage_adjust_factor"] = getVoltageAdjustFactor();
    serializeJson(jsonDocument, jsonString);
    server.send(http_status, "application/json", jsonString);
}

void serveNotFound() {
    server.send(404, "plain/text", "NOT FOUND");
}

void configModeCallback(WiFiManager *localWifiManager) {
    drd.stop();
    Serial.println("Entering Wifi Manager configuration mode...");
    preferences.clear();
    for (int blinks = 0; blinks < 15; blinks++) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
    }
    digitalWrite(LED_BUILTIN, LOW);
}

void setup() {

    // Preferences
    preferences.begin("chargeControl");
    
    // Basic pins and I/O setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BATTERY_VOLTAGE_PIN, INPUT);
    pinMode(ENABLE_INVERSOR_PIN, OUTPUT);
    pinMode(ENABLE_CHARGING_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(ENABLE_INVERSOR_PIN, HIGH);
    digitalWrite(ENABLE_CHARGING_PIN, HIGH);
    Serial.begin(9600);
    delay(1000);
    Serial.println("Charging controller started!");

    // Setup WiFi access
    Serial.println("Connecting to WiFi...");
    WiFiManager wifiManager;
    wifiManager.setAPStaticIPConfig(IPAddress(192,168,0,1), IPAddress(192,168,0,1), IPAddress(255,255,255,0));
    wifiManager.setAPCallback(configModeCallback);
    if (drd.detectDoubleReset()) {
        wifiManager.startConfigPortal(DEVICE_NAME);
    } else {
        if (!wifiManager.autoConnect(DEVICE_NAME)) {
            Serial.println("Failed to connect.");
            delay(10000);
            ESP.restart();
        } else {
            Serial.println("Connected!");
        }
    }
    delay(1000);
    drd.stop();
    timeClient.begin();
    setupMQTT();
    server.on("/status", serveStatusPage);
    server.on("/mqtt", serveMqttSettingsPage);
    server.on("/settings", serveSettingsPage);
    server.on("/voltage-adjust", serveVoltageAjustPage);
    server.onNotFound(serveNotFound);
    server.begin();
    Serial.println("HTTP server started!");

    // Initialize voltage level
    voltageMultiplier = 3.3 * (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / 1023.0 / VOLTAGE_DIVIDER_R2 * getVoltageAdjustFactor();
    voltage = getVoltage();
}

void loop() {
    updateTime();
    updateLedStatus();
    updateVoltage();
    updateMQTT();
    updateChargerStatus();
    updateInversorStatus();
    updateBuzzer();
    printSerialReport();
    server.handleClient();
}
