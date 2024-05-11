/*********************************************************************************
* MIT License                                                                    *
*                                                                                *
* Copyright (c) 2024 Fernando Lemes da Silva                                     *
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
#include <WiFi.h>
#include <WiFiManager.h>             // WiFiManager@2.0.16-rc.2 by tzapu
#include <WiFiUdp.h>                 //
#include <NTPClient.h>               // NTPClient@3.2.1 by Fabrice Weinberg version
#include <PubSubClient.h>            // PubSubClient@2.8.0 by Nick O'Leary
#include <WebServer.h>               // WebServer
#include <ArduinoJson.h>             // ArduinoJson@6.21.4 by Benoit Blanchon
#include <ESP_DoubleResetDetector.h> // ESP_DoubleResetDetector@1.3.2 by Khoi Hoang
#include <Preferences.h>             // Preferences@2.1.0 by Volodymyr Shymanskyy

// IO Pins
#define LED_BUILTIN                                         23
#define SENSOR_1_PIN                                        12
#define SENSOR_2_PIN                                        13
#define SENSOR_3_PIN                                        14
#define ENABLE_PUMP_PIN                                     16

// Software settings'
#define DEVICE_NAME                            "PumpController"
#define DEVICE_TOPIC_PREFIX                         "pump-ctrl"
#define NTP_REFRESH_INTERVAL                             86400 // Sync with NTP servers once a day
#define MQTT_MAX_ATTEMPTS                                    3 // Attempts to connect to MQTT server before give up
#define MQTT_BACKOFF_SECONDS                               300 // How many seconds to backoff in case of too many failures

// Default values
#define SENSOR_DELAY_SECONDS_KEY                 "sensor-delay"
#define SENSOR_DELAY_SECONDS_DEFAULT_VALUE                   5
#define MAX_TIME_SECONDS_KEY                         "max-time"
#define MAX_TIME_SECONDS_DEFAULT_VALUE                     600
#define COOLDOWN_TIME_SECONDS_KEY               "cooldown-time"
#define COOLDOWN_TIME_SECONDS_DEFAULT_VALUE                300
#define MAX_COOLDOWN_COUNT                                  16 // If the cooldown feature is triggered this much, there must be some failure on the sensors
#define MAX_COOLDOWN_PERIOD                              86400 // The amount of time to clean the cooldown feature counter (1 day by default)

// MQTT Settings
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

bool pumpEnabled                     = false;
bool tooManyCooldowns                = false;
unsigned long pumpEnabledUntilEpoch  = 0;
unsigned long pumpEnabledSince       = 0;
unsigned long pumpDisabledUntilEpoch = 0;
unsigned long pumpOverrideUntilEpoch = 0;
unsigned long startupTime            = 0;
unsigned long epochTime              = 0;
unsigned long nextNtpTimeUpdate      = 0;
unsigned long nextSettingsReport     = 0;
unsigned long nextMQTTMessage        = 0;
unsigned long ignoreMQTTUntil        = 0;
int cooldownCounter                  = 0;


// Web server for metrics
WebServer server(80);
StaticJsonDocument<255> jsonDocument;

// Double Reset detector
DoubleResetDetector* drd;

unsigned int getSensorDelaySeconds() {
    return preferences.getUInt(SENSOR_DELAY_SECONDS_KEY, SENSOR_DELAY_SECONDS_DEFAULT_VALUE);
}

unsigned int getMaxTimeSeconds() {
    return preferences.getUInt(MAX_TIME_SECONDS_KEY, MAX_TIME_SECONDS_DEFAULT_VALUE);
}

unsigned int getCooldownTimeSeconds() {
    return preferences.getUInt(COOLDOWN_TIME_SECONDS_KEY, COOLDOWN_TIME_SECONDS_DEFAULT_VALUE);
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
    if (ignoreMQTTUntil < epochTime) {
        String mqttServer = getMQTTHostname();
        if (!mqttServer.isEmpty()) {
            int mqttPort = getMQTTPort();
            String mqttUsername = getMQTTUsername();
            String mqttPassword = getMQTTPassword();
            mqttClient.setServer(mqttServer.c_str(), mqttPort);
            Serial.printf("Connecting to MQTT server %s:%d ", mqttServer, mqttPort);
            int attempts = 0;
            while (!mqttClient.connected() && attempts < MQTT_MAX_ATTEMPTS) {
                Serial.print(".");
                mqttClient.connect(DEVICE_NAME, mqttUsername.c_str(), mqttPassword.c_str());
                delay(500);
                attempts++;
                if (attempts >= MQTT_MAX_ATTEMPTS) {
                  ignoreMQTTUntil = epochTime + MQTT_BACKOFF_SECONDS;
                }
            }
            if (mqttClient.connected()) {
                Serial.printf(" connected!\n");
            } else {
                Serial.printf(" FAILURE!\n");
            }
        }
    }
}

void setupMQTT() {
    if (ignoreMQTTUntil < epochTime) {
        if (!getMQTTHostname().isEmpty()) {
            mqttClient.setBufferSize(512);
            if (!mqttClient.connected()) {
                connectToMQTTServer();
            }
            char mqttTopic[128];
            char newSensorInfo[512];

            // Pump
            sprintf(mqttTopic, "homeassistant/binary_sensor/%s-pump/%08X/config", DEVICE_TOPIC_PREFIX, ESP.getEfuseMac());
            sprintf(newSensorInfo, "{\"unique_id\":\"%s-%08X-pump\",\"name\":\"%s Pump (%08X)\",\"device_class\":\"power\",\"state_topic\":\"%s-pump/%08X/state\",\"expire_after\":\"330\"}",
                                    DEVICE_TOPIC_PREFIX, ESP.getEfuseMac(), DEVICE_NAME, ESP.getEfuseMac(), DEVICE_TOPIC_PREFIX, ESP.getEfuseMac());
            Serial.printf("Publishing to topic '%s' message: '%s'\n", mqttTopic, newSensorInfo);
            if (!mqttClient.publish(mqttTopic, newSensorInfo, true)) {
                Serial.println("ERROR: Could not publish setup message!");
            }

        }
    }
}

void updateMQTT() {
    if (nextMQTTMessage <= epochTime && ignoreMQTTUntil < epochTime) {
        if (!getMQTTHostname().isEmpty()) {
            if (!mqttClient.connected()) {
                connectToMQTTServer();
            }
            char mqttTopic[128];
            char data[128];

            sprintf(mqttTopic, "%s-pump/%08X/state", DEVICE_TOPIC_PREFIX, ESP.getEfuseMac());
            if (pumpEnabled) {
                Serial.printf("Publishing to topic '%s' message: 'ON'\n", mqttTopic);
                sprintf(data, "ON");
            } else {
                Serial.printf("Publishing to topic '%s' message: 'OFF'\n", mqttTopic);
                sprintf(data, "OFF");
            }
            if (!mqttClient.publish(mqttTopic, data)) {
                Serial.println("ERROR: Could not publish update message!");
            }
        }
        nextMQTTMessage = epochTime + 300;
    }
}

void updatePumpEnabled() {
    
    if (tooManyCooldowns) {
        digitalWrite(ENABLE_PUMP_PIN, LOW);
        return;
    }

    bool previousPumpStatus = pumpEnabled;

    if (pumpDisabledUntilEpoch <= epochTime) {
      
        if (digitalRead(SENSOR_1_PIN) == LOW || digitalRead(SENSOR_2_PIN) == LOW || digitalRead(SENSOR_3_PIN) == LOW) { 
            if (pumpEnabledSince == 0) {
                pumpEnabledSince = epochTime;
            }
            pumpEnabled = true;
            pumpEnabledUntilEpoch = epochTime + getSensorDelaySeconds();
            if (getMaxTimeSeconds() < epochTime - pumpEnabledSince) {
                pumpEnabled = false;
                pumpEnabledSince = 0;
                pumpEnabledUntilEpoch = 0;
                pumpDisabledUntilEpoch = epochTime + getCooldownTimeSeconds();
                cooldownCounter = cooldownCounter + 1;
            } else {
                pumpDisabledUntilEpoch = 0;
            }
        } else {
            if (epochTime <= pumpEnabledUntilEpoch) {
                // Keep it enabled a little bit
            } else {
                pumpEnabled = false;
                pumpEnabledSince = 0;
                pumpEnabledUntilEpoch = 0;
                pumpDisabledUntilEpoch = 0;
            }
        }
    } else {
        pumpEnabled = false;
        pumpEnabledSince = 0;
        pumpEnabledUntilEpoch = 0;
        if (pumpDisabledUntilEpoch <= epochTime) {
            pumpDisabledUntilEpoch = 0;
        }
    }

    if (MAX_COOLDOWN_COUNT <= cooldownCounter) {
        tooManyCooldowns = true;
    }
    
    unsigned long currentUptime = epochTime - startupTime;
    if (currentUptime % MAX_COOLDOWN_PERIOD == 0) {
        cooldownCounter = 0;
    }
    
    if (pumpEnabled) {
        digitalWrite(ENABLE_PUMP_PIN, HIGH);
    } else {
        digitalWrite(ENABLE_PUMP_PIN, LOW);
    }

    if (pumpEnabled != previousPumpStatus) {
        nextMQTTMessage = epochTime + 1; // To avoid any delay on turning the pump on
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
    unsigned long pumpRunningSeconds = epochTime - pumpEnabledSince;
    if (pumpEnabledSince == 0) pumpRunningSeconds = 0;
    unsigned long pumpDelaySeconds = pumpEnabledUntilEpoch - epochTime;
    if (pumpEnabledUntilEpoch == 0) pumpDelaySeconds = 0;
    unsigned long cooldownSeconds = pumpDisabledUntilEpoch - epochTime;
    if (pumpDisabledUntilEpoch == 0) cooldownSeconds = 0;
    if (nextSettingsReport <= epochTime) {
        Serial.printf("timestamp=%lu | uptime=%dd%02dh%02dm%02ds | pumpEnabled=%d | pumpRunningSeconds=%lu | pumpDelaySeconds=%lu | cooldownSeconds=%lu | cooldownCounter=%d | tooManyCooldowns=%d\n", epochTime, days, hours, minutes, seconds, pumpEnabled, pumpRunningSeconds, pumpDelaySeconds, cooldownSeconds, cooldownCounter, tooManyCooldowns);
        nextSettingsReport = epochTime + 1;
    }
}

void serveStatusPage() {

    unsigned long pumpRunningSeconds = epochTime - pumpEnabledSince;
    if (pumpEnabledSince == 0) pumpRunningSeconds = 0;

    unsigned long pumpDelaySeconds = pumpEnabledUntilEpoch - epochTime;
    if (pumpEnabledUntilEpoch == 0) pumpDelaySeconds = 0;

    unsigned long cooldownSeconds = pumpDisabledUntilEpoch - epochTime;
    if (pumpDisabledUntilEpoch == 0) cooldownSeconds = 0;

    String jsonString = "";
    JsonObject object = jsonDocument.to<JsonObject>();
    object["timestamp"] = epochTime;
    object["uptime"] = epochTime - startupTime;
    object["pumpEnabled"] = pumpEnabled;
    object["pumpRunningSeconds"] = pumpRunningSeconds;
    object["pumpDelaySeconds"] = pumpDelaySeconds;
    object["cooldownSeconds"] = cooldownSeconds;
    object["cooldownCounter"] = cooldownCounter;
    object["tooManyCooldowns"] = tooManyCooldowns;
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
            unsigned int sensor_delay_seconds  = jsonDocument["sensor_delay_seconds"];
            unsigned int max_time_seconds      = jsonDocument["max_time_seconds"];
            unsigned int cooldown_time_seconds = jsonDocument["cooldown_time_seconds"];
            if (0 < sensor_delay_seconds  && sensor_delay_seconds  <   30 &&
                0 < max_time_seconds      && max_time_seconds      < 1800 &&
                0 < cooldown_time_seconds && cooldown_time_seconds < 3600) {
                preferences.putUInt(SENSOR_DELAY_SECONDS_KEY, sensor_delay_seconds);
                preferences.putUInt(MAX_TIME_SECONDS_KEY, max_time_seconds);
                preferences.putUInt(COOLDOWN_TIME_SECONDS_KEY, cooldown_time_seconds);
                nextSettingsReport = 0;
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
    object["sensor_delay_seconds"]  = getSensorDelaySeconds();
    object["max_time_seconds"]      = getMaxTimeSeconds();
    object["cooldown_time_seconds"] = getCooldownTimeSeconds();
    serializeJson(jsonDocument, jsonString);
    server.send(http_status, "application/json", jsonString);
}


void serveNotFound() {
    server.send(404, "plain/text", "NOT FOUND");
}


void configModeCallback(WiFiManager *localWifiManager) {
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
    preferences.begin("pumpControl");

    // Basic pins and I/O setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SENSOR_1_PIN, INPUT_PULLUP);
    pinMode(SENSOR_2_PIN, INPUT_PULLUP);
    pinMode(SENSOR_3_PIN, INPUT_PULLUP);
    pinMode(ENABLE_PUMP_PIN, OUTPUT);
    digitalWrite(ENABLE_PUMP_PIN, LOW);
    Serial.begin(9600);
    delay(1000);
    Serial.println("Pump controller started!");

    // Setup WiFi access
    Serial.println("Connecting to WiFi...");
    WiFiManager wifiManager;
    wifiManager.setAPStaticIPConfig(IPAddress(192,168,0,1), IPAddress(192,168,0,1), IPAddress(255,255,255,0));
    wifiManager.setAPCallback(configModeCallback);
    drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
    if (drd->detectDoubleReset()) {
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
    timeClient.begin();
    setupMQTT();
    server.on("/status", serveStatusPage);
    server.on("/mqtt", serveMqttSettingsPage);
    server.on("/settings", serveSettingsPage);
    server.onNotFound(serveNotFound);
    server.begin();
    Serial.println("HTTP server started!");
}


void loop() {
    updateTime();
    updateLedStatus();
    updateMQTT();
    updatePumpEnabled();
    printSerialReport();
    server.handleClient();
    drd->loop();
}
