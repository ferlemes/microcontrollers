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
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <DoubleResetDetector.h>
#include <Preferences.h> // Preferences@2.1.0

// IO Pins
#define BATTERY_VOLTAGE_PIN A0
#define ENABLE_INVERSOR_PIN D0
#define ENABLE_CHARGING_PIN D1
#define BUZZER_PIN          D5

// Hardware settings
#define VOLTAGE_DIVIDER_R1       68000 // 68k ohm (to battery)
#define VOLTAGE_DIVIDER_R2        6800 // 6k8 ohm (to ground)


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

// Double Reset to reconfigure WiFi
#define DRD_TIMEOUT 10  // Number of seconds after reset during which a subsequent reset will be considered a double reset.
#define DRD_ADDRESS 0   // RTC Memory Address for the DoubleResetDetector to use

Preferences preferences;

float voltage                           = 0.0;
float voltageMultiplier                 = 0.0;
bool inverterEnabled                    = false;
bool chargerEnabled                     = false;
unsigned long uptimeSeconds             = 0;
unsigned long lastMillis                = 0;
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

float getVoltage() {
    return analogRead(BATTERY_VOLTAGE_PIN) * voltageMultiplier;
}

void updateUptime() {
    unsigned long currentMillis = millis();
    if (lastMillis + 1000 < currentMillis) {
        int seconds = (currentMillis - lastMillis) / 1000;
        uptimeSeconds += seconds;
        lastMillis += (seconds * 1000);
    } else {
        if (currentMillis < lastMillis) {
            int seconds = (4294967295 - lastMillis + currentMillis) / 1000;
            uptimeSeconds += seconds;
            lastMillis += (seconds * 1000);
        }
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

void updateVoltage() {
    if (nextVoltageUpdateSec <= uptimeSeconds) {
        voltage = (voltage * 9 + getVoltage()) / 10;
        nextVoltageUpdateSec = uptimeSeconds + 1;
    } 
}

void updateChargerStatus() {
    if (chargerStatusBlockedUntil <= uptimeSeconds) {
        if (voltage <= getActivateChargingVoltage()) {
            chargerStatusBlockedUntil = uptimeSeconds + getChargingTimeSeconds();
            digitalWrite(ENABLE_CHARGING_PIN, LOW);
            chargerEnabled = true;
        } else {
            chargerStatusBlockedUntil = uptimeSeconds + getIdleMinTimeSeconds();
            digitalWrite(ENABLE_CHARGING_PIN, HIGH);
            chargerEnabled = false;
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
    unsigned long currentUptime = uptimeSeconds;
    int seconds = currentUptime % 60;
    currentUptime = currentUptime / 60;
    int minutes = currentUptime % 60;
    currentUptime = currentUptime / 60;
    int hours = currentUptime % 24;
    int days = (int) currentUptime / 24;
    if (nextSettingsReport <= uptimeSeconds) {
      float activateChargingVoltage = getActivateChargingVoltage();
      float activateInversorVoltage = getActivateInversorVoltage();
      float activateWarningVoltage = getActivateWarningVoltage();
      unsigned int chargingTimeSecs = getChargingTimeSeconds();
      unsigned int idleMinTimeSecs = getIdleMinTimeSeconds();
      float voltageAdjustFactor = getVoltageAdjustFactor();
      Serial.printf("uptime: %dd%02dh%02dm%02ds | voltageAdjustFactor=%.3f | activateChargingVoltage=%.3f | activateInversorVoltage=%.3f | activateWarningVoltage=%.3f | chargingTimeSecs=%d | idleMinTimeSecs=%d\n", days, hours, minutes, seconds, voltageAdjustFactor, activateChargingVoltage, activateInversorVoltage, activateWarningVoltage, chargingTimeSecs, idleMinTimeSecs);
      nextSettingsReport = uptimeSeconds + 60;
    }
    if (nextSerialReport <= uptimeSeconds) {
        bool alarm = voltage < getActivateWarningVoltage();
        Serial.printf("uptime: %dd%02dh%02dm%02ds | voltage=%.3f | inverterEnabled=%d | chargerEnabled=%d | alarm=%d\n", days, hours, minutes, seconds, voltage, inverterEnabled, chargerEnabled, alarm);
        nextSerialReport = uptimeSeconds + 3;
    }
}

void serveStatusPage() {
    String jsonString = "";
    JsonObject object = jsonDocument.to<JsonObject>();
    object["uptime"] = uptimeSeconds;
    object["voltage"] = voltage;
    object["inverterEnabled"] = inverterEnabled;
    object["chargerEnabled"] = chargerEnabled;
    object["alarm"] = voltage < getActivateWarningVoltage();
    serializeJson(jsonDocument, jsonString);
    server.send(200, "application/json", jsonString);
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
    preferences.begin("chargeControl", false);
    
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
        wifiManager.startConfigPortal("ChargingController");
    } else {
        if (!wifiManager.autoConnect("ChargingController")) {
            Serial.println("Failed to connect.");
            delay(10000);
            ESP.restart();
        } else {
            Serial.println("Connected!");
        }
    }
    delay(1000);
    drd.stop();
    server.on("/status", serveStatusPage);
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
    updateUptime();
    updateLedStatus();
    updateVoltage();
    updateChargerStatus();
    updateInversorStatus();
    updateBuzzer();
    printSerialReport();
    server.handleClient();
}
