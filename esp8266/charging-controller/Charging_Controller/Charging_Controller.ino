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

// IO Pins
#define BATTERY_VOLTAGE_PIN A0
#define ENABLE_INVERSOR_PIN D0
#define ENABLE_CHARGING_PIN D1
#define BUZZER_PIN          D5

// Voltage thresholds
#define ACTIVATE_CHARGING_VOLTAGE 26.0 // 3.250v per cell
#define ACTIVATE_INVERSOR_VOLTAGE 24.4 // 3.050v per cell
#define ACTIVATE_WARNING_VOLTAGE  24.2 // 3.025v per cell

// Settings
#define CHARGING_TIME_SECONDS    14400 // 4 hours
#define IDLE_MIN_TIME_SECONDS     1800 // 30 minutes 
#define VOLTAGE_DIVIDER_R1       68000 // 68k ohm (to battery)
#define VOLTAGE_DIVIDER_R2        6800 // 6k8 ohm (to ground)
#define VOLTAGE_ADJUST_FACTOR    0.911 // Parameter for minor adjust

// Double Reset to reconfigure WiFi
#define DRD_TIMEOUT 10  // Number of seconds after reset during which a subsequent reset will be considered a double reset.
#define DRD_ADDRESS 0   // RTC Memory Address for the DoubleResetDetector to use


float voltage                           = 0.0;
float voltageMultiplier                 = 0.0;
bool inverterEnabled                    = false;
bool chargerEnabled                     = false;
unsigned long uptimeSeconds             = 0;
unsigned long lastMillis                = 0;
unsigned long nextVoltageUpdateSec      = 0;
unsigned long nextSerialReport          = 0;
unsigned long chargerStatusBlockedUntil = 0;

// Web server for metrics
ESP8266WebServer server(80);
StaticJsonDocument<255> jsonDocument;

// Double Reset detector
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

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
        if (voltage <= ACTIVATE_CHARGING_VOLTAGE) {
            chargerStatusBlockedUntil = uptimeSeconds + CHARGING_TIME_SECONDS;
            digitalWrite(ENABLE_CHARGING_PIN, LOW);
            chargerEnabled = true;
        } else {
            chargerStatusBlockedUntil = uptimeSeconds + IDLE_MIN_TIME_SECONDS;
            digitalWrite(ENABLE_CHARGING_PIN, HIGH);
            chargerEnabled = false;
        }
    }
}

void updateInversorStatus() {
    if (voltage >= ACTIVATE_INVERSOR_VOLTAGE) {
        digitalWrite(ENABLE_INVERSOR_PIN, LOW);
        inverterEnabled = true;
    } else {
        digitalWrite(ENABLE_INVERSOR_PIN, HIGH);
        inverterEnabled = false;
    }
}

void updateBuzzer() {
    if (voltage < ACTIVATE_WARNING_VOLTAGE) {
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
    if (nextSerialReport <= uptimeSeconds) {
        unsigned long currentUptime = uptimeSeconds;
        int seconds = currentUptime % 60;
        currentUptime = currentUptime / 60;
        int minutes = currentUptime % 60;
        currentUptime = currentUptime / 60;
        int hours = currentUptime % 24;
        int days = (int) currentUptime / 24;
        bool alarm = voltage < ACTIVATE_WARNING_VOLTAGE;
        Serial.printf("uptime: %dd%02dh%02dm%02ds | voltage=%f | inverterEnabled=%d | chargerEnabled=%d | alarm=%d\n", days, hours, minutes, seconds, voltage, inverterEnabled, chargerEnabled, alarm);
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
    object["alarm"] = (voltage < ACTIVATE_WARNING_VOLTAGE);
    serializeJson(jsonDocument, jsonString);
    server.send(200, "application/json", jsonString);
}

void serveNotFound() {
    server.send(404, "plain/text", "NOT FOUND");
}

void configModeCallback(WiFiManager *localWifiManager) {
    drd.stop();
    Serial.println("Entering Wifi Manager configuration mode...");
    for (int blinks = 0; blinks < 15; blinks++) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
    }
    digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
    
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
    server.onNotFound(serveNotFound);
    server.begin();
    Serial.println("HTTP server started!");

    // Initialize voltage level
    voltageMultiplier = 3.3 * (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / 1023.0 / VOLTAGE_DIVIDER_R2 * VOLTAGE_ADJUST_FACTOR;
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
