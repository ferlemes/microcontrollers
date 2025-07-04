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
#include <Bounce2.h>

// IO Pins
#define BUILTIN_LED_PIN 17
#define FLOW_SENSOR_PIN 3
#define VALVE_RELAY_PIN 4

// "enum"
#define STATUS_WAITING             0
#define STATUS_ALLOW_RECIRCULATION 1
#define STATUS_WAITING_TO_RESET    2

// Default values
#define VALVE_OPEN_INTERVAL_MILLIS 30000
#define FLOW_STOP_INTERVAL_MILLIS  600000

// Global variables
int status = STATUS_WAITING;
unsigned long recirculation_until = 0;
unsigned long wait_cycle_until = 0;
Bounce flow_sensor = Bounce();

// Reset function
void(* resetFunc) (void) = 0;

void setup() {
    Serial.begin(9600);
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    flow_sensor.attach(FLOW_SENSOR_PIN);
    flow_sensor.interval(20);
    pinMode(VALVE_RELAY_PIN, OUTPUT);
    delay(1000);
    Serial.println("Water recirculation valve control - v0.0.1");
    delay(1000);
    Serial.println("Waiting for water flow detection...");
}

void updateLedStatus() {

    int module = 0;
    int on_below = 0;
    switch (status) {
        case STATUS_WAITING:
            module = 2000;
            on_below = 1000;
            break;
        case STATUS_ALLOW_RECIRCULATION:
            module = 1000;
            on_below = 800;
            break;
        case STATUS_WAITING_TO_RESET:
            module = 500;
            on_below = 250;
            break;
    }
    
    unsigned long ledMillis = millis() % module;
    if (ledMillis < on_below) {
        digitalWrite(BUILTIN_LED_PIN, HIGH);
    } else {
        digitalWrite(BUILTIN_LED_PIN, LOW);
    }
}

void loop() {

    updateLedStatus();
    flow_sensor.update();

    switch (status) {
      
        case STATUS_WAITING:
            digitalWrite(VALVE_RELAY_PIN, LOW);
            if (!flow_sensor.read()) {
                Serial.println("Water flow detected. Allowing recirculation...");
                status = STATUS_ALLOW_RECIRCULATION;
                recirculation_until = millis() + VALVE_OPEN_INTERVAL_MILLIS;
            }
            break;
            
        case STATUS_ALLOW_RECIRCULATION:
            digitalWrite(VALVE_RELAY_PIN, HIGH);
            if (recirculation_until < millis()) {
                Serial.println("Recirculation interval reached. Waiting for flow interruption...");
                recirculation_until = 0;
                wait_cycle_until = millis() + FLOW_STOP_INTERVAL_MILLIS;
                status = STATUS_WAITING_TO_RESET;
            }
            break;
            
        case STATUS_WAITING_TO_RESET:
            digitalWrite(VALVE_RELAY_PIN, LOW);
            if (!flow_sensor.read()) {
                wait_cycle_until = millis() + FLOW_STOP_INTERVAL_MILLIS;
            } else {
                if (wait_cycle_until < millis()) {
                    Serial.println("Wait interval reached. Waiting for water flow detection...");
                    wait_cycle_until= 0;
                    status = STATUS_WAITING;
                }
            }
            break;

        default:
            Serial.println("Unexpected status for the controller. Rebooting...");
            delay(3000);
            resetFunc();
            
    }
    
}
