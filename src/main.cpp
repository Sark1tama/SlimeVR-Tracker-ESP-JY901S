/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "Wire.h"
#include "ota.h"
#include "sensor.h"
#include "configuration.h"
#include "wifihandler.h"
#include "udpclient.h"
#include "defines.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serialcommands.h"
#include "ledstatus.h"

JY901Sensor sensor{};
#define HAS_SECOND_IMU true
JY901Sensor sensor2{};

#ifndef HAS_SECOND_IMU
    EmptySensor sensor2{};
#endif

bool isCalibrating = false;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long IMU_last_ms_1, IMU_start_ms_1 = 0; //millis() timers
unsigned long IMU_last_ms_2, IMU_start_ms_2 = 0; //millis() timers
unsigned long last_battery_sample = 0;
bool secondImuActive = false;

void commandRecieved(int command, void * const commandData, int commandDataLength)
{
    switch (command)
    {
    case COMMAND_CALLIBRATE:
        isCalibrating = true;
        break;
    case COMMAND_SEND_CONFIG:
        sendConfig(getConfigPtr(), PACKET_CONFIG);
        break;
    case COMMAND_BLINK:
        blinking = true;
        blinkStart = now_ms;
        break;
    }
}

void setup()
{
    //wifi_set_sleep_type(NONE_SLEEP_T);
    // Glow diode while loading
    pinMode(LOADING_LED, OUTPUT);
    pinMode(CALIBRATING_LED, OUTPUT);
    digitalWrite(CALIBRATING_LED, LOW);
    digitalWrite(LOADING_LED, HIGH);
    
    Serial.begin(serialBaudRate);
    setUpSerialCommands();
    // join I2C bus
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
#ifdef ESP8266
    Wire.setClockStretchLimit(150000L); // Default streatch limit 150mS
#endif
    Wire.setClock(I2C_SPEED);

    getConfigPtr();
    setConfigRecievedCallback(setConfig);
    setCommandRecievedCallback(commandRecieved);
    // Wait for IMU to boot
    delay(500);
    

#ifdef HAS_SECOND_IMU
    uint8_t first = I2CSCAN::pickDevice(JY_ADDR_1, JY_ADDR_2, true);
    uint8_t second = I2CSCAN::pickDevice(JY_ADDR_1, JY_ADDR_2, false);
    if(first != second) {
        sensor.setupJY901(0, first);
        sensor2.setupJY901(1, second);
        secondImuActive = true;
    } else {
        sensor.setupJY901(0, first);
    }
#else
sensor.setupJY901(0, I2CSCAN::pickDevice(JY_ADDR_1, JY_ADDR_2, true));
#endif

    sensor.motionSetup();
#ifdef HAS_SECOND_IMU
    if(secondImuActive)
        sensor2.motionSetup();
#endif

    setUpWiFi();
    otaSetup(otaPassword);
    digitalWrite(LOADING_LED, LOW);
}

// AHRS loop

void loop()
{
    ledStatusUpdate();
    serialCommandsUpdate();
    wifiUpkeep();
    otaUpdate();
    clientUpdate(&sensor, &sensor2);
    if (isCalibrating)
    {
        sensor.startCalibration(0);
        //sensor2.startCalibration(0);
        isCalibrating = false;
    }
#ifndef UPDATE_IMU_UNCONNECTED
        if(isConnected()) {
#endif
IMU_start_ms_1=millis();
if((IMU_start_ms_1-IMU_last_ms_1)>IMU1_SAMPLE_RATE)
{
    sensor.motionLoop();
    IMU_last_ms_1=millis();
}
#if HAS_SECOND_IMU
    if(secondImuActive)
    IMU_start_ms_2=millis();
    if((IMU_start_ms_2-IMU_last_ms_2)>IMU2_SAMPLE_RATE)
    {
        sensor.motionLoop();
        IMU_last_ms_2=millis();
    }
#endif

#ifndef UPDATE_IMU_UNCONNECTED
        }
#endif
    // Send updates
    now_ms = millis();
#ifndef SEND_UPDATES_UNCONNECTED
    if(isConnected()) {
#endif
        sensor.sendData();
#ifdef HAS_SECOND_IMU
        sensor2.sendData();
#endif
#ifndef SEND_UPDATES_UNCONNECTED
    }
#endif
#ifdef PIN_BATTERY_LEVEL
    if(now_ms - last_battery_sample >= batterySampleRate) {
        last_battery_sample = now_ms;
        float battery = ((float) analogRead(PIN_BATTERY_LEVEL)) * batteryADCMultiplier;
        float batteryLevel = (125 * battery) * 0.5 - 162.5; // Not good probably
        send2Floats(battery, batteryLevel, PACKET_BATTERY_LEVEL);
    }
#endif
}
