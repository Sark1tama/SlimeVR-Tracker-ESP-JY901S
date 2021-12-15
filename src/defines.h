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
#ifndef SLIMEVR_DEFINES_H_
#define SLIMEVR_DEFINES_H_

#include "consts.h"
#include "debug.h"

// Set parameters of IMU and board used
#define IMU IMU_JY901
#define BOARD BOARD_CUSTOM
#define IMU_ROTATION DEG_0
#define SECOND_IMU_ROTATION DEG_0

#define IMU_NAME "JY901"
#define IMU_HAS_ACCELL true
#define IMU_HAS_GYRO true
#define IMU_HAS_MAG true
#define I2C_SPEED 400000
#define IMU1_SAMPLE_RATE 4
#define IMU2_SAMPLE_RATE 4

#define PIN_IMU_SDA 21
#define PIN_IMU_SCL 22
#define PIN_BATTERY_LEVEL 34
#define JY_ADDR_1 0x50
#define JY_ADDR_2 0x51

#define LOADING_LED 32
#define CALIBRATING_LED 33
#define STATUS_LED 25

#define batteryADCMultiplier 3.3 / 4095.0 * 2.1

#endif