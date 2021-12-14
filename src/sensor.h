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

#ifndef SLIMEVR_SENSOR_H_
#define SLIMEVR_SENSOR_H_

#include <JY901.h>
#include <quat.h>
#include <vector3.h>
#include "configuration.h"
#include "defines.h"

#define DATA_TYPE_NORMAL 1
#define DATA_TYPE_CORRECTION 2

class Sensor {
    public:
        Sensor() = default;
        virtual ~Sensor() = default;
        virtual void motionSetup() = 0;
        virtual void motionLoop() = 0;
        virtual void sendData() = 0;
        virtual void startCalibration(int calibrationType) = 0;
        bool isWorking() {
            return working;
        }
    protected:
        Quat quaternion {};
        Quat sensorOffset {Quat(Vector3(0, 0, 1), IMU_ROTATION)};
        Quat lastQuatSent {};
        bool working {false};
        uint8_t sensorId {0};
};

class EmptySensor : public Sensor {
    public:
        EmptySensor() = default;
        ~EmptySensor() override = default;
        void motionSetup() override final;
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
};

class JY901Sensor : public Sensor {
    public:
        JY901Sensor() = default;
        ~JY901Sensor() override  = default;
        void motionSetup() override final;
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
        void setupJY901(uint8_t sensorId = 0, uint8_t addr = 0x50);
    private:
        JY901 imu {JY901(0x50)};
        bool newData {false};
        uint8_t addr = 0x50;
        uint8_t tap;
        float a[3];
};

class JY901Sensor : public Sensor {
    public:
        JY901Sensor() = default;
        ~JY901Sensor() override  = default;
        void motionSetup() override final;
        void motionLoop() override final;
        void sendData() override final;
        void startCalibration(int calibrationType) override final;
        void setupJY901(uint8_t sensorId = 0, uint8_t addr = 0x50);
    private:
        JY901 imu {JY901(0x50)};
        bool newData {false};
        uint8_t addr = 0x50;
        uint8_t tap;
        float a[3];
};

#endif //SLIMEVR_SENSOR_H_
