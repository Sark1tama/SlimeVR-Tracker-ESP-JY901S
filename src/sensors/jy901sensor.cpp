/*
    MIT License

    Copyright (c) 2021 Sark1tama

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#include "sensors/jy901sensor.h"
#include "network/network.h"
#include "ledmgr.h"

void JY901Sensor::setupJY901(uint8_t sensorId, uint8_t addr)
{
    this->addr = addr;
    this->sensorId = sensorId;
    this->sensorOffset = {Quat(Vector3(0, 0, 1), sensorId == 0 ? IMU_ROTATION : SECOND_IMU_ROTATION)};
}

void JY901Sensor::motionSetup()
{
    if (!imu.StartIIC(addr))
    {
        Serial.print("[ERR] IMU: Can't connect to ");
        Serial.println(getIMUNameByType(sensorType));
        LEDManager::signalAssert();
        return;
    }
    Serial.print("[NOTICE] IMU: Connected to ");
    Serial.print(getIMUNameByType(sensorType));
    Serial.print(" on 0x");
    Serial.println(addr, HEX);
    working = true;
}
void JY901Sensor::motionLoop()
{
    imu.GetQuater();
    // imu.GetAcc();
    quaternion.set((float)imu.stcQuater.q1 / 32768, (float)imu.stcQuater.q2 / 32768, (float)imu.stcQuater.q3 / 32768, (float)imu.stcQuater.q0 / 32768);
    quaternion *= sensorOffset;
    // a[0] = (float)imu.stcAcc.a[0]/32768*16;
    // a[1] = (float)imu.stcAcc.a[1]/32768*16;
    // a[2] = (float)imu.stcAcc.a[2]/32768*16;
    // newData = true;
    if (!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion))
    {
        newData = true;
        lastQuatSent = quaternion;
    }
    tap = imu.GetTapDetector();
}

void JY901Sensor::sendData()
{
    if (newData)
    {
        newData = false;
        Network::sendRotationData(&quaternion, DATA_TYPE_NORMAL, 1, sensorId);
#ifdef FULL_DEBUG
        Serial.print("[DBG] Quaternion: ");
        Serial.print(quaternion.x);
        Serial.print(",");
        Serial.print(quaternion.y);
        Serial.print(",");
        Serial.print(quaternion.z);
        Serial.print(",");
        Serial.println(quaternion.w);
#endif
    }
    if (tap != 0)
    {
        Network::sendTap(tap, sensorId);
        tap = 0;
#ifdef FULL_DEBUG
        Serial.print("[DBG] Tap: ");
        Serial.println(tap);
#endif
    }
}
void JY901Sensor::startCalibration(int calibrationType)
{
    LEDManager::pattern(CALIBRATING_LED, 20, 20, 10);
    LEDManager::blink(CALIBRATING_LED, 2000);
    imu.Unlock();
    delay(2000);
    LEDManager::on(CALIBRATING_LED);
    imu.SetDirection(1);
    imu.SetCalsw(calibrationType);
    delay(6000);
    LEDManager::off(CALIBRATING_LED);
    imu.Save(0);
    delay(100);
}