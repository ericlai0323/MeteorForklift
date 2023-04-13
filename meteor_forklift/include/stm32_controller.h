#pragma once

#include <serial/serial.h>
#include <iostream>
#include <math.h>

class STM32Controller
{
private:
    typedef unsigned char byte;
    serial::Serial SerialPort;
    float AngularVelocityX, AngularVelocityY, AngularVelocityZ;
    float AccelerationWheelSpeed, AccelerationY, AccelerationZ;
public:
    void SendData(float data1, float data2, float data3, float data4, float data5, float data6,
                  float data7, float data8, float data9, float data10, float data11, float data12);
    float BinaryToFloat(byte m0, byte m1, byte m2, byte m3);
    size_t GetBufferSize();
    void ReadBufferData(uint8_t buffer[], size_t n);
    int SetSerialPort();
    void ClearBuffer();
    void CloseBuffer();
    void ReciveData();
};
