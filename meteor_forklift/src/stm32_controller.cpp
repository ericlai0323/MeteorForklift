#include <stm32_controller.h>

void STM32Controller::SendData(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10, float data11, float data12)
{
    isnan(data4) ? data4 = 0.0 : data4 = data4;
    uint8_t tbuf[53];
    unsigned char *p;

    p = (unsigned char *)&data1;
    tbuf[4] = (unsigned char)(*(p + 3));
    tbuf[5] = (unsigned char)(*(p + 2));
    tbuf[6] = (unsigned char)(*(p + 1));
    tbuf[7] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data2;
    tbuf[8] = (unsigned char)(*(p + 3));
    tbuf[9] = (unsigned char)(*(p + 2));
    tbuf[10] = (unsigned char)(*(p + 1));
    tbuf[11] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data3;
    tbuf[12] = (unsigned char)(*(p + 3));
    tbuf[13] = (unsigned char)(*(p + 2));
    tbuf[14] = (unsigned char)(*(p + 1));
    tbuf[15] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data4;
    tbuf[16] = (unsigned char)(*(p + 3));
    tbuf[17] = (unsigned char)(*(p + 2));
    tbuf[18] = (unsigned char)(*(p + 1));
    tbuf[19] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data5;
    tbuf[20] = (unsigned char)(*(p + 3));
    tbuf[21] = (unsigned char)(*(p + 2));
    tbuf[22] = (unsigned char)(*(p + 1));
    tbuf[23] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data6;
    tbuf[24] = (unsigned char)(*(p + 3));
    tbuf[25] = (unsigned char)(*(p + 2));
    tbuf[26] = (unsigned char)(*(p + 1));
    tbuf[27] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data7;
    tbuf[28] = (unsigned char)(*(p + 3));
    tbuf[29] = (unsigned char)(*(p + 2));
    tbuf[30] = (unsigned char)(*(p + 1));
    tbuf[31] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data8;
    tbuf[32] = (unsigned char)(*(p + 3));
    tbuf[33] = (unsigned char)(*(p + 2));
    tbuf[34] = (unsigned char)(*(p + 1));
    tbuf[35] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data9;
    tbuf[36] = (unsigned char)(*(p + 3));
    tbuf[37] = (unsigned char)(*(p + 2));
    tbuf[38] = (unsigned char)(*(p + 1));
    tbuf[39] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data10;
    tbuf[40] = (unsigned char)(*(p + 3));
    tbuf[41] = (unsigned char)(*(p + 2));
    tbuf[42] = (unsigned char)(*(p + 1));
    tbuf[43] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data11;
    tbuf[44] = (unsigned char)(*(p + 3));
    tbuf[45] = (unsigned char)(*(p + 2));
    tbuf[46] = (unsigned char)(*(p + 1));
    tbuf[47] = (unsigned char)(*(p + 0));

    p = (unsigned char *)&data12;
    tbuf[48] = (unsigned char)(*(p + 3));
    tbuf[49] = (unsigned char)(*(p + 2));
    tbuf[50] = (unsigned char)(*(p + 1));
    tbuf[51] = (unsigned char)(*(p + 0));

    // fun:功能字 0XA0~0XAF
    // data:数据缓存区，48字节
    // len:data区有效数据个数

    uint8_t len = 48;

    tbuf[len + 4] = 0; // 校验位置零
    tbuf[0] = 0XAA;    // 帧头
    tbuf[1] = 0XAA;    // 帧头
    tbuf[2] = 0XF1;    // 功能字
    tbuf[3] = len;     // 数据长度
    for (uint8_t i = 0; i < len + 4; i++)
        tbuf[len + 4] += tbuf[i]; // 计算和校验

    try
    {
        SerialPort.write(tbuf, len + 5); // 发送数据下位机
    }
    catch (serial::IOException &e)
    {
        printf("Unable to send data through serial port");
    }
}

float STM32Controller::BinaryToFloat(byte m0, byte m1, byte m2, byte m3)
{
    // 求符號位
    float sig = 1.;
    if (m0 >= 128.)
        sig = -1.;

    // 求價碼
    float jie = 0.;
    if (m0 >= 128.)
    {
        jie = m0 - 128.;
    }
    else
    {
        jie = m0;
    }
    jie = jie * 2.;
    if (m1 >= 128.)
        jie += 1.;

    jie -= 127.;
    // 求尾码
    float tail = 0.;
    if (m1 >= 128.)
        m1 -= 128.;
    tail = m3 + (m2 + m1 * 256.) * 256.;
    tail = (tail) / 8388608; //   8388608 = 2^23

    float f = sig * pow(2., jie) * (1 + tail);

    return f;
}

size_t STM32Controller::GetBufferSize()
{
    return SerialPort.available();
}

void STM32Controller::ReadBufferData(uint8_t Buffer[], size_t N)
{
    SerialPort.read(Buffer, N);
}

int STM32Controller::SetSerialPort()
{
    serial::Timeout TimeOut = serial::Timeout::simpleTimeout(100); // Create timeout

    SerialPort.setPort("/dev/ttyUSB0"); // Set the name of the serial port to open
    SerialPort.setBaudrate(115200);     // Set the baud rate of serial communication
    SerialPort.setTimeout(TimeOut);     // Serial set timeout

    try // Open serial port
    {
        SerialPort.open();
    }
    catch (serial::IOException &e)
    {
        printf("Unable to open port");
        printf("$ sudo chmod 777 /dev/ttyUSB0");

        return -1;
    }

    if (SerialPort.isOpen())
    {
        printf("/dev/ttyUSB0 is opened");
        return 0;
    }
    else
    {
        printf("$ sudo chmod 777 /dev/ttyUSB0");
        return -1;
    }
}

void STM32Controller::ClearBuffer()
{
    for (uint8_t j = 0; j < 3; j++)
    {
        SendData(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
}

void STM32Controller::CloseBuffer()
{
    SerialPort.close();
}

void STM32Controller::ReciveData()
{
    float Data[16];
    uint8_t Buffer[65];
    size_t n = GetBufferSize();
    if (n > 0)
    {
        STM32Controller::ReadBufferData(Buffer, n);
        if (Buffer[0] == 0XAA && Buffer[1] == 0XAA && Buffer[2] == 0XF1)
        {
            uint8_t sum;
            for (uint8_t j = 0; j < 64; j++)
            {
                sum += Buffer[j]; // 计算校验和
            }

            if (sum == Buffer[64])
            {
                Data[1] = BinaryToFloat(Buffer[4], Buffer[5], Buffer[6], Buffer[7]);      // 電機啟動停止控制位元（1/0 啟動/停止）
                Data[2] = BinaryToFloat(Buffer[8], Buffer[9], Buffer[10], Buffer[11]);    // 前輪線速度
                Data[3] = BinaryToFloat(Buffer[12], Buffer[13], Buffer[14], Buffer[15]);  // 前輪轉角
                Data[4] = BinaryToFloat(Buffer[16], Buffer[17], Buffer[18], Buffer[19]);  // 繞X軸角速度 gyro_Roll 原始数值
                Data[5] = BinaryToFloat(Buffer[20], Buffer[21], Buffer[22], Buffer[23]);  // 繞Y軸角速度 gyro_Pitch 原始数值
                Data[6] = BinaryToFloat(Buffer[24], Buffer[25], Buffer[26], Buffer[27]);  // 繞Z軸角速度 gyro_Yaw 原始数值
                Data[7] = BinaryToFloat(Buffer[28], Buffer[29], Buffer[30], Buffer[31]);  // X軸加速度 accel_x 原始数值
                Data[8] = BinaryToFloat(Buffer[32], Buffer[33], Buffer[34], Buffer[35]);  // Y軸加速度 accel_y 原始数值
                Data[9] = BinaryToFloat(Buffer[36], Buffer[37], Buffer[38], Buffer[39]);  // Z軸加速度 accel_z 原始数值
                Data[10] = BinaryToFloat(Buffer[40], Buffer[41], Buffer[42], Buffer[43]); // Yaw Z轴角度
                Data[11] = BinaryToFloat(Buffer[44], Buffer[45], Buffer[46], Buffer[47]); // 電池電壓              24-25   <24.3  low
                Data[12] = BinaryToFloat(Buffer[48], Buffer[49], Buffer[50], Buffer[51]); // 紅色緊急開關位元0/1 運行/停止
                Data[13] = BinaryToFloat(Buffer[52], Buffer[53], Buffer[54], Buffer[55]); // 起重電機編碼器原始數據（未轉換） 如果有需要可以添加發送指令去清0，上面的發送命令還有剩餘  gearrate 30  dt 5 ms
                Data[14] = BinaryToFloat(Buffer[56], Buffer[57], Buffer[58], Buffer[59]); // 起重電機下行限位開關（用於校準） 1代表開關被壓住
                Data[15] = BinaryToFloat(Buffer[60], Buffer[61], Buffer[62], Buffer[63]); // 起重電機上行限位開關（用於校準） 1代表開關被壓住
            }
            sum = 0;
            memset(Buffer, 0, sizeof(uint8_t) * 65);
        }
        AngularVelocityX = Data[4] * 0.001064;   // 转换成 rad/s
        AngularVelocityY = Data[5] * 0.001064;   // 转换成 rad/s
        AngularVelocityZ = Data[6] * 0.001064;   // 转换成 rad/s
        AccelerationWheelSpeed = Data[7] / 2048; // 转换成 g	,重力加速度定义为1g, 等于9.8米每平方秒
        AccelerationY = Data[8] / 2048;          // 转换            // if(Data1)
        AccelerationZ = Data[9] / 2048;
    }
    ClearBuffer();
    CloseBuffer();
}