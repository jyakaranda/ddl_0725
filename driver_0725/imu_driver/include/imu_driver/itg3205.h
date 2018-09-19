#ifndef __ITG3205__
#define __ITG3205__

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>

// ITG3205(ITG3205+ADXL345), itg3205: 0x68, adxl345: 0x53, hmc5883: 0x1e

#define ITG3205_MAGIC 'K'

#define ITG3205_Addr 0x68 //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

//定义ITG3205内部地址（参考芯片手册）********************
#define ITG3205_WHO 0x00  // who am i
#define ITG3205_SMPL 0x15 // 采样率
#define ITG3205_DLPF 0x16 // 低通滤波，典型值 0x1E（5Hz 滤波频率，1kHz 内部采样频率）
#define ITG3205_INT_C 0x17
#define ITG3205_INT_S 0x1A
#define ITG3205_TMP_H 0x1B // 传感器寄存器，温度
#define ITG3205_TMP_L 0x1C
#define ITG3205_GX_H 0x1D // 传感器寄存器，gyro xyz
#define ITG3205_GX_L 0x1E
#define ITG3205_GY_H 0x1F
#define ITG3205_GY_L 0x20
#define ITG3205_GZ_H 0x21
#define ITG3205_GZ_L 0x22
#define ITG3205_PWR_M 0x3E
#define ITG3205_SCALE 0.00121414209  //0.00121414209  0.0695652174=14.375 rad/s
//****************************

union imu_data {
    struct
    {
        float x;
        float y;
        float z;
    } accel;
    struct
    {
        float x;
        float y;
        float z;
    } gyro;
    float temp;
};

class ITG3205
{
  public:
    ITG3205(unsigned char bus=1);
    ~ITG3205();
    bool openITG3205();
    void closeITG3205();
    bool init();
    imu_data getGyro();   
    imu_data getTemp();

  private:
    int writeReg(int reg, int value);
    unsigned char readReg(int reg);
    unsigned short readWord(int reg, bool flip=false);
    int readBlockData(const int reg, const int length, unsigned char* buf);
    unsigned char kI2CBus;
    int ITGFileDescriptor;
    int error;

};

#endif