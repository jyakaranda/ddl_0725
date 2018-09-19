#ifndef __ADXL345__
#define __ADXL345__

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <cmath>
#include "imu_driver/itg3205.h"

// hw579(itg3205+ADXL345), itg3205: 0x68, adxl345: 0x53, hmc5883: 0x1e

#define ADXL345_Addr 0x53 //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

//定义ADXL345内部地址********************
#define ADXL345_REG_DEVID (0x00)          // Device ID
#define ADXL345_REG_THRESH_TAP (0x1D)     // Tap threshold
#define ADXL345_REG_OFSX (0x1E)           // X-axis offset
#define ADXL345_REG_OFSY (0x1F)           // Y-axis offset
#define ADXL345_REG_OFSZ (0x20)           // Z-axis offset
#define ADXL345_REG_DUR (0x21)            // Tap duration
#define ADXL345_REG_LATENT (0x22)         // Tap latency
#define ADXL345_REG_WINDOW (0x23)         // Tap window
#define ADXL345_REG_THRESH_ACT (0x24)     // Activity threshold
#define ADXL345_REG_THRESH_INACT (0x25)   // Inactivity threshold
#define ADXL345_REG_TIME_INACT (0x26)     // Inactivity time
#define ADXL345_REG_ACT_INACT_CTL (0x27)  // Axis enable control for activity and inactivity detection
#define ADXL345_REG_THRESH_FF (0x28)      // Free-fall threshold
#define ADXL345_REG_TIME_FF (0x29)        // Free-fall time
#define ADXL345_REG_TAP_AXES (0x2A)       // Axis control for single/double tap
#define ADXL345_REG_ACT_TAP_STATUS (0x2B) // Source for single/double tap
#define ADXL345_REG_BW_RATE (0x2C)        // Data rate and power mode control
#define ADXL345_REG_POWER_CTL (0x2D)      // Power-saving features control
#define ADXL345_REG_INT_ENABLE (0x2E)     // Interrupt enable control
#define ADXL345_REG_INT_MAP (0x2F)        // Interrupt mapping control
#define ADXL345_REG_INT_SOURCE (0x30)     // Source of interrupts
#define ADXL345_REG_DATA_FORMAT (0x31)    // Data format control
#define ADXL345_REG_DATAX0 (0x32)         // X-axis data 0
#define ADXL345_REG_DATAX1 (0x33)         // X-axis data 1
#define ADXL345_REG_DATAY0 (0x34)         // Y-axis data 0
#define ADXL345_REG_DATAY1 (0x35)         // Y-axis data 1
#define ADXL345_REG_DATAZ0 (0x36)         // Z-axis data 0
#define ADXL345_REG_DATAZ1 (0x37)         // Z-axis data 1
#define ADXL345_REG_FIFO_CTL (0x38)       // FIFO control
#define ADXL345_REG_FIFO_STATUS (0x39)    // FIFO status
#define ADXL345_SCALE 0.0390625 		//0.00390625  0.03835616438
//****************************

class ADXL345
{
  public:
    ADXL345(unsigned char bus=1, int accel_scale=16);
    ~ADXL345();
    bool openADXL345();
    void closeADXL345();
    bool init();
    imu_data getAccel();

  private:
    int writeReg(const int reg, const int value);
    unsigned char readReg(const int reg);
    int readBlockData(const int reg, const int length, unsigned char* buf);
    unsigned short readWord(int reg, bool flip);
    unsigned char kI2CBus;
    int ADXLFileDescriptor;
    int error;

    const int accel_scale;
};

#endif