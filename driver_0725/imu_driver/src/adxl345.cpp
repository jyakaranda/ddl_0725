#include "imu_driver/adxl345.h"

ADXL345::ADXL345(unsigned char bus, int accel_scale) : accel_scale(accel_scale), kI2CBus(bus)
{
  error = 0;
}

ADXL345::~ADXL345()
{
  closeADXL345();
}

bool ADXL345::openADXL345()
{
  char fileNameBuffer[32];
  sprintf(fileNameBuffer, "/dev/i2c-%d", kI2CBus);
  ADXLFileDescriptor = open(fileNameBuffer, O_RDWR);
  if (ADXLFileDescriptor < 0)
  {
    error = errno;
    printf("Can not open file /dev/i2c-%d, error: %s", kI2CBus, strerror(error));
    return false;
  }

  if (ioctl(ADXLFileDescriptor, I2C_SLAVE, ADXL345_Addr) < 0)
  {
    error = errno;
    printf("Can not open ADXL345: %x, error: %s", ADXL345_Addr, strerror(error));
    return false;
  }

  return true;
}

void ADXL345::closeADXL345()
{
  if (ADXLFileDescriptor > 0)
  {
    close(ADXLFileDescriptor);
    ADXLFileDescriptor = -1;
  }
}

unsigned char ADXL345::readReg(int reg)
{
  unsigned char toReturn;
  toReturn = i2c_smbus_read_byte_data(ADXLFileDescriptor, reg);
  return toReturn;
}

int ADXL345::readBlockData(int reg, int length, unsigned char *buf)
{
  int size = i2c_smbus_read_i2c_block_data(ADXLFileDescriptor, reg, length, buf);
  return size;
}

unsigned short ADXL345::readWord(int reg, bool flip)
{
  unsigned short toReturn;
  toReturn = i2c_smbus_read_word_data(ADXLFileDescriptor, reg);
  if (flip)
  {
    unsigned short tmp = (unsigned char)toReturn << 8 | (unsigned char)(toReturn >> 8);
    toReturn = tmp;
  }
  return toReturn;
}


int ADXL345::writeReg(int reg, int value)
{
  int toReturn;
  toReturn = i2c_smbus_write_byte_data(ADXLFileDescriptor, reg, value);
  usleep(1000);
  if (toReturn < 0)
  {
    error = errno;
    toReturn = -1;
    printf("Writing error: %s", strerror(error));
  }
  return toReturn;
}
/* 
bool ADXL345::init()
{
  // wake up
  writeReg(ADXL345_REG_POWER_CTL, 0x00);
  writeReg(ADXL345_REG_POWER_CTL, 0x08);
  // set scale 测量范围，+-16g
  if (accel_scale == 16)
  {
    writeReg(ADXL345_REG_DATA_FORMAT, 0x03);
  }
  else if (accel_scale == 8)
  {
    writeReg(ADXL345_REG_DATA_FORMAT, 0x02);
  }
  else if (accel_scale == 4)
  {
    writeReg(ADXL345_REG_DATA_FORMAT, 0x01);
  }
  else if (accel_scale == 2)
  {
    writeReg(ADXL345_REG_DATA_FORMAT, 0x00);
  }
  else
  {
    printf("Unsupported measurement scale!");
    return false;
  }
  // TODO: 作用不明
  writeReg(ADXL345_REG_THRESH_TAP, 0x30);
  writeReg(ADXL345_REG_DUR, 0x10);
  writeReg(ADXL345_REG_LATENT, 0x78);
  writeReg(ADXL345_REG_WINDOW, 0x50);
  int g = -1;
  char axis = 'z';
  float change = 0.5;
  imu_data tmp;
  tmp = getAccel();
  if (axis == 'x')
  {
    writeReg(ADXL345_REG_ACT_INACT_CTL, 0x40);
    g = fabs(tmp.accel.x) + change;
  }
  else if (axis == 'y')
  {
    writeReg(ADXL345_REG_ACT_INACT_CTL, 0x20);
    g = fabs(tmp.accel.y) + change;
  }
  else if (axis == 'z')
  {
    writeReg(ADXL345_REG_ACT_INACT_CTL, 0x10);
    g = fabs(tmp.accel.z) + change;
  }
  else
  {
    printf("Error axis!");
    return false;
  }
  writeReg(ADXL345_REG_THRESH_ACT, g / 0.0625);

  tmp = getAccel();
  change = 0.1;
  if (axis == 'x')
  {
    writeReg(ADXL345_REG_ACT_INACT_CTL, 0x40);
    g = fabs(tmp.accel.x) + change;
  }
  else if (axis == 'y')
  {
    writeReg(ADXL345_REG_ACT_INACT_CTL, 0x20);
    g = fabs(tmp.accel.y) + change;
  }
  else if (axis == 'z')
  {
    writeReg(ADXL345_REG_ACT_INACT_CTL, 0x10);
    g = fabs(tmp.accel.z) + change;
  }
  else
  {
    printf("Error axis!");
    return false;
  }
  writeReg(ADXL345_REG_THRESH_INACT, g / 0.0625);

  writeReg(ADXL345_REG_TIME_INACT, 0x01);
  writeReg(ADXL345_REG_THRESH_FF, 0x06);
  writeReg(ADXL345_REG_TIME_FF, 0x14);

  return true;
} */

bool ADXL345::init()
{
  writeReg(ADXL345_REG_POWER_CTL, 0x08);
  usleep(5);
  writeReg(ADXL345_REG_DATA_FORMAT, 0x09);
  usleep(5);
  writeReg(ADXL345_REG_BW_RATE, 0x09);
  usleep(5);
  // writeReg(ADXL345_REG_INT_ENABLE, 0x80);
  // writeReg(ADXL345_REG_OFSX, 0x00);
  // writeReg(ADXL345_REG_OFSY, 0x00);
  // writeReg(ADXL345_REG_OFSZ, 0x05);
  return true;
}

imu_data ADXL345::getAccel()
{
  imu_data imu;
  float scale_factor;
  // scale_factor = this->accel_scale * 2 / 1024;
  scale_factor = 3.9;
  unsigned char *data;
  data = new unsigned char[7];
  // int size = readBlockData(ADXL345_REG_DATAX0, 6, data);
  // if (size < 6)
  // {
  //   printf("Error in get accel. read block data size less than 6 \n");
  //   return imu;
  // }
  // float x = -1.0 * (short)((int)data[1] << 8 | data[0]) * ADXL345_SCALE;
  // float y = -1.0 * (short)((int)data[3] << 8 | data[2]) * ADXL345_SCALE;
  // float z = -1.0 * (short)((int)data[5] << 8 | data[4]) * ADXL345_SCALE;
  float x = 1.0 * (short)(readReg(ADXL345_REG_DATAX0) | ((int)readReg(ADXL345_REG_DATAX1) << 8)) * ADXL345_SCALE;
  float y = 1.0 * (short)(readReg(ADXL345_REG_DATAY0) | ((int)readReg(ADXL345_REG_DATAY1) << 8)) * ADXL345_SCALE;
  float z = 1.0 * (short)(readReg(ADXL345_REG_DATAZ0) | ((int)readReg(ADXL345_REG_DATAZ1) << 8)) * ADXL345_SCALE;
  // float x = (float)(1 * (short)readWord(ADXL345_REG_DATAX0, false)) * ADXL345_SCALE;
  // float y = (float)(1 * (short)readWord(ADXL345_REG_DATAY0, false)) * ADXL345_SCALE;
  // float z = (float)(1 * (short)readWord(ADXL345_REG_DATAZ0, false)) * ADXL345_SCALE;
  imu.accel.x = x;
  imu.accel.y = y;
  imu.accel.z = z;

  return imu;
}