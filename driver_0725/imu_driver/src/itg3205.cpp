#include "imu_driver/itg3205.h"

ITG3205::ITG3205(unsigned char bus) : kI2CBus(bus)
{
  error = 0;
}

ITG3205::~ITG3205()
{
  closeITG3205();
}

bool ITG3205::openITG3205()
{
  char fileNameBuffer[32];
  sprintf(fileNameBuffer, "/dev/i2c-%d", kI2CBus);
  ITGFileDescriptor = open(fileNameBuffer, O_RDWR);
  if (ITGFileDescriptor < 0)
  {
    error = errno;
    printf("Can not open file /dev/i2c-%d, error: %s", kI2CBus, strerror(error));
    return false;
  }

  if (ioctl(ITGFileDescriptor, I2C_SLAVE, ITG3205_Addr) < 0)
  {
    error = errno;
    printf("Can not open ITG3205: %x, error: %s", ITG3205_Addr, strerror(error));
    return false;
  }

  return true;
}

void ITG3205::closeITG3205()
{
  if (ITGFileDescriptor > 0)
  {
    close(ITGFileDescriptor);
    ITGFileDescriptor = -1;
  }
}

unsigned char ITG3205::readReg(int reg)
{
  unsigned char toReturn;
  toReturn = i2c_smbus_read_byte_data(ITGFileDescriptor, reg);
  return toReturn;
}

int ITG3205::writeReg(int reg, int value)
{
  int toReturn;
  toReturn = i2c_smbus_write_byte_data(ITGFileDescriptor, reg, value);
  usleep(1000);
  if (toReturn < 0)
  {
    error = errno;
    toReturn = -1;
    printf("Writing error: %s", strerror(error));
  }
  return toReturn;
}

unsigned short ITG3205::readWord(int reg, bool flip)
{
  unsigned short toReturn;
  toReturn = i2c_smbus_read_word_data(ITGFileDescriptor, reg);
  if (flip)
  {
    unsigned short tmp = (unsigned char)toReturn << 8 | (unsigned char)(toReturn >> 8);
    toReturn = tmp;
  }
  return toReturn;
}

int ITG3205::readBlockData(const int reg, const int length, unsigned char *buf)
{
  int size = i2c_smbus_read_i2c_block_data(ITGFileDescriptor, reg, length, buf);
  return size;
}

bool ITG3205::init()
{
  // set power management, 0x80: Reset device and internal registers to the power-up-default settings; 0x00:
  writeReg(ITG3205_PWR_M, 0x00);
  usleep(5);
  // set Sample Rate Divider,Fsample = Finternal / (divider+1), where Finternal is either 1kHz or 8kHz, 0x07: Fsample 125Hz if Finternel is 1kHz.
  writeReg(ITG3205_SMPL, 0x13);
  usleep(5);
  // set Low Pass Filter Bandwidth and Internal Sample Rate. 0x19: +-2000deg/sec, 188Hz low pass filter , 1kHz internel sample rate; 0x1e:
  writeReg(ITG3205_DLPF, 0x18 | 0x06);
  usleep(5);
  // set Interrupt Configuration.
  writeReg(ITG3205_INT_C, 0x20 | 0x04 | 0x01);
  usleep(5);
  // writeReg(ITG3205_PWR_M, 0x03);
  // usleep(5);

  return true;
}

imu_data ITG3205::getGyro()
{
  imu_data imu;
  unsigned char *data;
  data = new unsigned char[6];
  // int size = readBlockData(ITG3205_GX_H, 6, data);
  // if (size < 6)
  // {
  //   printf("Error in get gyro. read block data size less than 6 \n");
  //   return imu;
  // }
  // float x = (1 * (short)((int)data[0] << 8) | data[1]) * ITG3205_SCALE;
  // float y = (1 * (short)((int)data[2] << 8) | data[3]) * ITG3205_SCALE;
  // float z = (1 * (short)((int)data[4] << 8) | data[5]) * ITG3205_SCALE;
  float x = (float)(1 * (short)(((int)readReg(ITG3205_GX_H) << 8) | readReg(ITG3205_GX_L))) * ITG3205_SCALE;
  float y = (float)(1 * (short)(((int)readReg(ITG3205_GY_H) << 8) | readReg(ITG3205_GY_L))) * ITG3205_SCALE;
  float z = (float)(1 * (short)(((int)readReg(ITG3205_GZ_H) << 8) | readReg(ITG3205_GZ_L))) * ITG3205_SCALE;
  // float x = (float)(1 * (short)readWord(ITG3205_GX_H, false)) * ITG3205_SCALE;
  // float y = (float)(1 * (short)readWord(ITG3205_GY_H, false)) * ITG3205_SCALE;
  // float z = (float)(1 * (short)readWord(ITG3205_GZ_H, false)) * ITG3205_SCALE;

  imu.gyro.x = x;
  imu.gyro.y = y;
  imu.gyro.z = z;
  return imu;
}

imu_data ITG3205::getTemp()
{
  imu_data imu;
  short tmp = readReg(ITG3205_TMP_L) | (readReg(ITG3205_TMP_H) << 8);

  imu.temp = 35 + (tmp + 13200) / 280.0;
  return imu;
}