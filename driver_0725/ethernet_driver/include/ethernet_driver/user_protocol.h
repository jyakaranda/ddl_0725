#ifndef __USER_PROTCOL_H_
#define __USER_PROTCOL_H_

#include <stdio.h>

/*protocol*/
enum _ptl_type
{
	VAL_START = 0,
	VAL_POSE,
	VAL_IMU_RAW,
	VAL_VEL,
	VAL_PID,
	VAL_IMU,
	VAL_ARM_POSE,
	VAL_ARM_AXIS,
	VAL_END
};
#define _PACKET_SYN_CODE_START 0xFA
static const uint16_t PACKET_SIZE = 96;
#pragma pack(1)
typedef struct _packet_data
{
	uint8_t syn;
	uint8_t type;
	union {
		struct
		{
			float liner[3], angular[4];
		} vel;
		struct
		{
			bool rot_ok, acc_ok, mag_ok;
			double rot[3], acc[3], mag[3];
		} imu;
		float pid[3];
	} dat;
	uint8_t syn_CR;
	uint8_t syn_LF;
} PacketData;
#pragma pack()

#endif /* __USER_PROTCOL_H_ */
