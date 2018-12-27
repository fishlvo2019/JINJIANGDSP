/*
 * AgvWfifCan.h
 *
 *  Created on: 2018年7月19日
 *      Author: Long
 */

#ifndef AGVWIFIMANUAL_H_
#define AGVWIFIMANUAL_H_

#include "FHAGV.h"


struct WifiRemoter_t
{
	Uint16 FunctionTypeId;//0	给Agv的命令类型
	int16 Speed ;//1			0x01为前进0x11为后退
	int16 MaxAngularSpd ;//2	0x01为顺时针，0x11为逆时针
	Uint16 ManualSwitch;//3		0x20为自动，0x21手动
	Uint16 SpeedGrade;//4		速度等级1-4
	Uint16 CheckSum;//5			校验和
};

FH_ERROR_CODE WifiManualMove_tsk(void);

#endif /* AGVWFIFCAN_H_ */
