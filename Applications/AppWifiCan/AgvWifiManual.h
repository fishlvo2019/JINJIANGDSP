/*
 * AgvWfifCan.h
 *
 *  Created on: 2018��7��19��
 *      Author: Long
 */

#ifndef AGVWIFIMANUAL_H_
#define AGVWIFIMANUAL_H_

#include "FHAGV.h"


struct WifiRemoter_t
{
	Uint16 FunctionTypeId;//0	��Agv����������
	int16 Speed ;//1			0x01Ϊǰ��0x11Ϊ����
	int16 MaxAngularSpd ;//2	0x01Ϊ˳ʱ�룬0x11Ϊ��ʱ��
	Uint16 ManualSwitch;//3		0x20Ϊ�Զ���0x21�ֶ�
	Uint16 SpeedGrade;//4		�ٶȵȼ�1-4
	Uint16 CheckSum;//5			У���
};

FH_ERROR_CODE WifiManualMove_tsk(void);

#endif /* AGVWFIFCAN_H_ */
