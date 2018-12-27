/*
 * AgvAppActionState.h
 *
 *  Created on: 2018��1��24��
 *      Author: hwei
 */

#ifndef AGVAPPACTIONSTATE_H_
#define AGVAPPACTIONSTATE_H_


//AGV motion status
#define AGV_MOTION_STOP 0
#define AGV_MOTION_TURNING 1
#define AGV_MOTION_LIFTING 2
#define AGV_MOTION_SETDOWN 3
#define AGV_MOTION_MOVING 4
#define AGV_MOTION_PAUSE 5
#define AGV_MOTION_MOVE_ARC_IN 6
#define AGV_MOTION_MOVE_ARC_OUT 7
#define AGV_MOTION_POD_ZERO 8
#define AGV_MOTION_POD_ROTATE AGV_MOTION_POD_ZERO
#define AGV_MOTION_POD_LS 9
#define AGV_MOTION_UNCERTAIN 10
#define AGV_MOTION_SOFTSTOP 11
#define AGV_MOTION_PODCHANGE 12
#define AGV_MOTION_TURNING_ALIGN 13
#define AGV_MOTION_POD_JOGGING 14
#define AGV_MOTION_GYROZERO 15
#define AGV_MOTION_SEARCHCODE_MOVE 16
#define AGV_MOTION_SEARCHCODE_TURNING 17
#define AGV_MOTION_MEASUREDIA_MOVE 18
#define AGV_MOTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF 19//����ǰ����������ͷ
#define AGV_MOTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF 20//���������������ͷ
#define AGV_MOTION_SLAVEPC_VERSION 21
#define AGV_MOTION_EXIT_CHARGEZONE 22
#define AGV_MOTION_KEEPPACEWITH_AGVFRONT 23
#define AGV_MOTION_INITIAL_AFTERLIFT_CHECKSHELF 24
#define AGV_MOTION_RETURN_BACK 25 //HR,add,20160126

//panj-add
#define AGV_MOTION_JOYSTICK_OMEGA 26
#define AGV_MOTION_JOYSTICK_VEL 27
#define AGV_MOTION_JOYSTICK_POS 28
//HR,add,20160805
#define AGV_MOTION_SHELF_GUIDE 29 //Ѱ�һ���
#define AGV_MOTION_CHECK_GUIDE_COND 30 //����Ƿ�����Ѱ�һ��ܵ�����

//HR,add,20160907
#define AGV_MOTION_POWERRESTART_DSP 31//����DSP
#define AGV_MOTION_POWERRESTART_CAMERA 32//����Camera
#define AGV_MOTION_POWERRESTART_IPC 33//����IPC

#define AGV_MOTION_QUERY_ENERGY 34 //��ѯ����


#define AGV_MOTION_TURNING_WITHUP 35

#define AGV_MOTION_ENABLE_DRIVER_MOTOR 36 //ʹ����������������涯�����
#define AGV_MOTION_DISABLE_DRIVER_MOTOR 37 //�ر���������������涯�����
#define AGV_MOTION_ENABLE_UPDOWN_MOTOR 38 //ʹ���������
#define AGV_MOTION_DISABLE_UPDOWN_MOTOR 39 //�ر��������

//add sxp 20161029
#define AGV_MOTION_ZERORETURN 40 //��ԭ��

#define AGV_MOTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF_PH2 41
//���ܸ�λ���׶�2
#define AGV_MOTION_SHELF_GUIDE_PH2 42 

//���� ��HR��20170525
#define AGV_MOTION_BACK_OFF 43
//���ӵ���
#define AGV_MOTION_GUIDE_DOWNCAM 44
#define AGV_MOTION_AFTERLIFT_GUIDE_TO_DM 45

#define AGV_MOTION_LASER_ADJUST 46 //HR,add,20171205
#define AGV_MOTION_BEFORELIFT_CHECK_UPCAMERA 47

#define AGV_MOTION_LOAD 48
#define AGV_MOTION_UNLOAD 49
#define AGV_MOTION_MOVING_UNLOAD 50
#define AGV_MOTION_CONVEYOR_CONFIRM 51
#define AGV_MOTION_CONVEYOR_ZERO_OFFSET 52
#define AGV_MOTION_CONVEYOR_BACK 53
#define AGV_MOTION_RESET_LOAD 54
#define AGV_MOTION_MOVEOVER_UNLOAD 55
#define AGV_MOTION_ARC 56
#define AGV_MOTION_UCURVE 57

struct AppActionStateManager_t
{
	int CurActionState;
	int preActionState;
	int stopCount;
	int stopConfirmDelay;
	int laserSwitchDelay;
};
int AppActionStateManagerInit(struct AppActionStateManager_t* appActionStateManager);
int AppActionStateManager(struct AppActionStateManager_t* appActionStateManager);
int SetAppActionState(int actionState);
int GetAppActionState();


#endif /* AGVAPPACTIONSTATE_H_ */
