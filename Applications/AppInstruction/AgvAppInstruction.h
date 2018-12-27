/*
 * OldInstruction.h
 *
 *  Created on: 2017��9��14��
 *      Author: hwei
 */

#ifndef OLDINSTRUCTION_H_
#define OLDINSTRUCTION_H_
#include "AgvDataType.h"

//instruction cmd type
#define INSTRUCTION_RIGHTANGLEMOVE 6
#define INSTRUCTION_MOVE 4 // a move instruction
#define INSTRUCTION_LIFTUP 2 // a liftup instruction
#define INSTRUCTION_SETDOWN 3 // a setdown instruction
#define INSTRUCTION_CAMERA 1 // a calibration position instruction for PC camera
#define INSTRUCTION_RESET 5  // reset agv to a given position
//#define INSTRUCTION_MOVE_ARC_IN 60 // move to turning area
//#define INSTRUCTION_MOVE_ARC_OUT 70 // move out turning area
#define INSTRUCTION_RUN 8  //  run time mode
#define INSTRUCTION_COMMISSION 9 // commission mode
//#define INSTRUCTION_SETPODHEIGHT 10 // set pod height
#define INSTRUCTION_UPDATALOG 11    // upload log data
#define INSTRUCTION_POD_LS 12     // pod find zero point
//#define INSTRUCTION_STARTLOG 13 // start log
//#define INSTRUCTION_STOPLOG 14 // stop log

#define INSTRUCTION_ENABLE 13 // start log
#define INSTRUCTION_DISABLE 14 // stop log
#define INSTRUCTION_POD_ZERO 15 // stop log
#define INSTRUCTION_TURNING 16 // turning
#define INSTRUCTION_RESET_ERROR 17 // error reset
#define INSTRUCTION_SOFTSTOP 18   // to stop with deceleration
#define INSTRUCTION_GYRO_ZERO 19  // calculate the gyro zero bias
#define INSTRUCTION_ESTOP 20 // to stop ASAP
#define INSTRUCTION_CHANGEPODDIRECTION 21 // the pod turning with the agv
#define INSTRUCTION_SETDOWNLS 10
#define INSTRUCTION_REQUEST 22
#define INSTRUCTION_FORCEMOVE 23
#define INSTRUCTION_SETPARAM 24
#define INSTRUCTION_READPARAM 25
#define INSTRUCTION_CHARGEMOVE 26
#define INSTRUCTION_PODJOGGING 27
#define INSTRUCTION_SETERROR 30// start write the parameters to exflash
#define INSTRUCTION_STEPIN_ONESTEP 28
#define INSTRUCTION_STEPOUT_ONESTEP 29
#define INSTRUCTION_SEARCHCODE_MOVE 33
#define INSTRUCTION_SEARCHCODE_TURNING 34
#define INSTRUCTION_MEASUREDIA_MOVE 35
#define INSTRUCTION_DORMANCY 36
#define INSTRUCTION_WAKEUP 37
#define INSTRUCTION_DEBUGMODE 38
#define INSTRUCTION_MAP_INFORMATION 39
#define INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF 40//40����ǰ����������ͷ
#define INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF 41//41���������������ͷ
#define INSTRUCTION_MEASUREPOWER 42
#define INSTRUCTION_RUNMODE 43 
#define INSTRUCTION_SLAVEPC_VERSION 44
#define INSTRUCTION_EXIT_CHARGINGZONE 45
#define INSTRUCTION_READMILEAGE 46//46
#define INSTRUCTION_OPEN_UPCAMERA_BEFORELIFTUP 47
#define INSTRUCTION_CLOSE_UPCAMERA 48
#define INSTRUCTION_OPEN_UPCAMERA_AFTERLIFTUP 49
#define INSTRUCTION_CLEARMILEAGE 50
#define INSTRUCTION_CHOOSE_LOADCONDITION 51
#define INSTRUCTION_KEEPPACEWITH_AGVFRONT 52
#define INSTRUCTION_INITIAL_AFTERLIFT_CHECKSHELF 53
#define INSTRUCTION_SWITCH_FAN 54 // open/start FAN*/
#define INSTRUCTION_RETURNBACK 55 //????

//????,?????????
//panj-add
#define INSTRUCTION_JOYSTICK_MOVE_POS_MODE 56
#define INSTRUCTION_JOYSTICK_MOVE_VEL_MODE 57
#define INSTRUCTION_GUIDE_UPCAMERA_BEFORELIFTUP 58
#define INSTRUCTION_SWITCH_LOG 59
#define INSTRUCTION_CHECK_GUIDE_COND 60
//??????????
#define INSTRUCTION_SWITCH_LOGPOINT 61
#define INSTRUCTION_FLUSH_QUEUE 62
#define INSTRUCTION_GET_AGV_STATUS 63
#define INSTRUCTION_INITIAL 64
#define INSTRUCTION_SWITCH_METHOD 65

#define INSTRUCTION_RESTART_IPC 66 //�������ػ�,HR,add,20160823
#define INSTRUCTION_RESTART_CAMERA 67 //��������ͷ,HR,add,20170308
#define INSTRUCTION_RESTART_DSP 68 //����DSP,HR,add,20160823
#define INSTRUCTION_RESTART_DRIVER 69 //����������,HR,add,20160908

//panj-add
//���뵼������
#define INSTRUCTION_GUIDE_WITHOUTDM 70
#define INSTRUCTION_GUIDE_WITHOUTDM_PH2 71
//�ƻ�������ת
#define INSTRUCTION_ROT_WITHUP 72
#define INSTRUCTION_GUIDE_ONCE 73
#define INSTRUCTION_CHECK_UPCAM 74

//add sxp 20161029
#define INSTRUCTION_ZERORETURN 75


//�л�SLAMģʽ
#define INSTRUCTION_SWITCH_SLAMMODE 76
//��ѯģʽ
#define INSTRUCTION_QUERY_SLAMMODE 77
//������ͷ����
#define INSTRUCTION_GUIDE_DOWN_CAM 78
//�����Լ���ͼƫ��У���
#define INSTRUCTION_CHECK_PARAM_MAP 79


//�����ȼ��
//#define INSTRUCTION_FIND_LEG 80
//�ֶ�ң��ָ��
#define INSTRUCTION_MANUAL 80	

//�״ﳵ������
#define INSTRUCTION_RADAR_AGV_GUIDE 81
//�״���ܵ���
#define INSTRUCTION_RADAR_SHELF_GUIDE 82
//С������
#define INSTRUCTION_BACK_OFF 83
//LHZ
#define INSTRUCTION_RST_AGV_MODE 84 //��λ��λ������֡�ϴ�ģʽ
#define INSTRUCTION_SET_AGV_REQ  85 //LHZ add 20170510
#define INSTRUCTION_RST_AGV_REQ 86//LHZ add 20170510
#define INSTRUCTION_ONLINE_SUCCESS 87//LHZ add 20170510

#define INSTRUCTION_DO_CLEAN 88//LHZ 0610 cleaner


#define INSTRUCTION_CLEANER_DIRTY_DISPLAY 93
#define INSTRUCTION_BEFOREPUTDOWN_OPEN_UPCAMERA_CHECKSHEL 95
//��Ͳ����ָ��
//��Ͳģʽ0������ģʽ��1������ģʽ��2������ģʽ����������ms
#define INSTRUCTION_CONVEYER_CONTROL_MODE_TRANSFER 96//��Ͳ����ģʽ�л�
//���ʹ��_����:0_�ر� 1_ʹ��
#define INSTRUCTION_CONVEYER_MOTOR_SWITCH 97 //���ʹ����ʹ�ܿ���
#define INSTRUCTION_PLATFORM_MOTOR_SWITCH 98  //ƽ̨��ת���ʹ�ܿ���
#define INSTRUCTION_BACK_PLATE_MOTOR_SWITCH 99  //��Ͳ�����������ʹ�ܿ���_
//����:Ŀ���ٶȣ���λ��dm/min
#define INSTRUCTION_CONVEYER_VELOCITY_SET 100  //��Ͳ���ʹ������ٶ��趨
//����:Ŀ��Ƕȣ���λ��degree�������ת�ٶȣ���λ��degree/s
#define INSTRUCTION_PLATFORM_ROTATE_SET 101  //��Ͳƽ̨��ת�Ƕȼ��ٶ��趨
//����:0_��_1_��
#define INSTRUCTION_BACK_PLATE_LIFT 102  //��Ͳ������������
//����ģʽ��ִ��103-106�ĸ�ָ��������ʹ����
//����:0_�����ǰ����״̬��1_��ֹ��ǰ����
#define INSTRUCTION_REMOVE_CONVEYER_TASK 103  //������ʹ�������
//����:��
#define INSTRUCTION_INITIAL_CONVEYER 104  //��ʼ����Ͳ���ʹ�
//����:���ʹ�ָ���ٶȣ���λ��dm/min,ƽ̨��תĿ��Ƕȣ���λ��degree,ƽ̨��ת���ʱ�䣬��λ��ms
#define INSTRUCTION_SHIPPING_CONVEYER 105  //��Ͳ���ʹ�װ��
//����:���ʹ�ָ���ٶȣ���λ��dm/min,ƽ̨��תĿ��Ƕȣ���λ��degree,ƽ̨��ת���ʱ�䣬��λ��ms
#define INSTRUCTION_UNLOADING_CONVEYER 106  //��Ͳ���ʹ�ж��
//����:0_���ʹ���1_ƽ̨��2_����
#define INSTRUCTION_QUERY_CONVEYER 107  //��Ͳ�豸״̬��ѯ

#define INSTRUCTION_ADJUST_LASER 108 //�״�����λУ׼��HR��add,20171120
#define INSTRUCTION_BELTMOVE 119	//Ƥ�������˶�1s
#define INSTRUCTION_CONVEYOR_LOAD 120  //����ָ��
#define INSTRUCTION_CONVEYOR_UNLOAD 121  //����ָ��
#define INSTRUCTION_MOVING_UNLOAD 122  //���ñ��߱�Ͷ��Ϣָ��
#define INSTRUCTION_CONVEYOR_CONFIRM 123  //Ƥ���̻���ȷ��ָ��
#define INSTRUCTION_CONVEYOR_ZERO_OFFSET 124  //Ƥ���̴�����У׼ָ��
#define INSTRUCTION_CONVEYOR_BACK 125  //Ƥ���̻ع�
#define INSTRUCTION_RESET_LOAD 126  //���̻���״̬����
#define INSTRUCTION_CONVEYOR_CANCEL 127  //Ƥ���̻ع�
#define INSTRUCTION_FULL_LOAD_TEST 128  //���̻���״̬����
#define INSTRUCTION_SET_LOAD_POS 129  //����Ͷ��̨����λ��
#define INSTRUCTION_SET_QRCODEMAP 130 //��̬�л�����
/***********************************************************************/

struct INSTRUCTION_Normal_params_t
{
	int32 targetX;//uint 0.001������
	int32 targetY;//uint 0.001������
	int16 targetHeading;//uint 0.01��
	int16 PodDirection : 8;
	int16 checkSum : 8;
};
FH_ERROR_CODE DO_INSTRUCTION_EXIT_CHARGINGZONE(struct INSTRUCTION_Normal_params_t* moveParam);

FH_ERROR_CODE DO_INSTRUCTION_CAMERA (struct INSTRUCTION_Normal_params_t* Param);
FH_ERROR_CODE DO_INSTRUCTION_LIFTUP(struct INSTRUCTION_Normal_params_t* Param);
FH_ERROR_CODE DO_INSTRUCTION_SETDOWN(struct INSTRUCTION_Normal_params_t* Param);
FH_ERROR_CODE DO_INSTRUCTION_MOVE(struct INSTRUCTION_Normal_params_t* Param);
FH_ERROR_CODE DO_INSTRUCTION_RESET(struct INSTRUCTION_Normal_params_t* Param);
FH_ERROR_CODE DO_INSTRUCTION_RIGHTANGLEMOVE(struct INSTRUCTION_Normal_params_t* moveParam);

FH_ERROR_CODE DO_INSTRUCTION_RUN(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_COMMISSION(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SETDOWNLS(struct INSTRUCTION_Normal_params_t* setDownLSParam);
FH_ERROR_CODE DO_INSTRUCTION_UPDATALOG(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_POD_LS(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_ENABLE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_DISABLE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_POD_ZERO(int* Param);
struct INSTRUCTION_TURNING_PARAMS_t
{
		Uint16 omega_rank:4;
		Uint16 epsi_rank:4;
	int16 rsvd :8;
	int16 rsvd1;
	int32 targetY;
	int16 targetHeading;
		Uint16 PodDirection : 8;
	int16 rsvd2 : 8;
};
FH_ERROR_CODE DO_INSTRUCTION_TURNING(struct INSTRUCTION_TURNING_PARAMS_t* turningParam);
FH_ERROR_CODE DO_INSTRUCTION_RESET_ERROR(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SOFTSTOP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_GYRO_ZERO(int* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_ESTOP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_CHANGEPODDIRECTION(struct INSTRUCTION_TURNING_PARAMS_t* turningParam);
FH_ERROR_CODE DO_INSTRUCTION_REQUEST(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_FORCEMOVE(int* Param);
struct INSTRUCTION_SETPARAM_PARAMS_t
{
	int32 ParamIndexWrite;
	float ParamWrite;
};
FH_ERROR_CODE DO_INSTRUCTION_SETPARAM(struct INSTRUCTION_SETPARAM_PARAMS_t* Param);
FH_ERROR_CODE DO_INSTRUCTION_READPARAM(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_CHARGEMOVE(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_PODJOGGING(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_STEPIN_ONESTEP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_STEPOUT_ONESTEP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SETERROR(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_SEARCHCODE_MOVE(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_SEARCHCODE_TURNING(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_MEASUREDIA_MOVE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_DORMANCY(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_WAKEUP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_DEBUGMODE(int* Param);

struct INSTRUCTION_MAP_INFORMATION_PARAMS_t
{
	Uint16 X:8;
	Uint16 Y:8;
	Uint16 EXRamXYxp;
	Uint16 EXRamXYxn;
	Uint16 EXRamXYyp;
	Uint16 EXRamXYyn;
	Uint16 chargePoint:8;
	Uint16 rsvd:8;
};
FH_ERROR_CODE DO_INSTRUCTION_BEFOREPUTDOWN_OPEN_UPCAMERA_CHECKSHEL(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_MAP_INFORMATION(struct INSTRUCTION_MAP_INFORMATION_PARAMS_t* Param);
FH_ERROR_CODE DO_INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_MEASUREPOWER(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_RUNMODE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SLAVEPC_VERSION(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_READMILEAGE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_OPEN_UPCAMERA_BEFORELIFTUP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_CLOSE_UPCAMERA(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_OPEN_UPCAMERA_AFTERLIFTUP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_CLEARMILEAGE(int* Param);

FH_ERROR_CODE DO_INSTRUCTION_CHOOSE_LOADCONDITION(struct INSTRUCTION_Normal_params_t * normalParam);
FH_ERROR_CODE DO_INSTRUCTION_KEEPPACEWITH_AGVFRONT(struct INSTRUCTION_Normal_params_t * moveParam);
FH_ERROR_CODE DO_INSTRUCTION_INITIAL_AFTERLIFT_CHECKSHELF(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_FAN(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_RETURNBACK(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_JOYSTICK_MOVE_POS_MODE(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_JOYSTICK_MOVE_VEL_MODE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_UPCAMERA_BEFORELIFTUP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_LOG(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_CHECK_GUIDE_COND(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_LOGPOINT(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_FLUSH_QUEUE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_GET_AGV_STATUS(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_INITIAL(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_METHOD(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_RESTART_IPC(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_RESTART_CAMERA(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_RESTART_DSP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_RESTART_DRIVER(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_WITHOUTDM(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_WITHOUTDM_PH2(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_ROT_WITHUP(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_ONCE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_CHECK_UPCAM(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_ZERORETURN(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_SLAMMODE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_QUERY_SLAMMODE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_DOWN_CAM(int* Param);

FH_ERROR_CODE DO_INSTRUCTION_CHECK_PARAM_MAP(struct INSTRUCTION_Normal_params_t* Param);
	struct INSTRUCTION_MANUAL_Params_t
	{
		Uint16 FrameHead :8;//0����֡ͷ
		Uint16 FunctionTypeId :8;//1�����־
		Uint16 Speed :8;//2�ƶ�����
		Uint16 MaxAngularSpd :8;//3��ת����
		Uint16 ManualSwitch :8;//4�ֶ�/�Զ��л�
		Uint16 SpeedGrade :8;//5�ٶȵȼ�
		Uint16 CheckSum :8;//6ͨѶ��־λ,У���

	};
	FH_ERROR_CODE DO_INSTRUCTION_MANUAL(struct INSTRUCTION_MANUAL_Params_t* manualParam);
FH_ERROR_CODE DO_INSTRUCTION_RADAR_AGV_GUIDE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_RADAR_SHELF_GUIDE(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_BACK_OFF(struct INSTRUCTION_Normal_params_t* moveParam);
FH_ERROR_CODE DO_INSTRUCTION_RST_AGV_MODE(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_SET_AGV_REQ(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_RST_AGV_REQ(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_ONLINE_SUCCESS(int* Param);
FH_ERROR_CODE DO_INSTRUCTION_CHARGE(struct INSTRUCTION_Normal_params_t* param);
FH_ERROR_CODE DO_INSTRUCTION_ONLINE(struct INSTRUCTION_Normal_params_t* param);
FH_ERROR_CODE DO_INSTRUCTION_DO_CLEAN(struct INSTRUCTION_Normal_params_t* param);
FH_ERROR_CODE DO_INSTRUCTION_CLEANER_DIRTY_DISPLAY(struct INSTRUCTION_Normal_params_t* param);

FH_ERROR_CODE DO_INSTRUCTION_ADJUST_LASER(int* Param);//HR,add,20171121
FH_ERROR_CODE DO_INSTRUCTION_POWER_SWITCH(struct INSTRUCTION_Normal_params_t* param);

FH_ERROR_CODE DO_INSTRUCTION_BELTMOVE(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_LOAD(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_UNLOAD(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_MOVING_UNLOAD(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_CONVEYOR_CONFIRM(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_CONVEYOR_ZERO_OFFSET(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_CONVEYOR_BACK(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_RESET_LOAD(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_CONVEYOR_CANCEL(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_FULL_LOAD_Check(struct INSTRUCTION_Normal_params_t* normalParam);
FH_ERROR_CODE DO_INSTRUCTION_SET_LOAD_POS(struct INSTRUCTION_Normal_params_t* moveParam);
FH_ERROR_CODE DO_INSTRUCTION_KEEPPACE(struct INSTRUCTION_Normal_params_t * moveParam);

struct INSTRUCTION_SET_QRCODEMAP_Params_t
	{
		union codeMap_u
		{
			struct CodeMapOrigin_t
			{
				Uint16 originIDX;
				Uint16 originIDY;
				Uint16 rsv1;
				Uint16 rsv2;
			}CodeNumberInfo;
			struct CodeMap_t
			{
				Uint16 CodeStartIDX;
				Uint16 CodeStartIDY;
				Uint16 CodeEndIDX;
				Uint16 CodeEndIDY;
			}CodeIDInfo;
			struct CodeMapGap_t
			{
				FH_Float CodeGapX;
				FH_Float CodeGapY;
			}CodeGapInfo;
		}CodeMapInfo;
		Uint16 MapIndex :8;//index(0-255)
		Uint16 FunctionID :8;//0:Add,1:Remove,2,Modify,3,ClearAll,4,SetMapNum,5,SetGap
	};
	FH_ERROR_CODE DO_INSTRUCTION_SET_QRCODEMAP(struct INSTRUCTION_SET_QRCODEMAP_Params_t* Param);

union InstructionParams_u
{
	int buf[22];
	struct INSTRUCTION_MAP_INFORMATION_PARAMS_t INSTRUCTION_MAP_INFORMATION_PARAMS;
	struct INSTRUCTION_SETPARAM_PARAMS_t INSTRUCTION_SETPARAM_PARAMS;
	struct INSTRUCTION_Normal_params_t INSTRUCTION_Normal_params;
};
struct InstructionProtocl_t
{
	unsigned char Header : 8;
	unsigned char ID : 8;
    unsigned char Type : 8;
	unsigned char Len : 8;
	union InstructionParams_u params;
};		


struct AppInstruction_t
{
	int rsvd;
};
void CompatiableFun(void* params);
int WaitLastCmdComplete_fun();
int WaitUnloadComplete_fun();
int WaitDisComplete_fun();
#define WaitLastCmdComplete() if(0 != WaitLastCmdComplete_fun()) return -1
#define WaitUnloadComplete() if(0 != WaitUnloadComplete_fun()) return -1
#define WaitDisComplete() if(0 != WaitDisComplete_fun()) return -1

int IsTaskActionOK();

FH_ERROR_CODE AppInstructionInit(struct AppInstruction_t* appInstruction);
void NormalParamsEndianConvert(struct INSTRUCTION_Normal_params_t* normalParam);

#include "Basefunction.h"
extern struct MoveParamType movep;
FH_ERROR_CODE CalcMoveParams(struct MoveParamType* moveParam);

#define CustomID 14
extern int IsCustomIDVaild;

#define GetChissisTskParam(chassisTskParams,TargetX,TargetY,TargetHeading,MotionType)  	\
    (chassisTskParams)->targetX = (TargetX);\
    (chassisTskParams)->targetY = (TargetY);\
    (chassisTskParams)->targetHeading = (TargetHeading);\
    if(Motion_MOVE == (MotionType)){\
		(chassisTskParams)->pg.creepDistance = Param.AgvParam[PARAM_DeceDisCompensate];\
	    (chassisTskParams)->pg.creepVel = Param.AgvParam[PARAM_AgvMinSPD];\
	    (chassisTskParams)->pg.maxDec = (0 == IsCustomIDVaild)? LoadCondition.MaxAcc[LoadCondition.ID] : (Min(LoadCondition.MaxAcc[CustomID],LoadCondition.MaxAcc[LoadCondition.ID]));\
	    (chassisTskParams)->pg.maxVel = (0 == IsCustomIDVaild)? LoadCondition.MaxSpd[LoadCondition.ID] : (Min(LoadCondition.MaxSpd[CustomID],LoadCondition.MaxSpd[LoadCondition.ID]));\
 		if((SMALL_BELT == AgvParams.serviceParams.leftBeltParams.beltType)\
		||(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType))\
	    (chassisTskParams)->pg.maxAcc = Param.AgvParam[PARAM_LoadCondition_AccAjust]*(chassisTskParams)->pg.maxDec;\
		else\
	    (chassisTskParams)->pg.maxAcc = (LoadCondition.ID>=1)? Param.AgvParam[PARAM_LoadCondition_AccAjust]*(chassisTskParams)->pg.maxDec : (chassisTskParams)->pg.maxDec;\
	}\
	else if(Motion_TURN == (MotionType)){\
		(chassisTskParams)->pg.creepDistance = Param.AgvParam[PARAM_TurningDeceAngCompensate];\
		(chassisTskParams)->pg.creepVel = Param.AgvParam[PARAM_AgvMinAngSPD];\
		(chassisTskParams)->pg.maxAcc = (0 == IsCustomIDVaild)?LoadCondition.MaxAngularAcc[LoadCondition.ID] : (Min(LoadCondition.MaxAngularAcc[CustomID],LoadCondition.MaxAngularAcc[LoadCondition.ID]));\
		(chassisTskParams)->pg.maxVel = (0 == IsCustomIDVaild)?LoadCondition.MaxAngularSpd[LoadCondition.ID] : (Min(LoadCondition.MaxAngularSpd[CustomID],LoadCondition.MaxAngularSpd[LoadCondition.ID]));\
		(chassisTskParams)->pg.maxDec = (chassisTskParams)->pg.maxAcc;}\
    (chassisTskParams)->pg.motionType = (MotionType);\
    (chassisTskParams)->pg.targetType = TargetType_Abs;\
    (chassisTskParams)->pg.pgType = PGType_MultiAxialCurve_SCurve;\
    (chassisTskParams)->esParams.stopConditionType = StopCondition_Empty;\
    *((FH_Float*)&((chassisTskParams)->esParams.stopTarget)) = ((Motion_MOVE == (MotionType))? LoadCondition.SoftStopAcc[LoadCondition.ID]: 0.0)

 #define GetTrayTskParam(trayTskParams,Target,MotionType)  	\
	 (trayTskParams)->target = (Target);\
	 (trayTskParams)->pg.creepDistance = Param.AgvParam[PARAM_LiftUpDeceAngCompensate];\
	 (trayTskParams)->pg.creepVel = Param.AgvParam[PARAM_AgvMinAngSPD];\
	 (trayTskParams)->pg.maxAcc = (0 == IsTrayLiftUpState())? Param.AgvParam[PARAM_LiftAngelSpeedAcc] : 0.5*Param.AgvParam[PARAM_LiftAngelSpeedAcc];\
	 (trayTskParams)->pg.maxVel = (0 == IsTrayLiftUpState())? Param.AgvParam[PARAM_LiftMaxAngleSpeed] : Degree2Rad(5);\
	 (trayTskParams)->pg.maxDec = (0 == IsTrayLiftUpState())? Param.AgvParam[PARAM_LiftAngelSpeedAcc] : 0.5*Param.AgvParam[PARAM_LiftAngelSpeedAcc];\
	 (trayTskParams)->pg.motionType = (MotionType);\
	 (trayTskParams)->pg.targetType = TargetType_Abs;\
	 (trayTskParams)->pg.pgType = PGType_SCurve;\
	 (trayTskParams)->esParams.stopConditionType = StopCondition_Empty;\
 	*((FH_Float*)&((trayTskParams)->esParams.stopTarget)) = 0.0

 
#define GetTrayLiftTskParam(trayTskParams,Target)   \
	 (trayTskParams)->target = (Target);\
	 (trayTskParams)->pg.creepDistance = 0.0002;\
	 (trayTskParams)->pg.creepVel = 0.001;\
	 (trayTskParams)->pg.maxAcc = 0.03;\
	 (trayTskParams)->pg.maxVel = (1== IsTrayLiftLimitConfirm())?0.050:0.050; \
	 (trayTskParams)->pg.maxDec = 0.03;\
	 (trayTskParams)->pg.motionType = (NULL);\
	 (trayTskParams)->pg.targetType = TargetType_Abs;\
	 (trayTskParams)->pg.pgType = PGType_SCurve;\
	 (trayTskParams)->esParams.stopConditionType = (Target>0)? StopCondition_TrayUpLimit : StopCondition_TrayDownLimit;\
	(trayTskParams)->esParams.IsEmc = 1;\
	(trayTskParams)->esParams.stopTarget = 1;\


#endif /* OLDDO_INSTRUCTION_H_ */
