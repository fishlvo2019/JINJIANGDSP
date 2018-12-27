/*
 * AgvAppParams.h
 *
 *  Created on: 2017年10月25日
 *      Author: hwei
 */

#ifndef AGVAPPPARAMS_H_
#define AGVAPPPARAMS_H_
#include "AgvDataType.h"
#define PARAMS_VALID 1
#define PARAMMAXNUM 206
#define PARAM_AgvMaxSPD 0//Agv运动最大速度
#define PARAM_AgvMaxAcc 1//Agv运动最大加速度
#define PARAM_MAX_ACC_SENSOR 2 //防撞限位时加速度
#define PARAM_AgvMinSPD 3//AGV运动最小速度
#define PARAM_AgvMaxAngSPD 4//AGV最大角速度
#define PARAM_AgvMaxAngAcc 5//AGV最大角加速度
#define PARAM_AgvMinAngSPD 6//AGV最小角速度
#define PARAM_LiftTopCycles 7 //顶升位置圈数
#define PARAM_DeceDisCompensate 8//减速距离补偿值
#define PARAM_ChargeDeceDisCompensate 9//充电减速距离补偿值
#define PARAM_LiftUpDeceAngCompensate 10//LiftUp减速角度补偿值

#define PARAM_TargetTolX 11//目标到达判定x
#define PARAM_TargetTolY 12//目标到达判定y
#define PARAM_TargetMoveTolHeading 13//move目标到达判定Heading
#define PARAM_TargetLiftUpTolHeading 14//liftup目标到达判定Heading
#define PARAM_TargetTurningTolHeading 15//turning目标到达判定Heading
#define PARAM_TargetTolPod 16 // 托盘安全高度阈值
#define PARAM_Offset_P 17//Offset_P
#define PARAM_Offset_I 18//Offset_I
#define PARAM_Offset_D 19//Offset_D
#define PARAM_Heading_P 20  //Heading_P
#define PARAM_Heading_I 21//Heading _I
#define PARAM_Heading_D 22//Heading _D
#define PARAM_PodHeadingLiftUp_P 23//PodHeadingLiftup_P
#define PARAM_PodHeadingLiftUp_I 24//PodHeadingLiftup_I
#define PARAM_PodHeadingTurning_P 25//PodHeadingTurning_P
#define PARAM_PodHeadingTurning_I 26//PodHeadingTurning_I
#define PARAM_PodLSP 27//托盘限位位置
#define PARAM_WheelReduceRatio 28//车轮减速比jixu
#define PARAM_LiftReduceRatio 29//顶升减速比
#define PARAM_OffsetTol 30//偏离距离
#define PARAM_OffsetMoveTol 31 //偏离移动距离
#define PARAM_OffsetHeading_DecelEndTol 32//偏离角度减速结束阈值
#define PARAM_TempratureWarn 33//温度报警
#define PARAM_WheelDis 34//轮距
#define PARAM_WheelDia 35//轮径
#define PARAM_BatteryCur 36 //电池电流
#define PARAM_CameraStandardTx 37 //摄像头标定数fx
#define PARAM_CameraStandardTy 38//摄像头标定数fy
#define PARAM_CameraStandardRz 39//摄像头标定参数cx
#define PARAM_CameraStandardY2C 40//摄像头标定参数cy  
#define PARAM_TurningDeceAngCompensate 41//Turning减速角度补偿值
#define PARAM_TargetLiftUpTolPodHeading 42 //liftup托盘目标到达判定Heading  0.3
#define PARAM_TempratureErr 43    //温度报错
#define PARAM_PodReturnToZero_P 44 //托盘回零闭环P 1.0
#define PARAM_PodReturnToZero_I 45 //托盘回零闭环I 0.993
#define PARAM_Move_Decelerate_P 46 //移动减速闭环P 2.0
#define PARAM_Move_Decelerate_I 47 //移动减速闭环I 0.01
#define PARAM_Turning_Decelerate_P 48 //旋转减速闭环P 2.0
#define PARAM_Turning_Decelerate_I 49 //旋转减速闭环I 0.01
#define PARAM_StepInOut_MovespanDis 50 
#define PARAM_Move_MovespanDis 51
#define PARAM_ChargeInterfaceDis 52 //充电针默认判断距离:30-60
#define PARAM_PosUnstableLimit 53 //
#define PARAM_PosLightUnstableLimit 54 //
#define PARAM_HeadingUnstableLimit 55
#define PARAM_HeadingLightUnstableLimit 56
#define PARAM_Slip_CtlMode 57
#define PARAM_DormancyTime 58 //min
#define PARAM_ADDAngle 59
#define PARAM_ExposureTime 60
#define PARAM_LateralFieldView  61
#define PARAM_VerticalFieldView  62
#define PARAM_CameraChoose 63
#define PARAM_Offset_DecelMinSPD 64// 偏离最小速度
#define PARAM_Offset_DecelMaxAcc 65// 偏离减速度
#define PARAM_LaserSigleWidth 66  // 
#define PARAM_LaserLength 67  // 
#define PARAM_ShelfCollision_Angle 68  //
#define PARAM_DormancyDelayTime 69 //s
#define PARAM_UpCamera_Shelf_Offset 70 
#define PARAM_UpCamera_Shelf_OffsetHeading 71 
#define PARAM_UpCameraStandardTx 72 //摄像头标定数fx
#define PARAM_UpCameraStandardTy 73 //摄像头标定数fy
#define PARAM_UpCameraStandardRz 74 //摄像头标定参数cx
#define PARAM_MapBorderXmin 75 //m
#define PARAM_MapBorderXmax 76 //m
#define PARAM_MapBorderYmin 77  //m
#define PARAM_MapBorderYmax 78 //m
#define PARAM_UpCameraExposureTime 79
#define PARAM_LoadCondition_MaxSpd1 80
#define PARAM_LoadCondition_MaxAcc1 81
#define PARAM_LoadCondition_SoftStopAcc1 82
#define PARAM_LoadCondition_MaxChangeDirectionAngularSpd1 83
#define PARAM_LoadCondition_ChangeDirectionAngularAcc1 84//HR,modified,20160512

#define PARAM_LoadCondition_MaxSpd2 85
#define PARAM_LoadCondition_MaxAcc2 86
#define PARAM_LoadCondition_SoftStopAcc2 87
#define PARAM_LoadCondition_MaxChangeDirectionAngularSpd2 88
#define PARAM_LoadCondition_ChangeDirectionAngularAcc2 89//HR,modified,20160512

#define PARAM_LoadCondition_MaxSpd3 90
#define PARAM_LoadCondition_MaxAcc3 91
#define PARAM_LoadCondition_SoftStopAcc3 92
#define PARAM_LoadCondition_MaxChangeDirectionAngularSpd3 93
#define PARAM_LoadCondition_ChangeDirectionAngularAcc3 94//HR,modified,20160512

#define PARAM_LoadCondition_MaxSpd4 95
#define PARAM_LoadCondition_MaxAcc4 96
#define PARAM_LoadCondition_SoftStopAcc4 97
#define PARAM_LoadCondition_MaxChangeDirectionAngularSpd4 98
#define PARAM_LoadCondition_ChangeDirectionAngularAcc4 99//HR,modified,20160512

#define PARAM_UpCameraStandardTx_BeforeLift 100 //摄像头标定数fx
#define PARAM_UpCameraStandardTy_BeforeLift 101 //摄像头标定数fy
#define PARAM_UpCameraStandardRz_BeforeLift 102 //摄像头标定参数cx

#define PARAM_MotoTolCurrent 103//车辆静态下电机电流的容忍值：A
#define PARAM_MotoOverCurrentDuration 104//电机过流容忍时间：秒

#define PARAM_OffsetLimitSpeed 105
#define PARAM_Offset_StartDecel 106
#define PARAM_Offset_EndDecel 107
#define PARAM_Offset_Decel_P 108

#define PARAM_KeepPace_MaxSpeed 109
#define PARAM_KeepPace_P 110
#define PARAM_KeepPace_Dis 111
#define PARAM_ChargeCompensateDis 113
#define PARAM_BarrierCompensateDis 114
#define PARAM_LaserDelayCompensateTime 115
#define PARAM_KeepPace_OffsetCompensateDis 116
#define PARAM_OffsetCreepingTime 117
#define PARAM_LaserOpenNoDataTimeOutTol 118
#define PARAM_UseCorrectHeading_DMOffsetHeadingTol 119
#define PARAM_NotUseCorrectHeading_DMOffsetHeadingTol 120
#define PARAM_UseCorrectHeading_FrontOffsetTol 121
#define PARAM_LIFTMotoTolCurrent_Dynamic 122 //HR,add,20151022（顶升电机动态工况下过流参数：A）
#define PARAM_DriveMotorTolCurrent_Dynamic 123 //HR,add,20151022（驱动轮电机动态工况下过流参数：A）
#define PARAM_LIFTMotoTolCurrent_Static 124 //HR,add,20151023（顶升电机静态工况下过流参数：A）
#define PARAM_DriveMotorTolCurrent_Static 125 //HR,add,20151023（驱动轮电机静态工况下过流参数：A）
#define PARAM_LASER_SWITCH_BYPASS 126//HR,add,20151119，雷达开关：1表示旁路
#define PARAM_TargetTurnTolPodHeading 127//HR,add,20151221，旋转工况下，托盘目标角度到达判定：度
#define PARAM_ChargeTouchPointTol 128 //HR,add,20151221，充电接触点误差，车辆收到充上电后继续行驶的距离：m
#define PARAM_HighMove_OffsetAjustAngle_Limit 129 //HR,add,20151226,大于0.5倍额定转速的移动过程中横向偏差纠偏转向角度限值：3度
#define PARAM_SlowMove_OffsetAjustAngle_Limit 130 //HR,add,20151226,小于0.5倍额定转速的移动过程中横向偏差纠转向偏角度限值：10度
#define PARAM_STATE_AJUST_OFFSET_X 131 //HR,add,20160217,需要调整车辆姿态的判断标准：标码在视野X方向的偏差值，m
#define PARAM_STATE_AJUST_OFFSET_Y 132 //HR,add,20160217,需要调整车辆姿态的判断标准：标码在视野Y方向的偏差值，m
#define PARAM_CODE_GAP 133 //HR,add,20160222,标码间距标准值：m
#define PARAM_CAMERA_FRAMERATE_Normal 134 //HR,20160330,图像发送帧率：正常值50
#define PARAM_CAMERA_FRAMERATE_Medium 135 //HR,20160330,图像发送帧率：中间值65
#define PARAM_CAMERA_FRAMERATE_High 136 //HR,20160330,图像发送帧率：高值70
#define PARAM_DRIVER_CURRENT_NORMAL 137 //HR,20160412,驱动轮电机额定电流，A
#define PARAM_PODDRIVER_CURRENT_NORMAL 138 //HR,20160412,随动电机额定电流，A
#define PARAM_UPDOWN_SPEED 139 //HR,20160504,升降最大速度，rpm
#define PARAM_LoadCondition_MaxAngularSpd1 140  //HR,20160516,最大旋转角速度，degree/s
#define PARAM_LoadCondition_MaxAngularAcc1 141  //HR,20160516,最大旋转角加速度，degree/s2
#define PARAM_LoadCondition_MaxAngularSpd2 142  //HR,20160516,最大旋转角速度，degree/s
#define PARAM_LoadCondition_MaxAngularAcc2 143  //HR,20160516,最大旋转角加速度，degree/s2
#define PARAM_LoadCondition_MaxAngularSpd3 144  //HR,20160516,最大旋转角速度，degree/s
#define PARAM_LoadCondition_MaxAngularAcc3 145  //HR,20160516,最大旋转角加速度，degree/s2
#define PARAM_LoadCondition_MaxAngularSpd4 146  //HR,20160516,最大旋转角速度，degree/s
#define PARAM_LoadCondition_MaxAngularAcc4 147  //HR,20160516,最大旋转角加速度，degree/s2
#define PARAM_ShelfPositionToFloorCodeAjust 148 //HR,20160520,货架位置相对地面标码偏差修正门槛值，m
#define PARAM_LiftMaxAngleSpeed 149 //HR,20160527,一代车托盘顶升或下降时车辆的最大旋转速度，degree/s
#define PARAM_LiftAngelSpeedAcc 150 //HR,20160527,一代车托盘顶升或下降时车辆的旋转加速度，degree/s2
#define PARAM_LoadCondition_AccAjust 151 //HR,20160708,带载工况下车辆加速度调节比例值（必需小于1）
#define PARAM_LiftPitch 152 //HR,20160716,升降螺距(升降电机转一圈对应的升降高度，mm)
#define PARAM_LiftDistance 153 //HR,20160720,升降行程(mm)
#define PARAM_DriverMotorTorqueCoefficient 154 //驱动电机转矩系数
#define PARAM_DriverMotorTorqueCapacity 155 //驱动电机额定转矩
#define PARAM_LiftMotorTorqueCoefficient 156 //顶升电机转矩系数
#define PARAM_LiftMotorTorqueCapacity 157 //顶升电机额定转矩

//导引判别偏差
#define PARAM_GuideRange 158 
//圆半径，像素
#define PARAM_Cam_Radius 159 
//圆心距，像素
#define PARAM_Cam_Distance 160 

#define PARAM_LowTempWarn 161 //低温报警
//车辆角速度控制参数:比例系数
#define PARAM_OffsetOmg_P 162
//车辆角速度控制参数:积分系数
#define PARAM_OffsetOmg_I 163
//车辆角速度控制参数:微分系数
#define PARAM_OffsetOmg_D 164
//低电量保护电压值
#define PARAM_BATTERY_LOW 165
//是否上传偏载数据
#define PARAM_HighTempERR 166	//高温报错,默认80

#define PARAM_CODE_GAP_X 167 //X向码间距,xingll,20170503
#define PARAM_CODE_GAP_Y 168 //Y向码间距,xingll
#define PARAM_Laser_StorageRackLegLength 169 //Y??????,add by hwei 20170710
#define PARAM_Laser_StorageRackWidth 170 //Y??????,add by hwei 20170710
#define PARAM_Laser_StorageRackLength 171 //Y??????,add by hwei 20170710
#define PARAM_Laser_LaserPosY 172 //Y??????,add by hwei 20170710
#define PARAM_Laser_LaserPosX 173 //Y??????,add by hwei 20170710
#define PARAM_Laser_StoragePassSize 174 //Y??????,add by hwei 20170710
#define PARAM_Laser_StorageRackLegWidth 175 //Y??????,add by hwei 20170710
#define PARAM_DisLaserOfTarget 176 //禁止雷达避障距离
#define PARAM_PowerCoeff 177	//功率系数
#define PARAM_NewLaser 178 //

#define PARAM_UARTA 112//(0-COM2;1-Com1)PARAM_KeepPace_TimeOutTol
#define PARAM_KINCO_SYNCSTOP_SWITCH 179 //add wf 20170911
#define PARAM_WITH_SELF 180 //add wf 20170925
#define PARAM_CHARGE_CODE_GAP 181 //add wf 20170925
#define PARAM_AvoidBarrierDecelerate 182
#define PARAM_LASER_ADJUSTAngle 183 //20171202,雷达安装标定角度
#define PARAM_LASER_SCAN_HZ 184 //雷达扫描频率

#define PARAM_DM642_NEW_PROTOCOL 185
#define PARAM_HARDWARE_VERSION_ID 186
#define PARAM_UpLed_Luminance_Resistant 187
#define PARAM_DownLed_Luminance_Resistant 188
#define PARAM_StopOnDM_NoErr 189
//小皮带参数
#define PARAM_Belt_FullLoad_Threshold 190
#define PARAM_Belt_Sensor 191
#define PARAM_Belt_Calibration_Near 192
#define PARAM_Belt_Calibration_Far 193
#define PARAM_BeltIR_Threshold 194

#define PARAM_SlideRail_Width 195
#define PARAM_Belt_Width 196
#define PARAM_Unload_CompTimes 197
#define PARAM_Unload_Vel 198
#define PARAM_Unload_StopMove 199
#define PARAM_Unload_StopTurn 200
#define PARAM_Belt_PreSpeed 201
#define PARAM_Belt_LoadSpeed 202
#define PARAM_Belt_UnLoadSpeed 203
#define PARAM_FullScope_Move 204
#define PARAM_FullScope_Turn 205

/*===============================华丽的分割线================================*/
extern struct ParamType Param;

extern double ChangeCodeGapX;
extern double ChangeCodeGapXError;
extern double ChangeCodeGapY;
extern double ChangeCodeGapYError;
extern double CHARGE_CODE_GAP;
extern double CODE_GAP;
extern double CODE_GAP_Y;
extern double CODE_GAP_X;
extern unsigned char ID_CAMERA;
extern struct ParamSaveType offline_save;
extern struct MapSaveType offline_save_map;
extern struct ParamType Param;
extern int podZeroDir;
extern double TOL_DIS_MOVE_X;
extern double TOL_DIS_MOVE_Y;
extern double TOL_DIS_ARC;
extern double TOL_DIS_MOVE_X;
extern double TOL_DIS_MOVE_Y;
extern double TOL_DIS_ARC_X;
extern Uint16 cam_param_num[20];
extern Uint16 cam_index;
typedef enum
{
	MIN_TEMP_ID = 0,
	TEMP_Gyro = MIN_TEMP_ID,
	TEMP_MotorLeft,
	TEMP_MotorRight,
	TEMP_MotorTrayLift,
	TEMP_MotorTrayRotate,
	TEMP_Battery,
	TEMP_ChargeInterfacePin,
	TEMP_ChargeInterface,
	TEMP_FrontPanel,
	TEMP_BackPanel,
	TEMP_Belt,
	TEMP_Laser,
	MAX_TEMP_ID
}TempId;
struct TempCheckParam_t
{
    int tmpWarnCnt;
    int tmpErrCnt;
    FH_Float temp;
    FH_Float warntemp;
    FH_Float errtemp;
    FH_Float ContinueTime;
	int cutWaitCnt;
};
struct CurrentProtectParams_t
{
	FH_Float StaticThreshold;
	FH_Float MotionThreshold;
	FH_Float ChargeThreshold;
	FH_Float ContinueTime;
};

extern struct TempCheckParam_t tempCheck[MAX_TEMP_ID];
//extern struct TempCheckParam_t tempMotorLeftCheck;
//extern struct TempCheckParam_t tempMotorRightCheck;
//extern struct TempCheckParam_t tempMotorTrayLiftCheck;
//extern struct TempCheckParam_t tempMotorTrayRotateCheck;
//extern struct TempCheckParam_t tempBatteryCheck;
extern struct CurrentProtectParams_t CurrentProtectParams;

/*---------------------------------end---------------------------------------*/
#define	MaxAppParamsNumer 210

union AppRawParams_u
{
	long double ParamsDouble;
	float ParamsFloat;
	int ParamsBuf[4];
	char ParamsString[4];
};

struct AgvAppParamsManager_t
{
	int ParamsNum;
	Uint16 paramsFlag[MaxAppParamsNumer/16+1];
	int MD5CheckValid;
	int HostComputerSupportValid;
	int readValid;
	int checkValid;
	char VersionID[10];
	union AppRawParams_u* appRawParams;
//	Uint32 check_sum_eeprom;
};
/*-----------------------------------------------------------------------------	
*输.....入：agvAppParamsManager-参数管理,agvAppParams-参数
*输.....出：无
*输入/输出：无
*返.....回：无
*目.....的：初始化上位机参数
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
int AppParamsInit(struct AgvAppParamsManager_t* agvAppParamsManager);
/*-----------------------------------------------------------------------------	
*输.....入：agvAppParamsManager-参数管理,CameraIndex-参数索引号
*输.....出：无
*输入/输出：无
*返.....回：无
*目.....的：逐个更新上位机参数
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
int SetAppParam4Single(struct AgvAppParamsManager_t* agvAppParamsManager,int CameraIndex);
/*-----------------------------------------------------------------------------	
*输.....入：agvAppParamsManager-参数管理
*输.....出：无
*输入/输出：无
*返.....回：无
*目.....的：全部更新上位机参数
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
void SetAppParam4All(struct AgvAppParamsManager_t* agvAppParamsManager);

/*-----------------------------------------------------------------------------	
*输.....入：无
*输.....出：无
*输入/输出：无
*返.....回：无
*目.....的：对上位机下发参数进行转换到DSP系统参数
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
int UpdateSystemParamsFromApp();
/*-----------------------------------------------------------------------------	
*输.....入：无
*输.....出：无
*输入/输出：无
*返.....回：无
*目.....的：计算上位机下发参数的MD5
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
int CalcAppParamsMd5();
/*-----------------------------------------------------------------------------	
*输.....入：无
*输.....出：无
*输入/输出：无
*返.....回：无
*目.....的：计算Camera参数
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
int UpdataCameraParams();
int DM642CameraParamsTest();

/*-----------------------------------------------------------------------------	
*输.....入：agvAppParamsManager-参数管理
*输.....出：无
*输入/输出：无
*返.....回：无
*目.....的：从IIC中读取上位机历史保存数据
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
void GetAppParams(struct AgvAppParamsManager_t* agvAppParamsManager);

/*-----------------------------------------------------------------------------	
*输.....入：agvAppParamsManager-参数管理
*输.....出：无
*输入/输出：无
*返.....回：无
*目.....的：校验参数
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
void ValidateAppParams(struct AgvAppParamsManager_t* agvAppParamsManager);

int ParamsProtect(void);//struct MotorDriver_t* md,void* params
int ValidHLSParams(int ifBigTorque);
int ValidKincoParams(int ifBigTorque);
int DM642CameraParamsTest();
int ParamQuanti(int param_index);
void SetParamFlag(int paramIndex);
void ResetParamFlag(int paramIndex);
int IsAppParamsFlagZero();
int ValidateKincoParams();
int AppParamsReset(struct AgvAppParamsManager_t* agvAppParamsManager);
void ResetAllParamFlag();
#endif /* AGVAPPPARAMS_H_ */
