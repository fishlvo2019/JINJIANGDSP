/*
 * AgvAppParams.c
 *
 *  Created on: 2017年10月25日
 *      Author: hwei
 */
#include <stdio.h>
#include <string.h>
#include"AgvApp.h"
#include "userfunction.h"
#include"basefunction.h"
#include "MD5_calc.h"
#include "fun_protype.h"
//#include "Mannual.h"


extern int DM642_NEW_PROTOCOL;
#pragma DATA_SECTION(appRawParamsBuf,"ZONE6DATA");
struct TempCheckParam_t tempCheck[MAX_TEMP_ID] = 
{
	{0,0,0,64,70,5,0},//TEMP_Gyro
	{0,0,0,56,70,5,0},//TEMP_MotorLeft
	{0,0,0,56,70,5,0},
	{0,0,0,56,70,5,0},
	{0,0,0,56,70,5,0},
	{0,0,0,50,70,5,0},//TEMP_Battery
	{0,0,0,50,70,5,0},//TEMP_ChargeInterfacePin
	{0,0,0,50,70,5,0},//TEMP_ChargeInterface
	{0,0,0,50,70,5,0},//TEMP_FrontPanel
	{0,0,0,50,70,5,0},//TEMP_BackPanel
	{0,0,0,50,70,5,0},//TEMP_Belt
	{0,0,0,50,70,5,0}//Laser
};
//电池电流及温度保护参数
struct CurrentProtectParams_t CurrentProtectParams = {5,60,-18,5};
union AppRawParams_u appRawParamsBuf[MaxAppParamsNumer];
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
int AppParamsInit(struct AgvAppParamsManager_t* agvAppParamsManager)
{
	int i=0;
	agvAppParamsManager->ParamsNum = PARAMMAXNUM;
	for(i=0;i<agvAppParamsManager->ParamsNum;i++)
	{
		SetParamFlag(i);
	}
	MemSet(appRawParamsBuf,0x0,MaxAppParamsNumer*sizeof(union AppRawParams_u));
	agvAppParamsManager->appRawParams = appRawParamsBuf;
	
	//默认V100即非产品化小车
	strcpy(agvAppParamsManager->VersionID,"V100");
	return 0;
}
int AppParamsReset(struct AgvAppParamsManager_t* agvAppParamsManager)
{
	AppParamsInit(agvAppParamsManager);

	return 0;
}

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
int SetAppParam4Single(struct AgvAppParamsManager_t* agvAppParamsManager,int CameraIndex)
{
	int rel_val = 0;
	rel_val = EEPRomWrite(&Agv.device.eepRom,AppParamsAddress + 4*CameraIndex,(Uint16*)&Param.AgvParam[CameraIndex],2);
	if(rel_val) 
	{
		LogPrintf("=ERROR= WriteSingleParamData\n");
//		SetError(ERROR_MAP_SaveNumFail,3);
	}
	else
	{
		
	}
	return 0;
}
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
void SetAppParam4All(struct AgvAppParamsManager_t* agvAppParamsManager)
{
	int rel_val = 0;
	rel_val = EEPRomWrite(&Agv.device.eepRom,AppParamsAddress,(Uint16*)&Param.AgvParam,200*sizeof(double));
	if(rel_val) 
	{
		LogPrintf("=ERROR= WriteAllParamData\n");
//		SetError(ERROR_MAP_SaveNumFail,3);
	}
}

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
void GetAppParams(struct AgvAppParamsManager_t* agvAppParamsManager)
{
    Uint16 rel_val = 0;
	//读取IIC中的地图数据
	/*
		由于修改Param.AgvParam到agvAppParamsManager->base.params,改动太大，故暂时使用原来的Param.AgvParam
	*/
//	rel_val = IICRead(&Agv.board.i2c,RomAddress,SystemParamsAddress,(Uint16*)(&agvAppParamsManager->base.params),200*sizeof(double));
	rel_val = EEPRomRead(&Agv.device.eepRom,AppParamsAddress,(Uint16*)(&Param.AgvParam),PARAMMAXNUM*sizeof(double));
    if(rel_val)
    {
        SetError(ERROR_ParamRead_Fail,3); 
        agvAppParamsManager->readValid = 0;
    }
    else
    {
		agvAppParamsManager->readValid = 1;
    }
	LogPrintf("GetAppParams res:%d\n",rel_val);
}
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
void ValidateAppParams(struct AgvAppParamsManager_t* agvAppParamsManager)
{
    int param_index = 0;
	if(0 == agvAppParamsManager->readValid)
	{
		return ;
	}
	agvAppParamsManager->checkValid = 0;
    for(param_index=0;param_index<PARAMMAXNUM;param_index++)
    {
		if(0 == ParamQuanti(param_index))
		{
			LogPrintf("param:%d wrong\n",param_index);
			return;
		}
    } 
	if(0 == ParamsProtect())
	{
		SetError(ERROR_ParamValid_Fail, ErrorC);
		AppParamsReset(&App.appParamsManager);
		LogPrintf("ParamsProtect error\n");
		return ;
	}
	LogPrintf("ValidateAppParams ok\n");
	agvAppParamsManager->checkValid = 1;
}

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
int UpdateSystemParamsFromApp()
{
	if(0 == App.appParamsManager.checkValid)
	{
		SetError(ERROR_ParamValid_Fail, ErrorC);
		return -1;
	}
	strcpy(AgvParams.HardwareVersion, App.appParamsManager.VersionID);
	
	/*************解析版本号***************/
	if(CheckAndConfigAgv())
	{
		return -2;
	}
	
	/*****************************/

//电机参数
	AgvParams.deviceParams.mdLeftParams.mdParams.ReduceRatio = Param.AgvParam[PARAM_WheelReduceRatio];
	AgvParams.deviceParams.mdRightParams.mdParams.ReduceRatio = Param.AgvParam[PARAM_WheelReduceRatio];
	AgvParams.deviceParams.mdTrayRotateParams.mdParams.ReduceRatio = Param.AgvParam[PARAM_LiftReduceRatio];
	
	AgvParams.deviceParams.mdLeftParams.mdParams.enc2RoundFactor= 1/AgvParams.deviceParams.mdLeftParams.mdParams.encPerRound/AgvParams.deviceParams.mdLeftParams.mdParams.ReduceRatio;
	AgvParams.deviceParams.mdRightParams.mdParams.enc2RoundFactor= 1/AgvParams.deviceParams.mdRightParams.mdParams.encPerRound/AgvParams.deviceParams.mdRightParams.mdParams.ReduceRatio;
	AgvParams.deviceParams.mdTrayRotateParams.mdParams.enc2RoundFactor= 1/AgvParams.deviceParams.mdTrayRotateParams.mdParams.encPerRound/AgvParams.deviceParams.mdTrayRotateParams.mdParams.ReduceRatio;



	AgvParams.deviceParams.mdLeftParams.mdParams.StaticCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Static];
	AgvParams.deviceParams.mdLeftParams.mdParams.DynamicCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Dynamic];
	AgvParams.deviceParams.mdLeftParams.mdParams.CurrentOverLargeDuration = Param.AgvParam[PARAM_MotoOverCurrentDuration]*100;

	AgvParams.deviceParams.mdRightParams.mdParams.StaticCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Static];
	AgvParams.deviceParams.mdRightParams.mdParams.DynamicCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Dynamic];
	AgvParams.deviceParams.mdRightParams.mdParams.CurrentOverLargeDuration = Param.AgvParam[PARAM_MotoOverCurrentDuration]*100;

	AgvParams.deviceParams.mdTrayRotateParams.mdParams.StaticCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Static];
	AgvParams.deviceParams.mdTrayRotateParams.mdParams.DynamicCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Dynamic];
	AgvParams.deviceParams.mdTrayRotateParams.mdParams.CurrentOverLargeDuration = Param.AgvParam[PARAM_MotoOverCurrentDuration]*100;

	AgvParams.deviceParams.mdTrayLiftParams.mdParams.StaticCurrentOverLargeValue = Param.AgvParam[PARAM_LIFTMotoTolCurrent_Static];
	AgvParams.deviceParams.mdTrayLiftParams.mdParams.DynamicCurrentOverLargeValue = Param.AgvParam[PARAM_LIFTMotoTolCurrent_Dynamic];
	AgvParams.deviceParams.mdTrayLiftParams.mdParams.VelErrorOverLargeValue = Param.AgvParam[PARAM_UPDOWN_SPEED]*1.1;
	AgvParams.deviceParams.mdTrayLiftParams.mdParams.CurrentOverLargeDuration = Param.AgvParam[PARAM_MotoOverCurrentDuration]*100;

	
	AgvParams.deviceParams.mdBeltLeftParams.mdParams.StaticCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Static];
	AgvParams.deviceParams.mdBeltLeftParams.mdParams.DynamicCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Dynamic];
	AgvParams.deviceParams.mdBeltLeftParams.mdParams.CurrentOverLargeDuration = Param.AgvParam[PARAM_MotoOverCurrentDuration]*100;

	
	AgvParams.deviceParams.mdBeltRightParams.mdParams.StaticCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Static];
	AgvParams.deviceParams.mdBeltRightParams.mdParams.DynamicCurrentOverLargeValue = Param.AgvParam[PARAM_DriveMotorTolCurrent_Dynamic];
	AgvParams.deviceParams.mdBeltRightParams.mdParams.CurrentOverLargeDuration = Param.AgvParam[PARAM_MotoOverCurrentDuration]*100;

	if(MD_KinCo ==AgvParams.deviceParams.mdLeftParams.mdParams.Type)
	{
		AgvParams.deviceParams.mdLeftParams.mdKincoParams.IsSynStopAbility = Param.AgvParam[PARAM_KINCO_SYNCSTOP_SWITCH];
		AgvParams.deviceParams.mdRightParams.mdKincoParams.IsSynStopAbility = Param.AgvParam[PARAM_KINCO_SYNCSTOP_SWITCH];
	}
	else if(HLS_SingleFrame ==AgvParams.deviceParams.mdLeftParams.mdParams.Type)
	{
		AgvParams.deviceParams.mdLeftParams.mdParams.currentFactor = Param.AgvParam[PARAM_DRIVER_CURRENT_NORMAL]*0.001;
		AgvParams.deviceParams.mdRightParams.mdParams.currentFactor = Param.AgvParam[PARAM_DRIVER_CURRENT_NORMAL]*0.001;
		
		AgvParams.deviceParams.mdTrayLiftParams.mdParams.currentFactor = Param.AgvParam[PARAM_DRIVER_CURRENT_NORMAL]*0.001;
		AgvParams.deviceParams.mdTrayRotateParams.mdParams.currentFactor = Param.AgvParam[PARAM_DRIVER_CURRENT_NORMAL]*0.001;
		
		AgvParams.deviceParams.mdBeltLeftParams.mdParams.currentFactor = Param.AgvParam[PARAM_DRIVER_CURRENT_NORMAL]*0.001;
		AgvParams.deviceParams.mdBeltRightParams.mdParams.currentFactor = Param.AgvParam[PARAM_DRIVER_CURRENT_NORMAL]*0.001;
	}

	//dm642参数
	
	SetDm642ProtocolType((int)Param.AgvParam[PARAM_DM642_NEW_PROTOCOL]);
	AgvParams.deviceParams.qrcodeParams.CODE_GAP_X = Param.AgvParam[PARAM_CODE_GAP_X];
	AgvParams.deviceParams.qrcodeParams.CODE_GAP_Y = Param.AgvParam[PARAM_CODE_GAP_Y];
	//更新码间距不报错
	AgvParams.deviceParams.qrcodeParams.codeFlag = 1;
	ThreadSleep(500);
	AgvParams.deviceParams.qrcodeParams.codeFlag = 0;
	AgvParams.deviceParams.qrcodeParams.UpCameraStandardTx = Param.AgvParam[PARAM_UpCameraStandardTx];
	AgvParams.deviceParams.qrcodeParams.UpCameraStandardTy = Param.AgvParam[PARAM_UpCameraStandardTy];
	AgvParams.deviceParams.qrcodeParams.UpCameraStandardRz= Param.AgvParam[PARAM_UpCameraStandardRz];
	AgvParams.deviceParams.qrcodeParams.UpCameraStandardTx_BeforeLift = Param.AgvParam[PARAM_UpCameraStandardTx_BeforeLift];
	AgvParams.deviceParams.qrcodeParams.UpCameraStandardTy_BeforeLift = Param.AgvParam[PARAM_UpCameraStandardTy_BeforeLift];
	AgvParams.deviceParams.qrcodeParams.UpCameraStandardRz_BeforeLift= Param.AgvParam[PARAM_UpCameraStandardRz_BeforeLift];
	
//托盘参数
//零位
	AgvParams.serviceParams.trayParams.RotateLimitAngle = Param.AgvParam[PARAM_PodLSP];
	AgvParams.serviceParams.trayParams.LiftUpLimitPos = Param.AgvParam[PARAM_LiftDistance]*0.001;
	AgvParams.serviceParams.trayParams.ShelfCollisionAngle = Param.AgvParam[PARAM_ShelfCollision_Angle];
	if((Param.AgvParam[PARAM_NewLaser] == 1)||(Param.AgvParam[PARAM_NewLaser] == 3))
	{
		AgvParams.serviceParams.trayParams.IsShelfLegCheckEnable = 1;
	}
	else
	{
		AgvParams.serviceParams.trayParams.IsShelfLegCheckEnable = 0;
	}

//控制
	//AgvParams.serviceParams.trayParams.axisTrayLiftParams.pidCtrlParams.Kp = Param.AgvParam[PARAM_Turning_Decelerate_P];
	//AgvParams.serviceParams.trayParams.axisTrayLiftParams.pidCtrlParams.Ki = Param.AgvParam[PARAM_Turning_Decelerate_I]/AgvSamplePeriod;
	//AgvParams.serviceParams.trayParams.axisTrayRotateParams.pidCtrlParams.Kp = Param.AgvParam[PARAM_PodHeadingTurning_P];
	//AgvParams.serviceParams.trayParams.axisTrayRotateParams.pidCtrlParams.Ki = Param.AgvParam[PARAM_PodHeadingTurning_P]/AgvSamplePeriod;;
	AgvParams.serviceParams.trayParams.axisTrayRotateParams.ctrlErrOverLargeValue = Param.AgvParam[PARAM_ShelfCollision_Angle];


//PARAM_LiftPitch 普通2.16，大扭矩0.32，
	AgvParams.serviceParams.trayParams.Motor2LiftFactor = Param.AgvParam[PARAM_LiftPitch]*0.001;//转化为国际单位
	AgvParams.serviceParams.trayParams.Lift2MotorFactor = 60/AgvParams.serviceParams.trayParams.Motor2LiftFactor;

//底盘参数
	AgvParams.serviceParams.chassisParams.locationParams.DistanceOfTwoWheel = Param.AgvParam[PARAM_WheelDis];
	AgvParams.serviceParams.chassisParams.locationParams.wheelRadius = Param.AgvParam[PARAM_WheelDia];
	AgvParams.serviceParams.chassisParams.locationParams.motor2WheelFactor = Param.AgvParam[PARAM_WheelDia]*2*PI/60;
	AgvParams.serviceParams.chassisParams.locationParams.wheel2MotorFactor = 60/(Param.AgvParam[PARAM_WheelDia]*2*PI);
	AgvParams.serviceParams.chassisParams.locationParams.chassisMinSpd = Param.AgvParam[PARAM_AgvMinSPD];
	AgvParams.serviceParams.chassisParams.SearchCodeSpd =  Param.AgvParam[PARAM_AgvMinSPD]*2.5;
	AgvParams.serviceParams.chassisParams.MaxMoveSpeed = Param.AgvParam[PARAM_AgvMaxSPD]*1.1;
	AgvParams.serviceParams.chassisParams.MaxTurnSpeed = Param.AgvParam[PARAM_AgvMaxAngSPD]*1.1;

	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisXTargetPosTolerance = Param.AgvParam[PARAM_TargetTolX];
	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisYTargetPosTolerance = Param.AgvParam[PARAM_TargetTolX];
	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisHeadingTargetPosTolerance = Param.AgvParam[PARAM_TargetTurningTolHeading];

//控制参数
	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisYParams.ctrlErrOverLargeValue = 2*Param.AgvParam[PARAM_OffsetTol];//Max(Param.AgvParam[PARAM_OffsetTol],0.05);
	AgvParams.serviceParams.chassisParams.multiAxisParams.OffsetYLimitDecel = Param.AgvParam[PARAM_OffsetLimitSpeed];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OffsetYStartDecel = Param.AgvParam[PARAM_Offset_StartDecel];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OffsetYMoveTol = Param.AgvParam[PARAM_OffsetMoveTol];
	
	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisXParams.pidCtrlParams.Kp = Param.AgvParam[PARAM_Move_Decelerate_P];
	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisXParams.pidCtrlParams.Ki = Param.AgvParam[PARAM_Move_Decelerate_I]/AgvSamplePeriod;
	
	if(Param.AgvParam[PARAM_Offset_D]<ZeroFloat)
	{
		if(Param.AgvParam[PARAM_Slip_CtlMode]>=10)
		{
			AgvParams.serviceParams.chassisParams.multiAxisParams.offsetYControlMethodType = 1;
		}
		else
		{
			AgvParams.serviceParams.chassisParams.multiAxisParams.offsetYControlMethodType = 0;
		}
		//现场的参数微分都是0，由于策略变更，需要微分参数，纠偏控制参数不改
	//AgvParams.serviceParams.chassisParams.axisYParams.pidCtrlParams.Kp = Param.AgvParam[PARAM_Offset_P];
	//AgvParams.serviceParams.chassisParams.axisYParams.pidCtrlParams.Ki = Param.AgvParam[PARAM_Offset_I]/AgvSamplePeriod;
	//AgvParams.serviceParams.chassisParams.axisYParams.pidCtrlParams.Kd = Param.AgvParam[PARAM_Offset_D]*AgvSamplePeriod;
	}
	else
	{
		AgvParams.serviceParams.chassisParams.multiAxisParams.offsetYControlMethodType = 1;
		AgvParams.serviceParams.chassisParams.multiAxisParams.AxisYParams.pidCtrlParams.Kp = Param.AgvParam[PARAM_Offset_P];
		AgvParams.serviceParams.chassisParams.multiAxisParams.AxisYParams.pidCtrlParams.Ki = Param.AgvParam[PARAM_Offset_I];
		AgvParams.serviceParams.chassisParams.multiAxisParams.AxisYParams.pidCtrlParams.Kd = Param.AgvParam[PARAM_Offset_D];
	}
	AgvParams.serviceParams.chassisParams.multiAxisParams.HeadingLimitWhenMove = Param.AgvParam[PARAM_HighMove_OffsetAjustAngle_Limit];

	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisHeadingParams.pidCtrlParams.Kp = Param.AgvParam[PARAM_Turning_Decelerate_P];//Param.AgvParam[PARAM_Heading_P];
	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisHeadingParams.pidCtrlParams.Ki = Param.AgvParam[PARAM_Turning_Decelerate_I]/AgvSamplePeriod;//Param.AgvParam[PARAM_Heading_I]/AgvSamplePeriod;
	AgvParams.serviceParams.chassisParams.multiAxisParams.AxisHeadingParams.pidCtrlParams.Kd = 0;//Param.AgvParam[PARAM_Heading_D]*AgvSamplePeriod;

	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.slipCtlMode =Param.AgvParam[PARAM_Slip_CtlMode];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.offsetDecP =  Param.AgvParam[PARAM_Offset_Decel_P];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.offsetP = Param.AgvParam[PARAM_Offset_P];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.offsetI = Param.AgvParam[PARAM_Offset_I];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.HighMove_OffsetAjustAngle_Limit = Param.AgvParam[PARAM_HighMove_OffsetAjustAngle_Limit];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.SlowMove_OffsetAjustAngle_Limit = Param.AgvParam[PARAM_SlowMove_OffsetAjustAngle_Limit];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.Heading_P = Param.AgvParam[PARAM_Heading_P];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.Heading_I = Param.AgvParam[PARAM_Heading_I];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.OffsetOmg_P = Param.AgvParam[PARAM_OffsetOmg_P];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.OffsetOmg_I = Param.AgvParam[PARAM_OffsetOmg_I];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.OffsetOmg_D = Param.AgvParam[PARAM_OffsetOmg_I];
	AgvParams.serviceParams.chassisParams.multiAxisParams.OldControllerParams.WheelDis = Param.AgvParam[PARAM_WheelDis];
//休眠参数
	AgvParams.serviceParams.powerManagerParams.dormancyTime = Param.AgvParam[PARAM_DormancyTime] * 60;
	AgvParams.serviceParams.powerManagerParams.dormancyDelayTime = Param.AgvParam[PARAM_DormancyDelayTime];
	AgvParams.serviceParams.powerManagerParams.trayLiftState = Param.AgvParam[PARAM_LiftTopCycles]*0.5;
//避障参数
//	AgvParams.serviceParams.avoidObstacleParams.methodType = Param.AgvParam[PARAM_AvoidBarrierDecelerate];
	SetAvoidObstacleParams((int)Param.AgvParam[PARAM_AvoidBarrierDecelerate]);
//	if(Param.AgvParam[PARAM_LASER_SWITCH_BYPASS])
//	{
	AvoidObstacleSwitch((int)Param.AgvParam[PARAM_LASER_SWITCH_BYPASS]);
	SetLaserActive((int)Param.AgvParam[PARAM_LASER_SWITCH_BYPASS]);
	AgvParams.serviceParams.avoidObstacleParams.disLaserofTarget =Param.AgvParam[PARAM_DisLaserOfTarget];
	AgvParams.deviceParams.batteryParams.batterySRParams.ctrl48VCoeff =Param.AgvParam[PARAM_PowerCoeff];
	AgvParams.deviceParams.pcParams.pcCmdRecParams.logCom=Param.AgvParam[PARAM_UARTA];
	struct DiagnosticManager_t*  diagnosticInstance=&Agv.service.diagnosticManager;
	if(1 == AgvParams.deviceParams.pcParams.pcCmdRecParams.logCom)
		LoggerInit(&diagnosticInstance->board->myPrintf,&diagnosticInstance->board->sciA.sendFifo);
//	}
//	else
//	{
//		AvoidObstacleSwitch(1);
//	}
	//更新雷达参数
	
	AgvParams.deviceParams.laserParams.SpdCmd = Param.AgvParam[PARAM_LASER_SCAN_HZ];
	AgvParams.deviceParams.laserParams.Width = Param.AgvParam[PARAM_LaserSigleWidth]*100;//厘米
	AgvParams.deviceParams.laserParams.Length = Param.AgvParam[PARAM_LaserLength]*10;//分米
	AgvParams.deviceParams.laserParams.StorageRackWidth = Param.AgvParam[PARAM_Laser_StorageRackWidth];
	AgvParams.deviceParams.laserParams.StorageRackLegWidth = Param.AgvParam[PARAM_Laser_StorageRackLegWidth];//单位：mm
	AgvParams.deviceParams.laserParams.StorageRackLength= Param.AgvParam[PARAM_Laser_StorageRackLength];//单位：cm
	AgvParams.deviceParams.laserParams.StorageRackLegLength= Param.AgvParam[PARAM_Laser_StorageRackLegLength];//单位：mm
	AgvParams.deviceParams.laserParams.LaserPosY = Param.AgvParam[PARAM_Laser_LaserPosY];//单位：mm
	AgvParams.deviceParams.laserParams.LaserPosX= Param.AgvParam[PARAM_Laser_LaserPosX];//单位：mm
	AgvParams.deviceParams.laserParams.StoragePassSize = Param.AgvParam[PARAM_Laser_StoragePassSize];//单位：cm
	AgvParams.deviceParams.laserParams.LaserPosAangle =Param.AgvParam[PARAM_LASER_ADJUSTAngle]*10;//单位：0.1度
	AgvParams.deviceParams.laserParams.NoDataTimeOut =  Param.AgvParam[PARAM_LaserOpenNoDataTimeOutTol];
	AgvParams.deviceParams.laserParams.ProtocolType = Param.AgvParam[PARAM_NewLaser];
	SetLaserCycle(&Agv.device.laser,10,20);//小朱雀避障参数更新
//皮带参数
	if(Agv.service.beltManager.BeltType != NONE_BELT)
	{
		Agv.service.beltManager.BeltLoadStateMachine = BeltTaskFree;
		Agv.service.beltManager.BeltUnLoadStateMachine = UNLOAD_BeltTaskFree;
		Agv.service.beltManager.BeltMovingUnLoadStateMachine = UNLOAD_BeltTaskFree;
		AgvParams.serviceParams.leftBeltParams.beltSensorType = ((int)Param.AgvParam[PARAM_Belt_Sensor]==0)?IR_SENSOR:LIGHT_SENSOR;
		AgvParams.serviceParams.rightBeltParams.beltSensorType = ((int)Param.AgvParam[PARAM_Belt_Sensor]==0)?IR_SENSOR:LIGHT_SENSOR;
		AgvParams.serviceParams.leftBeltParams.loadPreSpeed = Param.AgvParam[PARAM_Belt_PreSpeed];
		AgvParams.serviceParams.leftBeltParams.loadSpeed = Param.AgvParam[PARAM_Belt_LoadSpeed];
		AgvParams.serviceParams.leftBeltParams.unLoadSpeed = Param.AgvParam[PARAM_Belt_UnLoadSpeed];
		AgvParams.serviceParams.rightBeltParams.loadPreSpeed = Param.AgvParam[PARAM_Belt_PreSpeed];
		AgvParams.serviceParams.rightBeltParams.loadSpeed = Param.AgvParam[PARAM_Belt_LoadSpeed];
		AgvParams.serviceParams.rightBeltParams.unLoadSpeed = Param.AgvParam[PARAM_Belt_UnLoadSpeed];
		BeltInitWithType(&Agv.service.leftBelt, &AgvParams.serviceParams.leftBeltParams);
		BeltInitWithType(&Agv.service.rightBelt, &AgvParams.serviceParams.rightBeltParams);
	}
	#if LaserRebuild
	if((Param.AgvParam[PARAM_NewLaser] == 1)||(Param.AgvParam[PARAM_NewLaser] == 3))
	{
		SendLaserParams(&Agv.device.laser);
	}
		
	#else
	Laser.SpdCmd =Param.AgvParam[PARAM_LASER_SCAN_HZ];
	Laser.Width = Param.AgvParam[PARAM_LaserSigleWidth]*100;//20cm
	Laser.Length = Param.AgvParam[PARAM_LaserLength]*10;//150cm
	Laser.storageRackInfo.StorageRackWidth = Param.AgvParam[PARAM_Laser_StorageRackWidth];//单位：cm
	Laser.storageRackInfo.StorageRackLegWidth = Param.AgvParam[PARAM_Laser_StorageRackLegWidth];//单位：mm
	Laser.storageRackInfo.StorageRackLength = Param.AgvParam[PARAM_Laser_StorageRackLength];//单位：cm
	Laser.storageRackInfo.StorageRackLegLength = Param.AgvParam[PARAM_Laser_StorageRackLegLength];//单位：mm
	Laser.storageRackInfo.LaserPosY = Param.AgvParam[PARAM_Laser_LaserPosY];//单位：mm
	Laser.storageRackInfo.LaserPosX = Param.AgvParam[PARAM_Laser_LaserPosX];//单位：mm
	Laser.storageRackInfo.StoragePassSize = Param.AgvParam[PARAM_Laser_StoragePassSize];//单位：cm
	Laser.storageRackInfoNew.StorageRackWidth =Param.AgvParam[PARAM_Laser_StorageRackWidth];//单位：cm
	Laser.storageRackInfoNew.StorageRackLength = Param.AgvParam[PARAM_Laser_StorageRackLength];//单位：cm
	Laser.storageRackInfoNew.StorageRackLegWidth = Param.AgvParam[PARAM_Laser_StorageRackLegWidth];//单位：mm
	Laser.storageRackInfoNew.StorageRackLegLength = Param.AgvParam[PARAM_Laser_StorageRackLegLength];//单位：mm
	Laser.storageRackInfoNew.LaserPosY = Param.AgvParam[PARAM_Laser_LaserPosY];//单位：mm
	Laser.storageRackInfoNew.LaserPosX = Param.AgvParam[PARAM_Laser_LaserPosX];//单位：mm
	Laser.storageRackInfoNew.StoragePassSize = Param.AgvParam[PARAM_Laser_StoragePassSize];//单位：cm
	Laser.storageRackInfoNew.LaserPosAangle =Param.AgvParam[PARAM_LASER_ADJUSTAngle]*10;//单位：0.1度

	SendLaserStorageRackInfo_PosAdjust(&Laser.storageRackInfoNew);
	#endif
    AgvParams.serviceParams.dymCodegapParams.mapBorderMax_X = Param.AgvParam[PARAM_MapBorderXmax];//dymCodegap->areaAddNormal[maxIndex].endNormalX;
    AgvParams.serviceParams.dymCodegapParams.mapBorderMax_Y = Param.AgvParam[PARAM_MapBorderYmax];//dymCodegap->areaAddNormal[maxIndex].endNormalY;
    AgvParams.serviceParams.dymCodegapParams.mapBorderMin_X = Param.AgvParam[PARAM_MapBorderXmin];//dymCodegap->areaAddNormal[0].startNormalX;
    AgvParams.serviceParams.dymCodegapParams.mapBorderMin_Y = Param.AgvParam[PARAM_MapBorderYmin];//dymCodegap->areaAddNormal[0].startNormalY;
//更新皮带参数
	AgvParams.serviceParams.beltManagerParams.beltUnloadParams.slideRailWidth = Param.AgvParam[PARAM_SlideRail_Width];
	AgvParams.serviceParams.beltManagerParams.beltUnloadParams.beltWidth = Param.AgvParam[PARAM_Belt_Width];
	AgvParams.serviceParams.beltManagerParams.beltUnloadParams.compTimes = Param.AgvParam[PARAM_Unload_CompTimes];
	AgvParams.serviceParams.beltManagerParams.beltUnloadParams.unloadVel = Param.AgvParam[PARAM_Unload_Vel];
	AgvParams.serviceParams.beltManagerParams.beltUnloadParams.stopMove = Param.AgvParam[PARAM_Unload_StopMove];
	AgvParams.serviceParams.beltManagerParams.beltUnloadParams.stopTurn = Param.AgvParam[PARAM_Unload_StopTurn]*PI/180;
	//iic保存参数等待
	ThreadSleep(100);
	if(SetSystemParams())
	{
		return -2;
	}
	LogPrintf("updateSystemParams ok\n");
	Agv.base.state = FHAGV_Initialized;
	return 0;
}
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
int CalcAppParamsMd5()
{
	calcMD5(Param.AgvParamInit,PARAMMAXNUM);
	return 0;
}
int DM642CameraParamsTest()
{
	int cam_index = 0;
	while(cam_param_num[cam_index] > 0)
	{
		Param.CameraParamIndex = cam_param_num[cam_index];
		switch(Param.CameraParamIndex)
		{
			case 1:                            
			    Param.CameraParam[1] = (unsigned short)(2004);
			    ID_CAMERA =0;
			    break;
			case 2:
			    Param.CameraParam[2] = (unsigned short)(1997);
			    ID_CAMERA =0;
			    break;
			case 3:
			    Param.CameraParam[3] = (unsigned short)(18000.0);
			    ID_CAMERA =0;
			    break;
			case 4:
			    Param.CameraParam[4] = (unsigned short)(18000.0);
			    ID_CAMERA =0;
			    break;
			case 5:
			    Param.CameraParam[5] = (unsigned short)(18000.0);
			    ID_CAMERA =0;
			    break;
			case 6:
			    Param.CameraParam[6] = (unsigned short)(20);
			    break;
			case 7:
			    Param.CameraParam[7] = (unsigned short)(60);
			    break;

			case 8:
			    Param.CameraParam[8] = (unsigned short)(60);
			    break;

			case 9:
			    Param.CameraParam[9] = (unsigned short)(0);             
			    break;
			    
			case 15:
			    Param.CameraParam[15] = (unsigned short)(20);             
			    break;
			case 17:
			    Param.CameraParam[17] = (unsigned short)(110);             
			    ID_CAMERA =0;
			    break;
			case 18:
			    Param.CameraParam[18] = (unsigned short)(105);             
			    ID_CAMERA =0;
			    break;
			default:
			    break;
		}
		SendDataDM642();
		cam_index++;
		ThreadSleep(30);
		DM642TrigSend(0,0,0,0,0,GetQRMode());
	}
	LogPrintf("updateCameraParams ok\n");
	
	QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_DOWN);
	return 0;
}

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
int UpdataCameraParams()
{
	cam_index = 0;
	unsigned short temp = 0;
	while(cam_param_num[cam_index] > 0)
	{
		Param.CameraParamIndex = cam_param_num[cam_index];
		switch(Param.CameraParamIndex)
		{
			case 1:                            
			    Param.CameraParam[1] = (unsigned short)(Param.AgvParam[PARAM_CameraStandardTx] * 1000.0 + 2000.0);
			    ID_CAMERA =0;
			    break;
			case 2:
			    Param.CameraParam[2] = (unsigned short)(Param.AgvParam[PARAM_CameraStandardTy] * 1000.0 + 2000.0);
			    ID_CAMERA =0;
			    break;
			case 3:
			    Param.CameraParam[3] = (unsigned short)(Param.AgvParam[PARAM_CameraStandardRz] * 100.0 + 18000.0);
			    ID_CAMERA =0;
			    break;
			case 4:
			    Param.CameraParam[4] = (unsigned short)(Param.AgvParam[PARAM_CameraStandardY2C] * 100.0 + 18000.0);
			    ID_CAMERA =0;
			    break;
			case 5:
			    Param.CameraParam[5] = (unsigned short)(Param.AgvParam[PARAM_ADDAngle] * 100.0 + 18000.0);
			    ID_CAMERA =0;
			    break;
			case 6:
			    Param.CameraParam[6] = (unsigned short)(Param.AgvParam[PARAM_ExposureTime]);
			    break;
			case 7:
			    Param.CameraParam[7] = (unsigned short)(Param.AgvParam[PARAM_LateralFieldView] * 1000.0);
			    break;

			case 8:
			    Param.CameraParam[8] = (unsigned short)(Param.AgvParam[PARAM_VerticalFieldView] * 1000.0);
			    break;

			case 9:
			    Param.CameraParam[9] = (unsigned short)(Param.AgvParam[PARAM_CameraChoose]);             
			    break;
			    
			case 15:
			    Param.CameraParam[15] = (unsigned short)(Param.AgvParam[PARAM_UpCameraExposureTime]);             
			    break;
			case 16:
				//切换图像解码帧率
				temp =50;
				temp =(temp<<8);
				Param.CameraParam[16] =temp;
				break;
			case 17:
			    Param.CameraParam[17] = (unsigned short)(Param.AgvParam[PARAM_Cam_Radius]);             
			    ID_CAMERA =0;
			    break;
			case 18:
			    Param.CameraParam[18] = (unsigned short)(Param.AgvParam[PARAM_Cam_Distance]);             
			    ID_CAMERA =0;
			    break;
			default:
			    break;
		}
		SendDataDM642();
		cam_index++;
		ThreadSleep(50);
	}
	QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_DOWN);
	LogPrintf("updateCameraParams ok\n");
	return 0;
}

int ParamQuanti(int param_index)
{
	int AValue = 0;
	int BValue = 0;
	int CValue = 0;
	//设置动态码间距默认值
	struct DymCodegap_t* dymCodegap = &Agv.service.dymCodegap;
	Param.AgvParamInit[param_index] = Param.AgvParam[param_index];
	switch(param_index)
	{			
        case PARAM_AgvMaxSPD:
			if(Param.AgvParam[param_index]>3)
			{
				LogPrintf("vel:%f\n",Param.AgvParam[param_index]);
				return 0;
			}
             LoadCondition.MaxSpd[0] = Param.AgvParam[PARAM_AgvMaxSPD];
             break;
             
        case PARAM_AgvMaxAcc:
			if(Param.AgvParam[param_index]>3)
			{
				LogPrintf("acc:%f\n",Param.AgvParam[param_index]);
				return 0;
			}
             LoadCondition.MaxAcc[0] = Param.AgvParam[PARAM_AgvMaxAcc];
             break;
             
        case PARAM_MAX_ACC_SENSOR:
             LoadCondition.SoftStopAcc[0] = Param.AgvParam[PARAM_MAX_ACC_SENSOR];
             break;             

		case PARAM_AgvMaxAngSPD:
			 Param.AgvParam[PARAM_AgvMaxAngSPD] = Param.AgvParam[PARAM_AgvMaxAngSPD]/180*PI;
			 LoadCondition.MaxChangeDirectionAngularSpd[0] = Param.AgvParam[PARAM_AgvMaxAngSPD];
			 LoadCondition.MaxAngularSpd[0] = Param.AgvParam[PARAM_AgvMaxAngSPD];
			 break;
			 
		case PARAM_AgvMaxAngAcc:
		     Param.AgvParam[PARAM_AgvMaxAngAcc] = Param.AgvParam[PARAM_AgvMaxAngAcc]/180*PI;
			 LoadCondition.ChangeDirectionAngularAcc[0] = Param.AgvParam[PARAM_AgvMaxAngAcc];
			 LoadCondition.MaxAngularAcc[0] = Param.AgvParam[PARAM_AgvMaxAngAcc];
			 break;
			 
		case PARAM_AgvMinAngSPD:
		     Param.AgvParam[PARAM_AgvMinAngSPD] = Param.AgvParam[PARAM_AgvMinAngSPD]/180*PI;
			 break;
			 
		case PARAM_LiftTopCycles:
		     Param.AgvParam[PARAM_LiftTopCycles] = Param.AgvParam[PARAM_LiftTopCycles]*PI*2.0;
			 break;
			 
		case PARAM_LiftUpDeceAngCompensate:
		     Param.AgvParam[PARAM_LiftUpDeceAngCompensate] = Param.AgvParam[PARAM_LiftUpDeceAngCompensate]/180.0*PI;
			 break;
		
		case PARAM_TurningDeceAngCompensate:
		     Param.AgvParam[PARAM_TurningDeceAngCompensate] = Param.AgvParam[PARAM_TurningDeceAngCompensate]/180.0*PI;
			 break;
			 	 
		case PARAM_TargetTolPod:
		     Param.AgvParam[PARAM_TargetTolPod] = Param.AgvParam[PARAM_TargetTolPod]/180.0*PI; 
			 break;
			 
		case PARAM_TargetMoveTolHeading:
			 Param.AgvParam[PARAM_TargetMoveTolHeading] = Param.AgvParam[PARAM_TargetMoveTolHeading]/180.0*PI; 
			 break;
			 
		case PARAM_TargetLiftUpTolHeading:
			 Param.AgvParam[PARAM_TargetLiftUpTolHeading] = Param.AgvParam[PARAM_TargetLiftUpTolHeading]/180.0*PI; 
			 break;
			 
		case PARAM_TargetTurningTolHeading:
			 Param.AgvParam[PARAM_TargetTurningTolHeading] = Param.AgvParam[PARAM_TargetTurningTolHeading]/180.0*PI; 
			 break;
			 	  	 
		case PARAM_PodLSP:
		     Param.AgvParam[PARAM_PodLSP] = Param.AgvParam[PARAM_PodLSP]/180.0*PI;
		     #if VEHICLE_VERSION_SECOND
		     Param.AgvParam[PARAM_PodLSP] =WrapToPiDe(Param.AgvParam[PARAM_PodLSP]);
		    
		     #endif
			 break;
			 
		case PARAM_LiftReduceRatio: //HR,add,20161021
		     //Pod.PodRoundCount =Param.AgvParam[PARAM_LiftReduceRatio]*4*2500;
		     break;
		     
		case PARAM_OffsetHeading_DecelEndTol:
			 Param.AgvParam[PARAM_OffsetHeading_DecelEndTol] = Param.AgvParam[PARAM_OffsetHeading_DecelEndTol]/180.0*PI;
			 break;
			 
		case PARAM_TargetTurnTolPodHeading:
		     Param.AgvParam[PARAM_TargetTurnTolPodHeading] = Param.AgvParam[PARAM_TargetTurnTolPodHeading]/180.0*PI;
			 break;
		case PARAM_TargetLiftUpTolPodHeading:
		     Param.AgvParam[PARAM_TargetLiftUpTolPodHeading] = Param.AgvParam[PARAM_TargetLiftUpTolPodHeading]/180.0*PI;
			 break;
		case PARAM_HeadingUnstableLimit:
		     Param.AgvParam[PARAM_HeadingUnstableLimit] = Param.AgvParam[PARAM_HeadingUnstableLimit]/180.0*PI;
			 break;
			 
	    case PARAM_HeadingLightUnstableLimit:
		     Param.AgvParam[PARAM_HeadingLightUnstableLimit] = Param.AgvParam[PARAM_HeadingLightUnstableLimit]/180.0*PI;
			 break;
			 
		case PARAM_ShelfCollision_Angle:	
		     Param.AgvParam[PARAM_ShelfCollision_Angle] = Param.AgvParam[PARAM_ShelfCollision_Angle]/180.0*PI;
			 break;
			  
	    case PARAM_CameraStandardTx: 
	         Param.CameraParamIndex = 1;
	         Param.CameraParam[1] = (unsigned short)(Param.AgvParam[PARAM_CameraStandardTx] * 1000.0 + 2000.0);
	         ID_CAMERA =0;
	         break;
	         
        case PARAM_CameraStandardTy:  
             Param.CameraParamIndex = 2;
             Param.CameraParam[2] = (unsigned short)(Param.AgvParam[PARAM_CameraStandardTy] * 1000.0 + 2000.0);
             ID_CAMERA =0;
             break;
             
        case PARAM_CameraStandardRz:  
             Param.CameraParamIndex = 3;
             Param.CameraParam[3] = (unsigned short)(Param.AgvParam[PARAM_CameraStandardRz] * 100.0 + 18000.0);
             ID_CAMERA =0;
             break;
             
        case PARAM_CameraStandardY2C: 
             Param.CameraParamIndex = 4; 
             Param.CameraParam[4] = (unsigned short)(Param.AgvParam[PARAM_CameraStandardY2C] * 100.0 + 18000.0);
             ID_CAMERA =0;
             break;
             
        case PARAM_ADDAngle: 
             Param.CameraParamIndex = 5;
             Param.CameraParam[5] = (unsigned short)(Param.AgvParam[PARAM_ADDAngle] * 100.0 + 18000.0);
             ID_CAMERA =0;
             break;
             
        case PARAM_ExposureTime: 
             Param.CameraParamIndex = 6;
             Param.CameraParam[6] = (unsigned short)(Param.AgvParam[PARAM_ExposureTime]);
             break;
             
        case PARAM_LateralFieldView: 
             Param.CameraParamIndex = 7;
             Param.CameraParam[7] = (unsigned short)(Param.AgvParam[PARAM_LateralFieldView] * 1000.0);
             break;
             
        case PARAM_VerticalFieldView: 
             Param.CameraParamIndex = 8;
             Param.CameraParam[8] = (unsigned short)(Param.AgvParam[PARAM_VerticalFieldView] * 1000.0);
             break;
             	
        case PARAM_CameraChoose:      
             Param.CameraParamIndex = 9;
             Param.CameraParam[9] = (unsigned short)(Param.AgvParam[PARAM_CameraChoose]);             
             break;
             
        case PARAM_UpCameraExposureTime:
             Param.CameraParamIndex = 15;
             Param.CameraParam[15] = (unsigned short)(Param.AgvParam[PARAM_UpCameraExposureTime]);             
             break;
             
        case PARAM_Cam_Radius:
             Param.CameraParamIndex = 17;
             Param.CameraParam[17] = (unsigned short)(Param.AgvParam[PARAM_Cam_Radius]);             
             break;
             
        case PARAM_Cam_Distance:
             Param.CameraParamIndex = 18;
             Param.CameraParam[18] = (unsigned short)(Param.AgvParam[PARAM_Cam_Distance]);             
             break;
             
//        case PARAM_UpCameraStandardTx:      
//             Param.CameraParamIndex = 10;
//             Param.CameraParam[10] = (unsigned short)(Param.AgvParamTem[PARAM_UpCameraStandardTx] * 1000.0 + 2000.0);             
//             break;
//             
//        case PARAM_UpCameraStandardTy:      
//             Param.CameraParamIndex = 11;
//             Param.CameraParam[11] = (unsigned short)(Param.AgvParamTem[PARAM_UpCameraStandardTy] * 1000.0 + 2000.0);             
//             break;
        
        case PARAM_UpCameraStandardRz:      
             Param.AgvParam[PARAM_UpCameraStandardRz] = Param.AgvParam[PARAM_UpCameraStandardRz]/180.0*PI;
//             Param.CameraParam[12] = (unsigned short)(Param.AgvParamTem[PARAM_UpCameraStandardRz]/180.0*PI);             
             break;
        case PARAM_UpCamera_Shelf_OffsetHeading:
             Param.AgvParam[PARAM_UpCamera_Shelf_OffsetHeading] = Param.AgvParam[PARAM_UpCamera_Shelf_OffsetHeading]/180.0*PI;
			 break;
        case PARAM_MapBorderXmin:
             if (Param.AgvParam[PARAM_MapBorderXmin]<1)
             {
             	SetError(ERROR_MapBorder_MinX,3);
             	return 0;
             }
//             if (Param.SetMapData!=0)
//             {
//             Param.SetMapData =0;
//             memset(mapdata,0,250000);
//             }
             break;
        case PARAM_MapBorderYmin:
             if (Param.AgvParam[PARAM_MapBorderYmin]<1)
             {
             	SetError(ERROR_MapBorder_MinY,3);
             	return 0;
             }
//             if (Param.SetMapData!=0)
//             {
//             Param.SetMapData =0;
//             memset(mapdata,0,250000);
//             }
             break;
        case PARAM_MapBorderXmax:
             if (Param.AgvParam[PARAM_MapBorderXmax]<1 || Param.AgvParam[PARAM_MapBorderXmax]<Param.AgvParam[PARAM_MapBorderXmin])
             {
             	SetError(ERROR_MapBorder_MaxX,3);
             	return 0;
             }
//             MapLimit_MaxX =(unsigned int)(Param.AgvParam[PARAM_MapBorderXmax]+0.5);
//             if (Param.SetMapData!=0)
//             {
//             Param.SetMapData =0;
//             memset(mapdata,0,250000);
//             }
             break;
        case PARAM_MapBorderYmax:
             if (Param.AgvParam[PARAM_MapBorderYmax]<1 || Param.AgvParam[PARAM_MapBorderYmax]<Param.AgvParam[PARAM_MapBorderYmin])
             {
             	SetError(ERROR_MapBorder_MaxY,3);
             	return 0;
             }
//             MapLimit_MaxY =(unsigned int)(Param.AgvParam[PARAM_MapBorderYmax]+0.5);
//             if (Param.SetMapData!=0)
//             {
//             Param.SetMapData =0;
//             memset(mapdata,0,250000);
//             }
             break;
        case PARAM_LoadCondition_MaxSpd1:
             LoadCondition.MaxSpd[1] = Param.AgvParam[PARAM_LoadCondition_MaxSpd1];
			 if(LoadCondition.MaxSpd[1]>LoadCondition.MaxSpd[0])
		 	 {
		 		LoadCondition.MaxSpd[1]=LoadCondition.MaxSpd[0];
		 	 }
             break;
             
        case PARAM_LoadCondition_MaxAcc1:
             LoadCondition.MaxAcc[1] = Param.AgvParam[PARAM_LoadCondition_MaxAcc1];
			 if(LoadCondition.MaxAcc[1]>LoadCondition.MaxAcc[0])
		 	 {
		 		LoadCondition.MaxAcc[1]=LoadCondition.MaxAcc[0];
		 	 }
             break;
             
        case PARAM_LoadCondition_SoftStopAcc1:
             LoadCondition.SoftStopAcc[1] = Param.AgvParam[PARAM_LoadCondition_SoftStopAcc1];
			 if(LoadCondition.SoftStopAcc[1]>LoadCondition.SoftStopAcc[0])
		 	 {
		 		LoadCondition.SoftStopAcc[1]=LoadCondition.SoftStopAcc[0];
		 	 }
             break;
             
        case PARAM_LoadCondition_MaxChangeDirectionAngularSpd1:
             LoadCondition.MaxChangeDirectionAngularSpd[1] = Param.AgvParam[PARAM_LoadCondition_MaxChangeDirectionAngularSpd1]/180*PI;
			 if(LoadCondition.MaxChangeDirectionAngularSpd[1]>LoadCondition.MaxChangeDirectionAngularSpd[0])
		 	 {
		 		LoadCondition.MaxChangeDirectionAngularSpd[1]=LoadCondition.MaxChangeDirectionAngularSpd[0];
		 	 }
             break;
        
        case PARAM_LoadCondition_ChangeDirectionAngularAcc1:
             LoadCondition.ChangeDirectionAngularAcc[1] = Param.AgvParam[PARAM_LoadCondition_ChangeDirectionAngularAcc1]/180*PI;
			 if(LoadCondition.ChangeDirectionAngularAcc[1]>LoadCondition.ChangeDirectionAngularAcc[0])
		 	 {
		 		LoadCondition.ChangeDirectionAngularAcc[1]=LoadCondition.ChangeDirectionAngularAcc[0];
		 	 }
             break;
             
        case PARAM_LoadCondition_MaxAngularSpd1:
             LoadCondition.MaxAngularSpd[1] = Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1]/180*PI;
			if(LoadCondition.MaxAngularSpd[1]>LoadCondition.MaxAngularSpd[0])
		 	 {
		 		LoadCondition.MaxAngularSpd[1]=LoadCondition.MaxAngularSpd[0];
		 	 }
			 break;
        
        case PARAM_LoadCondition_MaxAngularAcc1:
             LoadCondition.MaxAngularAcc[1] = Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1]/180*PI;
			if(LoadCondition.MaxAngularAcc[1]>LoadCondition.MaxAngularAcc[0])
		 	 {
		 		LoadCondition.MaxAngularAcc[1]=LoadCondition.MaxAngularAcc[0];
		 	 }
			 break;
             
        case PARAM_LoadCondition_MaxSpd2:
             LoadCondition.MaxSpd[2] = Param.AgvParam[PARAM_LoadCondition_MaxSpd2];
			 if(LoadCondition.MaxSpd[2]>LoadCondition.MaxSpd[0])
		 	 {
		 		LoadCondition.MaxSpd[2]=LoadCondition.MaxSpd[0];
		 	 }
             break;
             
        case PARAM_LoadCondition_MaxAcc2:
             LoadCondition.MaxAcc[2] = Param.AgvParam[PARAM_LoadCondition_MaxAcc2];
			 if(LoadCondition.MaxAcc[2]>LoadCondition.MaxAcc[0])
		 	 {
		 		LoadCondition.MaxAcc[2]=LoadCondition.MaxAcc[0];
		 	 }
             break;
             
        case PARAM_LoadCondition_SoftStopAcc2:
             LoadCondition.SoftStopAcc[2] = Param.AgvParam[PARAM_LoadCondition_SoftStopAcc2];
			 if(LoadCondition.SoftStopAcc[2]>LoadCondition.SoftStopAcc[0])
		 	 {
		 		LoadCondition.SoftStopAcc[2]=LoadCondition.SoftStopAcc[0];
		 	 }
             break;
             
        case PARAM_LoadCondition_MaxChangeDirectionAngularSpd2:
             LoadCondition.MaxChangeDirectionAngularSpd[2] = Param.AgvParam[PARAM_LoadCondition_MaxChangeDirectionAngularSpd2]/180*PI;
             break;
             
        case PARAM_LoadCondition_ChangeDirectionAngularAcc2:
             LoadCondition.ChangeDirectionAngularAcc[2] = Param.AgvParam[PARAM_LoadCondition_ChangeDirectionAngularAcc2]/180*PI;
             break;
             
        case PARAM_LoadCondition_MaxAngularSpd2:
             LoadCondition.MaxAngularSpd[2] = Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd2]/180*PI;
             break;
        
        case PARAM_LoadCondition_MaxAngularAcc2:
             LoadCondition.MaxAngularAcc[2] = Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc2]/180*PI;
             break;
             
        case PARAM_LoadCondition_MaxSpd3:
             LoadCondition.MaxSpd[3] = Param.AgvParam[PARAM_LoadCondition_MaxSpd3];
             break;
             
        case PARAM_LoadCondition_MaxAcc3:
             LoadCondition.MaxAcc[3] = Param.AgvParam[PARAM_LoadCondition_MaxAcc3];
             break;
             
        case PARAM_LoadCondition_SoftStopAcc3:
             LoadCondition.SoftStopAcc[3] = Param.AgvParam[PARAM_LoadCondition_SoftStopAcc3];
             break;
             
        case PARAM_LoadCondition_MaxChangeDirectionAngularSpd3:
             LoadCondition.MaxChangeDirectionAngularSpd[3] = Param.AgvParam[PARAM_LoadCondition_MaxChangeDirectionAngularSpd3]/180*PI;
             break;
             
        case PARAM_LoadCondition_ChangeDirectionAngularAcc3:
             LoadCondition.ChangeDirectionAngularAcc[3] = Param.AgvParam[PARAM_LoadCondition_ChangeDirectionAngularAcc3]/180*PI;
             break;
        
        case PARAM_LoadCondition_MaxAngularSpd3:
             LoadCondition.MaxAngularSpd[3] = Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd3]/180*PI;
             break;
        
        case PARAM_LoadCondition_MaxAngularAcc3:
             LoadCondition.MaxAngularAcc[3] = Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc3]/180*PI;
             break;
                      
        case PARAM_LoadCondition_MaxSpd4:
             LoadCondition.MaxSpd[4] = Param.AgvParam[PARAM_LoadCondition_MaxSpd4];
             break;
             
        case PARAM_LoadCondition_MaxAcc4:
             LoadCondition.MaxAcc[4] = Param.AgvParam[PARAM_LoadCondition_MaxAcc4];
             break;
             
        case PARAM_LoadCondition_SoftStopAcc4:
             LoadCondition.SoftStopAcc[4] = Param.AgvParam[PARAM_LoadCondition_SoftStopAcc4];
             break;
             
        case PARAM_LoadCondition_MaxChangeDirectionAngularSpd4:
             LoadCondition.MaxChangeDirectionAngularSpd[4] = Param.AgvParam[PARAM_LoadCondition_MaxChangeDirectionAngularSpd4]/180*PI;
             break;
        
        case PARAM_LoadCondition_ChangeDirectionAngularAcc4:
             LoadCondition.ChangeDirectionAngularAcc[4] = Param.AgvParam[PARAM_LoadCondition_ChangeDirectionAngularAcc4]/180*PI;
             break;
        
        case PARAM_LoadCondition_MaxAngularSpd4:
             LoadCondition.MaxAngularSpd[4] = Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd4]/180*PI;
             break;
        
        case PARAM_LoadCondition_MaxAngularAcc4:
             LoadCondition.MaxAngularAcc[4] = Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc4]/180*PI;
             break;
             
        case PARAM_UpCameraStandardRz_BeforeLift:      
             Param.AgvParam[PARAM_UpCameraStandardRz_BeforeLift] = Param.AgvParam[PARAM_UpCameraStandardRz_BeforeLift]/180.0*PI;
             break;
        
        case PARAM_UseCorrectHeading_DMOffsetHeadingTol: 
             Param.AgvParam[PARAM_UseCorrectHeading_DMOffsetHeadingTol] = Param.AgvParam[PARAM_UseCorrectHeading_DMOffsetHeadingTol]/180.0*PI;
             break;

        case PARAM_NotUseCorrectHeading_DMOffsetHeadingTol:                       
	         Param.AgvParam[PARAM_NotUseCorrectHeading_DMOffsetHeadingTol] = Param.AgvParam[PARAM_NotUseCorrectHeading_DMOffsetHeadingTol]/180.0*PI;
             break;
        
        case PARAM_HighMove_OffsetAjustAngle_Limit://HR,add,20151226
             Param.AgvParam[PARAM_HighMove_OffsetAjustAngle_Limit] = Param.AgvParam[PARAM_HighMove_OffsetAjustAngle_Limit]/180.0*PI;
             break;
        
        case PARAM_SlowMove_OffsetAjustAngle_Limit:
             Param.AgvParam[PARAM_SlowMove_OffsetAjustAngle_Limit] = Param.AgvParam[PARAM_SlowMove_OffsetAjustAngle_Limit]/180.0*PI;
             break;
        case PARAM_CODE_GAP:
             {
			ID_CAMERA = 0; //wf add 20170825
		 }
		 break;
	case PARAM_LiftMaxAngleSpeed:
		 Param.AgvParam[PARAM_LiftMaxAngleSpeed] = Param.AgvParam[PARAM_LiftMaxAngleSpeed]/180.0*PI;
		 break;
	case PARAM_LiftAngelSpeedAcc:
		 Param.AgvParam[PARAM_LiftAngelSpeedAcc] = Param.AgvParam[PARAM_LiftAngelSpeedAcc]/180.0*PI;
		 break;
	case PARAM_CODE_GAP_X://Yousong,add,20170503
			if(Param.AgvParam[param_index]>2)
				return 0;
	//wf add 20170825
		//if((AGV.MotionStatus != AGV_MOTION_STOP) && (AGV.MotionStatus != AGV_MOTION_UNCERTAIN))
		if(IsAgvMotionState()){
			SetError(ERROR_NonStop_ParamChg,1);
		} else {
			 if((0 == dymCodegap->areaInit.areaNumber)||
				(1 == dymCodegap->areaInit.areaNumber))
			 {
				ChangeCodeGapX = ROUND_POSITION(AGV.X, CODE_GAP_X,CameraReadCodeIDActived);
				ChangeCodeGapX = ChangeCodeGapX / CODE_GAP_X;
				ChangeCodeGapXError = AGV.X - ChangeCodeGapX * CODE_GAP_X;
				CODE_GAP_X = Param.AgvParam[PARAM_CODE_GAP_X];
				AgvParams.deviceParams.qrcodeParams.CODE_GAP_X = CODE_GAP_X;
				AGV.X = ChangeCodeGapXError + ChangeCodeGapX * CODE_GAP_X;
				TOL_DIS_MOVE_X = 2*CODE_GAP_X+0.1;

	            AGV.LastCorrectX = AGV.X;
			 }
		 }
		 break;
	case PARAM_CODE_GAP_Y://Yousong,add,20170503
		if(Param.AgvParam[param_index]>2)
			return 0;
	//wf add 20170825
		if(IsAgvMotionState()) {
			SetError(ERROR_NonStop_ParamChg,1);
		} else {
			if((0 == dymCodegap->areaInit.areaNumber)||
				(1 == dymCodegap->areaInit.areaNumber))
			 {
				ChangeCodeGapY = ROUND_POSITION(AGV.Y, CODE_GAP_Y,CameraReadCodeIDActived);
				ChangeCodeGapY = ChangeCodeGapY / CODE_GAP_Y;
				ChangeCodeGapYError = AGV.Y - ChangeCodeGapY * CODE_GAP_Y;
				CODE_GAP_Y = Param.AgvParam[PARAM_CODE_GAP_Y];
				AgvParams.deviceParams.qrcodeParams.CODE_GAP_Y = CODE_GAP_Y;
				AGV.Y = ChangeCodeGapYError + ChangeCodeGapY * CODE_GAP_Y;
				TOL_DIS_MOVE_Y = 2*CODE_GAP_Y+0.1;
				AGV.LastCorrectY = AGV.Y;
			}
		}
		break;
	case PARAM_KINCO_SYNCSTOP_SWITCH://wf,add,20170911
		if(1 == Param.AgvParam[PARAM_KINCO_SYNCSTOP_SWITCH]) {
			//EnableMotorSyncStop();
			EnableSynStop();
		}
		else
		{
			DisableSynStop();
		}
		break;
	case PARAM_CHARGE_CODE_GAP://wf,add,20170925
		CHARGE_CODE_GAP = Param.AgvParam[PARAM_CHARGE_CODE_GAP];//wf add 20170925
		break;
	case PARAM_LiftPitch:
		//Holding_Up = (int)(4*60/Param.AgvParam[PARAM_LiftPitch]);//4mm/s
		//Holding_Down = -Holding_Up;
		break;
	case PARAM_DM642_NEW_PROTOCOL:
		break;
	case PARAM_TempratureWarn:
		AValue = Param.AgvParam[PARAM_TempratureWarn]*0.0001;
		BValue = Param.AgvParam[PARAM_TempratureWarn]*0.01 - AValue*100;
		CValue = (int32)(Param.AgvParam[PARAM_TempratureWarn])%100;
		tempCheck[TEMP_Battery].warntemp = AValue;
		tempCheck[TEMP_Gyro].warntemp = BValue;
		tempCheck[TEMP_MotorLeft].warntemp = CValue;
		tempCheck[TEMP_MotorRight].warntemp = CValue;
		tempCheck[TEMP_MotorTrayLift].warntemp = CValue;
		tempCheck[TEMP_MotorTrayRotate].warntemp = CValue;
		LogPrintf("TempratrueWarn:A:%d,B:%d,C:%d\n",AValue,BValue,CValue);
		break;
	case PARAM_TempratureErr:
		AValue = Param.AgvParam[PARAM_TempratureErr]*0.0001;
		BValue = Param.AgvParam[PARAM_TempratureErr]*0.01 - AValue*100;
		CValue = (int32)(Param.AgvParam[PARAM_TempratureErr])%100;
		tempCheck[TEMP_Battery].errtemp= AValue;
		tempCheck[TEMP_Gyro].errtemp = BValue;
		tempCheck[TEMP_MotorLeft].errtemp = CValue;
		tempCheck[TEMP_MotorRight].errtemp = CValue;
		tempCheck[TEMP_MotorTrayLift].errtemp = CValue;
		tempCheck[TEMP_MotorTrayRotate].errtemp = CValue;
//		SetBatteryOverTempParams(AValue,10);
		LogPrintf("TempratureErr:A:%d,B:%d,C:%d\n",AValue,BValue,CValue);
		break;
	case PARAM_LowTempWarn:
		AValue = Param.AgvParam[PARAM_LowTempWarn]*0.0001;
		BValue = Param.AgvParam[PARAM_LowTempWarn]*0.01 - AValue*100;
		CValue = (int32)(Param.AgvParam[PARAM_LowTempWarn])%100;
		LogPrintf("LowTempWarn:A:%d,B:%d,C:%d\n",AValue,BValue,CValue);
		tempCheck[TEMP_ChargeInterfacePin].warntemp = AValue;
		tempCheck[TEMP_FrontPanel].warntemp = BValue;
		tempCheck[TEMP_BackPanel].warntemp = BValue;
		tempCheck[TEMP_Belt].warntemp = BValue;
		tempCheck[TEMP_ChargeInterface].warntemp = BValue;
		tempCheck[TEMP_Laser].warntemp = CValue;
		break;
	case PARAM_HighTempERR:
		AValue = Param.AgvParam[PARAM_HighTempERR]*0.0001;
		BValue = Param.AgvParam[PARAM_HighTempERR]*0.01 - AValue*100;
		CValue = (int32)(Param.AgvParam[PARAM_HighTempERR])%100;
		LogPrintf("HighTempERR:A:%d,B:%d,C:%d\n",AValue,BValue,CValue);
		tempCheck[TEMP_ChargeInterfacePin].errtemp = AValue;
		tempCheck[TEMP_FrontPanel].errtemp = BValue;
		tempCheck[TEMP_BackPanel].errtemp = BValue;
		tempCheck[TEMP_Belt].errtemp = BValue;
		tempCheck[TEMP_ChargeInterface].errtemp = BValue;
		tempCheck[TEMP_Laser].errtemp = CValue;
		break;
	case PARAM_HARDWARE_VERSION_ID:
		{
			int IDA = (int)Param.AgvParam[PARAM_HARDWARE_VERSION_ID];
			int IDB = (int)((Param.AgvParam[PARAM_HARDWARE_VERSION_ID]-IDA)*100+0.5);
			if((IDA>30000)||(IDA<10000)||(IDB>99)||(IDB<1))
				return 0;
			strcpy(App.appParamsManager.VersionID, "V00000F00");
			itoa(IDA, &App.appParamsManager.VersionID[1], 10);
			if(IDB<=9)
			{
			    itoa(IDB, &App.appParamsManager.VersionID[8], 10);
			}
			else
			{
			    itoa(IDB, &App.appParamsManager.VersionID[7], 10);
			}

			App.appParamsManager.VersionID[0] = 'V';
			App.appParamsManager.VersionID[6] = 'F';
		}
		break;
	case PARAM_BatteryCur:
		AValue = Param.AgvParam[PARAM_BatteryCur]*0.0001;
		BValue = Param.AgvParam[PARAM_BatteryCur]*0.01 - AValue*100;
		CValue = (int32)(Param.AgvParam[PARAM_BatteryCur])%100;
		CurrentProtectParams.StaticThreshold= AValue;
		CurrentProtectParams.MotionThreshold = BValue;
		CurrentProtectParams.ChargeThreshold = -1.0*CValue;
//		SetBatteryOverCurrentParams(BValue,10);
		LogPrintf("OverCurrent:A:%d,B:%d,C:%d\n",AValue,BValue,CValue);
		break;
	case PARAM_ChargeInterfaceDis:
		AValue = Param.AgvParam[PARAM_ChargeInterfaceDis]*0.01;
		BValue = (int32)(Param.AgvParam[PARAM_ChargeInterfaceDis])%100;
		AgvParams.deviceParams.chargeInterfaceParams.MinDis= AValue;
		AgvParams.deviceParams.chargeInterfaceParams.MaxDis= BValue;
		LogPrintf("ChargeInterfaceDis:A:%d,B:%d\n",AValue,BValue);
		if((0 == AValue)&&(0 == BValue))
		{
			AgvParams.deviceParams.chargeInterfaceParams.MinDis= 0;
			AgvParams.deviceParams.chargeInterfaceParams.MaxDis= 99;
		}
		LogPrintf("ChargeInterfaceDis:A:%d,B:%d\n",AValue,BValue);
		break;
	case PARAM_Belt_FullLoad_Threshold:
		AValue = Param.AgvParam[PARAM_Belt_FullLoad_Threshold]*0.0001;
		BValue = Param.AgvParam[PARAM_Belt_FullLoad_Threshold]*0.01 - AValue*100;
		CValue = (int32)(Param.AgvParam[PARAM_Belt_FullLoad_Threshold])%100;
		LogPrintf("Threshold:A:%d,B:%d,C:%d\n",AValue,BValue,CValue);
		AgvParams.serviceParams.beltManagerParams.FullCnt = AValue;
		AgvParams.serviceParams.beltManagerParams.FullLoadThresholdLow= BValue;
		AgvParams.serviceParams.beltManagerParams.FullLoadThreshold = CValue;
		break;
	case PARAM_FullScope_Move:
		AValue = Param.AgvParam[PARAM_FullScope_Move]*0.01;
		BValue = (int32)(Param.AgvParam[PARAM_FullScope_Move])%100;
		LogPrintf("FullScope_Move:A:%d,B:%d\n",AValue,BValue);
		AgvParams.serviceParams.beltManagerParams.beltUnloadParams.fullScopeMove.DisToCentre = AValue*0.01;
		AgvParams.serviceParams.beltManagerParams.beltUnloadParams.fullScopeMove.CheckScope = BValue*0.01;
		break;
	case PARAM_FullScope_Turn:
		AValue = Param.AgvParam[PARAM_FullScope_Turn]*0.01;
		BValue = (int32)(Param.AgvParam[PARAM_FullScope_Turn])%100;
		LogPrintf("FullScope_Turn:A:%d,B:%d\n",AValue,BValue);
		AgvParams.serviceParams.beltManagerParams.beltUnloadParams.fullScopeTurn.CheckAngleSmall = AValue;
		AgvParams.serviceParams.beltManagerParams.beltUnloadParams.fullScopeTurn.CheckAngleBig = BValue;
		break;
	default:
		 break;
	}

	return 1;
}

int ParamsProtect(void)
{
	int re = 0;
	LogPrintf("paramProtect-m:%d,v:%d\n",AgvParams.deviceParams.mdTrayLiftParams.mdParams.Type,App.appParamsManager.VersionID[2]);
	switch(AgvParams.deviceParams.mdTrayLiftParams.mdParams.Type)
	{
	case HLS_MultiFrame://和利时多包
		break;
	case HLS_SingleFrame://和利时单包
		if('1' != App.appParamsManager.VersionID[2])
			return 0;
		re = 1;
		//re = ValidHLSParams(Agv.device.motorTrayLift.mdHls.md.IfBigTorque);
//		device->MotorSupplyer[0] = 'L';
//		device->MotorSupplyer[1] = 'S';
		
		break;
	case MotorDriverSim:
	    re = 1;
		break;
	case MD_KinCo:
//		device->MotorSupplyer[0] = 'B';
//		device->MotorSupplyer[1] = 'K';
		if('2' != App.appParamsManager.VersionID[2])
			return 0;

		re = ValidKincoParams(Agv.device.motorTrayLift.mdKinco.md.IfBigTorque);

		break;

	default:
		break;
	}
	return re;
}

int ValidHLSParams(int ifBigTorque)
{
	if(ifBigTorque == 1)//大扭矩
	{
		if(Param.AgvParam[PARAM_UPDOWN_SPEED] <= 600 &&
		  (IsEqual( Param.AgvParam[PARAM_LiftPitch] ,2.16 ))&&
		   Param.AgvParam[PARAM_LiftDistance] <= 60 &&
		   Param.AgvParam[PARAM_LiftDistance] >= 50 &&
		   (((IsEqual(Param.AgvParam[PARAM_LiftReduceRatio] ,85.51724)) &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 180 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 200) ||
			((IsEqual(Param.AgvParam[PARAM_LiftReduceRatio] ,136.8275)) &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 110 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 110) ||
			((IsEqual(Param.AgvParam[PARAM_LiftReduceRatio] ,70.52631 ))&&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 180 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 300)))
		{
			//正常
		}
		else
		{
			//不正常
			//SetError(ERROR_ParamValid_Fail, 3);
			LogPrintf("hls bigtorque protect fail\n");
			SetAppErrorToQue(APP_ERROR_ParamValid_Fail_WhenHLSBigTCheck, ErrorC);
			return 0;
		}
	}
	else if(ifBigTorque == 2)//非大扭矩
	{
		if((IsEqual( Param.AgvParam[PARAM_UPDOWN_SPEED], 3000)) &&
		  (IsEqual( Param.AgvParam[PARAM_LiftPitch] , 0.32)) &&
		   Param.AgvParam[PARAM_LiftDistance] <= 60 &&
		   Param.AgvParam[PARAM_LiftDistance] >= 50 &&
		   (((IsEqual(Param.AgvParam[PARAM_LiftReduceRatio] , 85.51724)) &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 180 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 200) ||
			((IsEqual(Param.AgvParam[PARAM_LiftReduceRatio] , 136.8275)) &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 110 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 110)))
		{
			//正常
		}
		else
		{
			//不正常
			//SetError(ERROR_ParamValid_Fail, 3);
			LogPrintf("hls no bigtorque protect fail\n");
			//SetAppErrorToQue(APP_ERROR_ParamValid_Fail_WhenHLSCheck, ErrorC);
			//return 0;
		}
	}
	return 1;
}

int ValidKincoParams(int ifBigTorque)
{
	if(ifBigTorque == 1)
	{
		if(Param.AgvParam[PARAM_UPDOWN_SPEED] <= 600 &&
		  (IsEqual(  Param.AgvParam[PARAM_LiftPitch] , 2.16)) &&
		   Param.AgvParam[PARAM_LiftDistance] <= 60 &&
		   Param.AgvParam[PARAM_LiftDistance] >= 50 &&
		   (((IsEqual( Param.AgvParam[PARAM_LiftReduceRatio], 85.51724)) &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 180 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 200) ||
			((IsEqual( Param.AgvParam[PARAM_LiftReduceRatio] , 136.8275)) &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 110 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 110) ||
			((IsEqual( Param.AgvParam[PARAM_LiftReduceRatio] , 70.52631)) &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 180 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 300)))
		{
			//正常
		}
		else
		{
			//不正常
			//SetError(ERROR_ParamValid_Fail, 3);
			LogPrintf("kinco bigtorque protect fail\n");
			SetAppErrorToQue(APP_ERROR_ParamValid_Fail_WhenKincoBigTCheck, ErrorC);
			return 0;
		}
	}
	else if(ifBigTorque == 2)//非大扭矩
	{
		if((IsEqual( Param.AgvParam[PARAM_UPDOWN_SPEED] , 3000)) &&
		   (IsEqual( Param.AgvParam[PARAM_LiftPitch] , 0.32)) &&
		   Param.AgvParam[PARAM_LiftDistance] <= 60 &&
		   Param.AgvParam[PARAM_LiftDistance] >= 50 &&
		   (((IsEqual( Param.AgvParam[PARAM_LiftReduceRatio] , 85.51724 ))&&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 180 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 200) ||
			((IsEqual( Param.AgvParam[PARAM_LiftReduceRatio] , 136.8275)) &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularSpd1] <= 110 &&
				   Param.AgvParam[PARAM_LoadCondition_MaxAngularAcc1] <= 110)))
		{
			//正常
		}
		else
		{
			//不正常
			//SetError(ERROR_ParamValid_Fail, 3);
			LogPrintf("kinco no bigtorque protect fail\n");
			SetAppErrorToQue(APP_ERROR_ParamValid_Fail_WhenKincoCheck, ErrorC);
			return 0;
		}
	}
	return 1;
}


void SetParamFlag(int paramIndex)
{
	int tmpIndex = paramIndex/16;
	int tmpBit = paramIndex%16;

	App.appParamsManager.paramsFlag[tmpIndex]|=(0x1<<tmpBit);
}
void ResetParamFlag(int paramIndex)
{
	int tmpIndex = paramIndex/16;
	int tmpBit = paramIndex%16;

	App.appParamsManager.paramsFlag[tmpIndex]&=(~(0x1<<tmpBit));
}
void ResetAllParamFlag()
{
	int i=0;
	for(i=0;i<sizeof(App.appParamsManager.paramsFlag);i++)
	{
		App.appParamsManager.paramsFlag[i] = 0;
	}
	
}

int IsAppParamsFlagZero()
{
	int i=0;
	int tmp = 0;
	for(i=0;i<sizeof(App.appParamsManager.paramsFlag);i++)
	{
		tmp |= App.appParamsManager.paramsFlag[i];
	}
	return (0 == tmp);
}
