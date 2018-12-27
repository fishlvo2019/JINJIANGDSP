/*
 * OldInstruction.c
 *
 *  Created on: 2017年9月13日
 *      Author: hwei
 */
 
#include "AgvApp.h"
#include "FHAGV.h"
#include "Basefunction.h"
#include "userfunction.h"
#include "Cir_queue.h"
#include "AgvAppInstruction.h"
#include "./AppWifiCan/AgvWifiManual.h"


void instruction_enter_queue(void);
void SetError(unsigned char errorcode, unsigned char errorpriority1);
void SendDataDM642();
int IsTaskActionOK()
{
	if(FHAGV_Initialized != Agv.base.state)
	{
		return 0;
	}

	while(1)
	{
		ThreadSleep(20);
		if(GetTaskCnt(&Agv.service.mainControlTask) == 0)
			break;
		
	}
	if( (FHMAINTASK_ERROR != Agv.service.mainControlTask.base.state)&&
		(FHMAINTASK_CANCLE != Agv.service.mainControlTask.base.state))
		return 1;
	else
		return 0;

}
int WaitDisComplete_fun()
{
	if(FHAGV_Initialized != Agv.base.state)
	{
		return -1;
	}
	//等待上一条指令执行结束
	while(1)
	{
		
		FH_Float targetDis = PGGetTargetDistance(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg);
		ThreadSleep(20);
		if((targetDis < 0.1)
		||(IsAgvMotionState()== 0))
			break;
	}
	if(	(FHMAINTASK_ERROR != Agv.service.mainControlTask.base.state)&&
		(FHMAINTASK_CANCLE != Agv.service.mainControlTask.base.state))
		return 0;
	else
		return -1;
}

int WaitUnloadComplete_fun()
{
	if(FHAGV_Initialized != Agv.base.state)
	{
		return -1;
	}
	//等待上一条指令执行结束
	while(1)
	{
		ThreadSleep(20);
		if(((Agv.service.beltManager.beltUnloadInfo.startUnload == 0)
		&&(Agv.service.beltManager.BeltMovingUnLoadStateMachine == UnLoadSuccess))
		||(IsAgvMotionState()== 0))
			break;
	}
	if(	(FHMAINTASK_ERROR != Agv.service.mainControlTask.base.state)&&
		(FHMAINTASK_CANCLE != Agv.service.mainControlTask.base.state))
		return 0;
	else
		return -1;
	
}

int WaitLastCmdComplete_fun()
{
	if(FHAGV_Initialized != Agv.base.state)
	{
		return -1;
	}
	//等待上一条指令执行结束
		while(1)
		{
			ThreadSleep(20);
			if((IsAgvMotionState()== 0)&&
				(GetTaskCnt(&Agv.service.mainControlTask) == 0))
				break;
			
		}
		
		if(	(FHMAINTASK_ERROR != Agv.service.mainControlTask.base.state)&&
			(FHMAINTASK_CANCLE != Agv.service.mainControlTask.base.state))
			return 0;
		else
			return -1;
}
void NormalParamsEndianConvert(struct INSTRUCTION_Normal_params_t* normalParam)
{
	EndianConvert(&normalParam->targetX,sizeof(int32));
	EndianConvert(&normalParam->targetY,sizeof(int32));
	EndianConvert(&normalParam->targetHeading,sizeof(int16));
	return;
}

//#define DO_INSTRUCTION_CAMERA 1 // a calibration position instruction for PC camera
FH_ERROR_CODE DO_INSTRUCTION_CAMERA(struct INSTRUCTION_Normal_params_t* param)
{
	//CompatiableFun(Param);
	NormalParamsEndianConvert(param);
	CorrectPosition.CorrectX = SwitchQRCodeToAgv(param->targetX,DirX);
	CorrectPosition.CorrectY = SwitchQRCodeToAgv(param->targetY,DirY);
	CorrectPosition.CorrectHeading = param->targetHeading*0.01 * PI /180.0;
	CorrectPosition.DataArrive = 1;

	//WaitLastCmdComplete();

	return 0;
}

//#define DO_INSTRUCTION_RESET 5  // reset agv to a given position
FH_ERROR_CODE DO_INSTRUCTION_RESET(struct INSTRUCTION_Normal_params_t* Param)
{
	DisableMotorDevice();
	DisableBeltMotorDevice();
	ResetAgv();
	WaitLastCmdComplete();

	return 0;
}

 //#define DO_INSTRUCTION_MOVE_ARC_IN 60 // move to turning area
 //#define DO_INSTRUCTION_MOVE_ARC_OUT 70 // move out turning area
//#define DO_INSTRUCTION_RUN 8  //  run time mode
FH_ERROR_CODE DO_INSTRUCTION_RUN(int* Param)
{
	 AGV.OperateMode = AGV_OPMODE_RUN;
	 //CompatiableFun(Param);
	 //WaitLastCmdComplete();

	return 0;
}

//#define DO_INSTRUCTION_COMMISSION 9 // commission mode
FH_ERROR_CODE DO_INSTRUCTION_COMMISSION(int* Param)
{
	//ResetTask();
     AGV.OperateMode = AGV_OPMODE_COMMISSION;
	//CompatiableFun(Param);
	//WaitLastCmdComplete();

	return 0;
}

 //#define DO_INSTRUCTION_SETPODHEIGHT 10 // set pod height

//#define DO_INSTRUCTION_UPDATALOG 11    // upload log data
FH_ERROR_CODE DO_INSTRUCTION_UPDATALOG(int* Param)
{
	return 0;
}




//#define DO_INSTRUCTION_ENABLE 13 // start log
FH_ERROR_CODE DO_INSTRUCTION_ENABLE(int* Param)
 {
 
 	//struct INSTRUCTION_Normal_params_t* normalParam = (struct INSTRUCTION_Normal_params_t*)Param;
 	WaitLastCmdComplete_fun();
	EnableMotorDevice();
	if(Agv.service.beltManager.BeltType != NONE_BELT)
	{
		EnableBeltMotorDevice();
	}
	WaitLastCmdComplete_fun();
	return 0;
 }

//#define DO_INSTRUCTION_DISABLE 14 // stop log
FH_ERROR_CODE DO_INSTRUCTION_DISABLE(int* Param)
{

	WaitLastCmdComplete_fun();

	DisableMotorDevice();
	DisableBeltMotorDevice();
	WaitLastCmdComplete_fun();
	return 0;
}


extern double podHeight_Default;


//#define DO_INSTRUCTION_RESET_ERROR 17 // error reset
FH_ERROR_CODE DO_INSTRUCTION_RESET_ERROR(int* Param)
{
	if(IsAgvMotionState())
		return 0;
	TaskReset(&Agv.service.mainControlTask);
	WaitLastCmdComplete_fun();
	Agv.mainControlThread.tickOccupy = 0;
	SafeManagerReset();
	if(FHAGV_Initialized > Agv.base.state)
	{
		return 0;
	}

	if(!(IsMotorDriverState(&Agv.device.motorLeft.md,MD_Ready)&&
		IsMotorDriverState(&Agv.device.motorRight.md,MD_Ready)&&
		IsMotorDriverState(&Agv.device.motorTrayLift.md,MD_Ready)&&
		IsMotorDriverState(&Agv.device.motorTrayRotate.md, MD_Ready)))
	{
		LogPrintf("ResetMotor\n");
		ResetMotor();
		if(Agv.service.beltManager.BeltType != NONE_BELT)
			EnableBeltMotorDevice();
	}
	
	if(Agv.service.beltManager.BeltType != NONE_BELT)
	{
		if(!(IsMotorDriverState(&Agv.device.motorLeftBelt.md,MD_Ready)&&
			 IsMotorDriverState(&Agv.device.motorRightBelt.md, MD_Ready)))
		{
			LogPrintf("ResetBeltMotor\n");
			Agv.device.motorLeftBelt.md.ResetMotor(&Agv.device.motorLeftBelt.md);
			ThreadSleep(10);
			if(Agv.service.beltManager.BeltType == BIG_BELT)
			{
				Agv.device.motorRightBelt.md.ResetMotor(&Agv.device.motorRightBelt.md);
				ThreadSleep(10);
			}
			EnableBeltMotorDevice();
		}
	}
	if((0 == IsAllMotorEnable())&&(AGV_MOTION_STOP == GetAppActionState()))
	{
		EnableMotorDevice();
	}
 	ResetError_tsk();
	Agv.service.beltManager.LoadSuccessFlag = 0;
	Agv.service.beltManager.BeltMovingUnLoadStateMachine = UnLoadSuccess;
	 /*BeginDeploy;
	 //CompatiableFun(Param);
     DeployResetError();
	 EndDeploy;*/
	WaitLastCmdComplete();
	 return 0;
}
extern unsigned int recoverChargeProcessFlag;
//#define DO_INSTRUCTION_SOFTSTOP 18   // to stop with deceleration
FH_ERROR_CODE DO_INSTRUCTION_SOFTSTOP(int* Param)
{
	SoftStopAll();
	TaskReset(&Agv.service.mainControlTask);
	SetChargeRelay(OFF);
	SetError(ERROR_SoftStop,3);
	CleanerStop();
	return 0;
}

//#define DO_INSTRUCTION_GYRO_ZERO 19  // calculate the gyro zero bias
FH_ERROR_CODE DO_INSTRUCTION_GYRO_ZERO(int* normalParam)
{
	if(Agv.service.beltManager.BeltType == NONE_BELT)
	{
		/*---------------------托盘碰下限位---------------------------*/
		SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,10,10);
		ThreadSleep(100);
		if( Agv.device.motorTrayLift.md.SendMotorEnable(&Agv.device.motorTrayLift.md))
		{
		   SetError(ERROR_MotorDisableAbnormal_5,3);
		   return -1;
		}
		ThreadSleep(100);
		FH_Float target = 0;
		Uint16 waitCnt = 2000;
		struct TrayTskParams_t trayTskParams;
		//恢复上视曝光
		AdjustExposure_b_adjust_exposure(EXPOSURE_CAMERA_UP,FALSE_FH);
		SetAppActionState(AGV_MOTION_SETDOWN);
		target = -(Param.AgvParam[PARAM_LiftDistance]*0.001+0.01);
		GetTrayLiftTskParam(&trayTskParams,target);
		TrayLift_tsk(&trayTskParams);
		WaitMoveDone_tsk(&waitCnt);
		WaitLastCmdComplete_fun();
		Agv.device.motorTrayLift.md.SendMotorDisable(&Agv.device.motorTrayLift.md);
		SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,50,10);
		ThreadSleep(100);
		TraySetLiftZeroOffset();
	}

	/*-----------------陀螺仪回零----------------------*/
	 GyroZeroBiasCal(&Agv.device.gyro);
	 SetMapPointNUM(&App.map);
	 SetChargePointNUM(&App.map);
	 AGV.Heading = WrapTo2PiDe(AGV.Heading);
	 return 0;
}

//#define DO_INSTRUCTION_ESTOP 20 // to stop ASAP
FH_ERROR_CODE DO_INSTRUCTION_ESTOP(int* Param)
{
	TaskReset(&Agv.service.mainControlTask);
	
	if(IsTrayLiftUpState())//HR,modified,20160601,
	{
		SoftStopAll();
	}
	else
	{
		EmcStopAll();   	        
	}
	SetError(ERROR_EStop,4);
	CleanerStop();
	//WaitLastCmdComplete();
	 return 0;
}


//#define DO_INSTRUCTION_REQUEST 22
FH_ERROR_CODE DO_INSTRUCTION_REQUEST(int* Param)
{
	 return 0;
}

//#define DO_INSTRUCTION_SETPARAM 24
FH_ERROR_CODE DO_INSTRUCTION_SETPARAM(struct INSTRUCTION_SETPARAM_PARAMS_t* setParams)
{
	if(IsAgvMotionState())
		return -1;
	EndianConvert(&setParams->ParamIndexWrite,sizeof(setParams->ParamIndexWrite));
	EndianConvert(&setParams->ParamWrite,sizeof(setParams->ParamWrite));
	int i=0,tmpV;
	for(i=0;i<8;i++)
	{
		tmpV = FHCharRead((int*)&setParams->ParamWrite,i);
		FHCharWrite(App.appParamsManager.appRawParams->ParamsBuf,i,tmpV);
	}
	//参数设置
	Param.CameraParamIndex = 0;
	 if	(setParams->ParamIndexWrite > PARAMMAXNUM-1)
 	{
 		SetError(ERROR_ParamGiven,3);
		return -1;
 	}
	     
     //从中控下发的参数中读取
	 Param.IndexRead =setParams->ParamIndexWrite;
	 Param.AgvParam[setParams->ParamIndexWrite] = setParams->ParamWrite;
	 
    if(setParams->ParamIndexWrite < PARAMMAXNUM)
    {
//                    save_one_param(Instruction.ParamIndexWrite);
		SetAppParam4Single(&App.appParamsManager,setParams->ParamIndexWrite);
    }
	if((PARAM_CODE_GAP_X ==setParams->ParamIndexWrite)
	||(PARAM_CODE_GAP_Y ==setParams->ParamIndexWrite))
	{
		
		AgvParams.deviceParams.qrcodeParams.codeFlag = 1;
	}
     if(ParamQuanti(setParams->ParamIndexWrite)!=0)
	 {
		 /*if(Instruction.ParamIndexWrite < 64)
		     Param.SetFlag = Param.SetFlag | ((Uint64)1 << Instruction.ParamIndexWrite);
		 else if (Instruction.ParamIndexWrite < 128)
		     Param.SetFlag1 = Param.SetFlag1 | ((Uint64)1 << (Instruction.ParamIndexWrite-64));
		 else
		     Param.SetFlag2 = Param.SetFlag2 | ((Uint64)1 << (Instruction.ParamIndexWrite-128));*/
	     ResetParamFlag(setParams->ParamIndexWrite);

	 }
    if(setParams->ParamIndexWrite == PARAMMAXNUM - 1)  //LHZ add 20170518  兼容老的上位机版本
	 {
		 Agv.service.HMI.InitlizedFlag = 1;
		 ClearMapNum();
		 ClearChargePointNum();
		 Param.SetMapData = 0;
		 //参数保护，前提是电机各个指针已经初始化，并且已经获取到电机厂商和硬件型号
		if(ParamsProtect() == 0)//参数值范围不对，校验未通过
		{
			SetAppErrorToQue(APP_ERROR_ParamValid_Fail_WhenParamsProtect, ErrorC);
			AppParamsReset(&App.appParamsManager);
			return -1;
		}
		//参数下发成功
		 if(IsAppParamsFlagZero())
	 	{
	 		App.appParamsManager.checkValid = 1;
			if(UpdateSystemParamsFromApp())
			{
				SetAppErrorToQue(APP_ERROR_ParamValid_Fail_WhenUpdateSystemParams, ErrorC);
			}
			if(IsNeedResetDm642Params())
		 	{
		 		UpdataCameraParams();
		 	}
			CollideCheckSwitch(1);
	 	}
		 if((Param.AgvParam[PARAM_UpLed_Luminance_Resistant]>0)&&
		 	(Param.AgvParam[PARAM_UpLed_Luminance_Resistant]<255)&&
		 	(Param.AgvParam[PARAM_DownLed_Luminance_Resistant]>0)&&
		 	(Param.AgvParam[PARAM_DownLed_Luminance_Resistant]<255))
	 	{
	 		LightLampResAdjust(Param.AgvParam[PARAM_UpLed_Luminance_Resistant],Param.AgvParam[PARAM_DownLed_Luminance_Resistant]);
	 	}
		 ResetError_tsk();
		 //设置动态码间距默认值
		 AgvParams.deviceParams.qrcodeParams.codeFlag = 1;
		 struct DymCodegap_t* dymCodegap = &Agv.service.dymCodegap;
	 	 DymAreaSetDefault(dymCodegap,0);
		 struct AreaInit_t areaInit;
		 areaInit.areaNumber = 1;
		 areaInit.originIdX = 0;
		 areaInit.originIdY = 0;
		 DymAreaInit(dymCodegap,areaInit);
		 struct AreaAdd_t areaAdd;
		 areaAdd.areaID = 0;
		 areaAdd.startIdX = 0;
		 areaAdd.startIdY = 0;
		 areaAdd.endIdX = Param.AgvParam[PARAM_MapBorderXmax];
		 areaAdd.endIdY = Param.AgvParam[PARAM_MapBorderYmax];
		 DymAreaAdd(dymCodegap,areaAdd);
		 struct AreaCodeGap_t areaCodeGap;
		 areaCodeGap.areaID = 0;
		 areaCodeGap.CodeGapX = Param.AgvParam[PARAM_CODE_GAP_X];
		 areaCodeGap.CodeGapY = Param.AgvParam[PARAM_CODE_GAP_Y];
		 AreaCodeGap(dymCodegap,areaCodeGap);
		 ThreadSleep(500);
		 AgvParams.deviceParams.qrcodeParams.codeFlag = 0;
	 }
	 Param.SetComplete = 1;
	 //设置参数下发完成报告事件，系统会上传特殊帧;
	 ReportParamsEvent((int)setParams->ParamIndexWrite,Param.AgvParam[setParams->ParamIndexWrite]);
	 if(Param.CameraParamIndex > 0)
	 {
	     if(setParams->ParamIndexWrite == PARAM_CameraChoose)
	     {
	         if(Param.AgvParamTem[PARAM_CameraChoose] == 1)
	         {
	             //Power.CTRLWORD.bit.UpLED = 1;
				 LightLampSwitch(UPLighLamp,ON);
	         }
	         else
	         {
	             //Power.CTRLWORD.bit.UpLED = 0;
				 LightLampSwitch(UPLighLamp,OFF);
	         }
	     }
	     SendDataDM642();
		 ThreadSleep(30);
	     Param.CameraParamIndex = 0;
	 }
	 if((PARAM_CODE_GAP_X ==setParams->ParamIndexWrite)
	||(PARAM_CODE_GAP_Y ==setParams->ParamIndexWrite))
	{
		ThreadSleep(500);
		AgvParams.deviceParams.qrcodeParams.codeFlag = 0;
	}
	 App.appParamsManager.MD5CheckValid = 0;//下发参数，清MD5校验通过标志
	 App.appParamsManager.HostComputerSupportValid = 0;//下发参数，清上位机新版一键启动版本标志
	 return 0;
}
//#define DO_INSTRUCTION_READPARAM 25
FH_ERROR_CODE DO_INSTRUCTION_READPARAM(int* Param)
{
	 return 0;
}



//#define DO_INSTRUCTION_STEPIN_ONESTEP 28
FH_ERROR_CODE DO_INSTRUCTION_STEPIN_ONESTEP(int* Param)
{
 return 0;
}
//#define DO_INSTRUCTION_STEPOUT_ONESTEP 29
FH_ERROR_CODE DO_INSTRUCTION_STEPOUT_ONESTEP(int* Param)
{
	 return 0;
}
//#define DO_INSTRUCTION_SETERROR 30// start write the parameters to exflash
FH_ERROR_CODE DO_INSTRUCTION_SETERROR(struct INSTRUCTION_Normal_params_t* normalParam)
{
	
	unsigned char ErrorCode = normalParam->PodDirection;
	SetErrorToQue((ErrorTypeEnum)ErrorCode,ErrorC);

// CompatiableFun(Param);
 //WaitLastCmdComplete();
	 return 0;
}



//#define DO_INSTRUCTION_DORMANCY 36
FH_ERROR_CODE DO_INSTRUCTION_DORMANCY(int* Param)
{
	struct PowerManager_t* powerManager = &Agv.service.powerManager;
	if(powerManager->dormancyState == DORMANCY_WAKEUP)
	{
		powerManager->entryDormancy = 1;
		powerManager->neverSleep = 0;
		PowerWaitEntryComplete(powerManager);
	}

	 return 0;
}
//#define DO_INSTRUCTION_WAKEUP 37
FH_ERROR_CODE DO_INSTRUCTION_WAKEUP(int* Param)
{
	struct PowerManager_t* powerManager = &Agv.service.powerManager;
	if(powerManager->dormancyState == DORMANCY_SLEEP)
	{
		powerManager->exitDormancy = 1;
		PowerWaitExitComplete(powerManager);
	}
	 return 0;
}
//#define DO_INSTRUCTION_DEBUGMODE 38
FH_ERROR_CODE DO_INSTRUCTION_DEBUGMODE(int* Param)
{
	AGV.DebugMode = 1;
	LightLampSwitch(UPLighLamp,ON);
	int isOn = 1;
	LaserSwitch(&isOn);
	WaitLastCmdComplete();
	 return 0;
}
//#define DO_INSTRUCTION_MAP_INFORMATION 39

FH_ERROR_CODE DO_INSTRUCTION_MAP_INFORMATION(struct INSTRUCTION_MAP_INFORMATION_PARAMS_t* mapParam)
{
 
 //temxy = ((((Uint16)UART.DataFrameRaw[3])&0x00FF)<<8)|(((Uint16)UART.DataFrameRaw[4])&0x00FF);
 
 MapData.X = mapParam->X;//(Uint16)UART.DataFrameRaw[3];
 MapData.Y = mapParam->Y;//(Uint16)UART.DataFrameRaw[4];
	LogPrintf("MapData.X=%d,MapData.Y=%d\n",MapData.X,MapData.Y);
 MapData.EXRamXY = ((((Uint16)mapParam->X)&0x00FF)<<8)|(((Uint16)mapParam->Y)&0x00FF);	   
 MapData.EXRamXYxp = mapParam->EXRamXYxp;//((((Uint16)UART.DataFrameRaw[5])&0x00FF)<<8)|(((Uint16)UART.DataFrameRaw[6])&0x00FF);		
 EndianConvert(&MapData.EXRamXYxp,sizeof(MapData.EXRamXYxp));

 MapData.EXRamXYxn = mapParam->EXRamXYxn;//((((Uint16)UART.DataFrameRaw[7])&0x00FF)<<8)|(((Uint16)UART.DataFrameRaw[8])&0x00FF);		
 EndianConvert(&MapData.EXRamXYxn,sizeof(MapData.EXRamXYxn));

 MapData.EXRamXYyp = mapParam->EXRamXYyp;//((((Uint16)UART.DataFrameRaw[9])&0x00FF)<<8)|(((Uint16)UART.DataFrameRaw[10])&0x00FF); 	 
 EndianConvert(&MapData.EXRamXYyp,sizeof(MapData.EXRamXYyp));

 MapData.EXRamXYyn = mapParam->EXRamXYyn;//((((Uint16)UART.DataFrameRaw[11])&0x00FF)<<8)|(((Uint16)UART.DataFrameRaw[12])&0x00FF);
 EndianConvert(&MapData.EXRamXYyn,sizeof(MapData.EXRamXYyn));

 MapData.chargePoint = mapParam->chargePoint;//UART.DataFrameRaw[13];//HR,add,20160106
 //如果是下充电地图
 if(MapData.chargePoint)
{
	/* if((MapData.X == 311) && (MapData.Y == 3))
 	{
 		int tttt=0;
 	}*/
	if((0 != MapData.EXRamXYxn)&&((0xFF&MapData.EXRamXYxn)== MapData.X))
	{
		MapData.X = MapData.EXRamXYxn;
	}
	if((0 != MapData.EXRamXYyp)&&((0xFF&MapData.EXRamXYyp)== MapData.Y))
	{
		MapData.Y = MapData.EXRamXYyp;
	}

	if(SetChargePointInfo4Single(&App.map,MapData.X,MapData.Y))
	{
		Param.SetComplete = 0;
		Param.SetMapData =0;
	}
	else
	{
		Param.SetComplete = 1;
		Param.SetMapData =1;
	}
}
else//不是充电点，就是贴码偏差
{
	int i =0;
	for(i=0;i<App.map.MapPointNUM;i++)
	{
		if ((MapData.X*100)==App.map.mapPointType[i].baseX 
		&& (MapData.Y*100)==App.map.mapPointType[i].baseY)
		{//接收到已经存在的地图点，则更新该地图点数据
			App.map.mapPointType[i].offsetXp =MapData.EXRamXYxp;//MapData.EXRamXYxp;
			App.map.mapPointType[i].offsetXn =MapData.EXRamXYxn;
			App.map.mapPointType[i].offsetYp =MapData.EXRamXYyp;
			App.map.mapPointType[i].offsetYn =MapData.EXRamXYyn;
			App.map.mapPointType[i].chargePoint =MapData.chargePoint;

			SetMapInfo4Single(&App.map,i);
			Param.SetComplete = 1;
			Param.SetMapData =1;
			break;
		}
	}
	if (i>= App.map.MapPointNUM)//接收到新的地图点,保存该地图点数据
	{
		if (App.map.MapPointNUM >= MAX_MAP_POINT_NUM)
		{
			SetError(ERROR_MapPointNum_Exceed,3);
			Param.SetComplete = 0;
			Param.SetMapData =0;
		}
		else
		{

			App.map.mapPointType[App.map.MapPointNUM].baseX =MapData.X*100;
			App.map.mapPointType[App.map.MapPointNUM].baseY =MapData.Y*100;
			App.map.mapPointType[App.map.MapPointNUM].offsetXp =MapData.EXRamXYxp;
			App.map.mapPointType[App.map.MapPointNUM].offsetXn =MapData.EXRamXYxn;
			App.map.mapPointType[App.map.MapPointNUM].offsetYp =MapData.EXRamXYyp;
			App.map.mapPointType[App.map.MapPointNUM].offsetYn =MapData.EXRamXYyn;
			App.map.mapPointType[App.map.MapPointNUM].chargePoint =MapData.chargePoint;
			App.map.MapPointNUM++;
			SetMapInfo4Single(&App.map,i);
			Param.SetComplete = 1;
			Param.SetMapData =1;
		}
	}
}
 

	 return 0;
}


//#define DO_INSTRUCTION_MEASUREPOWER 42
FH_ERROR_CODE DO_INSTRUCTION_MEASUREPOWER(int* Param)
{
	SetAppActionState(AGV_MOTION_QUERY_ENERGY);
	SetPowerBoardMeasure();
	ReportPowerEvent(Agv.device.battery.batteryBase.QuantityElectric,Agv.device.battery.batteryBase.BatVoltage,Agv.device.battery.batteryBase.BatCapacity,
					Agv.device.gyro.Temperature,Agv.device.battery.batteryBase.Temperature,
					Agv.device.motorLeft.md.TempValue,Agv.device.motorRight.md.TempValue,
					Agv.device.motorTrayRotate.md.TempValue,Agv.device.motorTrayLift.md.TempValue);
	WaitLastCmdComplete();
	return 0;
}
//#define DO_INSTRUCTION_RUNMODE 43 
FH_ERROR_CODE DO_INSTRUCTION_RUNMODE(int* Param)
{
	AGV.DebugMode = 0;
	LightLampSwitch(UPLighLamp,0);
	int isOn = 0;
	LaserSwitch(&isOn);
	WaitLastCmdComplete();

	//InstructionRaw.Type = INSTRUCTION_RUNMODE;
	//CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_SLAVEPC_VERSION 44
FH_ERROR_CODE DO_INSTRUCTION_SLAVEPC_VERSION(int* Param)
{
	//int value;
	//GetDeviceHardwareInfo(&Agv.device);
	LaserAskVersion(&Agv.device.laser);
	PowerBoardAskVersion();
	DM642AskVersion();
	int i;
	for(i = 0;i<4;i++)
	{
		ReportVersionEvent(i);
	}
	//等特殊帧事件传出
	while(0!=GetSpecialFrameCnt())
	{
		ThreadSleep(10);
	}
	WaitLastCmdComplete();
	 return 0;
}

//#define DO_INSTRUCTION_READMILEAGE 46//46
FH_ERROR_CODE DO_INSTRUCTION_READMILEAGE(int* Param)
{
	ReportMileageEvent();
	//等特殊帧事件传出
	while(0!=GetSpecialFrameCnt())
	{
		ThreadSleep(10);
	}
	//CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_OPEN_UPCAMERA_BEFORELIFTUP 47
FH_ERROR_CODE DO_INSTRUCTION_OPEN_UPCAMERA_BEFORELIFTUP(int* Param)
{
	UpCamera.UpCameraDataType = 2;
	 LightLampSwitch(UPLighLamp,ON);
	//CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_CLOSE_UPCAMERA 48
FH_ERROR_CODE DO_INSTRUCTION_CLOSE_UPCAMERA(int* Param)
{

	QRCodeSetMode(&Agv.device.qrcode,QRCODEMODE_DOWN);			 
	//CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_OPEN_UPCAMERA_AFTERLIFTUP 49
FH_ERROR_CODE DO_INSTRUCTION_OPEN_UPCAMERA_AFTERLIFTUP(int* Param)
{
	 UpCamera.UpCameraDataType = 1; 
	 LightLampSwitch(UPLighLamp,ON);
	//CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_CLEARMILEAGE 50	//
FH_ERROR_CODE DO_INSTRUCTION_CLEARMILEAGE(int* Param)
{
	MileageInfoReset();
	
	//CompatiableFun(Param);
	 return 0;
}

//#define DO_INSTRUCTION_CHOOSE_LOADCONDITION 51
FH_ERROR_CODE DO_INSTRUCTION_CHOOSE_LOADCONDITION(struct INSTRUCTION_Normal_params_t * normalParam)
{
	unsigned char loadConditionID = normalParam->PodDirection;
	if (loadConditionID<3 || loadConditionID>4)
	{
		LoadCondition.ID =0;
		SetErrorToQue((ErrorTypeEnum)ERROR_LoadConditionID,WarningB);
		LogPrintf("ERROR_LoadConditionID\n");
	}
	else
	{
		LoadCondition.ID = loadConditionID;
	}    
	//CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_INITIAL_AFTERLIFT_CHECKSHELF 53
FH_ERROR_CODE DO_INSTRUCTION_INITIAL_AFTERLIFT_CHECKSHELF(int* Param)
{
	//no used
	/*BeginDeploy;
	InstructionRaw.Type = INSTRUCTION_INITIAL_AFTERLIFT_CHECKSHELF;
	//DeployInitialAfterLiftCheckShelf();
	EndDeploy;*/
 //CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_SWITCH_FAN 54 // open/start FAN*/
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_FAN(int* Param)
{
//	InstructionRaw.Type = INSTRUCTION_SWITCH_FAN;
//
	DO_INSTRUCTION_ADJUST_LASER(Param);
	WaitLastCmdComplete();
	 return 0;
}
//#define DO_INSTRUCTION_RETURNBACK 55 //????
FH_ERROR_CODE DO_INSTRUCTION_RETURNBACK(int* Param)
{
	 return 0;
}
 

//#define DO_INSTRUCTION_JOYSTICK_MOVE_VEL_MODE 57
FH_ERROR_CODE DO_INSTRUCTION_JOYSTICK_MOVE_VEL_MODE(int* Param)
{
// CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_GUIDE_UPCAMERA_BEFORELIFTUP 58
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_UPCAMERA_BEFORELIFTUP(int* Param)
{

	 return 0;
}
//define DO_INSTRUCTION_SWITCH_LOG 59
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_LOG(int* Param)
{
	LogSwitch();

 //CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_CHECK_GUIDE_COND 60
FH_ERROR_CODE DO_INSTRUCTION_CHECK_GUIDE_COND(int* Param)
{
	 return 0;
}
 //??????????
//#define DO_INSTRUCTION_SWITCH_LOGPOINT 61
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_LOGPOINT(int* Param)
{

// CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_FLUSH_QUEUE 62
FH_ERROR_CODE DO_INSTRUCTION_FLUSH_QUEUE(int* Param)
{
	 return 0;
}
//#define DO_INSTRUCTION_GET_AGV_STATUS 63
FH_ERROR_CODE DO_INSTRUCTION_GET_AGV_STATUS(int* Param)
{
	 return 0;
}
//#define DO_INSTRUCTION_INITIAL 64
FH_ERROR_CODE DO_INSTRUCTION_INITIAL(int* Param)
{

	 return 0;
}
//#define DO_INSTRUCTION_SWITCH_METHOD 65
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_METHOD(int* Param)
{

 //CompatiableFun(Param);
	 return 0;
}
 
//#define DO_INSTRUCTION_RESTART_IPC 66 //重启工控机,HR,add,20160823
FH_ERROR_CODE DO_INSTRUCTION_RESTART_IPC(int* Param)
{
	if(IsAgvMotionState())
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_NotInStop_RejectPowerRestart,WarningB);
		return -1;
	}
	PCPowerOff();
	ThreadSleep(3000);
	PCPowerOn();
	ThreadSleep(8000);
	
	//DeployRestartIPCInstruction();

 //CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_RESTART_CAMERA 67 //重启摄像头,HR,add,20170308
FH_ERROR_CODE DO_INSTRUCTION_RESTART_CAMERA(int* Param)
{
	ID_CAMERA =0;//HR,add,201610
	if(IsAgvMotionState())
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_NotInStop_RejectPowerRestart,WarningC);
	}
	else
	{
		Dm642PowerOff();
		ThreadSleep(10000);
		Dm642PowerOn();
		ThreadSleep(2000);
		QRCodeClearError(&Agv.device.qrcode);
		ThreadSleep(200);
		UpdataCameraParams();
		ThreadSleep(200);
	}

 //CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_RESTART_DSP 68 //重启DSP,HR,add,20160823
FH_ERROR_CODE DO_INSTRUCTION_RESTART_DSP(int* param)
{
	DO_INSTRUCTION_RESET((struct INSTRUCTION_Normal_params_t *) param);
 //CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_RESTART_DRIVER 69 //重启驱动器,HR,add,20160908
FH_ERROR_CODE DO_INSTRUCTION_RESTART_DRIVER(int* Param)
{
 //CompatiableFun(Param);
	 return 0;
}
 
 //panj-add
 //无码导引顶升
//#define DO_INSTRUCTION_GUIDE_WITHOUTDM 70
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_WITHOUTDM(int* Param)
{
	 return 0;
}
//#define DO_INSTRUCTION_GUIDE_WITHOUTDM_PH2 71
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_WITHOUTDM_PH2(int* Param)
{
	 return 0;
}
 //绕货架码旋转
//#define DO_INSTRUCTION_ROT_WITHUP 72
FH_ERROR_CODE DO_INSTRUCTION_ROT_WITHUP(int* Param)
{
 return 0;
}

//#define DO_INSTRUCTION_CHECK_UPCAM 74
FH_ERROR_CODE DO_INSTRUCTION_CHECK_UPCAM(int* Param)
{
	 return 0;
}
 
 //add sxp 20161029
//#define DO_INSTRUCTION_ZERORETURN 75
FH_ERROR_CODE DO_INSTRUCTION_ZERORETURN(int* Param)
{
	 return 0;
}

 
 //切换SLAM模式
//#define DO_INSTRUCTION_SWITCH_SLAMMODE 76
FH_ERROR_CODE DO_INSTRUCTION_SWITCH_SLAMMODE(int* Param)
{
	 return 0;
}
 //查询模式
//#define DO_INSTRUCTION_QUERY_SLAMMODE 77
FH_ERROR_CODE DO_INSTRUCTION_QUERY_SLAMMODE(int* Param)
{
	 if(slam_mode)
	 {
		 ChangeSlamMode(&App.appLog,1);
		 ReportMD5Event();
	 }
	 else
	 {
		 ReportVersionEvent(1);
	 }

	 return 0;
}

  //参数以及地图偏差校验和
//#define DO_INSTRUCTION_CHECK_PARAM_MAP 79
FH_ERROR_CODE DO_INSTRUCTION_CHECK_PARAM_MAP(struct INSTRUCTION_Normal_params_t* Param)
{
	  int32 tmpV = Param->targetX;
	  EndianConvert(&tmpV,sizeof(int32));

	  if(tmpV == 0)//校验下位机参数  老版 OneKeyStart
	  {
		  App.appParamsManager.HostComputerSupportValid = 0;//当做上位机不支持一键启动
		  ClearMd5();//上传一个错误的MD5，保证上位机一定下发参数
	  }
	  else if(tmpV == 1)//校验下位机参数 新版  OneKeyStart
	  {
		  App.appParamsManager.HostComputerSupportValid = 1;//上位机支持一键启动
		  CalcAppParamsMd5();
	  }

 	if((tmpV == 0)||
		(tmpV == 1))
	{
		slam_mode = AGV_CHECK_MD5;
	}
 
	 return 0;
}

//#define DO_INSTRUCTION_MANUAL 80
/*------------------------cmd,远程,使能功能标志-------------------------------*/
#define   MotorStop  0x10   //关闭左右motor
#define   MotorStart 0x11   //打开左右motor
#define   RemoteOut  0x20   //自动模式
#define   RemoteIn   0x21   //手柄模式
#define   HoldDown   0x30  //updown motor下降
#define   HoldUp     0x31  //updown motor上升
#define   HoldStop   0x32  //updown motor停止
#define   HighPriority   0x40   //重新开始,清楚错误
FH_ERROR_CODE SetWifiParams(struct WifiRemoter_t* remoterParams);
FH_ERROR_CODE DO_INSTRUCTION_MANUAL(struct INSTRUCTION_MANUAL_Params_t* manualParam)
{
	switch(manualParam->FunctionTypeId) //判断传入函数的电机标志位数据
	{
		case HighPriority://清除错误
			//ResetAgv();
			DO_INSTRUCTION_RESET_ERROR(NULL);	//调用清错函数
			if(GetTaskCnt(&Agv.service.mainControlTask) == 0)	//判断此任务是否正在执行
			{
				//ChassisManualMove_tsk();
			}
			break;
		case MotorStart://电机使能
			if(AGV_Remote == GetAgvRunMode())	//判断Agv状态是否处于手动状态
			{
				EnableMotorDevice();	//调用电机使能函数
				if(GetTaskCnt(&Agv.service.mainControlTask) == 0)	//判断此任务是否正在执行
				{
					//ChassisManualMove_tsk();
				}
				HMISetMode(&Agv.service.HMI, HMIMODE_MANUAL_1);//状态输出到LED和蜂鸣器
			}
			break;
		case MotorStop://电机非使能
			if(AGV_Remote == GetAgvRunMode())	//判断Agv是否处于手动状态
			{
				DisableMotorDevice();//关闭电机
				HMISetMode(&Agv.service.HMI, HMIMODE_MANUAL_2);//状态输出到LED和蜂鸣器
			}
			break;
		case HoldUp://顶升
		case HoldDown://降下
			if(AGV_Remote == GetAgvRunMode())	//判断Agv状态是否处于手动状态
			{
				if(IsMotorDriverState(Agv.service.tray.motorTrayLift,MD_Enable)!=1)//判断电机是否处于使能状态
				{
					EnableTrayLiftMotorDevice();	//顶升电机使能
				}
				else
				{
					if(GetTaskCnt(&Agv.service.mainControlTask) == 0)	//判断此任务是否正在执行
					{
						//ChassisManualMove_tsk();
					}
					HMISetMode(&Agv.service.HMI, HMIMODE_MANUAL_1);//状态输出到LED和蜂鸣器
				}
			}
			break;

	}
	/*********************************************************************************************/
	switch(manualParam->Speed)
	{
	case 0x01://前进
	case 0x11://后退
		if(AGV_Remote == GetAgvRunMode())	//判断Agv状态是否处于手动状态
		{
			if(0 == IsAllMotorEnable())//判断电机是否处于使能状态
			{
				EnableMotorDevice();	//调用电机使能函数
				//EnableTrayLiftMotorDevice();	//顶升电机使能
				if(GetTaskCnt(&Agv.service.mainControlTask) == 0)	//判断此任务是否正在执行
				{
					//ChassisManualMove_tsk();
				}
				HMISetMode(&Agv.service.HMI, HMIMODE_MANUAL_1);//状态输出到LED和蜂鸣器
			}
		}
		break;
	}
	/*********************************************************************************************/
	switch(manualParam->MaxAngularSpd)
	{
	case 0x01://顺时针
	case 0x11://逆时针
		if(AGV_Remote == GetAgvRunMode())	//判断Agv状态是否处于手动状态
		{
			if(0 == IsAllMotorEnable())//判断电机是否处于使能状态
			{
				EnableMotorDevice();	//调用电机使能函数
				if(GetTaskCnt(&Agv.service.mainControlTask) == 0)	//判断此任务是否正在执行
				{
					//ChassisManualMove_tsk();
				}
				HMISetMode(&Agv.service.HMI, HMIMODE_MANUAL_1);//状态输出到LED和蜂鸣器
			}
		}
		break;
	}


	FeedManualDog();	//遥控看门狗
	if(AGV_Remote != GetAgvRunMode())	//如果Agv处于非遥控状态
	{
		return 0;
	}
	else
	{
		struct WifiRemoter_t remoterParams;
		remoterParams.FunctionTypeId = manualParam->FunctionTypeId; //0 命令类型
		remoterParams.Speed = manualParam->Speed;	//1 Agv移动方向
		remoterParams.MaxAngularSpd = manualParam->MaxAngularSpd;	//2 Agv旋转方向
		remoterParams.ManualSwitch = manualParam->ManualSwitch; //3 手动/自动模式
		remoterParams.SpeedGrade = manualParam->SpeedGrade; //4 速度等级
		remoterParams.CheckSum = manualParam->CheckSum; //5 校验和
		SetWifiParams(&remoterParams);

	}
	 return 0;
}
 //雷达车辆导引
//#define DO_INSTRUCTION_RADAR_AGV_GUIDE 81
FH_ERROR_CODE DO_INSTRUCTION_RADAR_AGV_GUIDE(int* Param)
{
// CompatiableFun(Param);
	 return 0;
}
 //雷达货架导引
//#define DO_INSTRUCTION_RADAR_SHELF_GUIDE 82
FH_ERROR_CODE DO_INSTRUCTION_RADAR_SHELF_GUIDE(int* Param)
{

 //CompatiableFun(Param);
	 return 0;
}
 
 //LHZ
//#define DO_INSTRUCTION_RST_AGV_MODE 84 //复位下位机数据帧上传模式
FH_ERROR_CODE DO_INSTRUCTION_RST_AGV_MODE(struct INSTRUCTION_Normal_params_t* normalParam)
{
	EndianConvert(&normalParam->targetX,sizeof(normalParam->targetX));
	slam_mode = AGV_NORMAL;//退出MD5上传模式
	ChangeSlamMode(&App.appLog,0);
	if(normalParam->targetX == 0 && App.appParamsManager.HostComputerSupportValid == 1)//验证通过
	{
		App.appParamsManager.MD5CheckValid = 1;
		Param.SetMapData = 0;
		ClearChargePointNum();
		ClearMapNum();
		ResetAllParamFlag();
	}
	else if(normalParam->targetX == 1 && App.appParamsManager.HostComputerSupportValid == 1)//MD5验证不通过
	{
		App.appParamsManager.MD5CheckValid = 0;
		AppParamsInit(&App.appParamsManager);
		SetError(ERROR_ParamValid_Fail,3);
	}

// CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_SET_AGV_REQ  85 //LHZ add 20170510
FH_ERROR_CODE DO_INSTRUCTION_SET_AGV_REQ(int* Param)
{
	
	SetHMIMode(Init_REQ_SINGLE);
	LogPrintf("INSTRUCTION_SET_AGV_REQ\n");

	 return 0;
}
//#define DO_INSTRUCTION_RST_AGV_REQ 86//LHZ add 20170510
FH_ERROR_CODE DO_INSTRUCTION_RST_AGV_REQ(int* Param)
{
	SetHMIMode(Init_NORMAL);
	LogPrintf("INSTRUCTION_RST_AGV_REQ\n");
 //CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_ONLINE_SUCCESS 87//LHZ add 20170510
FH_ERROR_CODE DO_INSTRUCTION_ONLINE_SUCCESS(int* Param)
{
	SetHMIEvent(OnOnlineSuccess);

 //CompatiableFun(Param);
	 return 0;
}
//#define DO_INSTRUCTION_CHARGE 110
FH_ERROR_CODE DO_INSTRUCTION_CHARGE(struct INSTRUCTION_Normal_params_t* param)
{
	int32 tmpx = param->targetX;
	EndianConvert(&tmpx,sizeof(tmpx));
	int32 tmpy = param->targetY;
	EndianConvert(&tmpy,sizeof(tmpy));
	if((0 != tmpx)&&(0 !=tmpy))
		SetChargePointInfo4Single(&App.map,tmpx/1000,tmpy/1000);
	LogPrintf("tmpx=%d,tmpy=%d",tmpx,tmpy);
	int waitCnt = 5;
	while(1)
	{
		SetChargeRelay(ON);
		ThreadSleep(3000);
		if(Agv.device.battery.batteryBase.QuantityElectric>=51.5)
		{
			LogPrintf("QuantityElectric=%f\n",Agv.device.battery.batteryBase.QuantityElectric);
			break;
		}
		if(IsBatteryCharging())
		{
			break;
		}
		if(waitCnt-- <0)
		{
			SetChargeRelay(OFF);
			ThreadSleep(3000);
			LogPrintf("ChargeMove:Failed-%d\n",5-waitCnt);
			SetErrorToQue(SYSTEM_ERROR_Battery_WaitPowerTimeOut,ErrorC);
			SetSRBatteryDataFlag((struct BatterySR_t*)&Agv.device.battery.batteryBase,1);
			return -1;
		}
	}
	 return 0;
}
FH_ERROR_CODE DO_INSTRUCTION_ONLINE(struct INSTRUCTION_Normal_params_t* param)
{
	SetHMIMode(Init_REQ_ONLINE);
	ReportLoadEvent();
	SetHMIMode(Init_NORMAL);
	LogPrintf("Init_REQ_ONLINE\n");
	return 0;
}
//#define DO_INSTRUCTION_DO_CLEAN 88//LHZ 0610 cleaner
FH_ERROR_CODE DO_INSTRUCTION_DO_CLEAN(struct INSTRUCTION_Normal_params_t* param)
{
	int32 cleanerStatus = param->targetX;
	EndianConvert(&cleanerStatus,sizeof(int32));

	if(cleanerStatus)//打开吸尘器
	{
		CleanerSwitch(ON);
		/*ActionPutDown();
		FanControl(FAN_ON);
		AGV.CleanerStatus = 1;//暂时没用*/
		//CleanerUpDownControl = 200;
	}
	else//关闭吸尘器
	{
		CleanerSwitch(OFF);
		/*ActionLiftUp_HR();
		FanControl(FAN_OFF);
		AGV.CleanerStatus = 0;//暂时没用*/
		//CleanerUpDownControl = 200;
	}
 	//CompatiableFun(Param);
	 return 0;
}
 

//#define DO_INSTRUCTION_CLEANER_DIRTY_DISPLAY 93
FH_ERROR_CODE DO_INSTRUCTION_CLEANER_DIRTY_DISPLAY(struct INSTRUCTION_Normal_params_t* param)
{
	int32 displayStatus = param->targetX;
	EndianConvert(&displayStatus,sizeof(int32));
	//CompatiableFun(Param);
	if(displayStatus)//打开清理提示
	{
		SetHMIEvent(OnOpenCleanerFullBlink);
		//FeedBack.IOSTATUS.bit.CLEANER_DIRTY = 1;//等待清理
		CleanerSwitchDirtyState(1);
	}
	else//关闭清理提示
	{
		Agv.service.HMI.InitlizeEvent = OnCloseCleanerFullBlink;
	}
	return 0;
}


//#define D0_INSTRUCTION_ADJUST_LASER 108
FH_ERROR_CODE DO_INSTRUCTION_ADJUST_LASER(int* Param)
{
	FH_ERROR_CODE re;
	AGV.MotionStatus =AGV_MOTION_LASER_ADJUST;
	re =LaserAssambleAdjust(&Agv.device.laser,3);
	return re;
}
//#define DO_INSTRUCTION_POWER_SWITCH 109 //开、关机
FH_ERROR_CODE DO_INSTRUCTION_POWER_SWITCH(struct INSTRUCTION_Normal_params_t* param)
{
	int32 switchStatus = param->targetX;
	EndianConvert(&switchStatus,sizeof(switchStatus));
	if(IsAgvMotionState())
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_NotInStop_RejectPowerRestart,WarningB);
		return -1;
	}
	PowerSwitch(switchStatus);
	ThreadSleep(3000);

	return 0;
}
//动态切换码间距
FH_ERROR_CODE DO_INSTRUCTION_SET_QRCODEMAP(struct INSTRUCTION_SET_QRCODEMAP_Params_t* Param)
{
	const char * const func_name = "INSTRUCTION::SET_QRCODEMAP";
	const ErrorTypeEnum error_code = SYSTEM_ERROR_Config_DymMap;
	ErrorPriorityEnum priority = WarningB;
	struct DymCodegap_t* dymCodegap = &Agv.service.dymCodegap;
	//清供包台位置数量
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	beltmanager->mapPointNum = 0;
	//增加判断，至少需要下发两个区域块
	
	//接受上位机下位操作id
	int functionID =Param->FunctionID;
	AgvParams.deviceParams.qrcodeParams.codeFlag = 1;
	if(1 == functionID)
	{
		struct AreaInit_t areaInit;
		areaInit.areaNumber = Param->MapIndex;
		areaInit.originIdX = Param->CodeMapInfo.CodeNumberInfo.originIDX;
		EndianConvert(&areaInit.originIdX,sizeof(areaInit.originIdX));
		areaInit.originIdY = Param->CodeMapInfo.CodeNumberInfo.originIDY;
		EndianConvert(&areaInit.originIdY,sizeof(areaInit.originIdY));
		if(0 == areaInit.areaNumber)
		{
			DymAreaSetDefault(dymCodegap,0);
			SetErrorToQue(error_code,priority);
			LogPrintf("%s:err=%x_%d:areaNumber=%d,originIdX=%d,originIdY=%d\n",\
				func_name,error_code,priority,\
				areaInit.areaNumber,areaInit.originIdX,areaInit.originIdY);
		}
		else
		{
			DymAreaSetDefault(dymCodegap,1);
			DymAreaInit(dymCodegap,areaInit);
		}
	}
	else if((2 == functionID)&&(1 == dymCodegap->initFlag))
	{
		struct AreaAdd_t areaAdd;
		areaAdd.areaID = Param->MapIndex;
		areaAdd.startIdX = Param->CodeMapInfo.CodeIDInfo.CodeStartIDX;
		EndianConvert(&areaAdd.startIdX,sizeof(areaAdd.startIdX));
		areaAdd.startIdY = Param->CodeMapInfo.CodeIDInfo.CodeStartIDY;
		EndianConvert(&areaAdd.startIdY,sizeof(areaAdd.startIdY));
		areaAdd.endIdX = Param->CodeMapInfo.CodeIDInfo.CodeEndIDX;
		EndianConvert(&areaAdd.endIdX,sizeof(areaAdd.endIdX));
		areaAdd.endIdY = Param->CodeMapInfo.CodeIDInfo.CodeEndIDY;
		EndianConvert(&areaAdd.endIdY,sizeof(areaAdd.endIdY));
		DymAreaAdd(dymCodegap,areaAdd);
	}
	else if((3 == functionID)&&(1 == dymCodegap->initFlag))
	{
		struct AreaCodeGap_t areaCodeGap;
		areaCodeGap.areaID = Param->MapIndex;
		areaCodeGap.CodeGapX = Param->CodeMapInfo.CodeGapInfo.CodeGapX;
		EndianConvert(&areaCodeGap.CodeGapX,sizeof(areaCodeGap.CodeGapX));
		areaCodeGap.CodeGapY = Param->CodeMapInfo.CodeGapInfo.CodeGapY;
		EndianConvert(&areaCodeGap.CodeGapY,sizeof(areaCodeGap.CodeGapY));
		AreaCodeGap(dymCodegap,areaCodeGap);
	}
	ThreadSleep(1000);
	AgvParams.deviceParams.qrcodeParams.codeFlag = 0;
	return 0;
}

struct Cmd_t InstructionArray[Type_Private_Instruction] = 
{
	{0,NULL},
	{1,(InstructionFunc)DO_INSTRUCTION_CAMERA},
	{2,(InstructionFunc)DO_INSTRUCTION_LIFTUP},
	{3,(InstructionFunc)DO_INSTRUCTION_SETDOWN},
	{4,(InstructionFunc)DO_INSTRUCTION_MOVE},
	{5,(InstructionFunc)DO_INSTRUCTION_RESET},
	{6,(InstructionFunc)DO_INSTRUCTION_RIGHTANGLEMOVE},
	{7,NULL},
	{8,(InstructionFunc)DO_INSTRUCTION_RUN},
	{9,(InstructionFunc)DO_INSTRUCTION_COMMISSION},
	{10,(InstructionFunc)DO_INSTRUCTION_SETDOWNLS},
	{11,(InstructionFunc)DO_INSTRUCTION_UPDATALOG},
	{12,(InstructionFunc)DO_INSTRUCTION_POD_LS},
	{13,(InstructionFunc)DO_INSTRUCTION_ENABLE},
	{14,(InstructionFunc)DO_INSTRUCTION_DISABLE},
	{15,(InstructionFunc)DO_INSTRUCTION_POD_ZERO},
	{16,(InstructionFunc)DO_INSTRUCTION_TURNING},
	{17,(InstructionFunc)DO_INSTRUCTION_RESET_ERROR},
	{18,(InstructionFunc)DO_INSTRUCTION_SOFTSTOP},
	{19,(InstructionFunc)DO_INSTRUCTION_GYRO_ZERO},
	{20,(InstructionFunc)DO_INSTRUCTION_ESTOP},
	{21,(InstructionFunc)DO_INSTRUCTION_CHANGEPODDIRECTION},
	{22,(InstructionFunc)DO_INSTRUCTION_REQUEST},
	{23,(InstructionFunc)DO_INSTRUCTION_FORCEMOVE},
	{24,(InstructionFunc)DO_INSTRUCTION_SETPARAM},
	{25,(InstructionFunc)DO_INSTRUCTION_READPARAM},
	{26,(InstructionFunc)DO_INSTRUCTION_CHARGEMOVE},
	{27,(InstructionFunc)DO_INSTRUCTION_PODJOGGING},
	{28,(InstructionFunc)DO_INSTRUCTION_STEPIN_ONESTEP},
	{29,(InstructionFunc)DO_INSTRUCTION_STEPOUT_ONESTEP},
	{30,(InstructionFunc)DO_INSTRUCTION_SETERROR},
	{31,NULL},
	{32,NULL},
	{33,(InstructionFunc)DO_INSTRUCTION_SEARCHCODE_MOVE},
	{34,(InstructionFunc)DO_INSTRUCTION_SEARCHCODE_TURNING},
	{35,(InstructionFunc)DO_INSTRUCTION_MEASUREDIA_MOVE},
	{36,(InstructionFunc)DO_INSTRUCTION_DORMANCY},
	{37,(InstructionFunc)DO_INSTRUCTION_WAKEUP},
	{38,(InstructionFunc)DO_INSTRUCTION_DEBUGMODE},
	{39,(InstructionFunc)DO_INSTRUCTION_MAP_INFORMATION},
	{40,(InstructionFunc)DO_INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF},
	{41,(InstructionFunc)DO_INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF},
	{42,(InstructionFunc)DO_INSTRUCTION_MEASUREPOWER},
	{43,(InstructionFunc)DO_INSTRUCTION_RUNMODE},
	{44,(InstructionFunc)DO_INSTRUCTION_SLAVEPC_VERSION},
	{45,(InstructionFunc)DO_INSTRUCTION_EXIT_CHARGINGZONE},
	{46,(InstructionFunc)DO_INSTRUCTION_READMILEAGE},
	{47,(InstructionFunc)DO_INSTRUCTION_OPEN_UPCAMERA_BEFORELIFTUP},
	{48,(InstructionFunc)DO_INSTRUCTION_CLOSE_UPCAMERA},
	{49,(InstructionFunc)DO_INSTRUCTION_OPEN_UPCAMERA_AFTERLIFTUP},
	{50,(InstructionFunc)DO_INSTRUCTION_CLEARMILEAGE},
	{51,(InstructionFunc)DO_INSTRUCTION_CHOOSE_LOADCONDITION},
	{52,(InstructionFunc)DO_INSTRUCTION_KEEPPACEWITH_AGVFRONT},
	{53,(InstructionFunc)DO_INSTRUCTION_INITIAL_AFTERLIFT_CHECKSHELF},
	{54,(InstructionFunc)DO_INSTRUCTION_SWITCH_FAN},
	{55,(InstructionFunc)DO_INSTRUCTION_RETURNBACK},
	{56,(InstructionFunc)DO_INSTRUCTION_JOYSTICK_MOVE_POS_MODE},
	{57,(InstructionFunc)DO_INSTRUCTION_JOYSTICK_MOVE_VEL_MODE},
	{58,(InstructionFunc)DO_INSTRUCTION_GUIDE_UPCAMERA_BEFORELIFTUP},
	{59,(InstructionFunc)DO_INSTRUCTION_SWITCH_LOG},
	{60,(InstructionFunc)DO_INSTRUCTION_CHECK_GUIDE_COND},
	{61,(InstructionFunc)DO_INSTRUCTION_SWITCH_LOGPOINT},
	{62,(InstructionFunc)DO_INSTRUCTION_FLUSH_QUEUE},
	{63,(InstructionFunc)DO_INSTRUCTION_GET_AGV_STATUS},
	{64,(InstructionFunc)DO_INSTRUCTION_INITIAL},
	{65,(InstructionFunc)DO_INSTRUCTION_SWITCH_METHOD},
	{66,(InstructionFunc)DO_INSTRUCTION_RESTART_IPC},
	{67,(InstructionFunc)DO_INSTRUCTION_RESTART_CAMERA},
	{68,(InstructionFunc)DO_INSTRUCTION_RESTART_DSP},
	{69,(InstructionFunc)DO_INSTRUCTION_RESTART_DRIVER},
	{70,(InstructionFunc)DO_INSTRUCTION_GUIDE_WITHOUTDM},
	{71,(InstructionFunc)DO_INSTRUCTION_GUIDE_WITHOUTDM_PH2},
	{72,(InstructionFunc)DO_INSTRUCTION_ROT_WITHUP},
	{73,(InstructionFunc)DO_INSTRUCTION_GUIDE_ONCE},
	{74,(InstructionFunc)DO_INSTRUCTION_CHECK_UPCAM},
	{75,(InstructionFunc)DO_INSTRUCTION_ZERORETURN},
	{76,(InstructionFunc)DO_INSTRUCTION_SWITCH_SLAMMODE},
	{77,(InstructionFunc)DO_INSTRUCTION_QUERY_SLAMMODE},
	{78,(InstructionFunc)DO_INSTRUCTION_GUIDE_DOWN_CAM},
	{79,(InstructionFunc)DO_INSTRUCTION_CHECK_PARAM_MAP},
	{80,(InstructionFunc)DO_INSTRUCTION_MANUAL},
	{81,(InstructionFunc)DO_INSTRUCTION_RADAR_AGV_GUIDE},
	{82,(InstructionFunc)DO_INSTRUCTION_RADAR_SHELF_GUIDE},
	{83,(InstructionFunc)DO_INSTRUCTION_BACK_OFF},
	{84,(InstructionFunc)DO_INSTRUCTION_RST_AGV_MODE},
	{85,(InstructionFunc)DO_INSTRUCTION_SET_AGV_REQ},
	{86,(InstructionFunc)DO_INSTRUCTION_RST_AGV_REQ},
	{87,(InstructionFunc)DO_INSTRUCTION_ONLINE_SUCCESS},
	{88,(InstructionFunc)DO_INSTRUCTION_KEEPPACE},
	{89,NULL},
	{90,NULL},
	{91,NULL},
	{92,NULL},
	{93,NULL},
	{94,NULL},
	{95,(InstructionFunc)DO_INSTRUCTION_BEFOREPUTDOWN_OPEN_UPCAMERA_CHECKSHEL},
	{INSTRUCTION_CONVEYER_CONTROL_MODE_TRANSFER,NULL},
	{INSTRUCTION_CONVEYER_MOTOR_SWITCH,NULL},
	{INSTRUCTION_PLATFORM_MOTOR_SWITCH,NULL},
	{INSTRUCTION_BACK_PLATE_MOTOR_SWITCH,NULL},
	{INSTRUCTION_CONVEYER_VELOCITY_SET,NULL},
	{INSTRUCTION_PLATFORM_ROTATE_SET,NULL},
	{INSTRUCTION_BACK_PLATE_LIFT,NULL},
	{INSTRUCTION_REMOVE_CONVEYER_TASK,NULL},
	{INSTRUCTION_INITIAL_CONVEYER,NULL},
	{INSTRUCTION_SHIPPING_CONVEYER,NULL},
	{INSTRUCTION_UNLOADING_CONVEYER,NULL},
	{INSTRUCTION_QUERY_CONVEYER,NULL},
	{INSTRUCTION_ADJUST_LASER,(InstructionFunc)DO_INSTRUCTION_ADJUST_LASER},//HR,add,20171117
	{109,(InstructionFunc)DO_INSTRUCTION_POWER_SWITCH},
	{110,(InstructionFunc)DO_INSTRUCTION_CHARGE},
	{111,(InstructionFunc)DO_INSTRUCTION_ONLINE},
	{112,NULL},
	{113,NULL},
	{114,NULL},
	{115,NULL},
	{116,NULL},
	{117,NULL},
	{118,NULL},
	{119,(InstructionFunc)DO_INSTRUCTION_BELTMOVE},
	{INSTRUCTION_CONVEYOR_LOAD,(InstructionFunc)DO_INSTRUCTION_LOAD},
	{INSTRUCTION_CONVEYOR_UNLOAD,(InstructionFunc)DO_INSTRUCTION_UNLOAD},
	{INSTRUCTION_MOVING_UNLOAD,(InstructionFunc)DO_INSTRUCTION_MOVING_UNLOAD},
	{INSTRUCTION_CONVEYOR_CONFIRM,(InstructionFunc)DO_INSTRUCTION_CONVEYOR_CONFIRM},
	{INSTRUCTION_CONVEYOR_ZERO_OFFSET,(InstructionFunc)DO_INSTRUCTION_CONVEYOR_ZERO_OFFSET},
	{INSTRUCTION_CONVEYOR_BACK,(InstructionFunc)DO_INSTRUCTION_CONVEYOR_BACK},
	{INSTRUCTION_RESET_LOAD,(InstructionFunc)DO_INSTRUCTION_RESET_LOAD},
	{INSTRUCTION_CONVEYOR_CANCEL,(InstructionFunc)DO_INSTRUCTION_CONVEYOR_CANCEL},
	{INSTRUCTION_FULL_LOAD_TEST,(InstructionFunc)DO_INSTRUCTION_FULL_LOAD_Check},
	{INSTRUCTION_SET_LOAD_POS,(InstructionFunc)DO_INSTRUCTION_SET_LOAD_POS},
	{INSTRUCTION_SET_QRCODEMAP,(InstructionFunc)DO_INSTRUCTION_SET_QRCODEMAP}
};

//串口接收到指令首先执行检查，检查返回
/**************
CheckResult_Immediately : 立即执行，不会推入app线程，防止app线程正在执行sleep指令。例如软停
CheckResult_DoNothing : 往app线程推入一个空函数，即什么也不会做，直接给工控机返回ID号和指令执行错误状态

************************/
CmdCheckResult CheckImmediatelyCmd(int TypeID)
{
	//软停、急停、复位指令立即执行，不推入App线程
	if((TypeID == INSTRUCTION_SOFTSTOP)||
		(TypeID ==INSTRUCTION_RESET)||
		(TypeID ==INSTRUCTION_CONVEYOR_CANCEL)||
		(TypeID ==INSTRUCTION_MANUAL)||
		(TypeID ==INSTRUCTION_ESTOP)) 
	{
		return CheckResult_Immediately;
	}
	return CheckResult_Normal;
}
CmdCheckResult CheckDebugMode(int TypeID)
{
	//debug 模式
	if(AGV.DebugMode == 1)
	{
	    if(TypeID != INSTRUCTION_RUNMODE &&
			TypeID != INSTRUCTION_DEBUGMODE && 
			TypeID != INSTRUCTION_RESET_ERROR && 
			TypeID != INSTRUCTION_SETPARAM)
	    {
	        SetErrorToQue((ErrorTypeEnum)ERROR_DEBUGMODE_RejectRunOrders,ErrorC);
	        return CheckResult_DoNothing;
	    }
	}
	return CheckResult_Normal;
}
CmdCheckResult CheckForSafe(int TypeID)
{
	if(TypeID >255)
	{
		LogPrintf("same id\n");
		SetError(ERROR_SAMEIDCMD,1);
		return CheckResult_DoNothing;
	}
	if(GetTaskCnt(&App.appBase.AppTask)>=15)
	{
		LogPrintf("instruction cnt over large\n");
		return CheckResult_DoNothing;
	}
	//新板卡未读到正确的参数，只允许执行设置参数
	if(FHAGV_Initialized != Agv.base.state)
	{
		if((TypeID !=INSTRUCTION_SETPARAM)&&
			(TypeID !=INSTRUCTION_RESET)&&
			(TypeID !=INSTRUCTION_RESET_ERROR))
			return CheckResult_DoNothing;
	}
	//未完成设置参数
	if((TypeID !=INSTRUCTION_RESET) &&
		(TypeID != INSTRUCTION_SETPARAM) &&
		(TypeID != INSTRUCTION_SETDOWNLS) &&
		(TypeID != INSTRUCTION_RESET_ERROR)&&
	   (TypeID !=INSTRUCTION_SWITCH_FAN) &&
	   (TypeID != INSTRUCTION_SOFTSTOP) &&
	   (TypeID != INSTRUCTION_ESTOP )&&
	   (TypeID !=INSTRUCTION_CLOSE_UPCAMERA)&&
	   (TypeID !=INSTRUCTION_CLOSE_UPCAMERA) &&
	   (TypeID !=INSTRUCTION_OPEN_UPCAMERA_BEFORELIFTUP) &&
	   (TypeID !=INSTRUCTION_OPEN_UPCAMERA_AFTERLIFTUP)&&
	   (TypeID !=INSTRUCTION_SOFTSTOP) &&
	   (TypeID !=INSTRUCTION_ESTOP) &&
	   (TypeID !=INSTRUCTION_RUNMODE)&&
	   (TypeID !=INSTRUCTION_RESTART_IPC) &&
	   (TypeID !=INSTRUCTION_RESTART_CAMERA) &&
	   (TypeID !=INSTRUCTION_RESTART_DSP)&&
	   (TypeID !=INSTRUCTION_MEASUREPOWER) &&
	   (TypeID !=INSTRUCTION_SLAVEPC_VERSION) &&
	   (TypeID !=INSTRUCTION_ENABLE) &&
	   (TypeID !=INSTRUCTION_DISABLE)&&
	   (TypeID !=INSTRUCTION_SWITCH_LOG) &&
	   (TypeID !=INSTRUCTION_SWITCH_LOGPOINT)&&
	   (TypeID != INSTRUCTION_SET_AGV_REQ) &&
	   (TypeID != INSTRUCTION_CHECK_PARAM_MAP)&&
	   (TypeID != INSTRUCTION_QUERY_SLAMMODE) &&
	   (TypeID != INSTRUCTION_RST_AGV_REQ) &&
	   (TypeID != INSTRUCTION_RST_AGV_MODE)&&
	   (TypeID != INSTRUCTION_SETERROR)&&
	   (TypeID != INSTRUCTION_DO_CLEAN )&&
	   (TypeID != 109 )&&
	   (TypeID != INSTRUCTION_CONVEYOR_ZERO_OFFSET )&&
	   (TypeID != INSTRUCTION_MANUAL)&&
	   (TypeID != 130))
		{
			if(!IsAppParamsFlagZero())
			{
				SetError(ERROR_SetParam_NoComplete,3);
				return CheckResult_DoNothing;
			}
			if (Param.SetMapData!=1 && (TypeID != INSTRUCTION_MAP_INFORMATION))//HR,add,20160321
			{
				SetError(ERROR_MapDataNotSet,3);
				return CheckResult_DoNothing;
			}
			
			if(App.appParamsManager.MD5CheckValid != 1 && App.appParamsManager.HostComputerSupportValid == 1)
			{
				SetError(ERROR_ParamValid_Fail,3);
				//Instruction.DataArrive = 0;
				return CheckResult_DoNothing;
			}
		}
	if (IsCurPosInChargePoint())
	{//在充电点，仅允许执行退出充电区域指令及其它非动车指令,严禁执行其它动车指令
		if(TypeID == INSTRUCTION_MOVE || TypeID == INSTRUCTION_TURNING || 
			TypeID == INSTRUCTION_SETDOWNLS	|| TypeID == INSTRUCTION_LIFTUP ||
			TypeID == INSTRUCTION_SETDOWN || TypeID == INSTRUCTION_CHARGEMOVE ||
			TypeID == INSTRUCTION_SEARCHCODE_MOVE || TypeID == INSTRUCTION_SEARCHCODE_TURNING ||
			TypeID == INSTRUCTION_CHANGEPODDIRECTION || TypeID == INSTRUCTION_FORCEMOVE || 
			TypeID == INSTRUCTION_KEEPPACEWITH_AGVFRONT)//HR,modified,20160201
		{
			if (TypeID != INSTRUCTION_CHARGEMOVE)
			{
				  SetErrorToQue((ErrorTypeEnum)ERROR_ChargeZone_RejectOrders,ErrorC);
				  return CheckResult_DoNothing;
			}
		}
	}
	//

	int curState = GetAgvMotionState();
	if(MC_ErrorHandling & curState)
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_SoftStop_RejectOrders,ErrorC);
		return CheckResult_DoNothing;
	}
	return CheckResult_Normal;
}

int CheckBeforDoInstruction(struct InstructionProtocl_t* cmdBuf)
{
	int TypeID = cmdBuf->Type;
	int tempExitDormancy = 1;
	CmdCheckResult res = CheckResult_Normal;
	if(cmdBuf->ID >15)
	{
		SetError(ERROR_SAMEIDCMD,1);
		return CheckResult_DoNothing;
	}
	//check Dormancy
	if(TypeID != INSTRUCTION_DORMANCY)
	{
		InvokeFunction(PowerTriggerExitDormancy,&tempExitDormancy,sizeof(int));
	}
	if(res = CheckImmediatelyCmd(TypeID))
	{
		return res;
	}
	
	if(res = CheckForSafe(TypeID))
	{
		return res;
	}
	if(res = CheckDebugMode(TypeID))
	{
		return res;
	}
	
	return CheckResult_Normal;
}
FH_ERROR_CODE CheckBeforeAction(struct Task_t* actionTask)
{
	if((IsAgvMotionState())&&
		(INSTRUCTION_MOVE != actionTask->CurTypeId)&&
		(INSTRUCTION_KEEPPACEWITH_AGVFRONT!= actionTask->CurTypeId)&&
		(INSTRUCTION_MOVING_UNLOAD!= actionTask->CurTypeId)&&
		(INSTRUCTION_RIGHTANGLEMOVE != actionTask->CurTypeId)&&
		(INSTRUCTION_CONVEYOR_CONFIRM != actionTask->CurTypeId))
	{
		LogPrintf("motion get cmd:%d\n",actionTask->CurTypeId);
		SetErrorToQue((ErrorTypeEnum) ERROR_NewCMDWithRuning,WarningB);
		WaitLastCmdComplete();
	}
	return 0;
}

FH_ERROR_CODE AppInstructionInit(struct AppInstruction_t* appInstruction)
{
    //基本命令系统已经做好，这里可以注册新的命令，以及更改系统命令;
    //注册指令检查
	InstructionCheckRegister((InstructionCheck)CheckBeforDoInstruction);
	SetAppInstructionTable(InstructionArray);
	RegisterHookBeforFunc(&App.appBase.AppTask,(MCTask)CheckBeforeAction);
	return 0;
}



