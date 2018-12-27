/*
 * AgvAppInstructionTray.c
 *
 *  Created on: 2018年2月8日
 *      Author: hwei
 */
 
#include "AgvApp.h"
#include "Basefunction.h"
#include "math.h"

/***************************************************************************//*
tray 运动
*//***************************************************************************/


//#define DO_INSTRUCTION_PODJOGGING 27
FH_ERROR_CODE DO_INSTRUCTION_PODJOGGING(struct INSTRUCTION_Normal_params_t* moveParam)
{
 /************条件检查****************************/
 if(0 == IsAgvReadyToMove())
 {
	 SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
	 LogPrintf("=ERROR= agv not ready\n");
	 return -1;
 }

 //region 解析参数
 int16 tmpHeading = moveParam->targetHeading;
 EndianConvert(&tmpHeading,sizeof(tmpHeading));
 tmpHeading+= Agv.service.tray.Heading2AGV;
 SetAppActionState(AGV_MOTION_POD_ZERO);
 struct TrayTskParams_t trayTskParams;
 GetTrayTskParam(&trayTskParams,tmpHeading,Tray_RotateToAgv);
 TrayRotate_tsk(&trayTskParams);
 Uint16 waitCnt = 5000;
 WaitMoveDone_tsk(&waitCnt);
 WaitLastCmdComplete();
 return 0;
}
//#define DO_INSTRUCTION_POD_LS 12 	// pod find zero point
FH_ERROR_CODE DO_INSTRUCTION_POD_LS(int* podLsParam)
{
	SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,10,10);
	ThreadSleep(100);
	Agv.service.tray.base.state = Tray_Idle;
 if(0 == IsAllMotorEnable())
 {
	 SetError(ERROR_MotorDisable_RejectOrder,3);
	 return -1;
 }
 if(IsTrayLiftUpState())
 {
	 SetError(ERROR_LSTouchWhenShelfOn,3);
	 return -2;
 }
 int dir = Sign(Param.AgvParam[PARAM_PodLSP]);
 if(dir == 0)
 {
	SetError(Error_NoLS,3);
	 return -2;
 }
 struct TrayTskParams_t params;
 params.pg.creepDistance = 0;
 params.pg.creepVel = Degree2Rad(1);
 params.pg.maxAcc = Param.AgvParam[PARAM_LiftAngelSpeedAcc];
 params.pg.maxVel = Degree2Rad(15);
 params.pg.maxDec = Param.AgvParam[PARAM_LiftAngelSpeedAcc];
 params.pg.motionType = Tray_RotateToAgv;//对agv
 params.pg.targetType = TargetType_Offset;//相对位置
 params.pg.pgType = PGType_SCurve;
 Uint16 waitCnt = 1000;//单位10ms
 //快速搜零
 params.pg.maxVel = Degree2Rad(80);
 params.target = Degree2Rad(380)*dir;
 params.esParams.stopConditionType = StopCondition_TrayRotateLimit;
 params.esParams.IsEmc = 0;
 params.esParams.stopTarget = 1;//limit有效
 SetAppActionState(AGV_MOTION_POD_LS);
 TrayRotate_tsk(&params);
 waitCnt = 5000;//单位10ms
 WaitMoveDone_tsk(&waitCnt);
 WaitLastCmdComplete();
if(0 == IsTrayRotateLimit())
{
	LogPrintf("podls flag0\n");
	params.pg.maxVel = Degree2Rad(40);
	 params.target = -Degree2Rad(100)*dir;
	 params.esParams.stopConditionType = StopCondition_TrayRotateLimit;
	 params.esParams.IsEmc = 0;
	 params.esParams.stopTarget = 1;//limit有效
	 TrayRotate_tsk(&params);
	 WaitMoveDone_tsk(&waitCnt);

}
 WaitLastCmdComplete();
 if(1 == IsTrayRotateLimit())
{
	LogPrintf("podls flag1\n");
	params.pg.maxVel = Degree2Rad(40);
	 params.target = -Degree2Rad(100)*dir;
	 params.esParams.stopConditionType = StopCondition_TrayRotateLimit;
	 params.esParams.IsEmc = 0;
	  params.esParams.stopTarget = 0;//limit
	 TrayRotate_tsk(&params);
	 WaitMoveDone_tsk(&waitCnt);
	 WaitLastCmdComplete();
}
 
 ThreadSleep(300);
 params.pg.maxVel = Degree2Rad(10);
 params.esParams.stopConditionType = StopCondition_TrayRotateLimit;
 params.esParams.IsEmc = 1;
 params.esParams.stopTarget = 1;//limit有效
 //转一圈
 params.target = Degree2Rad(100)*dir;
 TrayRotate_tsk(&params);
 waitCnt = 2000;//单位10ms
 WaitMoveDone_tsk(&waitCnt);
 //等碰限位停止
 WaitLastCmdComplete();
  if(0 == IsTrayRotateLimit())
 {
	 LogPrintf("podls err2\n");
	 return -3;
 }

 //设置托盘转动零位
 TraySetRotateZeroOffset();
 Agv.service.tray.base.state = Tray_ZeroIdentified;
 //Pod.ZeroLSFlag = 1;
 WaitLastCmdComplete();
 	
	SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,50,10);
	ThreadSleep(100);
  return 0;
}
//#define DO_INSTRUCTION_POD_ZERO 15 // stop log
FH_ERROR_CODE DO_INSTRUCTION_POD_ZERO(int* podRotateParam)
{
	if(IsTrayLiftUpState())
	{
		SetError(ERROR_LSTouchWhenShelfOn,3);
		return -1;
	}

 struct TrayTskParams_t params;
 params.target = 0;
 params.pg.creepDistance = Degree2Rad(2);
 params.pg.creepVel = Param.AgvParam[PARAM_AgvMinAngSPD];
 //判断是否顶有货架，用于托盘回零的速度限制
 if(IsTrayLiftUpState())
 {
	 params.pg.maxVel = 5*PI/180.0;//HR,modified,20161029
	 params.pg.maxAcc = 0.5*Param.AgvParam[PARAM_LiftAngelSpeedAcc];
	 params.pg.maxDec = 0.5*Param.AgvParam[PARAM_LiftAngelSpeedAcc];
 }
 else
 {
	 params.pg.maxVel = Param.AgvParam[PARAM_LiftMaxAngleSpeed];
	 params.pg.maxAcc = Param.AgvParam[PARAM_LiftAngelSpeedAcc];
	 params.pg.maxDec = Param.AgvParam[PARAM_LiftAngelSpeedAcc];
 }
 params.pg.motionType = Tray_RotateToAgv;//对agv
 params.pg.targetType = TargetType_Abs;//abs位置
 params.pg.pgType = PGType_SCurve;
 params.esParams.stopConditionType = StopCondition_Empty;
 SetAppActionState(AGV_MOTION_POD_ZERO);
 TrayRotate_tsk(&params);
 Uint16 waitCnt = 2000;//单位10ms
 WaitMoveDone_tsk(&waitCnt);
  WaitLastCmdComplete();
  return 0;
}
//#define DO_INSTRUCTION_LIFTUP 2 // a liftup instruction
FH_ERROR_CODE DO_INSTRUCTION_LIFTUP(struct INSTRUCTION_Normal_params_t* normalParam)
{
	SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,10,10);
	ThreadSleep(100);
	/*******************start 解析参数******************************/
	int16 tmpHeading = normalParam->targetHeading;
	 EndianConvert(&tmpHeading,sizeof(normalParam->targetHeading));
	 FH_Float agvTargetHeading = tmpHeading*0.01*PI/180;
	 ConvertAngleNearest(&agvTargetHeading,Agv.service.chassis.location.curPosGnd.heading);
	 /*******************end 解析参数******************************/

	 /**********start**条件检查****************************/

  if(0 == IsAgvReadyToMove())
  {
	  SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
	  LogPrintf("=ERROR= agv not ready\n");
	  return -1;
  }
  if(IsTrayLiftUpState())
  {
	  return 0;
  }
  
   /**********end**条件检查****************************/
   
   /**********start*指令部署****************************/
  
    Uint16 waitCnt = 2000;
	//1，托盘旋转对齐货架或者地面
	FH_Float targetPodHeading=0;
	SetAppActionState(AGV_MOTION_POD_ZERO);
	
	if(INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF != GetPreInstructionTypeId())
	{
		//deploy_liftup_noshelf();
		GetNominalHeading(Agv.service.tray.Heading2Ground,&targetPodHeading);
		ConvertAngleNearest(&targetPodHeading,Agv.service.tray.Heading2Ground);
	}
	else if(UpCamera.ShelfDMRead == 1)
	{
		FH_Float temp1,temp2;
		//deploy_liftup_lookshelf();
		//根据上摄像头数据先将托盘对齐货架
		//要改20180609
		temp1 =GetAngleDiff(UpCamera.ShelfDM_liftbefore,Agv.service.tray.Heading2Ground);
		GetNominalHeading(temp1,&temp2);
		targetPodHeading =GetAngleDiff(UpCamera.ShelfDM_liftbefore,temp2);
		ConvertAngleNearest(&targetPodHeading,Agv.service.tray.Heading2Ground);
		
		FH_Float podHeadingDiff = GetAngleDiff(targetPodHeading,Agv.service.tray.Heading2Ground);
		//1.旋转托盘：角度到与货架正交
		//超过0.3度
		/*if (fabs(podHeadingDiff)>Param.AgvParam[PARAM_TargetLiftUpTolPodHeading])
		{
			
		}*/
		
	}
	LogPrintf("lift-PodHeading:%f\n",targetPodHeading);
	struct TrayTskParams_t trayTskParams;
	GetTrayTskParam(&trayTskParams,targetPodHeading,Tray_RotateToGround);
	UpCamera.ShelfDMRead = 0;
	TrayRotate_tsk(&trayTskParams);
	WaitMoveDone_tsk(&waitCnt);
	WaitLastCmdComplete();
	//2，顶升
	if( Agv.device.motorTrayLift.md.SendMotorEnable(&Agv.device.motorTrayLift.md))
	 {
		 SetError(ERROR_MotorDisableAbnormal_5,3);
		 return -2;
	 }
	ThreadSleep(100);
	SetAppActionState(AGV_MOTION_LIFTING);
	FH_Float target = Param.AgvParam[PARAM_LiftDistance]*0.001+0.01;
	GetTrayLiftTskParam(&trayTskParams,target);
	TrayLift_tsk(&trayTskParams);
	WaitMoveDone_tsk(&waitCnt);
	WaitLastCmdComplete();
	Agv.device.motorTrayLift.md.SendMotorDisable(&Agv.device.motorTrayLift.md);
	//3，托盘对齐地面
	SetAppActionState(AGV_MOTION_POD_ZERO);
	GetNominalHeading(Agv.service.tray.Heading2Ground,&targetPodHeading);
	ConvertAngleNearest(&targetPodHeading,Agv.service.tray.Heading2Ground);
	GetTrayTskParam(&trayTskParams,targetPodHeading,Tray_RotateToGround);
	TrayRotate_tsk(&trayTskParams);
	WaitMoveDone_tsk(&waitCnt);
	WaitLastCmdComplete();
	
	//4，turn到地面中心
	FH_Float targetX = Agv.service.chassis.location.XRoundToQR;
	FH_Float targetY = Agv.service.chassis.location.YRoundToQR;
	FH_Float tempDiffx =fabs(targetX-Agv.service.chassis.location.curPosGnd.x);
	FH_Float tempDiffy =fabs(targetY-Agv.service.chassis.location.curPosGnd.y);
	FH_Float offsetDis = (tempDiffx>tempDiffy)? tempDiffx:tempDiffy;
	if(offsetDis>Param.AgvParam[PARAM_ShelfPositionToFloorCodeAjust])
	{
		agvTargetHeading = agvTargetHeading+Sign(agvTargetHeading - Agv.service.chassis.location.curPosGnd.heading)*2*PI;
	}
	LogPrintf("lift-turnHeading:%f\n",agvTargetHeading);
	//设置action状态，对应原运动状态
	SetAppActionState(AGV_MOTION_TURNING);
	//闭环托盘
	int isCloseLoop = 1;
	TrayCloseLoop_tsk(&isCloseLoop);
	struct ChassisTskParams_t params;
	IsCustomIDVaild = 0;//取消自定义速度
	GetChissisTskParam(&params,targetX,targetY,agvTargetHeading,Motion_TURN);
	params.pg.maxAcc = Min(params.pg.maxAcc,LoadCondition.MaxAngularAcc[1]);
    params.pg.maxVel = Min(params.pg.maxVel,LoadCondition.MaxAngularSpd[1]);
    params.pg.maxDec = Min(params.pg.maxDec,LoadCondition.MaxAngularAcc[1]);
	ChassisTurning_tsk(&params);
	WaitMoveDone_tsk(&waitCnt);
	//开环托盘
	isCloseLoop = 0;
	TrayCloseLoop_tsk(&isCloseLoop);
	WaitLastCmdComplete();
  	SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,50,10);
	ThreadSleep(100);
  return 0;
}

//#define DO_INSTRUCTION_SETDOWN 3 // a setdown instruction
FH_ERROR_CODE DO_INSTRUCTION_SETDOWN(struct INSTRUCTION_Normal_params_t* normalParam)
{
	SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,10,10);
	ThreadSleep(100);
	/*******************start 解析参数******************************/
	int16 tmpHeading = normalParam->targetHeading;
	EndianConvert(&tmpHeading,sizeof(normalParam->targetHeading));
	FH_Float agvTargetHeading = tmpHeading*0.01*PI/180;
	ConvertAngleNearest(&agvTargetHeading,Agv.service.chassis.location.curPosGnd.heading);
	/*******************end 解析参数******************************/

	/**********start**条件检查****************************/

	if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
		LogPrintf("=ERROR= agv not ready\n");
		return -1;
	}
	if(0 == IsTrayLiftUpState())
	{
		return 0;
	}
	if(0 == AGV.FindDM)
	{
		LogPrintf("cant setdown no dm\n");
		return 0;
	}
	/**********end**条件检查****************************/

	/**********start*指令部署****************************/
	//恢复上视曝光
	AdjustExposure_b_adjust_exposure(EXPOSURE_CAMERA_UP,FALSE_FH);

	//1, 误差过大旋转多一圈
    Uint16 waitCnt = 2000;
	FH_Float targetX = Agv.service.chassis.location.XRoundToQR;
	FH_Float targetY = Agv.service.chassis.location.YRoundToQR;
	FH_Float temp1 =fabs(targetX-Agv.service.chassis.location.curPosGnd.x);
	FH_Float temp2 =fabs(targetY-Agv.service.chassis.location.curPosGnd.y);
	FH_Float offsetDis = (temp1>temp2)? temp1:temp2;
	if(offsetDis>Param.AgvParam[PARAM_ShelfPositionToFloorCodeAjust])
	{
		agvTargetHeading = agvTargetHeading+Sign(agvTargetHeading - Agv.service.chassis.location.curPosGnd.heading)*2*PI;
	}
	//闭环托盘
	int isCloseLoop = 1;
	TrayCloseLoop_tsk(&isCloseLoop);
	//设置action状态，对应原运动状态
	SetAppActionState(AGV_MOTION_TURNING);
	struct ChassisTskParams_t params;
	IsCustomIDVaild = 0;//取消自定义速度
	GetChissisTskParam(&params,targetX,targetY,agvTargetHeading,Motion_TURN);
	params.pg.maxAcc = Min(params.pg.maxAcc,LoadCondition.MaxAngularAcc[1]);
    params.pg.maxVel = Min(params.pg.maxVel,LoadCondition.MaxAngularSpd[1]);
    params.pg.maxDec = Min(params.pg.maxDec,LoadCondition.MaxAngularAcc[1]);
	ChassisTurning_tsk(&params);
	WaitMoveDone_tsk(&waitCnt);
		//闭环托盘
	isCloseLoop = 0;
	TrayCloseLoop_tsk(&isCloseLoop);
	WaitLastCmdComplete();
//2，托盘旋转，执行过检查上视，将货架对齐地面
	FH_Float targetPodHeading=0;
	SetAppActionState(AGV_MOTION_POD_ZERO);
	if(INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF!= GetPreInstructionTypeId())
	{
		GetNominalHeading(Agv.service.tray.Heading2Ground,&targetPodHeading);
		ConvertAngleNearest(&targetPodHeading,Agv.service.tray.Heading2Ground);
		LogPrintf("targetPodHeading1=%f\n",targetPodHeading);
	}
	else if(1 == UpCamera.ShelfDMRead_after)
	{
		if(fabs(UpCamera.ShelfDM_liftafter - UpCamera.DeltaHeading2GND)>2.0*PI/180)
		{
			SetError(ERROR_Angle_too_large,3);
		}
		GetNominalHeading(UpCamera.ShelfDM_liftafter,&targetPodHeading);
		FH_Float podheadingdiff =GetAngleDiff(targetPodHeading,UpCamera.ShelfDM_liftafter);
        targetPodHeading = podheadingdiff+Agv.service.tray.Heading2Ground;
		LogPrintf("targetPodHeading2=%f\n",targetPodHeading);
	}
    UpCamera.ShelfDMRead_after = 0;
	struct TrayTskParams_t trayTskParams;
	GetTrayTskParam(&trayTskParams,targetPodHeading,Tray_RotateToGround);
	TrayRotate_tsk(&trayTskParams);
	WaitMoveDone_tsk(&waitCnt);
	WaitLastCmdComplete();
//3，下降
	if( Agv.device.motorTrayLift.md.SendMotorEnable(&Agv.device.motorTrayLift.md))
	{
		SetError(ERROR_MotorDisableAbnormal_5,3);
		return -2;
	}
	ThreadSleep(100);
	SetAppActionState(AGV_MOTION_SETDOWN);
	FH_Float target = -(Param.AgvParam[PARAM_LiftDistance]*0.001+0.01);
	GetTrayLiftTskParam(&trayTskParams,target);
	TrayLift_tsk(&trayTskParams);
	WaitMoveDone_tsk(&waitCnt);
	WaitLastCmdComplete();
	
	Agv.device.motorTrayLift.md.SendMotorDisable(&Agv.device.motorTrayLift.md);
	SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,50,10);
	ThreadSleep(100);
  return 0;
}
//#define DO_INSTRUCTION_SETDOWNLS 10
FH_ERROR_CODE DO_INSTRUCTION_SETDOWNLS(struct INSTRUCTION_Normal_params_t* normalParam)
{
	SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,10,10);
	ThreadSleep(100);
	NormalParamsEndianConvert(normalParam);
	FH_Float targetPodHeading = 0;
	struct TrayTskParams_t trayTskParams;
	Uint16 waitCnt = 2000;
   //上一个指令不是顶升前确认,就不转托盘
	if((1 == UpCamera.Podneed_turn)&&(0 == IsTrayLiftUpState())&&
		(INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF== GetPreInstructionTypeId()))
	{
		targetPodHeading = UpCamera.DeltaHeading2GND;
		ConvertAngleNearest(&targetPodHeading,Agv.service.tray.Heading2Ground);
		
		GetTrayTskParam(&trayTskParams,targetPodHeading,Tray_RotateToGround);
		TrayRotate_tsk(&trayTskParams);
		WaitMoveDone_tsk(&waitCnt);
		WaitLastCmdComplete();
	}
	UpCamera.Podneed_turn = 0;
   //DeloySetDownLS(normalParam->targetX);
   
	if( Agv.device.motorTrayLift.md.SendMotorEnable(&Agv.device.motorTrayLift.md))
	{
	   SetError(ERROR_MotorDisableAbnormal_5,3);
	   return -1;
	}
	ThreadSleep(100);
	FH_Float target = 0;
	if(normalParam->targetX)
	{
		struct Tray_t* tray = &Agv.service.tray;
		if(tray->trayLiftState == Tray_UP)
		{
			LogPrintf("repetitive lift up/n");
			//SetErrorToQue(SYSTEM_ERROR_Tray_UPLimit,WarningC);
			WaitLastCmdComplete_fun();
			Agv.device.motorTrayLift.md.SendMotorDisable(&Agv.device.motorTrayLift.md);
			SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,50,10);
			ThreadSleep(100);
			return 0;
		}
		SetAppActionState(AGV_MOTION_LIFTING);
//	   target = (Param.AgvParam[PARAM_LiftDistance]*0.001+0.01);
		target = (Param.AgvParam[PARAM_LiftDistance]*0.001);
	   GetTrayLiftTskParam(&trayTskParams,target);
	   TrayLift_tsk(&trayTskParams);
	   WaitMoveDone_tsk(&waitCnt);
	}
	else
	{
		//恢复上视曝光
	    AdjustExposure_b_adjust_exposure(EXPOSURE_CAMERA_UP,FALSE_FH);
		SetAppActionState(AGV_MOTION_SETDOWN);
		target = -(Param.AgvParam[PARAM_LiftDistance]*0.001+0.01);
		GetTrayLiftTskParam(&trayTskParams,target);
		TrayLift_tsk(&trayTskParams);
		WaitMoveDone_tsk(&waitCnt);
	}
   
   WaitLastCmdComplete_fun();
   Agv.device.motorTrayLift.md.SendMotorDisable(&Agv.device.motorTrayLift.md);
	SetPanelCycle(&Agv.device.ledAgv.leftLed.panel,50,10);
	ThreadSleep(100);
   return 0;
}
#if 1
FH_ERROR_CODE DoAdjustExposure(int mode)
{
	static int cnt = 0;
	int logFlag = 0;
	cnt++;
	if(cnt >= 100)
	{
		cnt = 0;
		logFlag = 1;
	}
	int sucessConfirmCnt = 0;
	int failConfirmCnt = 0;
	int waitCnt = 1500;//超时30s = 20ms*waitCnt;
	SetAppActionState(AGV_MOTION_BEFORELIFT_CHECK_UPCAMERA);
	AdjustExposure_init();
	//AdjustExposure_b_adjust_exposure(EXPOSURE_CAMERA_UP,FALSE_FH);
	QRCodeMode_e qrCodeMode;
	if(0 == mode)
	{
		qrCodeMode = QRCODEMODE_UP_GUIDE;
	}
	else if(1 == mode)
	{
		qrCodeMode = QRCODEMODE_UP_BEFORE_LIFT;
	}
	QRCodeSetMode(&Agv.device.qrcode, qrCodeMode);
	ThreadSleep(1000);
	//int preGuideState = CAMERADATA_GUIDE_FAILED;
	do
	{
		if(waitCnt <= 0)
		{
			sucessConfirmCnt = 0;
			SetErrorToQue((ErrorTypeEnum)ERROR_CheckUpCameraActionTimeOut,WarningB);
			break;
		}
		
		 struct UpCameraAppData_t upCameraData;
		 struct UpGuideAppData_t upGuideData;
		 if(qrCodeMode == GetQRMode())
		{
			if(mode)
			{
				GetUpCameraAppData(&upCameraData);
				LogPrintf("upCameraData:%d\n",upCameraData.DataValid);
				if((upCameraData.DataValid >= DM642ReliabilityThreshold)&&
					(0 == upCameraData.ErrorState))
				{
					//failConfirmCnt = 0;
					if(++sucessConfirmCnt>=10)
					{
						break;
					}
					else
					{
						ThreadSleep(30);
						continue;
					}
				}
				else if(upCameraData.DataValid<0)
				{
					//没有接收到新数据
					ThreadSleep(30);
					continue;
				}
				else
				{
					//接收到数据，单数数据无效
					sucessConfirmCnt = 0;
				}
					
			}
			else
			{
				//preGuideState = upGuideData.UpGuideState;
				GetUpGuideAppData(&upGuideData);
				LogPrintf("UpGuideState:%d\n",upGuideData.UpGuideState);
				if(CAMERADATA_GUIDE_PROC_ONE <= upGuideData.UpGuideState)
				{
					//failConfirmCnt = 0;
					if(++sucessConfirmCnt>=10)
					{
						break;
					}
					else
					{
						ThreadSleep(100);
						continue;
					}
				}
				
			}
			
			//else
			//{
				sucessConfirmCnt = 0;
				if(++failConfirmCnt>= 10)
				{
					failConfirmCnt = 0;
					if(1==logFlag)
						LogPrintf("adjust exposure\n");
					char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
					LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
					if(0 == AdjustExposure_b_adjust_exposure(EXPOSURE_CAMERA_UP,TRUE_FH))
					{
						SetErrorToQue((ErrorTypeEnum)ERROR_AdustCameraExposure,WarningB);
						break;
					}
					ThreadSleep(200);
				}
			//}
		}
		else
		{
			failConfirmCnt++;
			sucessConfirmCnt = 0;
			if(failConfirmCnt>=400)
			{
				LogPrintf("up mode time out\n");
				SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_UP642_TIME_OUT,WarningA);
				break;
			}
			else if(0 == (failConfirmCnt%100))
			{
				LogPrintf("up mode set mode\n");
				QRCodeSetMode(&(Agv.device.qrcode), qrCodeMode);
			}
		}
		ThreadSleep(20);
	}while((waitCnt--)>0);
	//调完曝光不切下视
	/*waitCnt = 10;
	do
	{
		QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_DOWN);
		ThreadSleep(500);
		if((waitCnt --)<=0)
		{
			LogPrintf("wait down mode Cnt out\n");
			SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_DN642_TIME_OUT,ErrorC);
			return -1;
		}
	}while(QRCODEMODE_DOWN != GetQRMode());*/
	if(sucessConfirmCnt)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
#else
FH_ERROR_CODE DoAdjustExposure()
{
	int sucessConfirmCnt = 0;
	int failConfirmCnt = 0;
	int waitCnt = 1500;//超时30s = 20ms*waitCnt;
	SetAppActionState(AGV_MOTION_BEFORELIFT_CHECK_UPCAMERA);
	AdjustExposure_init();
	AdjustExposure_b_adjust_exposure(EXPOSURE_CAMERA_UP,FALSE_FH);
	QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_UP_BEFORE_LIFT);
	ThreadSleep(500);
	
	struct UpCameraAppData_t upCameraData;
	do
	{
		if(waitCnt <= 0)
		{
			sucessConfirmCnt = 0;
			SetErrorToQue((ErrorTypeEnum)ERROR_CheckUpCameraActionTimeOut,WarningB);
			break;
		}

		 GetUpCameraAppData(&upCameraData);
		 if(QRCODEMODE_UP_BEFORE_LIFT == upCameraData.UpCameraState)
		{
			if(upCameraData.DataValid)
			{
				//failConfirmCnt = 0;
				if(++sucessConfirmCnt>=10)
				{
					break;
				}
				
			}
			else
			{
				sucessConfirmCnt = 0;
				if(++failConfirmCnt>= 20)
				{
					failConfirmCnt = 0;
					LogPrintf("adjust exposure\n");
					char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
					LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
					if(0 == AdjustExposure_b_adjust_exposure(EXPOSURE_CAMERA_UP,TRUE_FH))
					{
						SetErrorToQue((ErrorTypeEnum)ERROR_AdustCameraExposure,WarningB);
						break;
					}
				}
			}
		}
		else
		{
			failConfirmCnt++;
			sucessConfirmCnt = 0;
			if(failConfirmCnt>=150)
			{
				LogPrintf("up mode time out\n");
				SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_UP642_TIME_OUT,WarningA);
				break;
			}
			else if(0 == (failConfirmCnt%50))
			{
				LogPrintf("up mode set mode\n");
				QRCodeSetMode(&(Agv.device.qrcode), QRCODEMODE_UP_BEFORE_LIFT);
			}
		}
		ThreadSleep(20);
	}while((waitCnt--)>0);

	waitCnt = 10;
	do
	{
		QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_DOWN);
		ThreadSleep(500);
		if((waitCnt --)<=0)
		{
			LogPrintf("wait down mode Cnt out\n");
			SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_DN642_TIME_OUT,ErrorC);
			return -1;
		}
	}while(QRCODEMODE_DOWN != GetQRMode());
	if(sucessConfirmCnt)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

#endif
//#define DO_INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF 40//40顶升前打开上置摄像头
FH_ERROR_CODE DO_INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF(int* normalParam)
{
	FH_ERROR_CODE rtn = 0;
	int podRotateParam = 0;
	FH_Float target=0;
	GetNominalHeading(Agv.service.tray.Heading2AGV,&target);
	if(fabs(fabs(target)-PI/2)<0.01)
	{
		rtn = DO_INSTRUCTION_POD_ZERO(&podRotateParam);
	}
	if(IsTrayLiftUpState())
	{
	 	SetErrorToQue((ErrorTypeEnum)ERROR_Liftpos_RejectOrder,ErrorC);
		return -1;
	}
	if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum)Error_NoLS,ErrorC);
		return -2;
	}
	#if 1
	//提前开启顶升电机
	EnableTrayLiftMotorFast_tsk();
	//1,adjust exposure
	//先用上视模式看数据调曝光,用上视引导
	rtn =0;//DoAdjustExposure(1);
	int dataInValidCnt = 0;
	if(0 == rtn)
	{
		int moveCnt = 3;//运动次数超过5报错
		SetAppActionState(AGV_MOTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF_PH2);
		QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_UP_BEFORE_LIFT);
		ThreadSleep(200);
		int waitCnt = 1000;
		int sucessConfirmCnt = 0;
		int failConfirmCnt = 0;
		
		FH_Float dis_limit = fabs(Param.AgvParam[PARAM_GuideRange]);
	    if(dis_limit > 0.01)
	    {
	        dis_limit = 0.01;
	    }
		struct UpCameraAppData_t upCameraData;
		do
		{
			 GetUpCameraAppData(&upCameraData);
			 if((QRCODEMODE_UP_BEFORE_LIFT == upCameraData.UpCameraState)
			 		&&(upCameraData.DataValid>=DM642ReliabilityThreshold)
	                && (fabs(upCameraData.DeltaX2AGV) < dis_limit)
	                && (fabs(upCameraData.DeltaY2AGV) < dis_limit))
	        {
				if(++sucessConfirmCnt>=5)
				{
					UpCamera.ShelfDMRead = 1;
					UpCamera.Podneed_turn = 1;
				 	UpCamera.ShelfDM_liftbefore = upCameraData.DeltaHeading2GND;
					LogPrintf("up guid ok x:%f,y:%f\n",upCameraData.DeltaX2AGV,upCameraData.DeltaY2AGV);
					UpCamera.ShelfPositionStatus = 0x02;
					int32 upCameraX =upCameraData.X*1000;//HR,add,20161110
					int32 upCameraY =upCameraData.Y*1000;
					int16 upCameraH =upCameraData.Heading/PI*180.0*100.0;
					ReportShelfCodeEvent(upCameraX,upCameraY,upCameraH,UpCamera.ShelfPositionStatus);
					LogPrintf("sucess Confirm\n");
					char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
					LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
					break;
				}
	        }
			else if(QRCODEMODE_UP_BEFORE_LIFT != upCameraData.UpCameraState)
			{
				sucessConfirmCnt = 0;
				failConfirmCnt++;
				if(failConfirmCnt>=150)
				{
					LogPrintf("up guid time out\n");
					SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_UP642_TIME_OUT,WarningB);
					rtn = -2;
					break;
				}
				else if(0 == (failConfirmCnt%50))
				{
					LogPrintf("up guid set mode\n");
					QRCodeSetMode(&(Agv.device.qrcode), QRCODEMODE_UP_BEFORE_LIFT);
				}
			}
			else if(upCameraData.DataValid<DM642ReliabilityThreshold)
			{
				//do nothing but wait;
				ThreadSleep(20);
				if(++dataInValidCnt>=10)
				{
					break;
				}
				continue;
			}
			else
			{
				if(--moveCnt<0)
				{
					SetErrorToQue((ErrorTypeEnum) ERROR_GUIDE_TIME_OUT,WarningB);
					LogPrintf("guid moveCnt out state:%d, x:%f,y:%f\n",upCameraData.UpCameraState,upCameraData.X,upCameraData.Y);
					rtn = -4;
					break;
				}
				sucessConfirmCnt = 0;
				LogPrintf("up guid state:%d,x:%f,y:%f,targetx:%f,targety:%f\n",
					upCameraData.UpCameraState,upCameraData.X,upCameraData.Y,
					upCameraData.TargetX,upCameraData.TargetY);
				char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
				LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
				int32 upCameraX =upCameraData.DeltaX2AGV*1000;//HR,add,20161110
				int32 upCameraY =upCameraData.DeltaY2AGV*1000;
				int tmpValue = upCameraData.Heading/PI*180.0*100.0;
				
				ReportShelfCodeEvent(upCameraX,upCameraY,tmpValue,0x0);
				 //导引的时候，可能有大于的情况
				if((fabs(upCameraData.DeltaX2AGV) > 0.06) || (fabs(upCameraData.DeltaY2AGV) > 0.06)||
					(fabs(upCameraData.TargetX - Agv.service.chassis.location.XRoundToQR) >= 0.1)||
					(fabs(upCameraData.TargetY - Agv.service.chassis.location.YRoundToQR) >= 0.1))
				{
					SetErrorToQue((ErrorTypeEnum)ERROR_MOVE_RANGE,WarningB);
					rtn = -3;
					break;
				}
				//无论切换下视是否成功都引导？没有下视码引导很难成功到精度
				SwitchToDownMode();
				if(1)//0 == SwitchToDownMode())
				{
					struct ChassisTskParams_t params;
					FH_Float tarH;
					//GetNominalHeading(Agv.service.chassis.location.curPosGnd.heading,&tarH)
					if(Agv.service.chassis.location.curPosGnd.heading>0)
					{
						tarH = Agv.service.chassis.location.curPosGnd.heading-2.0*PI;
					}
					else
					{
						tarH =Agv.service.chassis.location.curPosGnd.heading+2.0*PI;
					}
					IsCustomIDVaild = 0;//取消自定义速度
					GetChissisTskParam(&params,upCameraData.TargetX,upCameraData.TargetY,tarH,Motion_TURN);
					params.pg.maxAcc = Min(params.pg.maxAcc,LoadCondition.MaxAngularAcc[0]);
                    params.pg.maxVel = Min(params.pg.maxVel,LoadCondition.MaxAngularSpd[0]);
                    params.pg.maxDec = Min(params.pg.maxDec,LoadCondition.MaxAngularAcc[0]);
					ChassisTurning_tsk(&params);
					Uint16 waitCnt4Move = 5000;
					WaitMoveDone_tsk(&waitCnt4Move);
					WaitLastCmdComplete_fun();
					LogPrintf("curX:%f,curY:%f\n",Agv.service.chassis.location.curPosGnd.x,Agv.service.chassis.location.curPosGnd.y);
					QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_UP_GUIDE);
					ThreadSleep(200);
				}
				
			}
			ThreadSleep(20);
			//1s quit
			waitCnt--;
		}while(waitCnt>0);
	}
	//else
	//普通上视数据无效
	if(dataInValidCnt>=10)
	{
		//上视调曝光失败再用引导调
		/*if(rtn)
		{
			rtn = DoAdjustExposure(0);
		}*/
		//2,up guide
		//调曝光成功才进入下一步
		int waitCnt = 3000;
		int sucessConfirmCnt = 0;
		int failConfirmCnt = 0;
		if(0 == rtn)
		{
			int moveCnt = 5;//运动次数超过5报错
			SetAppActionState(AGV_MOTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF);
			QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_UP_GUIDE);
			ThreadSleep(200);
			FH_Float dis_limit = fabs(Param.AgvParam[PARAM_GuideRange]);
		    if(dis_limit > 0.01)
		    {
		        dis_limit = 0.01;
		    }
			struct UpGuideAppData_t upGuideData;
			do
			{
				 GetUpGuideAppData(&upGuideData);
				 if((CAMERADATA_GUIDE_OK == upGuideData.UpGuideState)
		                && (fabs(upGuideData.X) < dis_limit)
		                && (fabs(upGuideData.Y) < dis_limit))
		        {
					if(++sucessConfirmCnt>=5)
					{
						LogPrintf("up guid ok x:%f,y:%f\n",upGuideData.X,upGuideData.Y);
						
						int32 upCameraX =upGuideData.X*1000;//HR,add,20161110
						int32 upCameraY =upGuideData.Y*1000;
						ReportShelfCodeEvent(upCameraX,upCameraY,1234,0x2);
						break;
					}
		        }
				else if(CAMERADATA_GUIDE_FAILED == upGuideData.UpGuideState)
				{
					sucessConfirmCnt = 0;
					failConfirmCnt++;
					if(failConfirmCnt>=100)
					{
						LogPrintf("up guid time out\n");
						SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_UP642_TIME_OUT,WarningB);
						rtn = -2;
						break;
					}
					else if(0 == (failConfirmCnt%50))
					{
						LogPrintf("up guid set mode\n");
						QRCodeSetMode(&(Agv.device.qrcode), QRCODEMODE_UP_GUIDE);
					}
				}
				else
				{
					sucessConfirmCnt = 0;
					LogPrintf("up guid state:%d,x:%f,y:%f,targetx:%f,targety:%f\n",
						upGuideData.UpGuideState,upGuideData.X,upGuideData.Y,
						upGuideData.TargetX,upGuideData.TargetY);
					char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
					LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
					int32 upCameraX =upGuideData.X*1000;//HR,add,20161110
					int32 upCameraY =upGuideData.Y*1000;
					int tmpValue = 4444;
					if(CAMERADATA_GUIDE_PROC_ONE == upGuideData.UpGuideState)
				 	{
				 		tmpValue = 1111;
				 	}
					else if(CAMERADATA_GUIDE_PROC_TWO == upGuideData.UpGuideState)
				 	{
				 		tmpValue = 2222;
				 	}
					else if(CAMERADATA_GUIDE_PROC_THREE == upGuideData.UpGuideState)
				 	{
				 		tmpValue = 3333;
				 	}
					ReportShelfCodeEvent(upCameraX,upCameraY,tmpValue,0x0);
					 //导引的时候，可能有大于的情况
					if((fabs(upGuideData.X) > 0.06) || (fabs(upGuideData.Y) > 0.06)||
						(fabs(upGuideData.TargetX - Agv.service.chassis.location.XRoundToQR) >= 0.1)||
						(fabs(upGuideData.TargetY - Agv.service.chassis.location.YRoundToQR) >= 0.1))
					{
						SetErrorToQue((ErrorTypeEnum)ERROR_MOVE_RANGE,WarningB);
						rtn = -3;
						break;
					}
					//无论切换下视是否成功都引导？没有下视码引导很难成功到精度
					SwitchToDownMode();
					if(1)//0 == SwitchToDownMode())
					{
						struct ChassisTskParams_t params;
						FH_Float tarH;
						//GetNominalHeading(Agv.service.chassis.location.curPosGnd.heading,&tarH)
						if(Agv.service.chassis.location.curPosGnd.heading>0)
						{
							tarH = Agv.service.chassis.location.curPosGnd.heading-2.0*PI;
						}
						else
						{
							tarH =Agv.service.chassis.location.curPosGnd.heading+2.0*PI;
						}
						IsCustomIDVaild = 0;//取消自定义速度
						GetChissisTskParam(&params,upGuideData.TargetX,upGuideData.TargetY,tarH,Motion_TURN);
						params.pg.maxAcc = Min(params.pg.maxAcc,LoadCondition.MaxAngularAcc[0]);
	                    params.pg.maxVel = Min(params.pg.maxVel,LoadCondition.MaxAngularSpd[0]);
	                    params.pg.maxDec = Min(params.pg.maxDec,LoadCondition.MaxAngularAcc[0]);
						ChassisTurning_tsk(&params);
						Uint16 waitCnt4Move = 5000;
						WaitMoveDone_tsk(&waitCnt4Move);
						WaitLastCmdComplete_fun();
						LogPrintf("curX:%f,curY:%f\n",Agv.service.chassis.location.curPosGnd.x,Agv.service.chassis.location.curPosGnd.y);
						QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_UP_GUIDE);
						ThreadSleep(200);
					}
					if(--moveCnt<=0)
					{
						SetErrorToQue((ErrorTypeEnum) ERROR_GUIDE_TIME_OUT,WarningB);
						LogPrintf("guid moveCnt out state:%d, x:%f,y:%f\n",upGuideData.UpGuideState,upGuideData.X,upGuideData.Y);
						rtn = -4;
						break;
					}
				}
				ThreadSleep(20);
				//1s quit
				waitCnt--;
			}while(waitCnt>0);
		}
		else
		{
			SetErrorToQue((ErrorTypeEnum) ERROR_ReadUpCameraFailed,ErrorC);
		}
		//上视模式调曝光
		rtn = 0;//DoAdjustExposure(1);

		//3,confirm up camera
		waitCnt = 150;
		if(0 == rtn)
		{
			SetAppActionState(AGV_MOTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF_PH2);
			QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_UP_BEFORE_LIFT);
			ThreadSleep(500);
			failConfirmCnt = 0;
			sucessConfirmCnt = 0;
			int errConfirmCnt = 0;
			int DataValidCnt =0;
			struct UpCameraAppData_t upCameraData;
			do
			{
				if(waitCnt-- <=0)
				{
					LogPrintf("waitCnt out\n");
					SetErrorToQue((ErrorTypeEnum) ERROR_DM642_FAIL,WarningB);
					rtn = -1;
					break;
				}
				GetUpCameraAppData(&upCameraData);
				if(QRCODEMODE_UP_BEFORE_LIFT == upCameraData.UpCameraState)
				{
					if(upCameraData.DataValid >= DM642ReliabilityThreshold)
					{
						if(++DataValidCnt>=10)
						{
							UpCamera.ShelfDMRead = 1;
							UpCamera.Podneed_turn = 1;
						 	UpCamera.ShelfDM_liftbefore = upCameraData.DeltaHeading2GND;
							FH_Float tmpDis = sqrt(upCameraData.DeltaX2AGV*upCameraData.DeltaX2AGV + upCameraData.DeltaY2AGV*upCameraData.DeltaY2AGV);
							if(tmpDis < 2*Param.AgvParam[PARAM_UpCamera_Shelf_Offset])
							{//HR,modified,20160123
								if (++sucessConfirmCnt>=5)
								{
									UpCamera.ShelfPositionStatus = 0x02;
									int32 upCameraX =upCameraData.X*1000;//HR,add,20161110
									int32 upCameraY =upCameraData.Y*1000;
									int16 upCameraH =upCameraData.Heading/PI*180.0*100.0;
									ReportShelfCodeEvent(upCameraX,upCameraY,upCameraH,UpCamera.ShelfPositionStatus);
									LogPrintf("sucess Confirm\n");
									char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
									LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
									break;
								}

							}
							else
							{
								if(++errConfirmCnt>10)
								{
									LogPrintf("err Confirm\n");
									char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
									LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);

									int32 upCameraX =upCameraData.X*1000;//HR,add,20161110
									int32 upCameraY =upCameraData.Y*1000;
									int16 upCameraH =upCameraData.Heading/PI*180.0*100.0;
									ReportShelfCodeEvent(upCameraX,upCameraY,upCameraH,0);
									break;
								}
							}
							
						}
					}
					else if(upCameraData.DataValid<0)
					{
						//没有接收到新数据
					}
					else
					{
						//接收到数据，单数数据无效
						sucessConfirmCnt = 0;
						DataValidCnt = 0;
					}
				
				}
				else
				{
					failConfirmCnt++;
					sucessConfirmCnt = 0;
					if(failConfirmCnt>=150)
					{
						LogPrintf("up mode time out\n");
						char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
						LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
						SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_UP642_TIME_OUT,WarningB);
						break;
					}
					else if(0 == (failConfirmCnt%50))
					{
						LogPrintf("up mode set mode\n");
						QRCodeSetMode(&(Agv.device.qrcode), QRCODEMODE_UP_BEFORE_LIFT);
					}
				}
				ThreadSleep(20);
			}while(1);
		}
		else
		{
			SetErrorToQue((ErrorTypeEnum) ERROR_ReadUpCameraFailed,ErrorC);
		}
	}
	//4,switch down mode 
	
	rtn |= SwitchToDownMode();
	
	
	#else
	BeginDeploy;
	Instruction.Type = INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF;
	DeployBeforeLiftOpenUpcamera();
	EndDeploy;
	#endif
	WaitLastCmdComplete();
	return 0;// 出错也让过?，等顶升后再报错?
}
//#define DO_INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF 41//41顶升后打开上置摄像头
FH_ERROR_CODE DO_INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF(int* normalParam)
{
	FH_ERROR_CODE rtn = 0;

	//1,adjust exposure
	rtn = 0;//DoAdjustExposure(1);
	//2,
	//调曝光成功才进入下一步
	int waitCnt = 150;
	int failConfirmCnt = 0;
	int	sucessConfirmCnt = 0;
	struct UpCameraAppData_t upCameraData;
	if(0 == rtn)
	{
		SetAppActionState(AGV_MOTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF);
		QRCodeSetMode(&Agv.device.qrcode, QRCODEMODE_UP_AFTER_LIFT);
		ThreadSleep(200);
		
		int DataValidCnt =0;
		int errConfirmCnt = 0;
		do
		{
			if((waitCnt--) <=0)
			{
				LogPrintf("waitCnt out\n");
				SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_UP642_TIME_OUT,WarningA);
				rtn = -1;
				break;
			}
			GetUpCameraAppData(&upCameraData);
			if(QRCODEMODE_UP_AFTER_LIFT == upCameraData.UpCameraState)
			{
				if(upCameraData.DataValid)
				{
					if(++DataValidCnt>=10)
					{
						UpCamera.ShelfDMRead_after = 1;
						UpCamera.ShelfDM_liftafter = upCameraData.DeltaHeading2GND;
						FH_Float tmpH = GetAngleDiff(upCameraData.DeltaHeading2GND,Agv.service.tray.Heading2Ground);
						tmpH = WrapToHalfPiDe(tmpH);
						FH_Float tmpDis = sqrt(upCameraData.DeltaX2AGV*upCameraData.DeltaX2AGV + upCameraData.DeltaY2AGV*upCameraData.DeltaY2AGV);
						if((1==UpCamera.Flag_valid_shelf)&&
							(tmpDis < Param.AgvParam[PARAM_UpCamera_Shelf_Offset])&&
							(fabs(tmpH) < Param.AgvParam[PARAM_UpCamera_Shelf_OffsetHeading]))
		                 {//HR,modified,20160123
		                 	if (++sucessConfirmCnt>=5)
		                 	{
								UpCamera.ShelfPositionStatus = 0x02;
								int32 upCameraX =upCameraData.X*1000;//HR,add,20161110
								int32 upCameraY =upCameraData.Y*1000;
								int16 upCameraH =upCameraData.Heading/PI*180.0*100.0;
								ReportShelfCodeEvent(upCameraX,upCameraY,upCameraH,UpCamera.ShelfPositionStatus);
								LogPrintf("success Confirm\n");
								char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
								LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
								break;
		                 	}
		                 	
		                 }
						else
						{
							sucessConfirmCnt = 0;
							if(++errConfirmCnt>=10)
							{
								LogPrintf("err Confirm,dis:%f,DeltaX:%f,DeltaY:%f,h:%f\n",tmpDis,upCameraData.DeltaX2AGV,upCameraData.DeltaY2AGV,tmpH);
								char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
								LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
								if(0 == UpCamera.Flag_valid_shelf)
								{
									SetErrorToQue((ErrorTypeEnum) ERROR_Shelf_Number,ErrorC);
								}
								else
								{
									
									SetErrorToQue((ErrorTypeEnum) ERROR_AfterLift_ShelfBadPosition,ErrorC);
								}
								int32 upCameraX =upCameraData.X*1000;//HR,add,20161110
								int32 upCameraY =upCameraData.Y*1000;
								int16 upCameraH =upCameraData.Heading/PI*180.0*100.0;
								ReportShelfCodeEvent(upCameraX,upCameraY,upCameraH,0);
								
								break;
							}
						}
						
					}
				}
				else if(upCameraData.DataValid<0)
				{
					//没有接收到新数据
				}
				else
				{
					//接收到数据，单数数据无效
					sucessConfirmCnt = 0;
					DataValidCnt = 0;
				}
			
			}
			else
			{
				failConfirmCnt++;
				sucessConfirmCnt = 0;
				if(failConfirmCnt>=150)
				{
					LogPrintf("up mode time out\n");
					char* tmpChar = ByteArrayToString((unsigned char*)Agv.device.dm642.revBuffer,13);
					LogPrintf("%d,%s\n",Agv.device.dm642.uartMonitor,tmpChar);
					SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_UP642_TIME_OUT,WarningA);
					break;
				}
				else if(0 == (failConfirmCnt%50))
				{
					LogPrintf("up mode set mode\n");
					QRCodeSetMode(&(Agv.device.qrcode), QRCODEMODE_UP_AFTER_LIFT);
				}
			}
			ThreadSleep(20);
		}while(1);
	}
	else
	{
		SetErrorToQue((ErrorTypeEnum) ERROR_ReadUpCameraFailed,ErrorC);
	}
	WaitLastCmdComplete();
	//switch down mode 
	rtn |= SwitchToDownMode();
	if(sucessConfirmCnt)
	{
		//求出货架对地正交角度差
		FH_Float shelf2GND_nor,diff_shelf2GND,podheading_nor,diff_podheading2GND,shelf2pod;
		GetNominalHeading(upCameraData.DeltaHeading2GND,&shelf2GND_nor);
		diff_shelf2GND = GetAngleDiff(upCameraData.DeltaHeading2GND, shelf2GND_nor);
		//求出pod对GND正交角度
		GetNominalHeading(Agv.service.tray.Heading2Ground,&podheading_nor);
		diff_podheading2GND = GetAngleDiff(Agv.service.tray.Heading2Ground, podheading_nor);
		shelf2pod = WrapToHalfPiDe(GetAngleDiff(diff_shelf2GND, diff_podheading2GND));//范围 正负 PI/4
		//Pod.Shelf2Pod = WrapToHalfPiDe(Pod.Shelf2Pod + shelf2pod);//范围 正负 PI/4
		FH_Float angleToAGV = 0;
		int shelfDir = 0;
		GetNominalHeading(upCameraData.Heading, &angleToAGV);
		if(angleToAGV == 0 || angleToAGV == PI)
			shelfDir = 1;
		else
			shelfDir = 0;
		SetPod2ShelfAngle(shelf2pod,shelfDir);
	}
	else
	{
		rtn = -1;
	}
	return rtn;
}

//#define INSTRUCTION_BEFOREPUTDOWN_OPEN_UPCAMERA_CHECKSHEL 95
FH_ERROR_CODE DO_INSTRUCTION_BEFOREPUTDOWN_OPEN_UPCAMERA_CHECKSHEL(int* normalParam)
{
	/*******************start 解析参数******************************/
	/*******************end 解析参数******************************/

	/**********start**条件检查****************************/

	if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
		LogPrintf("=ERROR= agv not ready\n");
		return -1;
	}
	if(0 == IsTrayLiftUpState())
	{
		return 0;
	}

	/**********end**条件检查****************************/

	/**********start*指令部署****************************/
	//1看角度误差，，托盘纠正
 	DO_INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF(normalParam);
	WaitLastCmdComplete();
	//2，托盘旋转，执行过检查上视，将货架对齐地面
	FH_Float targetPodHeading=0;
	Uint16 waitCnt = 2000;
	SetAppActionState(AGV_MOTION_POD_ZERO);
	if(1 == UpCamera.ShelfDMRead_after)
	{
		if(fabs(UpCamera.ShelfDM_liftafter - UpCamera.DeltaHeading2GND)>2.0*PI/180)
		{
			SetError(ERROR_Angle_too_large,3);
		}
		GetNominalHeading(UpCamera.ShelfDM_liftafter,&targetPodHeading);
		FH_Float podheadingdiff =GetAngleDiff(targetPodHeading,UpCamera.ShelfDM_liftafter);
        targetPodHeading = podheadingdiff+Agv.service.tray.Heading2Ground;
	}
	else
	{
		SetError(ERROR_ReadUpCameraFailed, 3);
		UpCamera.ShelfDMRead_after = 0;
		return -2;
	}
    UpCamera.ShelfDMRead_after = 0;
	struct TrayTskParams_t trayTskParams;
	GetTrayTskParam(&trayTskParams,targetPodHeading,Tray_RotateToGround);
	TrayRotate_tsk(&trayTskParams);
	WaitMoveDone_tsk(&waitCnt);
	WaitLastCmdComplete();
	//3,看位置误差，底盘纠正
	DO_INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF(normalParam);
	WaitLastCmdComplete();
	//4,底盘纠正
	if(1==UpCamera.ShelfDMRead_after)
	{
//		FH_Float targetX = Agv.service.chassis.location.curPosGnd.x+ROUND_POSITION_UNIT(UpCamera.DeltaX2GND, CODE_GAP_X) - UpCamera.DeltaX2GND;
//		FH_Float targetY = Agv.service.chassis.location.curPosGnd.y+ROUND_POSITION_UNIT(UpCamera.DeltaY2GND, CODE_GAP_Y) - UpCamera.DeltaY2GND;
		FH_Float targetX = Agv.service.chassis.location.curPosGnd.x+RoundAgvToCurQRPos(UpCamera.DeltaX2GND,DirX,1) - UpCamera.DeltaX2GND;
		FH_Float targetY = Agv.service.chassis.location.curPosGnd.y+RoundAgvToCurQRPos(UpCamera.DeltaY2GND,DirY,1) - UpCamera.DeltaY2GND;
		
		GetNominalHeading(UpCamera.ShelfDMRead_after, &targetPodHeading);
		FH_Float temp =GetAngleDiff(UpCamera.ShelfDMRead_after, targetPodHeading);

		double adjustHeading = 0;

		GetNominalHeading(Agv.service.chassis.location.curPosGnd.heading, &adjustHeading);
		FH_Float temp2 = GetAngleDiff(adjustHeading, Agv.service.chassis.location.curPosGnd.heading);

		FH_Float agvTargetHeading = Agv.service.chassis.location.curPosGnd.heading + 2.0*PI + temp2;
		//闭环托盘
		int isCloseLoop = 1;
		TrayCloseLoop_tsk(&isCloseLoop);

		//设置action状态，对应原运动状态
		SetAppActionState(AGV_MOTION_TURNING);
		struct ChassisTskParams_t params;
		IsCustomIDVaild = 0;//取消自定义速度
		GetChissisTskParam(&params,targetX,targetY,agvTargetHeading,Motion_TURN);
		params.pg.maxAcc = Min(params.pg.maxAcc,LoadCondition.MaxAngularAcc[1]);
        params.pg.maxVel = Min(params.pg.maxVel,LoadCondition.MaxAngularSpd[1]);
        params.pg.maxDec = Min(params.pg.maxDec,LoadCondition.MaxAngularAcc[1]);
		ChassisTurning_tsk(&params);
		WaitMoveDone_tsk(&waitCnt);
		//开环托盘
		isCloseLoop = 0;
		TrayCloseLoop_tsk(&isCloseLoop);
	}
	else
	{
		SetError(ERROR_ReadUpCameraFailed, 3);
		UpCamera.ShelfDMRead_after = 0;
		return -3;
	}
	UpCamera.ShelfDMRead_after = 0;

	
	/*BeginDeploy;
	ResetTask();
	DeployAfterLiftGuideToDM();
	EndDeploy;*/
	WaitLastCmdComplete();
	//不设参数，必须降下
	//if(Instruction.TargetX == 1)
	{
		if( Agv.device.motorTrayLift.md.SendMotorEnable(&Agv.device.motorTrayLift.md))
		{
			SetError(ERROR_MotorDisableAbnormal_5,3);
			return -2;
		}
		SetAppActionState(AGV_MOTION_SETDOWN);
		FH_Float target = -(Param.AgvParam[PARAM_LiftDistance]*0.001+0.01);
		GetTrayLiftTskParam(&trayTskParams,target);
		TrayLift_tsk(&trayTskParams);
		WaitMoveDone_tsk(&waitCnt);
		WaitLastCmdComplete();
		Agv.device.motorTrayLift.md.SendMotorDisable(&Agv.device.motorTrayLift.md);
		
	}
 return 0;
}



