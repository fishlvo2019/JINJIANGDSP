/*
 * InstructionBelt.c
 *
 *  Created on: 2018��04��02��
 *      Author: Liu HZ
 */

#include "AgvApp.h"
#include "Basefunction.h"
#include "math.h"
#include "string.h"
FH_ERROR_CODE LoadStateMachiner(struct BeltManager_t* beltmanager);
FH_ERROR_CODE UnLoadStateMachiner(struct BeltManager_t* beltmanager);
FH_ERROR_CODE MoveInstruction(FH_Float targetX,FH_Float targetY,int motionState);
FH_ERROR_CODE TurnInstruction(int16 targetH);

FH_ERROR_CODE DO_INSTRUCTION_BELTMOVE(struct INSTRUCTION_Normal_params_t* normalParam)
{
	int32 tmpx = normalParam->targetX;
	EndianConvert(&tmpx,sizeof(tmpx));
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	BeltMotorEnable(beltmanager->leftBelt);//ʹ�ܵ��
	if(tmpx == 1)
	{
		beltmanager->leftBelt->SpeedFlag = 1;
	}
	else if(tmpx == -1)
	{
		beltmanager->leftBelt->SpeedFlag = -1;
	}
	/*���ʹ�ܼ��*/
	if(0 == IsBeltMotorEnable())
	{
		SetError(ERROR_MotorDisableAbnormal_3,3);
		return 0;
	}
	beltmanager->leftBelt->AllowLoad = 1;
	BeltPreMove(beltmanager->leftBelt);//Ƥ����ʼԤ�˶�
	ThreadSleep(1000);
	beltmanager->leftBelt->AllowLoad = 0;
	BeltMove(beltmanager->leftBelt);
	return 0;
}
FH_ERROR_CODE DO_INSTRUCTION_LOAD(struct INSTRUCTION_Normal_params_t* normalParam)
{
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	/*���ʹ�ܼ��*/
	if(0 == IsBeltMotorEnable())
	{
		SetError(ERROR_MotorDisableAbnormal_3,3);
		return 0;
	}
	if(!IsAppParamsFlagZero())
	{
		SetError(ERROR_SetParam_NoComplete,3);
		return CheckResult_DoNothing;
	}
	if (Param.SetMapData!=1)//HR,add,20160321
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
	LogPrintf("LoadPosition=%d\n",beltmanager->LoadPosition);
	beltmanager->BeltMotionFlag = 1;
	beltmanager->LoadException = NoneException;
	beltmanager->BeltLoadStateMachine = MachineStart;
	beltmanager->LoadPosition = log(fabs(normalParam->PodDirection))/log(2);//����ȷ�� ����λ�� 1~7
	beltmanager->LoadDirection = ((normalParam->targetHeading & 0xFF00)>>8 == 0)?FORWARD:BACKWARD;
	if(0 == IsBeltMotorEnable())
	{
		SetErrorToQue((ErrorTypeEnum) ERROR_MotorDisable_RejectOrder,ErrorC);
		LogPrintf("=ERROR= belt motor disable\n");
		return -1;
	}
	SetAppActionState(AGV_MOTION_LOAD);
	LoadStateMachiner(beltmanager);
	if(Agv.service.beltManager.LoadCancerFlag == 1)
		Agv.service.beltManager.LoadCancerFlag = 0;
	beltmanager->BeltMotionFlag = 0;

	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_UNLOAD(struct INSTRUCTION_Normal_params_t* normalParam)
{
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	/*���ʹ�ܼ��*/
	if(0 == IsBeltMotorEnable())
	{
		SetError(ERROR_MotorDisableAbnormal_3,3);
		return 0;
	}
	if(!IsAppParamsFlagZero())
	{
		SetError(ERROR_SetParam_NoComplete,3);
		return CheckResult_DoNothing;
	}
	if (Param.SetMapData!=1)//HR,add,20160321
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

	beltmanager->BeltMotionFlag = 1;
	beltmanager->LoadException = NoneException;
	beltmanager->BeltUnLoadStateMachine = UNLOAD_MachineStart;
	beltmanager->LoadDirection = ((normalParam->targetHeading & 0xFF00)>>8 == 0)?FORWARD:BACKWARD;
	beltmanager->LoadPosition = log(fabs(normalParam->PodDirection))/log(2);//����ȷ�� ����λ�� 1~7
	if(beltmanager->LoadPosition <= 0 || beltmanager->LoadPosition >7)
	{
		SetErrorToQue(SYSTEM_ERROR_LOAD_INSTRUCTION, WarningA);
		return 0;
	}
	else if(beltmanager->LoadPosition == 7)
	{
		beltmanager->FollowUnloadFlag = 1;
		beltmanager->UnloadTargetBelt = beltmanager->leftBelt;
		beltmanager->UnloadNotTargetBelt = beltmanager->rightBelt;
	}
	if(beltmanager->LoadPosition%2 == 0)
		beltmanager->UnloadTargetBelt = beltmanager->rightBelt;//��������
	else
		beltmanager->UnloadTargetBelt = beltmanager->leftBelt;
	SetAppActionState(AGV_MOTION_UNLOAD);
	UnLoadStateMachiner(beltmanager);
	
	beltmanager->BeltMotionFlag = 0;
	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_MOVING_UNLOAD(struct INSTRUCTION_Normal_params_t* moveParam)
{
	FH_ERROR_CODE rtn = 0;
	SetAppActionState(AGV_MOTION_MOVING_UNLOAD);
	Agv.service.beltManager.BeltMotionFlag = 1;
	Uint16 timeOutCnt = 2000;
	/*******************start ��������******************************/
	int32 tmpx = moveParam->targetX;
	EndianConvert(&tmpx,sizeof(tmpx));
	int32 tmpy = moveParam->targetY;
	EndianConvert(&tmpy,sizeof(tmpy));
//	FH_Float TargetX = ((FH_Float)tmpx)/1000.0*CODE_GAP_X;
//	FH_Float TargetY = ((FH_Float)tmpy)/1000.0*CODE_GAP_Y;
	FH_Float TargetX = SwitchQRCodeToAgv(tmpx,DirX);//((FH_Float)tmpx)/1000.0*CODE_GAP_X;
	FH_Float TargetY = SwitchQRCodeToAgv(tmpy,DirY);//((FH_Float)tmpy)/1000.0*CODE_GAP_Y;
	BeltUnLoadSetTargetDistance(TargetX,TargetY);
	//��ǰ���˶�״̬���Ա���ٶ�
	memset(&movep,0x0,sizeof(struct MoveParamType));
	rtn =  CalcMoveParams(&movep);
	if(rtn)
	{
		LogPrintf("=ERROR= move params wrong\n");
		return rtn ;
	}
	/*******************end ��������******************************/
	/*--------------------------����ж��----------------------------*/
	struct BeltUnloadInfo_t * beltUnload = &Agv.service.beltManager.beltUnloadInfo;
	//������ֵ
	beltUnload->unloadPos = log(fabs(moveParam->PodDirection))/log(2);//����ȷ�� ����λ�� 1~7
	beltUnload->unloadDir = ((moveParam->targetHeading & 0x0F00)>>8 == 0)?0:1;
	beltUnload->moveType = (BeltUnloadMoveType)(((moveParam->targetHeading & 0xF000)>>12) + 1);
	/*���ʹ�ܼ��*/
	if(0 == IsBeltMotorEnable())
	{
		SetError(ERROR_MotorDisableAbnormal_3,3);
		return 0;
	}
	LogPrintf("MOVING_UNLOAD>>%d\n",beltUnload->moveType);
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	beltmanager->LoadSuccessFlag = 0;
	if(beltUnload->moveType == BeltUnload_MOVE_STRAT)
	{
		beltmanager->LoadCheckTargetX = TargetX;
		beltmanager->LoadCheckTargetY = TargetY;
		beltmanager->FullLoadCheck = 1;
#ifdef SingleIO
		beltmanager->TargetBeltIOLaser = (beltUnload->unloadDir==1)?&beltmanager->singleBeltIO->LeftLaserDis:&beltmanager->singleBeltIO->RightLaserDis;
#else
		beltmanager->TargetBeltIOLaser = (beltUnload->unloadDir==1)?&beltmanager->rightBelt->beltio->LaserDis:&beltmanager->leftBelt->beltio->LaserDis;
#endif
	}
	else if(beltUnload->moveType == BeltUnload_MOVE_STOP)
	{
		beltmanager->LoadCheckTargetX = TargetX;
		beltmanager->LoadCheckTargetY = TargetY;
		beltmanager->FullLoadCheck = 2;
#ifdef SingleIO
		beltmanager->TargetBeltIOLaser = (beltUnload->unloadDir==1)?&beltmanager->singleBeltIO->LeftLaserDis:&beltmanager->singleBeltIO->RightLaserDis;
#else
		beltmanager->TargetBeltIOLaser = (beltUnload->unloadDir==1)?&beltmanager->rightBelt->beltio->LaserDis:&beltmanager->leftBelt->beltio->LaserDis;
#endif
	}
	else if(beltUnload->moveType == BeltUnload_TURN_STOP)
	{
		int16 tempH = (moveParam->targetHeading & 0xFF)*90;
		LogPrintf("tempHheading=%d\n",tempH);
		float targetHeading = tempH*PI/180.0;
		FH_Float diffHeading = GetAngleDiff(targetHeading, Agv.service.chassis.location.curPosGnd.heading);
		FH_Float diffnormalheading = 0;
		GetNominalHeading(diffHeading, &diffnormalheading);
		LogPrintf("MoveingUnload---diffHeading=%f\n",diffnormalheading);
		if(fabs(fabs(diffnormalheading)-PI/2)<0.01)
		{
			beltmanager->FullLoadCheckAfterUnload = 0;
			beltmanager->FullLoadCheck = 3;
			beltmanager->StartAngle = Agv.service.chassis.location.curPosGnd.heading;
#ifdef SingleIO
			beltmanager->TargetBeltIOLaser = (beltUnload->unloadDir==1)?&beltmanager->singleBeltIO->LeftLaserDis:&beltmanager->singleBeltIO->RightLaserDis;
#else
			beltmanager->TargetBeltIOLaser = (beltUnload->unloadDir==1)?&beltmanager->rightBelt->beltio->LaserDis:&beltmanager->leftBelt->beltio->LaserDis;
#endif
		}
		else
		{
			beltmanager->FullLoadCheckAfterUnload = 1;
		}
	}

	if(Agv.service.beltManager.ReActiveSensorCnt >= REACTIVESENSORCNT)//5min��֮��
		ReActiveSensor(&Agv.service.beltManager);

	if(BeltUnload_MOVE_STRAT ==beltUnload->moveType)
	{
		//״̬��λ
		beltUnload->isUnload = 1;
		beltUnload->startUnload = 1;
		beltUnload->startMove = 0;
		beltUnload->unloadSPGState = BeltUnload_SPG_Idle;
		//move instruction
		rtn = MoveInstruction(TargetX,TargetY,AGV_MOTION_MOVING_UNLOAD);
	}
	else if(BeltUnload_MOVE_STOP ==beltUnload->moveType)
	{
		//״̬��λ
		beltUnload->isUnload = 1;
		beltUnload->startUnload = 1;
		beltUnload->startMove = 0;
		//move instruction
		rtn = MoveInstruction(TargetX,TargetY,AGV_MOTION_MOVEOVER_UNLOAD);
		WaitLastCmdComplete_fun();
	}
	else if(BeltUnload_TURN_STOP ==beltUnload->moveType)
	{	
		LogPrintf("MoveingUnload---FullLoadCheckAfterUnload=%d\n",beltmanager->FullLoadCheckAfterUnload);
		if(beltmanager->FullLoadCheckAfterUnload == 0)
		{
			//״̬��λ
			beltUnload->isUnload = 1;
			beltUnload->startUnload = 1;
			beltUnload->startMove = 0;
			int16 tempH = (moveParam->targetHeading & 0xFF)*90;
			if(tempH == 270)
			{
				tempH = -90;
			}
			//turn instruction
			LogPrintf("tempHheading=%d\n",tempH);
			rtn = TurnInstruction(tempH);
			timeOutCnt = 3000;//��λ10ms
            WaitMoveDone_tsk(&timeOutCnt);
			WaitLastCmdComplete_fun();
		}
		else if(beltmanager->FullLoadCheckAfterUnload == 1)
		{
			int16 tempH = (moveParam->targetHeading & 0xFF)*90;
			if(tempH == 270)
			{
				tempH = -90;
			}
			int tempHJust = tempH + ((beltUnload->unloadDir==1)?-90:90);
			//turn instruction
			LogPrintf("After1 tempHheading=%d\n",tempHJust);
			rtn = TurnInstruction(tempHJust);
			timeOutCnt = 3000;//��λ10ms
            WaitMoveDone_tsk(&timeOutCnt);
			WaitLastCmdComplete_fun();
			//Ϊ��û����ת?
			//״̬��λ
			beltUnload->isUnload = 1;
			beltUnload->startUnload = 1;
			beltUnload->startMove = 0;
			
			beltmanager->FullLoadCheckAfterUnload = 0;
			beltmanager->FullLoadCheck = 3;
			beltmanager->StartAngle = Agv.service.chassis.location.curPosGnd.heading;
#ifdef SingleIO
			beltmanager->TargetBeltIOLaser = (beltUnload->unloadDir==1)?&beltmanager->singleBeltIO->LeftLaserDis:&beltmanager->singleBeltIO->RightLaserDis;
#else
			beltmanager->TargetBeltIOLaser = (beltUnload->unloadDir==1)?&beltmanager->rightBelt->beltio->LaserDis:&beltmanager->leftBelt->beltio->LaserDis;
#endif
			LogPrintf("After2 tempHheading=%d\n",tempH);
			rtn = TurnInstruction(tempH);
			timeOutCnt = 3000;//��λ10ms
            WaitMoveDone_tsk(&timeOutCnt);
			WaitLastCmdComplete_fun();
		}
	}
	Agv.service.beltManager.FullLoadCheck = 0;
	Agv.service.beltManager.BeltMotionFlag = 0;
	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_CONVEYOR_CONFIRM(struct INSTRUCTION_Normal_params_t* normalParam)
{
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	int i = 0;
	NormalParamsEndianConvert(normalParam);
	SetAppActionState(AGV_MOTION_CONVEYOR_CONFIRM);	
	beltmanager->BeltState = normalParam->PodDirection;
	UpdateBeltState(beltmanager);

	beltmanager->LoadConflict = 0;
	if(beltmanager->BeltType == BIG_BELT)
	{
		for(i=5;i<8;i++)
		{
			if(((beltmanager->BeltState&(0x01<<i))==0) && ((beltmanager->BeltStateSelf&(0x01<<i))!=0))
			{
				if((beltmanager->BeltState&(0x01<<7)) == 0)
				{
					beltmanager->LoadConflict = i;
					beltmanager->BeltInfo.ConflictInfo.Flag = 1;
					beltmanager->BeltInfo.ConflictInfo.Info = beltmanager->LoadConflict;
					break;
				}
			}
		}
		if(i==8)
		{
			beltmanager->LoadConflict = 0;
			beltmanager->BeltInfo.ConflictInfo.Flag = 1;
			beltmanager->BeltInfo.ConflictInfo.Info = 0;
		}
	}
	else if(beltmanager->BeltType == SMALL_BELT)
	{
		if(((beltmanager->BeltState&(0x01<<1))==0) && ((beltmanager->BeltStateSelf&(0x01<<1))!=0))
		{
				beltmanager->LoadConflict = 1;
				beltmanager->BeltInfo.ConflictInfo.Flag = 1;
				beltmanager->BeltInfo.ConflictInfo.Info = beltmanager->LoadConflict;
		}
	}
	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_CONVEYOR_ZERO_OFFSET(struct INSTRUCTION_Normal_params_t* normalParam)
{
#ifndef SingleIO
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	NormalParamsEndianConvert(normalParam);
	SetAppActionState(AGV_MOTION_CONVEYOR_ZERO_OFFSET);	
	struct Belt_t* targetBelt = beltmanager->rightBelt;
	if(normalParam->targetHeading == 243)
	{
		ReActiveSensor(beltmanager);
		return 0;
	}

	if(!IsAppParamsFlagZero())
	{
		SetError(ERROR_SetParam_NoComplete,3);
		return CheckResult_DoNothing;
	}
	if (Param.SetMapData!=1)//HR,add,20160321
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

	if(normalParam->targetHeading == 255)
	{
		beltmanager->CalibrationSide = normalParam->targetX;
		beltmanager->CalibrationDis = (normalParam->targetY==0)?Param.AgvParam[PARAM_Belt_Calibration_Near]:Param.AgvParam[PARAM_Belt_Calibration_Far];
		targetBelt = (beltmanager->CalibrationSide==0)?beltmanager->leftBelt : beltmanager->rightBelt;
		targetBelt->beltio->CalibrationDis = beltmanager->CalibrationDis;
		targetBelt->beltio->CalibrationAction = 0xFF;
		targetBelt->beltio->SensorThreshold = Param.AgvParam[PARAM_BeltIR_Threshold];
		SendBeltIOCmd_s(targetBelt->beltio, Type_WriteCalibration);
	}
	else if(normalParam->targetHeading == 244)
	{
		beltmanager->CalibrationSide = normalParam->targetX;
		targetBelt = (beltmanager->CalibrationSide==0)?beltmanager->leftBelt : beltmanager->rightBelt;
		targetBelt->beltio->CalibrationDis = 0xFF;
		targetBelt->beltio->CalibrationAction = 0xFF;
		targetBelt->beltio->SensorThreshold = Param.AgvParam[PARAM_BeltIR_Threshold];
		SendBeltIOCmd_s(targetBelt->beltio, Type_WriteCalibration);
	}
	else if(normalParam->targetHeading == 0 || normalParam->targetHeading == 1)
	{
		beltmanager->CalibrationSide = normalParam->targetX;
		targetBelt = (beltmanager->CalibrationSide==0)?beltmanager->leftBelt : beltmanager->rightBelt;
		targetBelt->beltio->CalibrationDis = 255;
		targetBelt->beltio->CalibrationAction = normalParam->PodDirection;
		targetBelt->beltio->SensorThreshold = 255;
		SendBeltIOCmd_s(targetBelt->beltio, Type_WriteCalibration);
	}
#endif
	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_CONVEYOR_BACK(struct INSTRUCTION_Normal_params_t* normalParam)
{
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	NormalParamsEndianConvert(normalParam);
	SetAppActionState(AGV_MOTION_CONVEYOR_BACK);
	beltmanager->BeltMotionFlag = 1;

	struct Belt_t* targetBelt = (normalParam->targetX==0)?beltmanager->leftBelt:beltmanager->rightBelt;
	struct BeltParams_t* targetBeltParam = targetBelt->base.params;
	targetBelt->LoadDirection = (normalParam->targetY==0)?FORWARD:BACKWARD;
	targetBelt->SpeedFlag = (int)targetBelt->LoadDirection * targetBeltParam->beltSpeedRatio;
	targetBelt->SpeedOutput = targetBelt->SpeedFlag * targetBeltParam->loadSpeed;
	ThreadSleep(2000);
	targetBelt->SpeedFlag = 0;
	targetBelt->SpeedOutput = targetBelt->SpeedFlag * targetBeltParam->loadSpeed;
	beltmanager->BeltMotionFlag = 0;
	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_RESET_LOAD(struct INSTRUCTION_Normal_params_t* normalParam)
{
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	NormalParamsEndianConvert(normalParam);
	
	SetAppActionState(AGV_MOTION_RESET_LOAD);
	beltmanager->BeltState = normalParam->PodDirection;
//	service->BeltStateSelf = service->BeltState;
	UpdateBeltState(beltmanager);

	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_CONVEYOR_CANCEL(struct INSTRUCTION_Normal_params_t* normalParam)
{
//	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	NormalParamsEndianConvert(normalParam);
	if(normalParam->targetX == INSTRUCTION_CONVEYOR_LOAD && Agv.service.beltManager.BeltMotionFlag == 1)
	{
		Agv.service.beltManager.LoadCancerFlag = 1;
	}

	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_FULL_LOAD_Check(struct INSTRUCTION_Normal_params_t* moveParam)
{
	struct BeltManager_t* beltmanager = &Agv.service.beltManager;
	if(beltmanager->BeltType != SMALL_BELT)
	{
		beltmanager->IfFullLoad = 0;
		beltmanager->BeltInfo.FullBoxInfo.Flag = 1;
		beltmanager->BeltInfo.FullBoxInfo.Info = 0;
		return 0;
	}

#ifdef SingleIO
	if(moveParam->PodDirection == 1)
		beltmanager->TargetBeltIOLaser = &Agv.device.singleBeltIO.LeftLaserDis;
	else
		beltmanager->TargetBeltIOLaser = &Agv.device.singleBeltIO.RightLaserDis;
#else
	if(moveParam->PodDirection == 0)
		beltmanager->TargetBeltIOLaser = &Agv.device.leftBeltIO.LaserDis;
	else
		beltmanager->TargetBeltIOLaser = &Agv.device.rightBeltIO.LaserDis;
#endif

	beltmanager->LoadCheckTargetX = Agv.service.chassis.location.curPosGnd.x;
	beltmanager->LoadCheckTargetY = Agv.service.chassis.location.curPosGnd.y;
	
	moveParam->targetHeading = 0;
	moveParam->PodDirection = 0;
	FH_ERROR_CODE re = DO_INSTRUCTION_MOVE(moveParam);
	beltmanager->FullLoadCheck = 1;
	WaitLastCmdComplete();
	beltmanager->FullLoadCheck = 0;
	return 0;
}
//����Ͷ��̨����λ��
FH_ERROR_CODE DO_INSTRUCTION_SET_LOAD_POS(struct INSTRUCTION_Normal_params_t* moveParam)
{
    /*******************start ��������******************************/
     int32 tmpx = moveParam->targetX;
     EndianConvert(&tmpx,sizeof(tmpx));
     int32 tmpy = moveParam->targetY;
     EndianConvert(&tmpy,sizeof(tmpy));
	 LogPrintf("targetX=%d,targetY=%d\n",(int16)tmpx,(int16)tmpy);
     double tempX = SwitchQRCodeToAgv(tmpx*1000,DirX);//((FH_Float)tmpx)/1000.0*CODE_GAP_X;
     double tempY = SwitchQRCodeToAgv(tmpy*1000,DirY);//((FH_Float)tmpy)/1000.0*CODE_GAP_Y;
     SetMapPoint(tempX,tempY);
	 return 0;
}

//
FH_ERROR_CODE LoadStateMachiner(struct BeltManager_t* beltmanager)
{
	int runFlag = 1;
	Load_Exception tempException = NoneException;
//	struct Belt_t *belt = &Agv.service.leftBelt;
	if(beltmanager->BeltLoadStateMachine != MachineStart)
		beltmanager->BeltLoadStateMachine = Exception;

	while(runFlag)
	{
		if(Agv.service.safeManager.ErrorState >= ErrorC && beltmanager->BeltLoadStateMachine != Exception)
		{
			LogPrintf("Load SystemError...\n");
			beltmanager->BeltLoadStateMachine = Exception;
			beltmanager->LoadException = NoneException;
			continue;
		}
		if(Agv.service.beltManager.LoadCancerFlag == 1)
		{
			LogPrintf("LoadCancelled...\n");
			beltmanager->BeltLoadStateMachine = Exception;
			beltmanager->LoadException = LoadCancelled;
		}
		switch(beltmanager->BeltLoadStateMachine)
		{
		case BeltTaskFree:
			beltmanager->LoadException = NoneTask;
			beltmanager->BeltLoadStateMachine = Exception;
			break;
		case MachineStart://1. ״̬����ʼ
			if(SMALL_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
			{
//				if((ACTIVE == belt->BeltLocationState.LocationOne.SensorFlag)
//				||(ACTIVE == belt->BeltLocationState.LocationTwo.SensorFlag)
//				||(ACTIVE == belt->BeltLocationState.LocationThree.SensorFlag))
//				{
//					beltmanager->LoadException = Conflict;
//					beltmanager->BeltLoadStateMachine = Exception;
//					SetErrorToQue(SYSTEM_ERROR_BELT_Load_confligct,ErrorC);
//				}
//				else
//				{
					beltmanager->LoadSuccessFlag = 0;
					if(beltmanager->ReActiveSensorCnt >= REACTIVESENSORCNT)//5min��֮��
					{
						if(ReActiveSensor(beltmanager) != 0)
						//if(0)
						{
							beltmanager->LoadException = SenserError;
							beltmanager->BeltLoadStateMachine = Exception;
						}
						else
						{
							beltmanager->LoadException = NoneException;
							beltmanager->BeltLoadStateMachine = ParseInstruction;
						}
					}
					else
					{
						beltmanager->LoadException = NoneException;
						beltmanager->BeltLoadStateMachine = ParseInstruction;
					}
//				}
			}
			else
			{
				if(beltmanager->ReActiveSensorCnt >= REACTIVESENSORCNT)//5min��֮��
				{
					if(ReActiveSensor(beltmanager) != 0)
					{
						beltmanager->LoadException = SenserError;
						beltmanager->BeltLoadStateMachine = Exception;
					}
					else
					{
						beltmanager->LoadException = NoneException;
						beltmanager->BeltLoadStateMachine = ParseInstruction;
					}
				}
				else
				{
					beltmanager->LoadException = NoneException;
					beltmanager->BeltLoadStateMachine = ParseInstruction;
				}
			}
			break;
		case ParseInstruction://2. ����ָ���ȡ��������ȡ���Ϸ���
			//ParseLoadInstruction(service, normalParam); //�ŵ�LoadStateMachiner����ִ��
			beltmanager->LoadException = NoneException;
			beltmanager->BeltLoadStateMachine = PreCheck;
			break;
		case PreCheck://3. �ж�ָ���Ƿ��ִ�У����Ϸ�������Ƿ��п�λ����Ƥ��������������λ�Ƿ���У�СƤ����
					  //   ��ʱֻ֧��СƤ���������ϡ���Ƥ��������һ���ϣ���֧�ִ�Ƥ���ϴ��Ϻ͵�����������
					  //   ��ʱû��Ƥ������λ�û���״̬ά������������λ������ά����������������λ������ά��
					  //   ��ʱȱ�ٻ���FullIn��LoadFinish�������ж�
			tempException = LoadConditionPreCheck(beltmanager);//����ǰ���������ж��趨λ�á������ܷ����ϣ��ܷ�Ԥ�˶�����Ҫ���Ǵ�СƤ��������λ�á����Ϸ������ϴ�����
			if(tempException == NoneException)
			{
				beltmanager->LoadException = NoneException;
				beltmanager->BeltLoadStateMachine = PreMove;
			}
			else
			{
				
				beltmanager->LoadException = tempException;
				beltmanager->BeltLoadStateMachine = Exception;
			}
			break;
		case PreMove://4. ʹ�ܵ�������õ���ٶȣ���ʼԤ�˶�
			BeltMotorEnable(beltmanager->leftBelt);//ʹ�ܵ��
			BeltPreMove(beltmanager->leftBelt);//Ƥ����ʼԤ�˶�
			if(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
			{
				BeltMotorEnable(beltmanager->rightBelt);//ʹ�ܵ��
				BeltPreMove(beltmanager->rightBelt);//Ƥ����ʼԤ�˶�
			}
			beltmanager->BeltLoadStateMachine = WaittingStartTrigger;
			break;
		case WaittingStartTrigger://4. �ȴ����ϴ����źţ�ѡ���˶�Ƥ����������������PreCheck����
								  //����
								  //��ʱʱ�����
			tempException = LoadBeltChoice(beltmanager,10000);
			if(tempException == NoneException)
			{
				beltmanager->LoadException = NoneException;
				beltmanager->BeltLoadStateMachine = MoveCheck;
			}
			else if(tempException == TimeOut)
			{
				struct Belt_t* leftbelt = beltmanager->leftBelt;
				leftbelt->AllowLoad = 0;
				beltmanager->LoadException = NoneException;
				beltmanager->BeltLoadStateMachine = LoadSuccess;
			}
			else
			{
				beltmanager->LoadException = tempException;
				beltmanager->BeltLoadStateMachine = Exception;
			}
			
			break;
		case MoveCheck://5. ѡ��Ƥ���󣬼����ѡƤ���Ƿ��������
					   //	���Ϸ�����治���ж���
			if((beltmanager->LoadChosenBelt->LoadPositionOne != Full &&
				beltmanager->LoadChosenBelt->LoadPositionMiddle != Full &&
				beltmanager->LoadDirection == BACKWARD) ||
			   (beltmanager->LoadChosenBelt->LoadPositionTwo != Full &&
				beltmanager->LoadChosenBelt->LoadPositionMiddle != Full &&
				beltmanager->LoadDirection == FORWARD))
				beltmanager->BeltLoadStateMachine = Move;
			else
			{
				beltmanager->LoadException = Conflict;
				beltmanager->BeltLoadStateMachine = Exception;
			}
			
			LogPrintf("LoadState:State=%d,Exception=%d\n", beltmanager->BeltLoadStateMachine,beltmanager->LoadException);
			break;
		case Move://6. Ƥ�������˶�����ʼ����
			BeltMove(beltmanager->leftBelt);//Ƥ����ʼ�˶� �ٶȺͷ�������LoadBeltChoice���ú�
			if(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
				BeltMove(beltmanager->rightBelt);//Ƥ����ʼ�˶� �ٶȺͷ�������LoadBeltChoice���ú�
			beltmanager->BeltLoadStateMachine = LoadFullInCheck;
			break;
		case LoadFullInCheck://7. �ȴ�������ȫ����Ƥ�����ɹ�ʧ����������PreCheck����
			tempException = LoadInCheck(beltmanager);
			if(tempException == NoneException)
			{
				beltmanager->LoadException = NoneException;
				beltmanager->BeltLoadStateMachine = WaitLoadFinish;
			}
			else if(tempException == TimeOut)
			{
				struct Belt_t* leftbelt = beltmanager->leftBelt;
				leftbelt->AllowLoad = 0;
				beltmanager->LoadException = NoneException;
				beltmanager->BeltLoadStateMachine = WaitLoadFinish;
			}
			else
			{
				beltmanager->LoadException = tempException;
				beltmanager->BeltLoadStateMachine = Exception;
			}
			break;
		case WaitLoadFinish://8. �ȴ����ﵽ��ֹͣ�㣬�ɹ�ʧ����������PreCheck����
			tempException = LoadFinishCheck(beltmanager);
			if(tempException == NoneException)
			{
//				if(((NAGTIVE == belt->BeltLocationState.LocationOne.SensorFlag)
//				&&(NAGTIVE == belt->BeltLocationState.LocationTwo.SensorFlag)
//				&&(NAGTIVE == belt->BeltLocationState.LocationThree.SensorFlag))
//				&&(SMALL_BELT == AgvParams.serviceParams.leftBeltParams.beltType))
//				{
//					beltmanager->LoadException = LoadOutofStock;
//					beltmanager->BeltLoadStateMachine = Exception;
//				}
//				else
//				{
//					beltmanager->LoadSuccessFlag = 1;
					beltmanager->LoadException = NoneException;
					beltmanager->BeltLoadStateMachine = LoadSuccess;
//				}
				
			}
			else if(tempException == TimeOut)
			{
				struct Belt_t* leftbelt = beltmanager->leftBelt;
				leftbelt->AllowLoad = 0;
				beltmanager->LoadException = NoneException;
				beltmanager->BeltLoadStateMachine = LoadSuccess;
			}
			else
			{
				beltmanager->LoadException = tempException;
				beltmanager->BeltLoadStateMachine = Exception;
			}
			
			break;
		//case AfterProcess: break;//�Ȳ���
		//case StopMove: break;//�Ȳ���
		case LoadSuccess://״̬������1. ���һ�����˳��ȵ���ٶ���0���رյ��
			{
				int temp = 0x00;
				LogPrintf("Load Success...\n");
				beltmanager->FollowLoadFlag = 0;
				beltmanager->leftBelt->SpeedFlag = 0;
				beltmanager->rightBelt->SpeedFlag = 0;

				if(beltmanager->leftBelt->AllowLoad == 1)
					temp |= 0x01;
				if(beltmanager->rightBelt->AllowLoad == 1)
					temp |= 0x02;
				LogPrintf("Load Temp %d...\n", temp);
				beltmanager->leftBelt->AllowLoad = 0;
				beltmanager->rightBelt->AllowLoad = 0;
				BeltMove(beltmanager->leftBelt);
				if(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
					BeltMove(beltmanager->rightBelt);
				beltmanager->BeltLoadStateMachine = BeltTaskFree;
				beltmanager->LoadException = NoneException;
				//Final Check
				if(beltmanager->BeltType == BIG_BELT)
				{
					if(beltmanager->LoadChosenBelt->BeltLocationState.LocationOne.SensorFlag == NAGTIVE &&
					   beltmanager->LoadChosenBelt->BeltLocationState.LocationFive.SensorFlag == NAGTIVE &&
					  (beltmanager->LoadChosenBelt->BeltLocationState.LocationTwo.SensorFlag == ACTIVE ||
					   beltmanager->LoadChosenBelt->BeltLocationState.LocationThree.SensorFlag == ACTIVE ||
					   beltmanager->LoadChosenBelt->BeltLocationState.LocationFour.SensorFlag == ACTIVE))
					{ }
					else
					{
						LogPrintf("Load Still Run...\n");
						ThreadSleep(1000);
						if(beltmanager->LoadChosenBelt->BeltLocationState.LocationOne.SensorFlag == NAGTIVE &&
						   beltmanager->LoadChosenBelt->BeltLocationState.LocationFive.SensorFlag == NAGTIVE &&
						  (beltmanager->LoadChosenBelt->BeltLocationState.LocationTwo.SensorFlag == ACTIVE ||
						   beltmanager->LoadChosenBelt->BeltLocationState.LocationThree.SensorFlag == ACTIVE ||
						   beltmanager->LoadChosenBelt->BeltLocationState.LocationFour.SensorFlag == ACTIVE))
						{
							//����
						}
						else
						{
							//����
							LogPrintf("Load Error...\n");
							SetErrorToQue(SYSTEM_ERROR_BELT_LOAD_ERROR, ErrorC);
							beltmanager->BeltInfo.LoadInfo.Flag = 1;
							beltmanager->BeltInfo.LoadInfo.Info = 0;
							runFlag = 0;
							break;
						}
					}
				}
				if(beltmanager->BeltType == BIG_BELT)
				{
					if(temp == 0x01)
					{
						beltmanager->BeltInfo.LoadInfo.Flag = 1;
						beltmanager->BeltInfo.LoadInfo.Info = 5;
					}
					else if(temp == 0x02)
					{
						beltmanager->BeltInfo.LoadInfo.Flag = 1;
						beltmanager->BeltInfo.LoadInfo.Info = 6;
					}
					else if(temp == 0x03)
					{
						beltmanager->BeltInfo.LoadInfo.Flag = 1;
						beltmanager->BeltInfo.LoadInfo.Info = 7;
					}
					else
					{
						beltmanager->BeltInfo.LoadInfo.Flag = 1;
						beltmanager->BeltInfo.LoadInfo.Info = 0;
						LogPrintf("Load Info Error...\n");
						SetErrorToQue(SYSTEM_ERROR_UNKNOWNERROR, ErrorC);
					}
				}
				else if(beltmanager->BeltType == SMALL_BELT)
				{
					if(temp == 0x01)
					{
						beltmanager->BeltInfo.LoadInfo.Flag = 1;
						beltmanager->BeltInfo.LoadInfo.Info = 1;
					}
					else
					{
						beltmanager->BeltInfo.LoadInfo.Flag = 1;
						beltmanager->BeltInfo.LoadInfo.Info = 0;
						LogPrintf("Load Info Error...\n");
						//SetErrorToQue(SYSTEM_ERROR_UNKNOWNERROR, ErrorC);
					}
				}
				runFlag = 0;

			}
			break;
		case Exception://״̬������2. ���һ�����˳��ȵ���ٶ���0���رյ��
#ifdef YiKe
			beltmanager->leftBelt->SpeedOutput = 0;
			beltmanager->rightBelt->SpeedOutput = 0;
#else
			if(Agv.service.safeManager.ErrorState < ErrorC && beltmanager->LoadException == LoadTooLong)
			{
				LogPrintf("Load Too Long...\n");
				beltmanager->leftBelt->SpeedOutput = 0;
				beltmanager->rightBelt->SpeedOutput = 0;
				ThreadSleep(1000);
				beltmanager->leftBelt->SpeedFlag = -beltmanager->leftBelt->SpeedFlag;
				beltmanager->rightBelt->SpeedFlag = -beltmanager->rightBelt->SpeedFlag;
				BeltPreMove(beltmanager->leftBelt);
				if(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
					BeltPreMove(beltmanager->rightBelt);
				int cnt = 200;
				while(cnt-- && !WaitForSensor(beltmanager->LoadChosenBelt, beltmanager->LoadFinishFailure, 0));
				if(cnt >= 0)
				{
					cnt = 200;
					while(cnt--)
					{
						ThreadSleep(10);
						if((beltmanager->LoadChosenBelt->BeltLocationState.LocationThree.SensorFlag == NAGTIVE && beltmanager->BeltType == BIG_BELT) ||
						   (beltmanager->LoadChosenBelt->BeltLocationState.LocationTwo.SensorFlag == NAGTIVE && beltmanager->BeltType == SMALL_BELT))
							continue;
						else
							break;
					}
					beltmanager->LoadException = NoneException;
					beltmanager->BeltLoadStateMachine = LoadSuccess;
					break;
				}
			}
#endif
			beltmanager->FollowLoadFlag = 0;
	 		beltmanager->leftBelt->SpeedFlag = 0;
			beltmanager->rightBelt->SpeedFlag = 0;
			beltmanager->leftBelt->AllowLoad = 0;
			beltmanager->rightBelt->AllowLoad = 0;
			BeltMove(beltmanager->leftBelt);
			if(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
				BeltMove(beltmanager->rightBelt);
			beltmanager->BeltLoadStateMachine = BeltTaskFree;
			beltmanager->LoadException = beltmanager->LoadException;
			runFlag = 0;
			break;
		default:
			beltmanager->BeltLoadStateMachine = Exception;
			break;
		}
		LogPrintf("LoadState:State=%d,Exception=%d\n", beltmanager->BeltLoadStateMachine,beltmanager->LoadException);
		ThreadSleep(10);
	}
	LogPrintf("Load Info Before Update: %d  %d...\n", beltmanager->BeltInfo.LoadInfo.Flag, beltmanager->BeltInfo.LoadInfo.Info);
	UpdateBeltException(beltmanager);
	LogPrintf("Load Info After Update: %d  %d...\n", beltmanager->BeltInfo.LoadInfo.Flag, beltmanager->BeltInfo.LoadInfo.Info);
	CheckIfLoadInfoValid(&beltmanager->BeltInfo.LoadInfo);
	return 0;
}

FH_ERROR_CODE UnLoadStateMachiner(struct BeltManager_t* beltmanager)
{
	int runFlag = 1;
 	Load_Exception tempException = NoneException;

	beltmanager->LoadChosenBelt = beltmanager->UnloadTargetBelt;
	if(beltmanager->BeltUnLoadStateMachine != UNLOAD_MachineStart)
		beltmanager->BeltUnLoadStateMachine = UNLOAD_Exception;

	while(runFlag)
	{
		if(Agv.service.safeManager.ErrorState >= ErrorC && beltmanager->BeltUnLoadStateMachine != UNLOAD_Exception)
		{
			beltmanager->BeltUnLoadStateMachine = UNLOAD_Exception;
			beltmanager->LoadException = SystemError;
		}

		switch(beltmanager->BeltUnLoadStateMachine)
		{
		case UNLOAD_BeltTaskFree:
			beltmanager->LoadException = NoneTask;
			beltmanager->BeltUnLoadStateMachine = UNLOAD_Exception;
			break;
		case UNLOAD_MachineStart://1. ״̬����ʼ
			if(beltmanager->ReActiveSensorCnt >= REACTIVESENSORCNT)//5min��֮��
			{
				if(ReActiveSensor(beltmanager) != 0)
				//if(0)
				{
					beltmanager->LoadException = SenserError;
					beltmanager->BeltUnLoadStateMachine = UNLOAD_Exception;
				}
				else
				{
					beltmanager->LoadException = NoneException;
					beltmanager->BeltUnLoadStateMachine = UNLOAD_ParseInstruction;
				}
			}
			else
			{
				beltmanager->LoadException = NoneException;
				beltmanager->BeltUnLoadStateMachine = UNLOAD_ParseInstruction;
			}
			break;
		case UNLOAD_ParseInstruction://2. ����ָ���ȡ��������ȡ���Ϸ���
			beltmanager->LoadException = NoneException;
			beltmanager->BeltUnLoadStateMachine = UNLOAD_PreCheck;
			break;
		case UNLOAD_PreCheck://3. �ж�ָ���Ƿ��ִ�У����Ϸ�������Ƿ��п�λ����Ƥ��������������λ�Ƿ���У�СƤ����
							 //   ��ʱֻ֧��СƤ���������ϡ���Ƥ��������һ���ϣ���֧�ִ�Ƥ���´��Ϻ͵�����������
							 //   ��ʱû��Ƥ������λ�û���״̬ά������������λ������ά����������������λ������ά��
			tempException = UnLoadConditionPreCheck(beltmanager);//����ǰ���������ж��趨λ�á������ܷ����ϣ��ܷ�Ԥ�˶�����Ҫ���Ǵ�СƤ��������λ�á����Ϸ������ϴ�����
			if(tempException == NoneException)
			{
				beltmanager->LoadException = NoneException;
				beltmanager->BeltUnLoadStateMachine = UNLOAD_Move;
			}
			else
			{
				beltmanager->LoadException = tempException;
				beltmanager->BeltUnLoadStateMachine = UNLOAD_Exception;
			}
			break;
		case UNLOAD_Move://4. Ƥ�������˶�����ʼ����
			//���ϳɹ��󽫼�ػ����־λ��0
			beltmanager->LoadSuccessFlag = 0;
			BeltMove(beltmanager->leftBelt);//Ƥ����ʼ�˶� �ٶȺͷ�������LoadBeltChoice���ú�
			if(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
				BeltMove(beltmanager->rightBelt);//Ƥ����ʼ�˶� �ٶȺͷ�������LoadBeltChoice���ú�
			beltmanager->BeltUnLoadStateMachine = WaitUnLoadFinish;
			break;
		case WaitUnLoadFinish://5. �ȴ��������ϳɹ����ɹ�ʧ����������PreCheck����
			tempException = UnLoadFinishCheck(beltmanager);
			if(tempException == NoneException)
			{
				beltmanager->LoadException = NoneException;
				beltmanager->BeltUnLoadStateMachine = UnLoadSuccess;
			}
			else
			{
				beltmanager->LoadException = tempException;
				beltmanager->BeltUnLoadStateMachine = UNLOAD_Exception;
			}
			break;
		//case UNLOAD_AfterProcess: break;//�Ȳ���
		//case UNLOAD_StopMove: break;//�Ȳ���
		case UnLoadSuccess://״̬������1. ���һ�����˳��ȵ���ٶ���0���رյ��
			{
				
				/*-----------------------------------*/
				beltmanager->FollowUnloadFlag = 0;
				beltmanager->leftBelt->SpeedFlag = 0;
				beltmanager->rightBelt->SpeedFlag = 0;
				int temp = 0x00;
				if(beltmanager->leftBelt->AllowLoad == 1)
					temp |= 0x01;
				if(beltmanager->rightBelt->AllowLoad == 1)
					temp |= 0x02;
				beltmanager->leftBelt->AllowLoad = 0;
				beltmanager->rightBelt->AllowLoad = 0;
				BeltMove(beltmanager->leftBelt);
				if(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
					BeltMove(beltmanager->rightBelt);
				beltmanager->BeltUnLoadStateMachine = UNLOAD_BeltTaskFree;
				beltmanager->LoadException = NoneException;


				if(beltmanager->BeltType == BIG_BELT)
				{
					if(temp == 0x01)
					{
						beltmanager->BeltInfo.UnloadInfo.Flag = 1;
						beltmanager->BeltInfo.UnloadInfo.Info = 5;
					}
					else if(temp == 0x02)
					{
						beltmanager->BeltInfo.UnloadInfo.Flag = 1;
						beltmanager->BeltInfo.UnloadInfo.Info = 6;
					}
					else if(temp == 0x03)
					{
						beltmanager->BeltInfo.UnloadInfo.Flag = 1;
						beltmanager->BeltInfo.UnloadInfo.Info = 7;
					}
				}
				else if(beltmanager->BeltType == SMALL_BELT)
				{
					if(temp == 0x01)
					{
						beltmanager->BeltInfo.UnloadInfo.Flag = 1;
						beltmanager->BeltInfo.UnloadInfo.Info = 1;
					}
				}
				runFlag = 0;
			}
			break;
		case UNLOAD_Exception://״̬������2. ���һ�����˳��ȵ���ٶ���0���رյ��		
			if(beltmanager->BeltType == BIG_BELT)
			{
				if(beltmanager->FollowUnloadFlag == 1)
				{
					if((BeltCleanState(beltmanager->leftBelt) || BeltCleanState(beltmanager->rightBelt)) == 0)
					{
						beltmanager->LoadException = NoneException;
						beltmanager->BeltUnLoadStateMachine = UnLoadSuccess;
						break;
					}
				}
				else
				{
					if(BeltCleanState(beltmanager->UnloadTargetBelt) == 0)
					{
						beltmanager->LoadException = NoneException;
						beltmanager->BeltUnLoadStateMachine = UnLoadSuccess;
						break;
					}
				}
			}
			else if(beltmanager->BeltType == SMALL_BELT)
			{
				if(BeltCleanState(beltmanager->leftBelt) == 0)
				{
					beltmanager->LoadException = NoneException;
					beltmanager->BeltUnLoadStateMachine = UnLoadSuccess;
					break;
				}	
			}

			beltmanager->FollowUnloadFlag = 0;
			beltmanager->leftBelt->SpeedFlag = 0;
			beltmanager->rightBelt->SpeedFlag = 0;
			beltmanager->leftBelt->AllowLoad = 0;
			beltmanager->rightBelt->AllowLoad = 0;
			BeltMove(beltmanager->leftBelt);
			if(BIG_BELT == AgvParams.serviceParams.leftBeltParams.beltType)
				BeltMove(beltmanager->rightBelt);
			beltmanager->BeltUnLoadStateMachine = UNLOAD_BeltTaskFree;
			beltmanager->LoadException = beltmanager->LoadException;
			beltmanager->BeltInfo.UnloadInfo.Flag = 1;
			runFlag = 0;
			break;
		default:
			beltmanager->BeltUnLoadStateMachine = UNLOAD_Exception;
			break;
		}
		LogPrintf("UnLoadState:State%d,Exception=%d\n", beltmanager->BeltUnLoadStateMachine,beltmanager->LoadException);
		ThreadSleep(10);
	}
	LogPrintf("UnLoad Info Before Update: %d  %d...\n", beltmanager->BeltInfo.UnloadInfo.Flag, beltmanager->BeltInfo.UnloadInfo.Info);
	UpdateBeltException(beltmanager);
	LogPrintf("UnLoad Info After Update: %d  %d...\n", beltmanager->BeltInfo.UnloadInfo.Flag, beltmanager->BeltInfo.UnloadInfo.Info);
	
	CheckIfLoadInfoValid(&beltmanager->BeltInfo.UnloadInfo);
	return 0;
}

FH_ERROR_CODE ParseLoadInstruction(struct BeltManager_t* beltmanager, struct INSTRUCTION_Normal_params_t* normalParam)
{
	//Load_Exception rtn = NoneException;
	beltmanager->LoadPosition = 2;//����ȷ�� ����λ�� 1~7
	beltmanager->LoadDirection = (normalParam->targetX==0)?FORWARD:BACKWARD;
	return 0;
}

FH_ERROR_CODE MoveInstruction(FH_Float targetX,FH_Float targetY,int motionState)
{	
	FH_Float TargetX = targetX;
	FH_Float TargetY = targetY;
	/**********start**�������****************************/
	//�˶�Ŀ��λ��Ϊ��ǰλ��ֱ�ӷ��سɹ�
	if ((fabs(TargetX - Agv.service.chassis.location.curPosGnd.x)<(0.1))&&
		 (fabs(TargetY - Agv.service.chassis.location.curPosGnd.y)<(0.1)))
	{
		return 0;
	}
	//����ֱ��׷�ӣ�����һ���������ִ��
	int isOneLineMove = IsTargetInFront(TargetX,TargetY);
	if(0 == isOneLineMove)
	{
		WaitLastCmdComplete();
	}
	//�жϵ�ǰ�״�̽�ⷶΧ�Ƿ����Ҫ��HR��20180305
	if (1==CheckLaserLength())
	{
		SetErrorToQue(SYSTEM_ERROR_Laser_BarrierDetectPara,ErrorC);
	}
	if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
		LogPrintf("=ERROR= agv not ready\n");
		return -1;
	}
	if (IsChargePoint(TargetX,TargetY))
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_MoveTargetPointInChargeZone,ErrorC);
		return -1;
	}
	if(AGV.DebugMode == 1)
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_DEBUGMODE_RejectRunOrders,ErrorC);
		return -3;
	}
	if(IsOutOfBoard(TargetX,TargetY))
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_TargetOutBorder,ErrorC);
		return -4;
	}
	/**********end**�������****************************/
	
	/**********start*ָ���****************************/
	Uint16 timeOutCnt = 2000;
	struct ChassisTskParams_t chassisTskparams;
	//�趨��������ͣ�������������Ƥ�����ϲ���:30cm+�����
	float32 keepPaceDis =Param.AgvParam[PARAM_CODE_GAP_X]>Param.AgvParam[PARAM_CODE_GAP_Y]? Param.AgvParam[PARAM_CODE_GAP_X]:Param.AgvParam[PARAM_CODE_GAP_Y];
	keepPaceDis =keepPaceDis > Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis] ? keepPaceDis:Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis];
	if(IsTargetInFront(TargetX,TargetY))
	{
		struct Chassis_t * chassis = &Agv.service.chassis;
    	FH_Float targetDis = PGGetTargetDistance(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg);
		FH_Float xDis = fabs(TargetX - chassis->location.curPosGnd.x);
		FH_Float yDis = fabs(TargetY - chassis->location.curPosGnd.y);
		FH_Float MoveDistance = Max(xDis,yDis);
		
		if(targetDis>MoveDistance)
		{
			LogPrintf("small target\n");
			SetErrorToQue((ErrorTypeEnum)APP_ERROR_INSTRUCTION_Move_Target_Small,WarningC);
			return 0;
		}
		else if((MoveDistance-targetDis)<0.1)
		{
			LogPrintf("equal target\n");
			SetErrorToQue((ErrorTypeEnum)APP_ERROR_INSTRUCTION_Move_Target_Equal,WarningC);
			return 0;
		}
		//�жϵ�ǰĿ����Ƿ�����һ��Ŀ���ǰ��
		FH_Float targetheading = 0.0;
		FH_Float nominaltargetx = RoundAgvToCurQRPos(TargetX,DirX,1);//ROUND_POSITION_UNIT(TargetX,CODE_GAP_X);
		FH_Float nominaltargety = RoundAgvToCurQRPos(TargetY,DirY,1);//ROUND_POSITION_UNIT(TargetY,CODE_GAP_Y);
		FH_Float nominaltargetx_pre =RoundAgvToCurQRPos(chassis->TargetX,DirX,1);//ROUND_POSITION_UNIT(chassis->TargetX,CODE_GAP_X);
		FH_Float nominaltargety_pre =RoundAgvToCurQRPos(chassis->TargetY,DirY,1);//ROUND_POSITION_UNIT(chassis->TargetY,CODE_GAP_Y);
		if (0 == GetNominalHeading(chassis->location.curPosGnd.heading,&targetheading))
		{
		   	SetError(ERROR_MoveDirection,3);
		   	return 0;
		}	
		if(((nominaltargetx_pre - nominaltargetx)*cos(targetheading)>0)
		||((nominaltargety_pre - nominaltargety)*sin(targetheading)>0))
		{
		    SetError(ERROR_Moving_RejectNearEndTarget,1);
		    return 0;
		}
		SetKeepPaceParams(keepPaceDis);
		TaskReset(&Agv.service.mainControlTask);
		GetChissisTskParam(&chassisTskparams,TargetX,TargetY,0,Motion_MOVE);
		SetAppActionState(motionState);
		SetChassisMoveDecType(0);
		ChassisMove_tsk(&chassisTskparams);
		if(FH_FALSE == IsTaskActionOK())
		{
			LogPrintf("move exception, wait last move done\n");
			WaitLastCmdComplete_fun();
			ChassisMove_tsk(&chassisTskparams);
		}
//		struct Chassis_t* chassis = &Agv.service.chassis;
//		FH_Float MoveDistance = Max(fabs(TargetY-chassis->location.YRoundToQR),fabs(TargetX-chassis->location.XRoundToQR));
//		timeOutCnt = 1000+100*(MoveDistance/chassisTskparams.pg.maxVel+chassisTskparams.pg.maxAcc/chassisTskparams.pg.maxVel);
		timeOutCnt = 1000+100*PGGetTargetTime(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg);
		LogPrintf("newtime=%d\n",timeOutCnt);
		WaitMoveDone_tsk(&timeOutCnt);
		//StopCurrentWaitMove();
		//move����������
		WaitUnloadComplete_fun();
		LogPrintf("Wait Move Unload Complete 3\n");
	}
	else
	{
		//�����״�
		int isOn = 1;
		if(!LaserSwitch(&isOn))
		{	
			SetKeepPaceParams(keepPaceDis);
		}
		if(FH_FALSE == IsDownCameraExposureDefault())
		{
			AdjustExposure_b_adjust_exposure(EXPOSURE_CAMERA_DOWN,FALSE_FH);
		}
		FH_Float target=0;
		struct TrayTskParams_t trayTskParams;
		GetTrayTskParam(&trayTskParams,target,Tray_RotateToGround);
		SetAppActionState(AGV_MOTION_POD_ROTATE);
		int isTrayLift = IsTrayLiftUpState();
		if(isTrayLift)
		{
			trayTskParams.pg.motionType = Tray_RotateToGround;//��ground
			GetNominalHeading(Agv.service.tray.Heading2Ground,&target);
			ConvertAngleNearest(&target,Agv.service.tray.Heading2Ground);
			trayTskParams.pg.maxVel = Degree2Rad(5);
		}
		else
		{
			trayTskParams.pg.motionType = Tray_RotateToAgv;//��agv
			GetNominalHeading(Agv.service.tray.Heading2AGV,&target);
			ConvertAngleNearest(&target,Agv.service.tray.Heading2AGV);
			trayTskParams.pg.maxVel = Param.AgvParam[PARAM_LiftMaxAngleSpeed];
		}
		trayTskParams.target = target;
		SetAppActionState(AGV_MOTION_POD_ROTATE);
		TrayRotate_tsk(&trayTskParams);
		timeOutCnt = 3000;//��λ10ms
		WaitMoveDone_tsk(&timeOutCnt);
		//������ʱ�ջ�
		int isCloseLoop = isTrayLift;
		TrayCloseLoop_tsk(&isCloseLoop);
		
		FH_Float curX = Agv.service.chassis.location.XRoundToQR;
		FH_Float curY = Agv.service.chassis.location.YRoundToQR;
		FH_Float targetHeading = 0;
		FH_Float MoveDistance = 0;
		if(IsEqual(TargetX,curX))
		{
			targetHeading = (TargetY>curY)? PI*0.5:-0.5*PI;
			ConvertAngleNearest(&targetHeading,Agv.service.chassis.location.curPosGnd.heading);
			MoveDistance = fabs(TargetY-curY);
		}
		else if(IsEqual(TargetY,curY))
		{
			targetHeading = (TargetX>curX)?0:-PI;
			targetHeading = ConvertAngleNearest(&targetHeading,Agv.service.chassis.location.curPosGnd.heading);
			MoveDistance = fabs(TargetX-curX);
		}
		else
		{
			LogPrintf("heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
			SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,ErrorC);
			return -1;
			
		}
		LogPrintf("heading target:%f\n",targetHeading);
		SetAppActionState(AGV_MOTION_TURNING);
		GetChissisTskParam(&chassisTskparams,curX,curY,targetHeading,Motion_TURN);
		chassisTskparams.pg.creepVel = 4.0*chassisTskparams.pg.creepVel;
		ChassisTurning_tsk(&chassisTskparams);
		timeOutCnt = 3000;//��λ10ms
		WaitMoveDone_tsk(&timeOutCnt);
		WaitLastCmdComplete();
		SetAppActionState(motionState);
		GetChissisTskParam(&chassisTskparams,TargetX,TargetY,0,Motion_MOVE);
		ChassisMove_tsk(&chassisTskparams);
		//timeOutCnt = 2000+100*(MoveDistance/chassisTskparams.pg.maxVel+chassisTskparams.pg.maxAcc/chassisTskparams.pg.maxVel);
		timeOutCnt = 2000+100*(MoveDistance/chassisTskparams.pg.maxVel+chassisTskparams.pg.maxAcc/chassisTskparams.pg.maxVel);
		LogPrintf("MoveDis=%f,Movetime=%d\n",MoveDistance,timeOutCnt);
		WaitMoveDone_tsk(&timeOutCnt);
		LogPrintf("Wait Move Unload Complete 1\n");
		//move����������
		WaitUnloadComplete_fun();
		LogPrintf("Wait Move Unload Complete 2\n");
	}
	return 0;
}
FH_ERROR_CODE TurnInstruction(int16 targetH)
{
	/*******************start ��������******************************/
	FH_Float tarH = targetH*PI/180;
	FH_ERROR_CODE rtn =  CalcMoveParams(&movep);
	if(rtn)
	{
		return rtn ;
	}
	/*******************end ��������******************************/
	if(0 == IsAllMotorEnable())
	{
		SetError(ERROR_MotorDisable_RejectOrder,3);
		return -1;
	}
	if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
		LogPrintf("=ERROR= agv not ready\n");
		return -1;
	}
	
	/**********start*ָ���****************************/
	Uint16 waitCnt = 5000;//��λ10ms
	int isCloseLoop = 0;
	if (IsTrayLiftUpState())
	{
		SetAppActionState(AGV_MOTION_POD_ZERO);
		FH_Float target=0;
		GetNominalHeading(Agv.service.tray.Heading2Ground,&target);
		ConvertAngleNearest(&target,Agv.service.tray.Heading2Ground);
		struct TrayTskParams_t trayTskParams;
		GetTrayTskParam(&trayTskParams,target,Tray_RotateToGround);
		trayTskParams.pg.maxVel = Degree2Rad(5);
		TrayRotate_tsk(&trayTskParams);
		WaitMoveDone_tsk(&waitCnt);
		//�ջ�����
		isCloseLoop = 1;
		TrayCloseLoop_tsk(&isCloseLoop);
		WaitLastCmdComplete();
	}
	//����action״̬����Ӧԭ�˶�״̬
	SetAppActionState(AGV_MOTION_TURNING);
	
	struct ChassisTskParams_t params;
	FH_Float targetX = Agv.service.chassis.location.XRoundToQR;
	FH_Float targetY = Agv.service.chassis.location.YRoundToQR;
	ConvertAngleNearest(&tarH,Agv.service.chassis.location.curPosGnd.heading);
	GetChissisTskParam(&params,targetX,targetY,tarH,Motion_TURN);
	ChassisTurning_tsk(&params);

	WaitMoveDone_tsk(&waitCnt);
	//��������
	isCloseLoop = 0;
	TrayCloseLoop_tsk(&isCloseLoop);
	//move����������
	WaitUnloadComplete();
	LogPrintf("Wait Turn Unload Complete\n");
	return 0;
}

