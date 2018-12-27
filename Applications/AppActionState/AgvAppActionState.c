/*
 * AgvAppActionState.c
 *
 *  Created on: 2018年1月24日
 *      Author: hwei
 */
#include "AgvApp.h"
#include "basefunction.h"
#include "Cir_queue.h"


int AppActionStateManager(struct AppActionStateManager_t* appActionStateManager)
{
	int cmdTypeId = GetCurInstructionTypeId();
	//int oldActionState = AGV.MotionStatus;
	FH_Float leftMotorTargetRpm = GetLeftMotorTargetRPM();
	FH_Float leftWheelSpeed = Agv.service.chassis.location.leftSpeed;//GetLeftMotorRPM();
	FH_Float rightMotorTargetRpm = GetRightMotorTargetRPM();
	FH_Float rightWheelSpeed = Agv.service.chassis.location.rightSpeed;//GetRightMotorRPM();
	Uint64 tickNow = GetTimerTick();
	if ((appActionStateManager->CurActionState != AGV_MOTION_STOP) && 
		(GetTaskCnt(&Agv.service.mainControlTask) == 0) &&
		(GetTaskCnt(& App.appBase.AppTask) == 0)&&
		(0 == IsAgvMotionState())&&
		(leftMotorTargetRpm == 0) && 
		(rightMotorTargetRpm == 0))
	{
		float biasSpeed = 10*Param.AgvParam[PARAM_AgvMinSPD];
		appActionStateManager->stopCount++;
		if ((fabs(leftWheelSpeed)<biasSpeed) && (fabs(rightWheelSpeed)<biasSpeed))
		{
			appActionStateManager->stopConfirmDelay++;
			if (appActionStateManager->stopConfirmDelay >= 20)
			{
				
				appActionStateManager->CurActionState = AGV_MOTION_STOP;
				AGV.MotionStatus = AGV_MOTION_STOP;
				appActionStateManager->stopConfirmDelay =0;
				
			}
		}
		else
		{
			 appActionStateManager->stopConfirmDelay =0;
			 if (appActionStateManager->stopCount>500)
			 {
			 	appActionStateManager->stopCount = 0;
			 	SetErrorToQue((ErrorTypeEnum)ERROR_StopStatusConfirmTimeOut,ErrorA);
			 }
		}
	}
	else
	{
		appActionStateManager->stopCount = 0;
		//根据重构进度逐步替换，更改了Action方式，这里需要添加状态
		if((appActionStateManager->CurActionState)&&
			(appActionStateManager->CurActionState != appActionStateManager->preActionState))
		{
			AGV.MotionStatus = appActionStateManager->CurActionState;
		}
		
	}
	
	appActionStateManager->preActionState = appActionStateManager->CurActionState;
	if(AGV_MOTION_STOP == appActionStateManager->CurActionState)
	{
		if(IsTrayCloseLoop() &&
			(!((FHMAINTASK_DOING == App.appBase.AppTask.base.state)||
		(FHMAINTASK_WAITING == App.appBase.AppTask.base.state)||
		(GetTaskCnt(&Agv.service.mainControlTask))||
		(IsAgvMotionState()))))
		{
			//非运动状态，托盘不闭环。运动时开启闭环，才这里结束关掉
			int isCloseLoop = 0;
			TrayCloseLoop_tsk(&isCloseLoop);
		}
		if((++appActionStateManager->laserSwitchDelay >= 1000)&&
			(!((FHMAINTASK_DOING == App.appBase.AppTask.base.state)||
		(FHMAINTASK_WAITING == App.appBase.AppTask.base.state)||
		(GetTaskCnt(&Agv.service.mainControlTask))||
		(IsAgvMotionState()))))
		{
			appActionStateManager->laserSwitchDelay = 0;
			if(0 == AGV.DebugMode)
			{
				LaserSwitch_s(0);
				//ConfigDateConfirmCnt(1);
			}
		}
	}
	else
	{
		appActionStateManager->laserSwitchDelay = 0;
	}
	return 0;
}
//设置action状态，对应原运动状态
int SetAppActionState(int actionState)
{
	Uint64 tmpTick = GetTimerTick();
	int16 tmp = (int16)(tmpTick*0.001);//单位0.1s
	LogPrintf("=Action: %d= time:%d\n",actionState,tmp);
	App.appActionStateManager.CurActionState = actionState;
	return 0;
}
int GetAppActionState()
{
	return App.appActionStateManager.CurActionState;
}
int AppActionStateManagerInit(struct AppActionStateManager_t* appActionStateManager)
{
	return 0;
}



