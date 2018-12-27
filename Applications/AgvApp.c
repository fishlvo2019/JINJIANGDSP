/*
 * FHAgvApp.c
 *
 *  Created on: 2017年10月26日
 *      Author: hwei
 */
#include <string.h>
#include "AgvApp.h"
#include "FHAGV.h"

struct AgvApp_t App;
struct AppParams_t AppParams = 
{
 .AppVersion = 0,
};
FH_ERROR_CODE AppCmdErrorDoNothing(void* nothing)
{
	SetErrorToQue((ErrorTypeEnum)ERROR_CleanerInstruction,WarningC);
    return 0;
}

FH_ERROR_CODE CheckAndConfigAgvApp()
{
	if('V' !=  App.appParamsManager.VersionID[0])
		return -1;
	if('F' !=  App.appParamsManager.VersionID[6])
		return -1;
	if((0 == strncmp(&App.appParamsManager.VersionID[6],"F02",3))
	||(0 == strncmp(&App.appParamsManager.VersionID[6],"F03",3))
	||(0 == strncmp(&App.appParamsManager.VersionID[6],"F04",3)))
	{
		//清扫车:F02
		//清除托盘相关指令
		InstructionRegister(INSTRUCTION_LIFTUP,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_SETDOWN,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_POD_LS,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_POD_ZERO,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_CHANGEPODDIRECTION,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_SETDOWNLS,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_PODJOGGING,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_OPEN_UPCAMERA_AFTERLIFTUP,AppCmdErrorDoNothing);
		InstructionRegister(INSTRUCTION_INITIAL_AFTERLIFT_CHECKSHELF,AppCmdErrorDoNothing);
		//增加清扫指令
		InstructionRegister(INSTRUCTION_DO_CLEAN,(InstructionFunc)DO_INSTRUCTION_DO_CLEAN);
		InstructionRegister(INSTRUCTION_CLEANER_DIRTY_DISPLAY,(InstructionFunc)DO_INSTRUCTION_CLEANER_DIRTY_DISPLAY);
		
	}
	
	return 0;
}

 //应用线程
int AppThreadMain(struct Thread_t* thread)
{
	struct AgvApp_t* app = (struct AgvApp_t*)thread->user;
	#if 1//仿真时0
	//等agv初始化完成
	while(Agv.base.state<FHAGV_WaitParamSet)
	{
		ThreadSleep(10);
	}
	
	//读取地图数据
	GetMapInfo(&app->map);
	
	GetChargePointInfo(&app->map);
	ThreadSleep(10);
	//读取参数
	GetAppParams(&App.appParamsManager);
	ThreadSleep(10);
	//参数验证及公式换算
	ValidateAppParams(&App.appParamsManager);
	//ThreadSleep(10);
	//计算MD5并校验
	CalcAppParamsMd5();
	//更新Camera参数
	int tmpCnt = 0;
	while(0 == Agv.device.dm642.isInitialized)
	{
		UpdataCameraParams();
		ThreadSleep(200);
		if(++tmpCnt>=3)
		{
			break;
		}
	}
	UpdataCameraParams();
	ThreadSleep(200);
	//根据硬件版本号配置应用
	CheckAndConfigAgvApp();
	//app参数到system参数转换
	UpdateSystemParamsFromApp();

	#endif
	int cnt1s = 0,cnt5s = 0;
	if(FHAGV_Initialized == Agv.base.state )
	{
		LogPrintf("app init ok\n");
	}
	else
	{
		LogPrintf("app init fail\n");
	}
	ThreadSleep(50);
	SafeManagerReset();
	while(1)
	{
	//消息机制，处理从其他线程推送过来的任务
		TaskAction_s(&app->appBase.AppTask);
	//处理从主线程推送过来的任务
		TaskAction_s(&app->appBase.AppTaskFromMainThread);
		cnt1s++;
		if (IsAllMotorEnable() && (cnt1s>=100))
		{
			ReportLoadEvent();
			cnt1s = 0;
		}
		cnt5s++;
		if (cnt5s>=500)
		{
			ReportPowerEvent(Agv.device.battery.batteryBase.QuantityElectric,Agv.device.battery.batteryBase.BatVoltage,Agv.device.battery.batteryBase.BatCapacity,
							Agv.device.gyro.Temperature,Agv.device.battery.batteryBase.Temperature,
							Agv.device.motorLeft.md.TempValue,Agv.device.motorRight.md.TempValue,
							Agv.device.motorTrayRotate.md.TempValue,Agv.device.motorTrayLift.md.TempValue);
			cnt5s = 0;
		}
		/*串口A打印*/
		if(1 == AgvParams.deviceParams.pcParams.pcCmdRecParams.logCom)
		{
			/*串口发送功能*/
			struct SCI_t* sci = &Agv.board.sciA;
			while((sci->sendFifo.dataLen>0)&&
				(sci->sciRegs->SCIFFTX.bit.TXFFST < 16))
			{
				sci->sciRegs->SCITXBUF = FIFOPop8(&sci->sendFifo);
			}
		}
		/***********************************电池电流异常保护end******************************************/
		ThreadSleep(10);
	}
	//return 0;
}

void AppRealTimeTaskPeriod(void)
{
	AppActionStateManager(&App.appActionStateManager);
	AppSafeCheck();
	WifiManualMove_tsk();	//在此处插入需要放入线程的APP程序(遥控速度下发)
	return;
}
int AgvAppInit(struct AgvApp_t* app,struct AppParams_t* appParams)
{
    MemSet((int*)&App,0,sizeof(App));
	app->params = appParams;
	app->appBase.AppThread = &Agv.AppThread;
	Agv.app = &app->appBase;

	
	appParams->appBaseParams.appTaskParams.typeId = TASK_ForApp;
	TaskInit(&app->appBase.AppTask,&appParams->appBaseParams.appTaskParams);

	appParams->appBaseParams.appTaskFromMainThreadParams.typeId = TASK_ForApp_FromMainThread;
	TaskInit(&app->appBase.AppTaskFromMainThread,&appParams->appBaseParams.appTaskFromMainThreadParams);
	app->appBase.RealTimeTaskRun_s = AppRealTimeTaskPeriod;

	
	//app 线程
	app->appBase.AppThread->Run = AppThreadMain;
	app->appBase.AppThread->user = app;
	app->appBase.AppThread->stackSize = 512;//该线程运行原main函数，栈扩大，后期可调整缩小
	app->appBase.AppThread->priority = DefaultPriority;
	ThreadRun(app->appBase.AppThread);
	
	AppParamsInit(&app->appParamsManager);
    AppMapInfoInit(&app->map,&appParams->mapParams);
	AppInstructionInit(&app->appInstruction);
	AppLogInit(&app->appLog);
	AppActionStateManagerInit(&app->appActionStateManager);
	AppEventHandlerInit();
	
	return 0;
}
int GetCurInstructionId()
{
	return App.appBase.AppTask.CurId;
}
int GetCurInstructionTypeId()
{
	return App.appBase.AppTask.CurTypeId;
}
int GetPreInstructionTypeId()
{
	return App.appBase.AppTask.PreTypeId;
}

