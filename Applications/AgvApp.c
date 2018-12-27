/*
 * FHAgvApp.c
 *
 *  Created on: 2017��10��26��
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
		//��ɨ��:F02
		//����������ָ��
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
		//������ɨָ��
		InstructionRegister(INSTRUCTION_DO_CLEAN,(InstructionFunc)DO_INSTRUCTION_DO_CLEAN);
		InstructionRegister(INSTRUCTION_CLEANER_DIRTY_DISPLAY,(InstructionFunc)DO_INSTRUCTION_CLEANER_DIRTY_DISPLAY);
		
	}
	
	return 0;
}

 //Ӧ���߳�
int AppThreadMain(struct Thread_t* thread)
{
	struct AgvApp_t* app = (struct AgvApp_t*)thread->user;
	#if 1//����ʱ0
	//��agv��ʼ�����
	while(Agv.base.state<FHAGV_WaitParamSet)
	{
		ThreadSleep(10);
	}
	
	//��ȡ��ͼ����
	GetMapInfo(&app->map);
	
	GetChargePointInfo(&app->map);
	ThreadSleep(10);
	//��ȡ����
	GetAppParams(&App.appParamsManager);
	ThreadSleep(10);
	//������֤����ʽ����
	ValidateAppParams(&App.appParamsManager);
	//ThreadSleep(10);
	//����MD5��У��
	CalcAppParamsMd5();
	//����Camera����
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
	//����Ӳ���汾������Ӧ��
	CheckAndConfigAgvApp();
	//app������system����ת��
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
	//��Ϣ���ƣ�����������߳����͹���������
		TaskAction_s(&app->appBase.AppTask);
	//��������߳����͹���������
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
		/*����A��ӡ*/
		if(1 == AgvParams.deviceParams.pcParams.pcCmdRecParams.logCom)
		{
			/*���ڷ��͹���*/
			struct SCI_t* sci = &Agv.board.sciA;
			while((sci->sendFifo.dataLen>0)&&
				(sci->sciRegs->SCIFFTX.bit.TXFFST < 16))
			{
				sci->sciRegs->SCITXBUF = FIFOPop8(&sci->sendFifo);
			}
		}
		/***********************************��ص����쳣����end******************************************/
		ThreadSleep(10);
	}
	//return 0;
}

void AppRealTimeTaskPeriod(void)
{
	AppActionStateManager(&App.appActionStateManager);
	AppSafeCheck();
	WifiManualMove_tsk();	//�ڴ˴�������Ҫ�����̵߳�APP����(ң���ٶ��·�)
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

	
	//app �߳�
	app->appBase.AppThread->Run = AppThreadMain;
	app->appBase.AppThread->user = app;
	app->appBase.AppThread->stackSize = 512;//���߳�����ԭmain������ջ���󣬺��ڿɵ�����С
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

