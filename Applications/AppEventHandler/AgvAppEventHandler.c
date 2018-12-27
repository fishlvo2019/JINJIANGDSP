/*
 * AgvAppEventHandler.c
 *
 *  Created on: 2018年7月11日
 *      Author: hwei
 */
#include "FHAGV.h"
#include "./AppWifiCan/AgvWifiManual.h"
extern int ManualKey;

int Event_Key_Handler(struct EventArg_t* eventArg)
{
	//if(eventArg->subEventId == Event_LeftKeyLongPress)	//遥控按键事件,右键长按
	if(eventArg->subEventId == Event_LeftRightKeyLongPress)//遥控按键事件,左右键长按
	{
		/*
		if(AGV_Remote != GetAgvRunMode())	//如果Agv不在遥控模式下
		{
			SetAgvRunMode(AGV_Remote);	//切换Agv状态为遥控模式
			return 0;
		}
		else
		{
			SetAgvRunMode(AGV_Auto);	//切换Agv状态为自动模式
			return 0;
		}
		*/
		ManualKey = ~ManualKey;
		if(ManualKey == 0)
		{
			SetAgvRunMode(AGV_Auto);
			DisableMotorDevice();	//关闭电机
		    DisableTrayLiftMotorDevice();	//关闭顶升电机
		}
		if(ManualKey != 0)	//一旦切换遥控模式后就清错并使能电机
		{
			SetAgvRunMode(AGV_Remote);
			HMISetMode(&Agv.service.HMI, HMIMODE_MANUAL_1);//状态输出到LED和蜂鸣器
		}
	}
	
    LogPrintf("key event subID:%d\n",eventArg->subEventId);
    return 0;
}
FH_ERROR_CODE AppEventHandlerInit()
{
    RegisterAgvEventHandler(Event_Key,Event_Key_Handler);
    return 0;
}
