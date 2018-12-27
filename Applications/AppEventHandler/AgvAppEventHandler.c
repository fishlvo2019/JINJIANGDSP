/*
 * AgvAppEventHandler.c
 *
 *  Created on: 2018��7��11��
 *      Author: hwei
 */
#include "FHAGV.h"
#include "./AppWifiCan/AgvWifiManual.h"
extern int ManualKey;

int Event_Key_Handler(struct EventArg_t* eventArg)
{
	//if(eventArg->subEventId == Event_LeftKeyLongPress)	//ң�ذ����¼�,�Ҽ�����
	if(eventArg->subEventId == Event_LeftRightKeyLongPress)//ң�ذ����¼�,���Ҽ�����
	{
		/*
		if(AGV_Remote != GetAgvRunMode())	//���Agv����ң��ģʽ��
		{
			SetAgvRunMode(AGV_Remote);	//�л�Agv״̬Ϊң��ģʽ
			return 0;
		}
		else
		{
			SetAgvRunMode(AGV_Auto);	//�л�Agv״̬Ϊ�Զ�ģʽ
			return 0;
		}
		*/
		ManualKey = ~ManualKey;
		if(ManualKey == 0)
		{
			SetAgvRunMode(AGV_Auto);
			DisableMotorDevice();	//�رյ��
		    DisableTrayLiftMotorDevice();	//�رն������
		}
		if(ManualKey != 0)	//һ���л�ң��ģʽ������ʹ�ܵ��
		{
			SetAgvRunMode(AGV_Remote);
			HMISetMode(&Agv.service.HMI, HMIMODE_MANUAL_1);//״̬�����LED�ͷ�����
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
