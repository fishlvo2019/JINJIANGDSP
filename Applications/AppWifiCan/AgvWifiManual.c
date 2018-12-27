/*
 * AgvWifiManual.c
 *
 *  Created on: 2018��4��11��
 *      Author: hwei
 */

#include "AgvWifiManual.h"
#include "..\AppParams\AgvAppParams.h"
//extern float PARAM_LiftPitch
int  MaxSpeedGrade = 0;	//����һ���ٶȵȼ�ȫ�ֱ���
int  misHeartBeat = 0;	//����һ��ȫ�ֱ���������У���Ϊ0��ʱ��
int  ManualKey = 0;


struct WifiRemoter_t remoter=
{
		.Speed=0,
};


FH_ERROR_CODE SetWifiParams(struct WifiRemoter_t* remoterParams)
{
	remoter.FunctionTypeId = remoterParams->FunctionTypeId;	//0 ��������
	remoter.Speed = remoterParams->Speed;	//1 Agv�ƶ�����
	remoter.MaxAngularSpd = remoterParams->MaxAngularSpd;	//2 Agv��ת����
	remoter.ManualSwitch = remoterParams->ManualSwitch;	//3�ֶ�/�Զ�ģʽ
	remoter.SpeedGrade = remoterParams->SpeedGrade;	//4�ٶȵȼ�
	remoter.CheckSum = remoterParams->CheckSum;	//5У���
	return 0;

}

FH_Float acc_time=1.5,dec_time=1.0,ts=0.01;//����ʱ�䣬����ʱ�䣬��������
FH_Float tmpVL = 0.0,tmpVR=0.0,tmpCmdV = 0.0,tmpOmg = 0.0,tmpVP= 0.0,tmpFeedbackV=0.0;//ʵ���ٶ�
FH_Float TmpAcc=3.0, TmpDec=3.2;//���ٶȡ����ٶ�
int cnt_test=0,UpFlag=0,DownFlag=0;//	����λ��־λ������λ��־λ
int com_Flag=0;//ͨѶ��־λ
float MotoFlag=0.0,MotoType=0.0;//������ͻ�ȡ��־λ��������ͼ����־λ
FH_ERROR_CODE WifiManualMove_tsk()
{
	if(AGV_Remote == GetAgvRunMode())	//���Agv����ң��ģʽ��
	{
		struct Tray_t* tray = &Agv.service.tray;	//��ȡ�û��·��������ٶ�  add hw 20180717
		struct Chassis_t* chassis = &Agv.service.chassis;
		//struct ChassisParams_t* chassisParams = (struct ChassisParams_t*)chassis->base.params;
		//����������

//		FH_Float acc_time=1.5,dec_time=1,ts=0.01;//����ʱ�䣬����ʱ�䣬��������
//		FH_Float tmpVL = 0.0,tmpVR=0.0,tmpCmdV = 0.0,tmpOmg = 0.0,tmpVP= 0.0,tmpFeedbackV=0.0;//ʵ���ٶ�
//		FH_Float TmpAcc=3.0, TmpDec=3.2;//���ٶȡ����ٶ�
//		int cnt_test=0,UpFlag=0,DownFlag=0;//	����λ��־λ������λ��־λ
//		int com_Flag=0;//ͨѶ��־λ
//		float MotoFlag=0.0,MotoType=0.0;//������ͻ�ȡ��־λ��������ͼ����־λ



		tmpVP = (tray->motorTrayLift->RPMSet);	//��ȡAgv���嶥������ٶ�
		tmpCmdV = (chassis->motorLeft->RPMSet+chassis->motorRight->RPMSet)*0.5;//��ȡAgvָ���ٶ�
		tmpFeedbackV =(chassis->motorLeft->RPM+chassis->motorRight->RPM)*0.5;//��ȡAgvʵʱ����ٶ�
		tmpOmg = (chassis->motorRight->RPMSet - chassis->motorLeft->RPMSet)*0.5;//��ȡAgvָ����ٶ�
		UpFlag = tray->LiftUpLimitFlag;	//����λ
		DownFlag = tray->LiftDownLimitFlag;//����λ
		MotoFlag = AgvParams.serviceParams.trayParams.Motor2LiftFactor*1000;//��ȡ������ͣ���Ť�ص������2.16�������������0.32��
		MotoType = fabs(MotoFlag-2.16);//���������ͣ�С��0.01�Ǵ�Ť�أ��������������

		//��ȡ�û����������ٶȵȼ�������
		switch(remoter.SpeedGrade)
		{
			case 0x01:
				MaxSpeedGrade = 30;	//�����ٶȣ�תÿ��
				break;

			case 0x02:
				MaxSpeedGrade = 60;
				break;

			case 0x03:
				MaxSpeedGrade = 120;
				break;

			case 0x04:
				MaxSpeedGrade = 165;
				break;
		}
		//
		if(remoter.CheckSum)	//��У����ж�PCͨѶ�Ƿ��ж�
		{
			remoter.CheckSum = 0;	//����У���
			misHeartBeat = 0;	//���ó�ʱ��־
			com_Flag=1;
		}
		else
		{
			misHeartBeat++;	//����ͨѶ�жϵĴ���
		}

		if(misHeartBeat>25)
		{
			com_Flag=0;
		}


		if(com_Flag)	//��У����ж�PCͨѶ�Ƿ��ж�
		{
			cnt_test++;
			remoter.CheckSum = 0;	//����У���
			//misHeartBeat = 0;	//���ó�ʱ��־
			TmpAcc =MaxSpeedGrade/acc_time;
			TmpDec =MaxSpeedGrade/dec_time;
			if(remoter.Speed == 0x01)	//�жϿͻ����Ƿ��·�ǰ������
			{
				tmpCmdV += TmpAcc*ts;
			}
			else if(remoter.Speed == 0x11)	//�жϿͻ����Ƿ��·���������
			{
				tmpCmdV -= TmpAcc*ts;
			}

			else
			{
				if(fabs(tmpFeedbackV)<= 3)
				{
					tmpCmdV = 0;
				}
				else
				{
					tmpCmdV -= (Sign(tmpCmdV)*TmpDec*ts);
				}

			}

			if(remoter.MaxAngularSpd == 0x01)	//˳ʱ��
			{
				tmpOmg -= TmpAcc*ts;
			}
			else if(remoter.MaxAngularSpd == 0x11)	//��ʱ��
			{
				tmpOmg += TmpAcc*ts;
			}

			else
			{
				
				if(fabs(tmpOmg)<= 3)
				{
					tmpOmg = 0;
				}
				else
				{
					tmpOmg -= (Sign(tmpOmg)*TmpDec*ts);
				}
			}


			if(remoter.FunctionTypeId == 0x31)	//���̶���
			{
				if(UpFlag == 1)
				{
					tmpVP = 0;
				}
				else
				{
					if(MotoType <= 0.01)
					{
						tmpVP = 100;	//��������ٶȸ�100
					}
					else
					{
						tmpVP = 750;	//��������ٶȸ�100
					}

				}

			}
			else if(remoter.FunctionTypeId == 0x30)
			{
				if(DownFlag == 1)
				{
					tmpVP = 0;
				}
				else
				{
					if(MotoType <= 0.01)
					{
						tmpVP = -100;
					}
					else
					{
						tmpVP = -750;
					}

				}

			}
			if(remoter.FunctionTypeId == 0x41)	//��ͣ
			{
				tmpCmdV =0.0;
				tmpOmg =0.0;
				tmpVP = 0.0;
			}
			if(remoter.FunctionTypeId == 0x00)	//û���·��κ�����
			{
				tmpVP = 0.0;
			}

		}

		/******************************************************************************/

		//��ʱδͨѶ
		if(misHeartBeat>=500)	//ͨѶ��ʱ����
		{
			tmpCmdV= 0.0;
			tmpVP = 0.0;	//ͨѶ�ж϶�������ٶȵ���0

		}
		else if(misHeartBeat >=50)
		{
			if(fabs(tmpFeedbackV) <= 3)
			{
				tmpCmdV = 0;
			}
			else
			{
				tmpCmdV = tmpCmdV - Sign(tmpCmdV)*TmpDec*ts;
			}
			if(fabs(tmpOmg) <= TmpDec*0.1)
			{
				tmpOmg = 0;
			}
			else
			{
				tmpOmg = tmpOmg - Sign(tmpOmg)*TmpDec*ts;
			}
		}
		else if(misHeartBeat >=40)
		{
			//�Զ�������ٶȽ����޸�
			tmpVP = tray->motorTrayLift->RPMSet;	//��������ٶȻ�ȡ

			if(fabs(tmpVP) <= TmpDec*0.5)
			{
				tmpVP = 0.0;
			}
			else
			{
				tmpVP = tmpVP - Sign(tray->motorTrayLift->RPMSet)*TmpDec;
			}
			tmpVP = 0;
			//���������բ�ɿ�
			if(IsMotorDriverState(Agv.service.tray.motorTrayLift,MD_Enable)==1)//�жϵ���Ƿ���ʹ��״̬
			{
				DisableTrayLiftMotorFast_tsk();	//�رն������
				if(GetTaskCnt(&Agv.service.mainControlTask) == 0)	//�жϴ������Ƿ�����ִ��
				{
					//ChassisManualMove_tsk();
				}
			}

		}


		//ͨѶδ��ʱ���ٶȴ���
		if(fabs(tmpCmdV) >= MaxSpeedGrade)
		{
			tmpCmdV = Sign(tmpCmdV)*MaxSpeedGrade;
		}
		if(fabs(tmpOmg) >= MaxSpeedGrade*0.5)
		{
			tmpOmg = Sign(tmpOmg)*MaxSpeedGrade*0.5;
		}
		/*
		//��ͨѶ����������¶���������ٶ�����(����������������)
		if(fabs(tmpVP) >= 150)
		{
			tmpVP = Sign(tmpVP)*10;
		}
		*/

		tmpVL = tmpCmdV - tmpOmg;	//??
		tmpVR = tmpCmdV + tmpOmg;

		chassis->leftZeroOffsetSpeed =tmpVL;
		chassis->rightZeroOffsetSpeed =tmpVR;
		tray->LiftZeroOffsetSpeed = tmpVP;	//����������ٶ�


		return TASK_WAIT;
	}
	//LogPrintf("quit chassis manual tsk\n");
	return TASK_DONE;
}
