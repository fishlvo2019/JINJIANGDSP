/*
 * AgvWifiManual.c
 *
 *  Created on: 2018年4月11日
 *      Author: hwei
 */

#include "AgvWifiManual.h"
#include "..\AppParams\AgvAppParams.h"
//extern float PARAM_LiftPitch
int  MaxSpeedGrade = 0;	//定义一个速度等级全局变量
int  misHeartBeat = 0;	//定义一个全局变量，计算校验和为0的时间
int  ManualKey = 0;


struct WifiRemoter_t remoter=
{
		.Speed=0,
};


FH_ERROR_CODE SetWifiParams(struct WifiRemoter_t* remoterParams)
{
	remoter.FunctionTypeId = remoterParams->FunctionTypeId;	//0 命令类型
	remoter.Speed = remoterParams->Speed;	//1 Agv移动方向
	remoter.MaxAngularSpd = remoterParams->MaxAngularSpd;	//2 Agv旋转方向
	remoter.ManualSwitch = remoterParams->ManualSwitch;	//3手动/自动模式
	remoter.SpeedGrade = remoterParams->SpeedGrade;	//4速度等级
	remoter.CheckSum = remoterParams->CheckSum;	//5校验和
	return 0;

}

FH_Float acc_time=1.5,dec_time=1.0,ts=0.01;//加速时间，减速时间，控制周期
FH_Float tmpVL = 0.0,tmpVR=0.0,tmpCmdV = 0.0,tmpOmg = 0.0,tmpVP= 0.0,tmpFeedbackV=0.0;//实际速度
FH_Float TmpAcc=3.0, TmpDec=3.2;//加速度。减速度
int cnt_test=0,UpFlag=0,DownFlag=0;//	上限位标志位。下限位标志位
int com_Flag=0;//通讯标志位
float MotoFlag=0.0,MotoType=0.0;//电机类型获取标志位。电机类型计算标志位
FH_ERROR_CODE WifiManualMove_tsk()
{
	if(AGV_Remote == GetAgvRunMode())	//如果Agv处于遥控模式下
	{
		struct Tray_t* tray = &Agv.service.tray;	//获取用户下发的托盘速度  add hw 20180717
		struct Chassis_t* chassis = &Agv.service.chassis;
		//struct ChassisParams_t* chassisParams = (struct ChassisParams_t*)chassis->base.params;
		//来了新数据

//		FH_Float acc_time=1.5,dec_time=1,ts=0.01;//加速时间，减速时间，控制周期
//		FH_Float tmpVL = 0.0,tmpVR=0.0,tmpCmdV = 0.0,tmpOmg = 0.0,tmpVP= 0.0,tmpFeedbackV=0.0;//实际速度
//		FH_Float TmpAcc=3.0, TmpDec=3.2;//加速度。减速度
//		int cnt_test=0,UpFlag=0,DownFlag=0;//	上限位标志位。下限位标志位
//		int com_Flag=0;//通讯标志位
//		float MotoFlag=0.0,MotoType=0.0;//电机类型获取标志位。电机类型计算标志位



		tmpVP = (tray->motorTrayLift->RPMSet);	//获取Agv本体顶升电机速度
		tmpCmdV = (chassis->motorLeft->RPMSet+chassis->motorRight->RPMSet)*0.5;//获取Agv指令速度
		tmpFeedbackV =(chassis->motorLeft->RPM+chassis->motorRight->RPM)*0.5;//获取Agv实时电机速度
		tmpOmg = (chassis->motorRight->RPMSet - chassis->motorLeft->RPMSet)*0.5;//获取Agv指令角速度
		UpFlag = tray->LiftUpLimitFlag;	//下限位
		DownFlag = tray->LiftDownLimitFlag;//上限位
		MotoFlag = AgvParams.serviceParams.trayParams.Motor2LiftFactor*1000;//获取电机类型（大扭矩电机等于2.16，正常电机等于0.32）
		MotoType = fabs(MotoFlag-2.16);//计算电机类型，小于0.01是大扭矩，否则是正常电机

		//获取用户输入输入速度等级并处理
		switch(remoter.SpeedGrade)
		{
			case 0x01:
				MaxSpeedGrade = 30;	//车轮速度，转每分
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
		if(remoter.CheckSum)	//用校验和判断PC通讯是否中断
		{
			remoter.CheckSum = 0;	//重置校验和
			misHeartBeat = 0;	//重置超时标志
			com_Flag=1;
		}
		else
		{
			misHeartBeat++;	//计算通讯中断的次数
		}

		if(misHeartBeat>25)
		{
			com_Flag=0;
		}


		if(com_Flag)	//用校验和判断PC通讯是否中断
		{
			cnt_test++;
			remoter.CheckSum = 0;	//重置校验和
			//misHeartBeat = 0;	//重置超时标志
			TmpAcc =MaxSpeedGrade/acc_time;
			TmpDec =MaxSpeedGrade/dec_time;
			if(remoter.Speed == 0x01)	//判断客户端是否下发前进命令
			{
				tmpCmdV += TmpAcc*ts;
			}
			else if(remoter.Speed == 0x11)	//判断客户端是否下发后退命令
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

			if(remoter.MaxAngularSpd == 0x01)	//顺时针
			{
				tmpOmg -= TmpAcc*ts;
			}
			else if(remoter.MaxAngularSpd == 0x11)	//逆时针
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


			if(remoter.FunctionTypeId == 0x31)	//托盘顶升
			{
				if(UpFlag == 1)
				{
					tmpVP = 0;
				}
				else
				{
					if(MotoType <= 0.01)
					{
						tmpVP = 100;	//顶升电机速度给100
					}
					else
					{
						tmpVP = 750;	//顶升电机速度给100
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
			if(remoter.FunctionTypeId == 0x41)	//急停
			{
				tmpCmdV =0.0;
				tmpOmg =0.0;
				tmpVP = 0.0;
			}
			if(remoter.FunctionTypeId == 0x00)	//没有下发任何命令
			{
				tmpVP = 0.0;
			}

		}

		/******************************************************************************/

		//超时未通讯
		if(misHeartBeat>=500)	//通讯超时处理
		{
			tmpCmdV= 0.0;
			tmpVP = 0.0;	//通讯中断顶升电机速度等于0

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
			//对顶升电机速度进行修改
			tmpVP = tray->motorTrayLift->RPMSet;	//顶升电机速度获取

			if(fabs(tmpVP) <= TmpDec*0.5)
			{
				tmpVP = 0.0;
			}
			else
			{
				tmpVP = tmpVP - Sign(tray->motorTrayLift->RPMSet)*TmpDec;
			}
			tmpVP = 0;
			//顶升电机抱闸松开
			if(IsMotorDriverState(Agv.service.tray.motorTrayLift,MD_Enable)==1)//判断电机是否处于使能状态
			{
				DisableTrayLiftMotorFast_tsk();	//关闭顶升电机
				if(GetTaskCnt(&Agv.service.mainControlTask) == 0)	//判断此任务是否正在执行
				{
					//ChassisManualMove_tsk();
				}
			}

		}


		//通讯未超时的速度处理
		if(fabs(tmpCmdV) >= MaxSpeedGrade)
		{
			tmpCmdV = Sign(tmpCmdV)*MaxSpeedGrade;
		}
		if(fabs(tmpOmg) >= MaxSpeedGrade*0.5)
		{
			tmpOmg = Sign(tmpOmg)*MaxSpeedGrade*0.5;
		}
		/*
		//在通讯正常的情况下顶升电机的速度限制(顶升电机不允许调速)
		if(fabs(tmpVP) >= 150)
		{
			tmpVP = Sign(tmpVP)*10;
		}
		*/

		tmpVL = tmpCmdV - tmpOmg;	//??
		tmpVR = tmpCmdV + tmpOmg;

		chassis->leftZeroOffsetSpeed =tmpVL;
		chassis->rightZeroOffsetSpeed =tmpVR;
		tray->LiftZeroOffsetSpeed = tmpVP;	//给顶升电机速度


		return TASK_WAIT;
	}
	//LogPrintf("quit chassis manual tsk\n");
	return TASK_DONE;
}
