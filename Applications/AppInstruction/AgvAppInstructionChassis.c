/*
 * InstructionMove.c
 *
 *  Created on: 2017��12��27��
 *      Author: hwei
 */

#include "AgvApp.h"
#include "Basefunction.h"
#include "math.h"
#include "string.h"
 /*
 	chassisTskParams:�ṹ��ָ��
 	TargetX:
 */
 
int IsCustomIDVaild = 0;
int NewTargetWhenDecCnt = 0;	//���ٶ��յ���ָ����������ڼ�����ٶ�����ʱ�䡣
struct MoveParamType movep = {0};
 
float maxVelocity_PC[16]={0,0.2,0.4,0.5,0.8,1.0,1.2,0.9,1.1,1.3,1.5,1.7,1.8,1.9,2,2.1};
float maxDecVelocity_PC[16]={0,0.1,0.2,0.3,0.4,0.5,0.7,0.9,1.1,1.3,1.5,1.7,1.8,1.9,2,2.1};
float maxRotateOmg_PC[16]={0,20*PI/180,40*PI/180,60*PI/180,80*PI/180,100*PI/180,150*PI/180,200*PI/180,250*PI/180,300*PI/180,350*PI/180,360*PI/180,370*PI/180,380*PI/180,390*PI/180,400*PI/180};
float maxDecRotateOmg_PC[16]={0,10*PI/180,20*PI/180,30*PI/180,40*PI/180,50*PI/180,100*PI/180,150*PI/180,200*PI/180,250*PI/180,300*PI/180,350*PI/180,400*PI/180,450*PI/180,500*PI/180,600*PI/180};
float maxVelocity_U[2]={0.291,0.36};

double SpeedLimit=1,RotateLimit=50*PI/180.0,DecLimit=0.6,DecRotateLimit=50*PI/180;//HR,������ʻ����(m/s)�����ٶȡ�����ת����(rad/s)����ת���ٶ��趨

//#define INSTRUCTION_MOVE 4 // a move instruction
FH_ERROR_CODE CalcMoveParams(struct MoveParamType* moveParam)
{
	IsCustomIDVaild = 0;
	if((0 != movep.dec_rank)
	|| (0 != movep.omega_rank)
	|| (0 != movep.epsi_rank))
	{
		if(IsAgvMotionState())//(Instruction.Status == INSTRUCTION_STATUS_DOING)
		{
			SetErrorToQue((ErrorTypeEnum)ERROR_NonStop_ParamChg,WarningB);
			//return -4;
		}
		LogPrintf("rank vel:%d,dec:%d,omg:%d,epsi:%d\n",movep.vel_rank,movep.dec_rank,movep.omega_rank,movep.epsi_rank);
	}
	else if(0 != movep.vel_rank)
	{
	//�������ٶ�
	}
	else
	{
		return 0;
	}
     double speedLimit_noLoad=0,rotateLimit_noLoad=0,decLimit_noLoad=0,decRotateLimit_noLoad=0,speedLimit_load=0,rotateLimit_load=0,decLimit_load=0,decRotateLimit_load=0;//HR,add,20170414

     //�趨�����������������ٶ�,HR,add,20161213
     if(IsTrayLiftUpState())//�����½�λ
     {
     	//modify by hwei ��ָ�����1.1*1.5������ٶȸ����ϣ�
         speedLimit_load =LoadCondition.MaxSpd[1];
         rotateLimit_load =Max(LoadCondition.MaxAngularSpd[1], LoadCondition.MaxChangeDirectionAngularSpd[1]);//?(1.1*LoadCondition.MaxAngularSpd[1]):(1.1*LoadCondition.MaxChangeDirectionAngularSpd[1]);
         decLimit_load =LoadCondition.MaxAcc[1];
         decRotateLimit_load = LoadCondition.MaxAngularAcc[1];
 
         speedLimit_noLoad =LoadCondition.MaxSpd[0];
         rotateLimit_noLoad =Max(LoadCondition.MaxAngularSpd[0], LoadCondition.MaxChangeDirectionAngularSpd[0]);//?(1.1*LoadCondition.MaxAngularSpd[0]):(1.1*LoadCondition.MaxChangeDirectionAngularSpd[0]);
         decLimit_noLoad =LoadCondition.MaxAcc[0];
         decRotateLimit_noLoad = Max(LoadCondition.MaxAngularAcc[0], LoadCondition.ChangeDirectionAngularAcc[0]);//?(1.1*LoadCondition.MaxAngularAcc[0]):(1.1*LoadCondition.ChangeDirectionAngularAcc[0]);
 
         SpeedLimit = Min(speedLimit_load,speedLimit_noLoad);// ? speedLimit_load:speedLimit_noLoad;//��������²��ܳ����ճ�
         RotateLimit = Min(rotateLimit_load,rotateLimit_noLoad );//? rotateLimit_load:rotateLimit_noLoad;
         DecLimit = Min(decLimit_load,decLimit_noLoad );//? decLimit_load:decLimit_noLoad;
         DecRotateLimit = Min(decRotateLimit_load,decRotateLimit_noLoad);//?decRotateLimit_load:decRotateLimit_noLoad;
     }
     else
     {
         SpeedLimit = LoadCondition.MaxSpd[0];
         RotateLimit =Max(LoadCondition.MaxAngularSpd[0], LoadCondition.MaxChangeDirectionAngularSpd[0]);//?(1.1*LoadCondition.MaxAngularSpd[0]):(1.1*LoadCondition.MaxChangeDirectionAngularSpd[0]);
         DecLimit =LoadCondition.MaxAcc[0];
         DecRotateLimit =Max(LoadCondition.MaxAngularAcc[0], LoadCondition.ChangeDirectionAngularAcc[0]);//?(1.1*LoadCondition.MaxAngularAcc[0]):(1.1*LoadCondition.ChangeDirectionAngularAcc[0]);
     }

 //ȷ��������Ϊ0
  //��֤����ٶ������������Բ���fabs
  if(Param.AgvParam[PARAM_AgvMaxSPD] > 0.0001)
  {
      if(0 != moveParam->vel_rank)
      {
         if (maxVelocity_PC[moveParam->vel_rank]>SpeedLimit)
         {
             moveParam->vel_max =SpeedLimit;
             SetError(ERROR_MoveParam_Wrong,1);
         }
         else
         {
             moveParam->vel_max =maxVelocity_PC[moveParam->vel_rank];
         }
      }
      else
      {
          moveParam->vel_max =SpeedLimit;
      }
  }


  //ȷ�����ٶ�ϵ������Ҫ��
  if(Param.AgvParam[PARAM_AgvMaxAcc] > 0.0001)
  {
      if(0 != moveParam->dec_rank)
      {
         if (maxDecVelocity_PC[moveParam->dec_rank]>DecLimit)
         {
             moveParam->dec_max =DecLimit;
             SetError(ERROR_MoveParam_Wrong,1);
         }
         else
         {
             moveParam->dec_max =maxDecVelocity_PC[movep.dec_rank];
         }
      }
      else
      {
          moveParam->dec_max =DecLimit;
      }
  }

  if(Param.AgvParam[PARAM_AgvMaxAngSPD] > 0.0001)
  {
      if(0 != moveParam->omega_rank)
      {
         if (maxRotateOmg_PC[moveParam->omega_rank]>RotateLimit)
         {
             moveParam->omega_max =RotateLimit;
             SetError(ERROR_MoveParam_Wrong,1);
         }
         else
         {
             moveParam->omega_max =maxRotateOmg_PC[movep.omega_rank];
         }
      }
      else
      {
          moveParam->omega_max =RotateLimit;
      }
  }

  if(Param.AgvParam[PARAM_AgvMaxAngAcc] > 0.0001)
  {
      if(0 != moveParam->epsi_rank)
      {
         if (maxDecRotateOmg_PC[movep.epsi_rank]>DecRotateLimit)
         {
             moveParam->epsi_max =DecRotateLimit;
             SetError(ERROR_MoveParam_Wrong,1);
         }
         else
         {
             moveParam->epsi_max = maxDecRotateOmg_PC[moveParam->epsi_rank];
         }
      }
      else
      {
          moveParam->epsi_max = DecRotateLimit;
      }
  }
	if(IsAgvMotionState())
	{
		if(0 == IsEqual(LoadCondition.MaxAcc[CustomID], moveParam->dec_max))
		{
			SetErrorToQue((ErrorTypeEnum)ERROR_NonStop_ParamChg,WarningB);
			return -4;
		}
	}
  
  LoadCondition.MaxSpd[CustomID] = moveParam->vel_max;
  LoadCondition.MaxAcc[CustomID] = moveParam->dec_max;
  LoadCondition.MaxAngularSpd[CustomID] = moveParam->omega_max;
  LoadCondition.MaxAngularAcc[CustomID] = moveParam->epsi_max;
  IsCustomIDVaild = 1;
 return 0;
}
extern enum ExtraTaskEnum extra_task;
CalMoveTime()
{
}

FH_ERROR_CODE WaitDecState()
{
	while(1)
	{
		ThreadSleep(10);
		//��ֹ�����޷�������ٽ׶�
		if((IsAgvMotionState()== 0)&&
		(GetTaskCnt(&Agv.service.mainControlTask) == 0))
		{
			LogPrintf("state ret\n");
			return 0;
		}
		//������ٽ׶ν������¹滮
		if(1 == IsSPGDecState(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.spg))
		{
			LogPrintf("IsSPGDecState\n");
			return 0;
		}
	}
}
FH_ERROR_CODE DO_INSTRUCTION_RIGHTANGLEMOVE(struct INSTRUCTION_Normal_params_t* moveParam)
{	
    /*******************start ��������******************************/
	int32 tmpx = moveParam->targetX;
	EndianConvert(&tmpx,sizeof(tmpx));
	int32 tmpy = moveParam->targetY;
	EndianConvert(&tmpy,sizeof(tmpy));
	int16 tmpHeading = moveParam->targetHeading;
	EndianConvert(&tmpHeading,sizeof(tmpHeading));
	FH_Float TargetX = SwitchQRCodeToAgv(tmpx,DirX);
	FH_Float TargetY = SwitchQRCodeToAgv(tmpy,DirY);

	FH_Float curX = Agv.service.chassis.location.XRoundToQR;
	FH_Float curY = Agv.service.chassis.location.YRoundToQR;
	FH_Float tmpAng,cornerX,cornerY,cornerHeading;
	FH_Float chargePointX,chargePointY;
	FH_Float cornerNextX,cornerNextY;
	struct Chassis_t * chassis = &Agv.service.chassis;	
	FH_Float nominaltargetx = RoundAgvToCurQRPos(TargetX,DirX,1);//ROUND_POSITION_UNIT(TargetX,CODE_GAP_X);
	FH_Float nominaltargety = RoundAgvToCurQRPos(TargetY,DirY,1);//ROUND_POSITION_UNIT(TargetY,CODE_GAP_Y);
	FH_Float nominaltargetx_pre =RoundAgvToCurQRPos(chassis->TargetX,DirX,1);//ROUND_POSITION_UNIT(chassis->TargetX,CODE_GAP_X);
	FH_Float nominaltargety_pre =RoundAgvToCurQRPos(chassis->TargetY,DirY,1);//ROUND_POSITION_UNIT(chassis->TargetY,CODE_GAP_Y);

	int axis = GetNominalHeadingOutAxis(Agv.service.chassis.location.curPosGnd.heading,&tmpAng);
	FH_Float compareX = curX;
	FH_Float compareY = curY;
	FH_Float targetH = 0.0;
	GetNominalHeading(Agv.service.chassis.location.curPosGnd.heading,&targetH);
	if(IsAgvMovingState())
	{
		compareX = nominaltargetx_pre;
		compareY = nominaltargety_pre;
	}
	if(Axis_X == axis)
	{
		if(2 == moveParam->PodDirection)
		{
			cornerX = SwitchQRCodeToAgv(tmpx+2000*cos(targetH),DirX);
			cornerY = curY;
			chargePointX = SwitchQRCodeToAgv(tmpx+1000*cos(targetH),DirX);
			chargePointY = curY;
			if((fabs(TargetY-compareY)<0.1)||((TargetX-compareX)*cos(targetH)<-0.1))
			{
				LogPrintf("UCurve:heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
				SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,ErrorC);
				return -1;
			}
			if((TargetY-compareY)>0.1)
			{
				cornerHeading = PI*cos(targetH);//��ת180��
			}
			else
			{
				cornerHeading = -PI*cos(targetH);//��ת180��
			}
		}
		else
		{
			cornerX = TargetX;
			cornerY = curY;
			chargePointX = TargetX;
			chargePointY = TargetY;
			if((compareX - nominaltargetx)*cos(tmpAng)> -0.1)
			{
				LogPrintf("Arc:heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
				SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,ErrorC);
				return -1;
			}
			if(fabs(TargetY - curY)<0.1)
			{
				LogPrintf("Arc:heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
				SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,ErrorC);
				return -1;
			}
			if(((TargetY>curY)&&(TargetX>curX))||
			((TargetY<curY)&&(TargetX<curX)))
			{
				cornerHeading = 0.5*PI;//��ת90��
			}
			else
			{
				cornerHeading = -0.5*PI;//��ת90��
			}
		}
		
	}
	else if(Axis_Y == axis)
	{
		if(2 == moveParam->PodDirection)
		{
			cornerX = curX;
			cornerY = SwitchQRCodeToAgv(tmpy+2000*sin(targetH),DirY);
			chargePointX = curX;
			chargePointY = SwitchQRCodeToAgv(tmpy+1000*sin(targetH),DirY);
			if((fabs(TargetX-compareX)<0.1)||((TargetY-compareY)*sin(targetH)<-0.1))
			{
				LogPrintf("UCurve:heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
				SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,ErrorC);
				return -1;
			}
			if((TargetX-compareX)>0.1)
			{
				cornerHeading = -PI*sin(targetH);//��ת180��
			}
			else
			{
				cornerHeading = PI*sin(targetH);//��ת180��
			}
		}
		else
		{
			cornerX = curX;
			cornerY = TargetY;
			
			chargePointX = TargetX;
			chargePointY = TargetY;
			if((compareY - nominaltargety)*sin(tmpAng)> -0.1)
			{
				LogPrintf("Arc:heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
				SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,ErrorC);
				return -1;
			}
			if(fabs(TargetX - curX)<0.1)
			{
				LogPrintf("Arc:heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
				SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,ErrorC);
				return -1;
			}
			if(((TargetY>curY)&&(TargetX>curX))||
			((TargetY<curY)&&(TargetX<curX)))
			{
				cornerHeading = -0.5*PI;//��ת90��
			}
			else
			{
				cornerHeading = 0.5*PI;//��ת90��
			}
		}
		
	}
	else
	{
		LogPrintf("error rightanglemove\n");
	}
	//�ж��Ƿ��ڳ���
	if(IsChargePoint(chargePointX,chargePointY))
	{
		int i = 0;
		for(i=0;i < App.map.ChargePointNUM;i++)
	    {
			LogPrintf("chargePoint:id=%d,X:%f,TargetY:%f\n",i,App.map.chargePointInfo[i].baseX,App.map.chargePointInfo[i].baseY);
			ThreadSleep(10);
		}
		SetErrorToQue((ErrorTypeEnum)ERROR_MoveTargetPointInChargeZone,ErrorC);
		return -1;
	}
	LogPrintf("cornerX:%f,Y:%f,H:%f\n",cornerX,cornerY,cornerHeading);
	LogPrintf("PodDirection=%d\n",moveParam->PodDirection);
	if(0 == moveParam->PodDirection)
	{
		moveParam->targetX = (int32)(GetCurCodeID(cornerX,DirX));//(cornerX/CODE_GAP_X*1000);
		EndianConvert(&moveParam->targetX,sizeof(moveParam->targetX));
		moveParam->targetY = (int32)(GetCurCodeID(cornerY,DirY));//(cornerY/CODE_GAP_Y*1000);
		EndianConvert(&moveParam->targetY,sizeof(moveParam->targetY));
		DO_INSTRUCTION_MOVE(moveParam);
		WaitLastCmdComplete();
		moveParam->targetX = (int32)(GetCurCodeID(TargetX,DirX));//(cornerX/CODE_GAP_X*1000);
		EndianConvert(&moveParam->targetX,sizeof(moveParam->targetX));
		moveParam->targetY = (int32)(GetCurCodeID(TargetY,DirY));//(cornerY/CODE_GAP_Y*1000);
		EndianConvert(&moveParam->targetY,sizeof(moveParam->targetY));
		DO_INSTRUCTION_MOVE(moveParam);
	}
	else if(1 == moveParam->PodDirection)
	{
		//1,�жϵ�ǰ�ٶ���λ���Ƿ�����ֱ�ǻ���ת������
//		if(AGV.RdDMXYCnt<2)
		LogPrintf("SpeedControlMode=%d\n",AGV.SpeedControlMode);
		moveParam->PodDirection = 0;
		if(AGV.SpeedControlMode != MOVE_SPEED_MODE_HIGH)
		{
			moveParam->targetX = (int32)(GetCurCodeID(cornerX,DirX));//(cornerX/CODE_GAP_X*1000);
			EndianConvert(&moveParam->targetX,sizeof(moveParam->targetX));
			moveParam->targetY = (int32)(GetCurCodeID(cornerY,DirY));//(cornerY/CODE_GAP_Y*1000);
			EndianConvert(&moveParam->targetY,sizeof(moveParam->targetY));
			DO_INSTRUCTION_MOVE(moveParam);
			WaitLastCmdComplete();
			moveParam->targetX = (int32)(GetCurCodeID(TargetX,DirX));//(cornerX/CODE_GAP_X*1000);
			EndianConvert(&moveParam->targetX,sizeof(moveParam->targetX));
			moveParam->targetY = (int32)(GetCurCodeID(TargetY,DirY));//(cornerY/CODE_GAP_Y*1000);
			EndianConvert(&moveParam->targetY,sizeof(moveParam->targetY));
			DO_INSTRUCTION_MOVE(moveParam);

		}
		else
		{
			if(GetAngleDiff(tmpAng,Agv.service.chassis.location.curPosGnd.heading)> (5.0/180*PI))
			{
				LogPrintf("AngDiff=%f\n",GetAngleDiff(tmpAng,Agv.service.chassis.location.curPosGnd.heading));
				SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Angle,ErrorC);
				return -1;
			}
			//1,ֹͣ����µ��ã��迪���״���ܶԵرջ�
			//����ת��ʱ�״���Ͻϸ��ӣ�ֹͣ���ٹ滮�ϸ��ӣ������״Ｔ��ͣ��
			struct ChassisTskParams_t chassisTskparams;
			if(0 == IsAgvMotionState())
			{
				//�����״�
				//�趨��������ͣ�����������HR��20180305
				float32 keepPaceDis =Param.AgvParam[PARAM_CODE_GAP_X]>Param.AgvParam[PARAM_CODE_GAP_Y]? 0.5*Param.AgvParam[PARAM_CODE_GAP_X]:0.5*Param.AgvParam[PARAM_CODE_GAP_Y];
				keepPaceDis =keepPaceDis > Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis] ? keepPaceDis:Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis];
				int isOn = 1;
				if(!LaserSwitch(&isOn))
				{	
					SetKeepPaceParams(keepPaceDis);
				}
				else
				{
					SetErrorToQue((ErrorTypeEnum)ERROR_LaserClose,ErrorC);
					return -1;
				}
				

				//
				FH_Float target=0;
	            struct TrayTskParams_t trayTskParams;
				GetTrayTskParam(&trayTskParams,target,Tray_RotateToGround);
				//SetAppActionState(AGV_MOTION_POD_ROTATE);
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
	            Uint16 timeOutCnt = 3000;//��λ10ms
	            WaitMoveDone_tsk(&timeOutCnt);
				//������ʱ�ջ�
				int isCloseLoop = isTrayLift;
	            TrayCloseLoop_tsk(&isCloseLoop);
				
				ConvertAngleNearest(&tmpAng,Agv.service.chassis.location.curPosGnd.heading);
				GetChissisTskParam(&chassisTskparams,curX,curY,tmpAng,Motion_TURN);
				ChassisTurning_tsk(&chassisTskparams);
				WaitMoveDone_tsk(&timeOutCnt);
			}
			else
			{
				
				 //���ϼ���ʱ���ϰ�����ʧ��ִ�б��
				if(IsPGTakeOver(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg))
				{
					 WaitLastCmdComplete();
				}
//				WaitDecState();
				TaskReset(&Agv.service.mainControlTask);
			}
			//2,chassismove to corner
			//����action״̬����Ӧԭ�˶�״̬
			SetAppActionState(AGV_MOTION_ARC);
			FH_Float newCorner = 0.0;
			int rank = (tmpHeading & 0xF000)>>12;
			if(2 == rank)
			{
				Arc.factor = 1.3745;
				Arc.maxR = 0.5498;
				CalcSPGFixParam(maxVelocity_PC[rank],0.5*PI,50);
			}
			else if(3 == rank)
			{
				Arc.factor = 0.80493;
				Arc.maxR = 0.402;
				CalcSPGFixParam(maxVelocity_PC[rank],0.5*PI,100);
			}
			if(Axis_X == axis)
			{
				newCorner = cornerX-(Arc.factor*maxVelocity_PC[rank]-0.05)*cos(targetH);
				GetChissisTskParam(&chassisTskparams,newCorner,cornerY,0,Motion_MOVE);
			}
			else if(Axis_Y == axis)
			{
				newCorner = cornerY-(Arc.factor*maxVelocity_PC[rank]-0.05)*sin(targetH);
				GetChissisTskParam(&chassisTskparams,cornerX,newCorner,0,Motion_MOVE);
			}
			chassisTskparams.pg.creepVel = maxVelocity_PC[rank];
			chassisTskparams.pg.creepDistance = 0.1;
			LogPrintf("curCreepVel=%f\n",chassisTskparams.pg.creepVel);
			
			//chassisTskparams.esParams.stopConditionType = StopCondition_IgnorErrorC;
			SetChassisMoveDecType(0);
			ChassisMove_tsk(&chassisTskparams);
			//3,waitmove speed to connect arc motion
			chassisTskparams.targetX = TargetX;
	    	chassisTskparams.targetY = TargetY;
			chassisTskparams.targetHeading = cornerHeading;
			//GetChissisTskParam(&chassisTskparams,TargetX,TargetY,0,Motion_MOVE);
			//chassisTskparams.pg.maxVel = 0.2;
			
			WaitVel4ArcMove_tsk(&chassisTskparams);
			while(1)
			{
				ThreadSleep(20);
				if(GetTaskCnt(&Agv.service.mainControlTask) == 0)
					break;
			}
		}
	}
	else if(2 == moveParam->PodDirection)
	{
		//1,�жϵ�ǰ�ٶ���λ���Ƿ�����ֱ�ǻ���ת������
//		if(AGV.RdDMXYCnt<2)
		LogPrintf("SpeedControlMode=%d\n",AGV.SpeedControlMode);
		moveParam->PodDirection = 0;
		if(AGV.SpeedControlMode != MOVE_SPEED_MODE_HIGH)
		{
			if(Axis_X == axis)
			{
				cornerX = SwitchQRCodeToAgv(tmpx+1000*cos(targetH),DirX);
				cornerY = curY;
				cornerNextX = cornerX;
				cornerNextY = TargetY;
			}
			else if(Axis_Y == axis)
			{
				cornerX = curX;
				cornerY = SwitchQRCodeToAgv(tmpy+1000*sin(targetH),DirY);
				cornerNextX = TargetX;
				cornerNextY = cornerY;
			}
			moveParam->targetX = (int32)(GetCurCodeID(cornerX,DirX));//(cornerX/CODE_GAP_X*1000);
			EndianConvert(&moveParam->targetX,sizeof(moveParam->targetX));
			moveParam->targetY = (int32)(GetCurCodeID(cornerY,DirY));//(cornerY/CODE_GAP_Y*1000);
			EndianConvert(&moveParam->targetY,sizeof(moveParam->targetY));
			DO_INSTRUCTION_MOVE(moveParam);
			WaitLastCmdComplete();
			moveParam->targetX = (int32)(GetCurCodeID(cornerNextX,DirX));//(cornerX/CODE_GAP_X*1000);
			EndianConvert(&moveParam->targetX,sizeof(moveParam->targetX));
			moveParam->targetY = (int32)(GetCurCodeID(cornerNextY,DirY));//(cornerY/CODE_GAP_Y*1000);
			EndianConvert(&moveParam->targetY,sizeof(moveParam->targetY));
			DO_INSTRUCTION_MOVE(moveParam);
			WaitLastCmdComplete();
			moveParam->targetX = (int32)(GetCurCodeID(TargetX,DirX));//(cornerX/CODE_GAP_X*1000);
			EndianConvert(&moveParam->targetX,sizeof(moveParam->targetX));
			moveParam->targetY = (int32)(GetCurCodeID(TargetY,DirY));//(cornerY/CODE_GAP_Y*1000);
			EndianConvert(&moveParam->targetY,sizeof(moveParam->targetY));
			DO_INSTRUCTION_MOVE(moveParam);
		}
		else
		{
			struct ChassisTskParams_t chassisTskparams;
			if(0 == IsAgvMotionState())
			{
				//�����״�
				//�趨��������ͣ�����������HR��20180305
				float32 keepPaceDis =Param.AgvParam[PARAM_CODE_GAP_X]>Param.AgvParam[PARAM_CODE_GAP_Y]? 0.5*Param.AgvParam[PARAM_CODE_GAP_X]:0.5*Param.AgvParam[PARAM_CODE_GAP_Y];
				keepPaceDis =keepPaceDis > Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis] ? keepPaceDis:Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis];
				int isOn = 1;
				if(!LaserSwitch(&isOn))
				{	
					SetKeepPaceParams(keepPaceDis);
				}
				else
				{
					SetErrorToQue((ErrorTypeEnum)ERROR_LaserClose,ErrorC);
					return -1;
				}
				FH_Float target=0;
	            struct TrayTskParams_t trayTskParams;
				GetTrayTskParam(&trayTskParams,target,Tray_RotateToGround);
				//SetAppActionState(AGV_MOTION_POD_ROTATE);
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
	            Uint16 timeOutCnt = 3000;//��λ10ms
	            WaitMoveDone_tsk(&timeOutCnt);
				//������ʱ�ջ�
				int isCloseLoop = isTrayLift;
	            TrayCloseLoop_tsk(&isCloseLoop);
				
				ConvertAngleNearest(&tmpAng,Agv.service.chassis.location.curPosGnd.heading);
				GetChissisTskParam(&chassisTskparams,curX,curY,tmpAng,Motion_TURN);
				ChassisTurning_tsk(&chassisTskparams);
				WaitMoveDone_tsk(&timeOutCnt);
			}
			else
			{
				//���ϼ���ʱ���ϰ�����ʧ��ִ�б��
				if(IsPGTakeOver(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg))
				{
					 WaitLastCmdComplete();
				}
//				WaitDecState();
				TaskReset(&Agv.service.mainControlTask);
			}
			SetAppActionState(AGV_MOTION_UCURVE);
			
			Arc.factor = 0.861326;
			FH_Float curOffsetYCodeGap = 0.0;
			FH_Float newCorner = 0.0;
			if(Axis_X == axis)
			{
				Arc.targetCodegap = fabs(SwitchQRCodeToAgv(tmpx+1000*cos(targetH),DirX)-TargetX);
				newCorner = cornerX-(1.2*Arc.targetCodegap-0.05)*cos(targetH);
				GetChissisTskParam(&chassisTskparams,newCorner,cornerY,0,Motion_MOVE);
				curOffsetYCodeGap = GetCurCodeGap(TargetY,DirY);
			}
			else if(Axis_Y == axis)
			{
				Arc.targetCodegap = fabs(SwitchQRCodeToAgv(tmpy+1000*sin(targetH),DirY)-TargetY);
				newCorner = cornerY-(1.2*Arc.targetCodegap-0.05)*sin(targetH);
				GetChissisTskParam(&chassisTskparams,cornerX,newCorner,0,Motion_MOVE);
				curOffsetYCodeGap = GetCurCodeGap(TargetX,DirX);
			}
			FH_Float targetVel = curOffsetYCodeGap*Arc.factor;
			LogPrintf("axis=%d,curOffsetYCodeGap=%f,vel=%f\n",axis,curOffsetYCodeGap,targetVel);
			chassisTskparams.pg.creepVel = targetVel;
			chassisTskparams.pg.creepDistance = 0.1;
			CalcSPGFixParam(chassisTskparams.pg.creepVel,PI,100);
			LogPrintf("curCreepVel=%f\n",chassisTskparams.pg.creepVel);
			SetChassisMoveDecType(0);
			Arc.targetAng = targetH;
			ChassisMove_tsk(&chassisTskparams);
			//3,waitmove speed to connect arc motion
			chassisTskparams.targetX = TargetX;
	    	chassisTskparams.targetY = TargetY;
			chassisTskparams.targetHeading = cornerHeading;
			WaitVel4UCurveMove_tsk(&chassisTskparams);
			while(1)
			{
				ThreadSleep(20);
				if(GetTaskCnt(&Agv.service.mainControlTask) == 0)
					break;
			}
		}

	}
	else
	{
		SetError(ERROR_MoveParam_Wrong,1);
		LogPrintf("rightangle move params err\n");
	}
	return 0;
}

FH_ERROR_CODE DO_INSTRUCTION_MOVE(struct INSTRUCTION_Normal_params_t* moveParam)
{
	const char * const func_name = "INSTRUCTION_MOVE";
	int MapPointFlag = 0;
    /*******************start ��������******************************/
     int32 tmpx = moveParam->targetX;
     EndianConvert(&tmpx,sizeof(tmpx));
     int32 tmpy = moveParam->targetY;
     EndianConvert(&tmpy,sizeof(tmpy));
     int16 tmpHeading = moveParam->targetHeading;
     EndianConvert(&tmpHeading,sizeof(tmpHeading));
     FH_Float TargetX = SwitchQRCodeToAgv(tmpx,DirX);//((FH_Float)tmpx)/1000.0*CODE_GAP_X;
     FH_Float TargetY = SwitchQRCodeToAgv(tmpy,DirY);//((FH_Float)tmpy)/1000.0*CODE_GAP_Y;
	 
     movep.vel_rank = (tmpHeading & 0xF000)>>12;
     movep.dec_rank = (tmpHeading & 0xF00)>>8;
     movep.omega_rank = (tmpHeading & 0xF0)>>4;
     movep.epsi_rank = (tmpHeading & 0xF);
     extra_task = (enum ExtraTaskEnum)(moveParam->PodDirection & 0xFF);
    
	LogPrintf("%s:TargetX:%f,TargetY:%f\n",func_name,TargetX,TargetY);
     /*******************end ��������******************************/

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
		//��ǰ���˶�״̬���Ա���ٶ�
	    FH_ERROR_CODE rtn =  CalcMoveParams(&movep);
		if(rtn)
		{
			LogPrintf("=ERROR= move params wrong\n");
			return rtn ;
		}
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
	//�ж��Ƿ��ڳ���
	if (IsChargePoint(TargetX,TargetY))
	{
		int i = 0;
		for(i=0;i < App.map.ChargePointNUM;i++)
	    {
			LogPrintf("%s:id=%d,X:%f,TargetY:%f\n",func_name,i,App.map.chargePointInfo[i].baseX,App.map.chargePointInfo[i].baseY);
			ThreadSleep(10);
		}
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
	//�趨��������ͣ�����������HR��20180305
	float32 keepPaceDis =Param.AgvParam[PARAM_CODE_GAP_X]>Param.AgvParam[PARAM_CODE_GAP_Y]? 0.5*Param.AgvParam[PARAM_CODE_GAP_X]:0.5*Param.AgvParam[PARAM_CODE_GAP_Y];
	keepPaceDis =keepPaceDis > Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis] ? keepPaceDis:Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis];
	LogPrintf("WaitUnloadComplete3 stage=%d,isUnload=%d\n",Agv.service.beltManager.beltUnloadInfo.unloadSPGState,
		Agv.service.beltManager.beltUnloadInfo.isUnload);
	if(IsTargetInFront(TargetX,TargetY))
	{
		 //���ϼ���ʱ���ϰ�����ʧ��ִ�б��
		if(IsPGTakeOver(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg))
		{
			 WaitLastCmdComplete();
		}
		LogPrintf("WaitUnloadComplete4\n");
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
		//StopCurrentWaitMove();
		TaskReset(&Agv.service.mainControlTask);
		GetChissisTskParam(&chassisTskparams,TargetX,TargetY,0,Motion_MOVE);
		if(0 != extra_task)
			chassisTskparams.pg.creepVel = 4*chassisTskparams.pg.creepVel;
	    SetAppActionState(AGV_MOTION_MOVING);
		//if(IsSPGDecState(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.spg))
		if(FALSE_FH)
		{
			if(++NewTargetWhenDecCnt>=3)
			{
				NewTargetWhenDecCnt = 3;
			}
			//���ٶ��յ�ָ��ʱ�����ٶȼ����㣬Ȼ�󰴵�ǰ�ٶ��ߵ�Ŀ��λ
			
			ChassisMove_tsk(&chassisTskparams);
			LogPrintf("wait dec stable:%d\n",1000*NewTargetWhenDecCnt);
			ThreadSleep(1000*NewTargetWhenDecCnt);
			//1��ʱ��������Ȼ���˶����˶��������жΣ������˶��ٶ�
			targetDis = PGGetTargetDistance(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg);
			if((targetDis>0.1)&&(IsAgvMotionState()))
			{
				LogPrintf("vel change:%f\n",chassisTskparams.pg.maxVel);
				ChangeCurMoveVel_tsk(&chassisTskparams.pg.maxVel);
			}
			
		}
		else
		{
			NewTargetWhenDecCnt = 0;
			SetChassisMoveDecType(0);
			ChassisMove_tsk(&chassisTskparams);
		}
	    
		if(FH_FALSE == IsTaskActionOK())
		{
			LogPrintf("move exception, wait last move done\n");
			WaitLastCmdComplete_fun();
			ChassisMove_tsk(&chassisTskparams);
		}
		timeOutCnt = 1000+100*PGGetTargetTime(&Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg);
		//timeOutCnt = 1000+100*(MoveDistance/chassisTskparams.pg.maxVel+chassisTskparams.pg.maxAcc/chassisTskparams.pg.maxVel);
		LogPrintf("newDis=%f,newtime=%d\n",MoveDistance,timeOutCnt);
		WaitMoveDone_tsk(&timeOutCnt);
	}
	else
	{
		NewTargetWhenDecCnt = 0;
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
		MapPointFlag = GetMapPoint(Agv.service.chassis.location.XRoundToQR,Agv.service.chassis.location.YRoundToQR);
		if((MapPointFlag==0)&&
			((fabs(Agv.service.chassis.location.curPosGnd.x - Agv.service.chassis.location.XRoundToQR)>=Param.AgvParam[PARAM_STATE_AJUST_OFFSET_X])||
			(fabs(Agv.service.chassis.location.curPosGnd.y -Agv.service.chassis.location.YRoundToQR)>=Param.AgvParam[PARAM_STATE_AJUST_OFFSET_Y])))
		{
			if(0 == isTrayLift)
			{
				targetHeading += ((targetHeading>PI)? -2.0*PI : 2.0*PI);
			}
		}

		LogPrintf("heading target:%f\n",targetHeading);
		SetAppActionState(AGV_MOTION_TURNING);
		GetChissisTskParam(&chassisTskparams,curX,curY,targetHeading,Motion_TURN);
		//�ƶ�ǰ��ת,��ת����ٶ�4*creepV(�ж���ת�Ƕ�?)
		chassisTskparams.pg.creepVel = 4*chassisTskparams.pg.creepVel;//4*Param.AgvParam[PARAM_AgvMinSPD]/Param.AgvParam[PARAM_WheelDis];
		ChassisTurning_tsk(&chassisTskparams);
		timeOutCnt = 3000;//��λ10ms
	    WaitMoveDone_tsk(&timeOutCnt);
		WaitLastCmdComplete();
		SetAppActionState(AGV_MOTION_MOVING);
		GetChissisTskParam(&chassisTskparams,TargetX,TargetY,0,Motion_MOVE);
		if(0 != extra_task)
			chassisTskparams.pg.creepVel = 4*chassisTskparams.pg.creepVel;
	    ChassisMove_tsk(&chassisTskparams);
		timeOutCnt = 2000+100*(MoveDistance/chassisTskparams.pg.maxVel+chassisTskparams.pg.maxAcc/chassisTskparams.pg.maxVel);
		LogPrintf("MoveDis=%f,Movetime=%d\n",MoveDistance,timeOutCnt);
		WaitMoveDone_tsk(&timeOutCnt);
		//��ԭĬ���ٶ�
		IsCustomIDVaild = 0;
	}

	struct ChassisSearchCodeTskParams_t searchCodeParams;
	searchCodeParams.DeltaDis = 0.02;
	searchCodeParams.curDis = 0;
	searchCodeParams.tickTimeOut = 2000;
	searchCodeParams.IsFindDmStop = 1;
	SerchCodeAdjustExposureAfterMove_tsk(&searchCodeParams);

	if(0 == extra_task)
	{
		// �˴�Ϊ����ʵʱ��������
		MapPointFlag = GetMapPoint(TargetX,TargetY);
		if(MapPointFlag)
		{
			SetError(ERROR_LoadPosPoint,1);//����111������ʾ
			LogPrintf("MapPoint:TargetX=%f,TargetY=%f\n",TargetX,TargetY);
			
		}
		else
		{
			//�ж������Ƿ���
			if(0 == IsTrayLiftUpState())
			{
				timeOutCnt = 2000;
				GetChissisTskParam(&chassisTskparams,Param.AgvParam[PARAM_STATE_AJUST_OFFSET_X],Param.AgvParam[PARAM_STATE_AJUST_OFFSET_Y],0,Motion_TURN);
				ChassisTurnAfterMove_tsk(&chassisTskparams);
				WaitMoveDone_tsk(&timeOutCnt);
			}
		}
	}
	else
	{
		LogPrintf("wait to do extra_task,vel=%f,omg=%f\n",AGV.V,AGV.Omg);
		FH_Float adjusttargetheading = 0;
		switch(extra_task)
		{
			case TURN_0:
			case PODCHG_0:
				adjusttargetheading = 0;
				break;
			case TURN_P90:
			case PODCHG_P90:
				adjusttargetheading= PI/2;
				break;
			case TURN_N90:
			case PODCHG_N90:
				adjusttargetheading= -PI/2;
				break;
			case TURN_180:
			case PODCHG_180:
				adjusttargetheading= PI;
				break;
		}
		timeOutCnt = 1000;//��λ10ms
		int isCloseLoop = 1;
		if(extra_task>=PODCHG_0)
		{
			if(0 == IsTrayLiftUpState())
			{
				SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Extra_Task,WarningB);
			}
			//���ܻ���
			//����action״̬����Ӧԭ�˶�״̬
			SetAppActionState(AGV_MOTION_PODCHANGE);
			isCloseLoop = 0;
			TrayCloseLoop_tsk(&isCloseLoop);
		}
		else
		{
			//�ջ�����
			isCloseLoop = IsTrayLiftUpState();
			TrayCloseLoop_tsk(&isCloseLoop);
		}
		FH_Float targetX = TargetX;//Agv.service.chassis.location.XRoundToQR;
		FH_Float targetY = TargetY;//Agv.service.chassis.location.YRoundToQR;
		ConvertAngleNearest(&adjusttargetheading,Agv.service.chassis.location.curPosGnd.heading);
		GetChissisTskParam(&chassisTskparams,targetX,targetY,adjusttargetheading,Motion_TURN);
		ChassisTurning_tsk(&chassisTskparams);
		WaitMoveDone_tsk(&timeOutCnt);
		//��������
		isCloseLoop = 0;
		TrayCloseLoop_tsk(&isCloseLoop);
		WaitLastCmdComplete();
	}
	/**********end*ָ���****************************/
     return 0;
 }
//#define DO_INSTRUCTION_KEEPPACEWITH_AGVFRONT 52
FH_ERROR_CODE DO_INSTRUCTION_KEEPPACEWITH_AGVFRONT(struct INSTRUCTION_Normal_params_t * moveParam)
{
  
	FH_ERROR_CODE re;
	re =DO_INSTRUCTION_MOVE(moveParam);
	if (re !=0)
		return re;
	//�趨�����������ģʽ�������Ƿ�����趨�����
	int isTrayLift = IsTrayLiftUpState();
	if (isTrayLift)
	{
		SetKeepPaceParams(Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis]);
	}
	else
	{
		SetKeepPaceParams(Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis]);
		//SetKeepPaceParams(���ز���-(���ܳߴ�-�����ߴ�));
	}
	return 0;
}
//#define DO_INSTRUCTION_KEEPPACE 88
FH_ERROR_CODE DO_INSTRUCTION_KEEPPACE(struct INSTRUCTION_Normal_params_t * moveParam)
{
	FH_ERROR_CODE re;
	int16 targetHeading = moveParam->targetHeading;
	int16 PodDirection = moveParam->PodDirection;
	moveParam->targetHeading = 0;
	moveParam->PodDirection = 0;
		re =DO_INSTRUCTION_MOVE(moveParam);
	if (re !=0)
		return re;
	//�趨�����������ģʽ�������Ƿ�����趨�����
	int isTrayLift = IsTrayLiftUpState();
	if (isTrayLift)
	{
		SetKeepPaceParams(Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis]);
	}
	else
	{
		SetKeepPaceParams(Param.AgvParam[PARAM_KeepPace_OffsetCompensateDis]);
		//SetKeepPaceParams(���ز���-(���ܳߴ�-�����ߴ�));
	}
	WaitDisComplete();
	moveParam->targetHeading = targetHeading;
	moveParam->PodDirection = PodDirection;
	DO_INSTRUCTION_LOAD(moveParam);
	return 0;
}

//С������ 
//��move��������ǲ���ת���෴
//#define DO_INSTRUCTION_BACK_OFF 83
FH_ERROR_CODE DO_INSTRUCTION_BACK_OFF(struct INSTRUCTION_Normal_params_t* moveParam)
{
  /*******************start ��������******************************/
     int32 tmpx = moveParam->targetX;
     EndianConvert(&tmpx,sizeof(tmpx));
     int32 tmpy = moveParam->targetY;
     EndianConvert(&tmpy,sizeof(tmpy));
     int16 tmpHeading = moveParam->targetHeading;
     EndianConvert(&tmpHeading,sizeof(tmpHeading));
     FH_Float TargetX = SwitchQRCodeToAgv(tmpx,DirX);//((FH_Float)tmpx)/1000.0*CODE_GAP_X;
     FH_Float TargetY = SwitchQRCodeToAgv(tmpy,DirY);//((FH_Float)tmpy)/1000.0*CODE_GAP_Y;
	 
     movep.vel_rank = (tmpHeading & 0xF000)>>12;
     movep.dec_rank = (tmpHeading & 0xF00)>>8;
     movep.omega_rank = (tmpHeading & 0xF0)>>4;
     movep.epsi_rank = (tmpHeading & 0xF);
     extra_task = (enum ExtraTaskEnum)(moveParam->PodDirection & 0xFF);
     /*******************end ��������******************************/
	 /**********start**�������****************************/
		
	//����ֱ��׷�ӣ�����һ���������ִ��
	int isOneLineMove = IsTargetInFront(TargetX,TargetY);
	if(0 == isOneLineMove)
	{
		WaitLastCmdComplete();
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
	Uint16 timeOutCnt = 1000;
	struct ChassisTskParams_t chassisTskparams;

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
	 timeOutCnt = 1000;//��λ10ms
	 WaitMoveDone_tsk(&timeOutCnt);
	 //������ʱ�ջ�
	 int isCloseLoop = isTrayLift;
	 TrayCloseLoop_tsk(&isCloseLoop);
	 
	 FH_Float curX = Agv.service.chassis.location.XRoundToQR;
	 FH_Float curY = Agv.service.chassis.location.YRoundToQR;
	 FH_Float targetHeading = 0;
	 if(IsEqual(TargetX,curX))
	 {
		 targetHeading = (TargetY>curY)? -PI*0.5:0.5*PI;
		 ConvertAngleNearest(&targetHeading,Agv.service.chassis.location.curPosGnd.heading);
	 }
	 else if(IsEqual(TargetY,curY))
	 {
		 targetHeading = (TargetX>curX)?-PI:0;
		 ConvertAngleNearest(&targetHeading,Agv.service.chassis.location.curPosGnd.heading);
	 }
	 else
	 {
		 LogPrintf("heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
		 SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,WarningA);
		 return -1;
		 
	 }
	SetAppActionState(AGV_MOTION_TURNING);
	GetChissisTskParam(&chassisTskparams,curX,curY,targetHeading,Motion_TURN);
	ChassisTurning_tsk(&chassisTskparams);
	WaitMoveDone_tsk(&timeOutCnt);
	SetAppActionState(AGV_MOTION_MOVING);
	GetChissisTskParam(&chassisTskparams,TargetX,TargetY,0,Motion_MOVE);
	ChassisMove_tsk(&chassisTskparams);
	WaitMoveDone_tsk(&timeOutCnt);
	//��������
	isCloseLoop = 0;
	TrayCloseLoop_tsk(&isCloseLoop);
	WaitLastCmdComplete();
 //CompatiableFun(Param);
	 return 0;
}

//#define DO_INSTRUCTION_CHARGEMOVE 26
FH_ERROR_CODE DO_INSTRUCTION_CHARGEMOVE(struct INSTRUCTION_Normal_params_t* moveParam)
{
	/*******************start ��������******************************/
	int32 tmpx = moveParam->targetX;
	EndianConvert(&tmpx,sizeof(tmpx));
	int32 tmpy = moveParam->targetY;
	EndianConvert(&tmpy,sizeof(tmpy));
	int16 tmpHeading = moveParam->targetHeading;
	EndianConvert(&tmpHeading,sizeof(tmpHeading));
	FH_Float TargetX = SwitchQRCodeToAgv(tmpx,DirX);//((FH_Float)tmpx)/1000.0*CODE_GAP_X;
	FH_Float TargetY = SwitchQRCodeToAgv(tmpy,DirY);//((FH_Float)tmpy)/1000.0*CODE_GAP_Y;

	if ((fabs(TargetX - Agv.service.chassis.location.curPosGnd.x)>(1.1*GetCurCodeGap(TargetX,DirX)))||
		(fabs(TargetY - Agv.service.chassis.location.curPosGnd.y)>(1.1*GetCurCodeGap(TargetY,DirY))))
	{
		SetError(ERROR_ChargeMoveParam, 3);
	}
	
	if(0 == tmpHeading) 
	{
		CHARGE_CODE_GAP = Param.AgvParam[PARAM_CHARGE_CODE_GAP];
	} 
	else 
	{
		CHARGE_CODE_GAP = ((double)tmpHeading)/1000.0;
	}
	if(fabs(CHARGE_CODE_GAP) < 0.001) 
	{
    	CHARGE_USE_HALF_CODE_GAP = 0;
    }
	else
    {
    	CHARGE_USE_HALF_CODE_GAP = 1;
	}
	// if()
	 /*******************end ��������******************************/
	Agv.device.battery.batteryBase.PowerOff48V = 1;
	 /**********start**�������****************************/
	if (IsAgvMotionState())
	{
		//ֱ�ӱ�������˶����յ����ָ��
		SetErrorToQue((ErrorTypeEnum)ERROR_CHARGECMD_IN_MOVING,ErrorC);
		return -1;
	}

	if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
		LogPrintf("=ERROR= agv not ready\n");
		return -1;
	}
	if(IsTrayLiftUpState())
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_LoadedDisapproveChargedMove,ErrorC);
		LogPrintf("=ERROR= tray lift cant charge\n");
		return -2;
	}
	//�Բ������б���������С��һֱǰ��
    if((Param.AgvParam[PARAM_TargetTolX]-Param.AgvParam[PARAM_ChargeCompensateDis])<0.001)
    {
        SetErrorToQue((ErrorTypeEnum)ERROR_Charge_Params,ErrorC);
        return -3;
    }
	if(IsOutOfBoard(TargetX,TargetY))
	{
        SetErrorToQue((ErrorTypeEnum)ERROR_TargetOutBorder,ErrorC);
		return -4;
	}
	/**********end**�������****************************/
	
	/**********start*ָ���****************************/
	IsCustomIDVaild = 0;//ȡ���Զ����ٶ�
	if ((fabs(TargetX - Agv.service.chassis.location.curPosGnd.x)<(0.1))&&
			 (fabs(TargetY - Agv.service.chassis.location.curPosGnd.y)<(0.1)))
	{
		LogPrintf("do not move\n");
		
	}
	else
	{
		Uint16 timeOutCnt = 2000;
		struct ChassisTskParams_t chassisTskparams;
		 FH_Float curX = Agv.service.chassis.location.XRoundToQR;
		 FH_Float curY = Agv.service.chassis.location.YRoundToQR;
		 FH_Float targetHeading = 0;
		 FH_Float halfTargetX = TargetX;
		 FH_Float halfTargetY = TargetY;
		 FH_Float halfCodeGap= 0;
		 if(IsEqual(TargetX,curX))
		 {
			 targetHeading = (TargetY>curY)? -PI*0.5:0.5*PI;
			if(1==CHARGE_USE_HALF_CODE_GAP)
			{
				targetHeading = -1*targetHeading;
			}
			 ConvertAngleNearest(&targetHeading,Agv.service.chassis.location.curPosGnd.heading);
			 halfTargetX = TargetX;
			 halfTargetY = TargetY - Sign(TargetY - curY)*CHARGE_CODE_GAP;
			 
//			 halfCodeGap = (TargetY>curY)? (CODE_GAP_Y - CHARGE_CODE_GAP) : CHARGE_CODE_GAP;
			 halfCodeGap = (TargetY>curY)? (GetCurCodeGap(TargetY,DirY) - CHARGE_CODE_GAP) : CHARGE_CODE_GAP;
		 }
		 else if(IsEqual(TargetY,curY))
		 {
			 targetHeading = (TargetX>curX)?-PI:0;
			 if(1==CHARGE_USE_HALF_CODE_GAP)
			{
				targetHeading = targetHeading+PI;
			}
			 ConvertAngleNearest(&targetHeading,Agv.service.chassis.location.curPosGnd.heading);
			 halfTargetX = TargetX - Sign(TargetX - curX)*CHARGE_CODE_GAP;
			 halfTargetY = TargetY;
// 		  halfCodeGap = (TargetX>curX)?(CODE_GAP_X - CHARGE_CODE_GAP): CHARGE_CODE_GAP;
		 	 halfCodeGap = (TargetX>curX)?(GetCurCodeGap(TargetX,DirX) - CHARGE_CODE_GAP): CHARGE_CODE_GAP;

		 }
		 else
		 {
			 LogPrintf("heading exception,targetx:%f.targetY:%f\n",TargetX,TargetY);
			 SetAppErrorToQue(APP_ERROR_INSTRUCTION_Move_Target,WarningA);
			 return -1;
			 
		 }
		SetAppActionState(AGV_MOTION_TURNING);
		GetChissisTskParam(&chassisTskparams,curX,curY,targetHeading,Motion_TURN);
		ChassisTurning_tsk(&chassisTskparams);
		WaitMoveDone_tsk(&timeOutCnt);
		if(1 == CHARGE_USE_HALF_CODE_GAP)
		{
			LogPrintf("halfcode x:%f,y:%f\n",halfTargetX,halfTargetY);
			//�����״�
			int isOn = 1;
			if(LaserSwitch(&isOn))
			{
				LogPrintf("laser open faild\n");
				return -1;
			}
			QRCodeSetDecodeMode(&Agv.device.qrcode,QRCODE_DECODEMODE_NORMAL);
			ThreadSleep(30);
			SetQRHalfCodeGap(halfCodeGap);
			SetAppActionState(AGV_MOTION_MOVING);
			GetChissisTskParam(&chassisTskparams,halfTargetX,halfTargetY,0,Motion_MOVE);
			ChassisMove_tsk(&chassisTskparams);
			WaitMoveDone_tsk(&timeOutCnt);
			struct ChassisSearchCodeTskParams_t searchCodeParams;
			searchCodeParams.DeltaDis = 0.02;
			searchCodeParams.curDis = 0;
			searchCodeParams.tickTimeOut = 2000;
			searchCodeParams.IsFindDmStop = 1;
			SerchCodeAdjustExposureAfterMove_tsk(&searchCodeParams);
			WaitLastCmdComplete();
			SetAppActionState(AGV_MOTION_TURNING);
			targetHeading = targetHeading+PI;
			GetChissisTskParam(&chassisTskparams,halfTargetX,halfTargetY,targetHeading,Motion_TURN);
			ChassisTurning_tsk(&chassisTskparams);
			WaitMoveDone_tsk(&timeOutCnt);
		}
		else
		{
			int tempClose = 0;
			LaserSwitch(&tempClose);
		}
		WaitLastCmdComplete();
		LogPrintf("charge move x:%f,y:%f\n",TargetX,TargetY);
		SetAppActionState(AGV_MOTION_MOVING);
		GetChissisTskParam(&chassisTskparams,TargetX,TargetY,0,Motion_MOVE);
		chassisTskparams.pg.maxAcc = LoadCondition.MaxAcc[2];
	    chassisTskparams.pg.maxVel = LoadCondition.MaxSpd[2];
	    chassisTskparams.pg.maxDec = LoadCondition.MaxAcc[2];
		chassisTskparams.pg.creepDistance = Param.AgvParam[PARAM_ChargeDeceDisCompensate];
		ChassisMove_tsk(&chassisTskparams);
		WaitMoveDone_tsk(&timeOutCnt);
		WaitLastCmdComplete();
		//PARAM_ChargeCompensateDis+PARAM_ChargeTouchPointTol
		if(Param.AgvParam[PARAM_ChargeCompensateDis]>=0.001)
		{
			struct ChassisSearchCodeTskParams_t searchCodeParams;
			searchCodeParams.DeltaDis = -1*(Param.AgvParam[PARAM_ChargeCompensateDis]);
			searchCodeParams.curDis = 0;
			searchCodeParams.tickTimeOut = 2000;
			searchCodeParams.IsFindDmStop = 0;
			SerchCodeMove_tsk(&searchCodeParams);
			WaitLastCmdComplete();
		}
	}
	/***********�������Ⲣ���ó�ʼֵ*******************/
	int waitCnt = 5;
	/***********���̵�������*******************/
	while(1)
	{
		if(Agv.device.battery.batteryBase.QuantityElectric>=51.5)
			break;
		SetChargeRelay(ON);
		ThreadSleep(2000);
		if(IsBatteryCharging())
		{
			break;
		}
		else if((2 == waitCnt)&&(Param.AgvParam[PARAM_ChargeTouchPointTol]>=0.001))
		{
			struct ChassisSearchCodeTskParams_t searchCodeParams;
			searchCodeParams.DeltaDis = -1*Param.AgvParam[PARAM_ChargeTouchPointTol];
			searchCodeParams.curDis = 0;
			searchCodeParams.tickTimeOut = 2000;
			searchCodeParams.IsFindDmStop = 0;
			SerchCodeMove_tsk(&searchCodeParams);
			WaitLastCmdComplete();
		}
		if(waitCnt-- <0)
		{
			SetChargeRelay(OFF);
			ThreadSleep(2000);
			LogPrintf("ChargeMove:Failed-%d\n",5-waitCnt);
			SetErrorToQue(SYSTEM_ERROR_Battery_WaitPowerTimeOut,ErrorC);
			SetSRBatteryDataFlag((struct BatterySR_t*)&Agv.device.battery.batteryBase,1);
			return -1;
		}
	}
	 return 0;
}
//#define DO_INSTRUCTION_EXIT_CHARGINGZONE 45
FH_ERROR_CODE DO_INSTRUCTION_EXIT_CHARGINGZONE(struct INSTRUCTION_Normal_params_t* moveParam)
{
	/*******************start ��������******************************/
	int32 tmpx = moveParam->targetX;
	EndianConvert(&tmpx,sizeof(tmpx));
	int32 tmpy = moveParam->targetY;
	EndianConvert(&tmpy,sizeof(tmpy));
	int16 tmpHeading = moveParam->targetHeading;
	EndianConvert(&tmpHeading,sizeof(tmpHeading));
//		 FH_Float TargetX = ((FH_Float)tmpx)/1000.0*CODE_GAP_X;
//		 FH_Float TargetY = ((FH_Float)tmpy)/1000.0*CODE_GAP_Y;
	FH_Float TargetX = SwitchQRCodeToAgv(tmpx,DirX);
	FH_Float TargetY = SwitchQRCodeToAgv(tmpy,DirY);

	movep.vel_rank = (tmpHeading & 0xF000)>>12;
	movep.dec_rank = (tmpHeading & 0xF00)>>8;
	movep.omega_rank = (tmpHeading & 0xF0)>>4;
	movep.epsi_rank = (tmpHeading & 0xF);
	extra_task = (enum ExtraTaskEnum)(moveParam->PodDirection & 0xFF);
	/*******************end ��������******************************/

	/**********start**�������****************************/

	if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum)Error_NoLS,ErrorC);
		return -1;
	}
	if(AGV.DebugMode == 1)
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_DEBUGMODE_RejectRunOrders,ErrorC);
		return -2;
	}
	 if(IsOutOfBoard(TargetX,TargetY))
	{
        SetErrorToQue((ErrorTypeEnum)ERROR_TargetOutBorder,ErrorC);
		return -4;
	}
	 //�˳��������Ŀ��λ�����ϸ��߼�Ҫ��
	 FH_Float outHeading = 0;
	 GetNominalHeadingOutAxis(Agv.service.chassis.location.curPosGnd.heading,&outHeading);
    if(IsEqual(outHeading,0))
	{
	    if((fabs(TargetY - Agv.service.chassis.location.YRoundToQR)>0.01)||
			(TargetX<Agv.service.chassis.location.XRoundToQR))
	    {
	        SetErrorToQue((ErrorTypeEnum)ERROR_TargetPos,ErrorC);
		   	return -1;
	    }
	}
	else if(IsEqual(outHeading,PI))
	{
		if((fabs(TargetY - Agv.service.chassis.location.YRoundToQR)>0.01)||
			(TargetX>Agv.service.chassis.location.XRoundToQR))
	    {
	        SetErrorToQue((ErrorTypeEnum)ERROR_TargetPos,ErrorC);
		   	return -1;
	    }
	}
	else if(IsEqual(outHeading,PI/2.0))
	{
	    if((fabs(TargetX - Agv.service.chassis.location.XRoundToQR)>0.01)||
			(TargetY<Agv.service.chassis.location.YRoundToQR))
	    {
	        SetErrorToQue((ErrorTypeEnum)ERROR_TargetPos,ErrorC);
		   	return -1;
	    }
	}
	else if(IsEqual(outHeading,-PI/2.0))
	{
	    if((fabs(TargetX - Agv.service.chassis.location.XRoundToQR)>0.01)||
			(TargetY>Agv.service.chassis.location.YRoundToQR))
	    {
	        SetErrorToQue((ErrorTypeEnum)ERROR_TargetPos,ErrorC);
		   	return -1;
	    }
	}
	/**********end**�������****************************/
	/**********start*ָ���****************************/
	int waitCnt = 20;
	while(1)
	{
		SetChargeRelay(OFF);
		ThreadSleep(2000);
		if(0 == IsBatteryCharging())
		{
			break;
		}
		if(waitCnt-- <0)
		{
			SetErrorToQue((ErrorTypeEnum)ERROR_Wait_Exit_Time_Out,ErrorC);
			return -1;
		}
	}
	ThreadSleep(1000);

	SetAppActionState(AGV_MOTION_MOVING);
	Uint16 timeOutCnt = 1000;
	struct ChassisTskParams_t chassisTskparams;
	GetChissisTskParam(&chassisTskparams,TargetX,TargetY,0,Motion_MOVE);
	chassisTskparams.pg.maxAcc = LoadCondition.MaxAcc[2];
    chassisTskparams.pg.maxVel = LoadCondition.MaxSpd[2];
    chassisTskparams.pg.maxDec = LoadCondition.MaxAcc[2];
    ChassisMove_tsk(&chassisTskparams);
    WaitMoveDone_tsk(&timeOutCnt);
	WaitLastCmdComplete();
	Agv.device.battery.batteryBase.PowerOff48V = 0;
	 return 0;
}

//#define DO_INSTRUCTION_FORCEMOVE 23
FH_ERROR_CODE DO_INSTRUCTION_FORCEMOVE(int* param)
{
	/*if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
		LogPrintf("=ERROR= agv not ready\n");
		return -1;
	}
	BeginDeploy;
	InstructionRaw.Type = INSTRUCTION_FORCEMOVE;
	AGV.ForceMove = 1;
	AGV.ComDecelDis = Param.AgvParam[PARAM_ChargeDeceDisCompensate];				
	if (AGV.MotionStatus ==  AGV_MOTION_MOVING)
		DeployMoveInstructionWithCMD();
	else
	{
		ResetTask();	 
		DeployMoveInstruction();   
	}
	EndDeploy;*/

//CompatiableFun(Param);
//WaitLastCmdComplete();
  return 0;
}

 //#define DO_INSTRUCTION_TURNING 16 // turning
FH_ERROR_CODE DO_INSTRUCTION_TURNING(struct INSTRUCTION_TURNING_PARAMS_t* turningParam)
{
	 /*******************start ��������******************************/
	 int16 tmpHeading = turningParam->targetHeading;
	 EndianConvert(&tmpHeading,sizeof(turningParam->targetHeading));
	 FH_Float tarH = tmpHeading*0.01*PI/180;
	  movep.vel_rank = 0;//(tmpHeading & 0xF000)>>12;
     movep.dec_rank = 0;//(tmpHeading & 0xF00)>>8;
     movep.omega_rank = turningParam->omega_rank;
     movep.epsi_rank = turningParam->epsi_rank;
     //extra_task = (enum ExtraTaskEnum)(turningParam->PodDirection & 0xFF);
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

	WaitLastCmdComplete();
	
	AGV.Heading = WrapTo2PiDe(AGV.Heading);
	return 0;
}
 //#define DO_INSTRUCTION_CHANGEPODDIRECTION 21 // the pod turning with the agv
FH_ERROR_CODE DO_INSTRUCTION_CHANGEPODDIRECTION(struct INSTRUCTION_TURNING_PARAMS_t* turningParam)
{
	 /*******************start ��������******************************/

	int16 tmpHeading = turningParam->targetHeading;
	EndianConvert(&tmpHeading,sizeof(turningParam->targetHeading));
	FH_Float tarH = tmpHeading*0.01*PI/180;
	 movep.vel_rank = 0;//(tmpHeading & 0xF000)>>12;
     movep.dec_rank = 0;//(tmpHeading & 0xF00)>>8;
     movep.omega_rank = turningParam->omega_rank;
     movep.epsi_rank = turningParam->epsi_rank;
	//extra_task = (enum ExtraTaskEnum)(turningParam->PodDirection & 0xFF);
	FH_ERROR_CODE rtn =  CalcMoveParams(&movep);
	if(rtn)
	{
		return rtn ;
	}
	 /*******************end ��������******************************/

	if(0 == IsAllMotorEnable())
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_MotorDisable_RejectOrder,ErrorC);
		return -1;
	}
	if(0 == IsAgvReadyToMove())
	{
		SetErrorToQue((ErrorTypeEnum) Error_NoLS,ErrorC);
		LogPrintf("=ERROR= agv not ready\n");
		return -1;
	}
	if (0 == IsTrayLiftUpState())
	{
		return -2;
	}
	/**********start*ָ���****************************/
	
	Uint16 waitCnt = 2000;//��λ10ms

	SetAppActionState(AGV_MOTION_POD_ROTATE);
	FH_Float target=0;
	GetNominalHeading(Agv.service.tray.Heading2Ground,&target);
	ConvertAngleNearest(&target,Agv.service.tray.Heading2Ground);
	struct TrayTskParams_t trayTskParams;
	GetTrayTskParam(&trayTskParams,target,Tray_RotateToGround);
	trayTskParams.pg.maxVel = Degree2Rad(5);
	TrayRotate_tsk(&trayTskParams);
	WaitMoveDone_tsk(&waitCnt);
	WaitLastCmdComplete();
	//����action״̬����Ӧԭ�˶�״̬
	SetAppActionState(AGV_MOTION_PODCHANGE);
	struct ChassisTskParams_t params;
	FH_Float targetX = Agv.service.chassis.location.XRoundToQR;
	FH_Float targetY = Agv.service.chassis.location.YRoundToQR;
	ConvertAngleNearest(&tarH,Agv.service.chassis.location.curPosGnd.heading);
	GetChissisTskParam(&params,targetX,targetY,tarH,Motion_TURN);
	params.pg.maxAcc = (0 == movep.epsi_rank)?LoadCondition.ChangeDirectionAngularAcc[LoadCondition.ID] : movep.epsi_max;
	params.pg.maxVel = (0 == movep.omega_rank)?LoadCondition.MaxChangeDirectionAngularSpd[LoadCondition.ID] : movep.omega_max;
	if(params.pg.maxAcc>LoadCondition.ChangeDirectionAngularAcc[LoadCondition.ID])
	{
		params.pg.maxAcc = LoadCondition.ChangeDirectionAngularAcc[LoadCondition.ID];
	}
	if(params.pg.maxVel>LoadCondition.MaxChangeDirectionAngularSpd[LoadCondition.ID])
	{
		params.pg.maxVel = LoadCondition.MaxChangeDirectionAngularSpd[LoadCondition.ID];
	}
	params.pg.maxDec = params.pg.maxAcc;
	ChassisTurning_tsk(&params);

	WaitMoveDone_tsk(&waitCnt);
	WaitLastCmdComplete();
	/*BeginDeploy;
		Instruction.Type = INSTRUCTION_CHANGEPODDIRECTION;
		//CompatiableFun(turningParam);
		ResetTask();
		DeployPodChange();
	EndDeploy;

	WaitLastCmdComplete();*/
	return 0;
}

//#define DO_INSTRUCTION_SEARCHCODE_MOVE 33
FH_ERROR_CODE DO_INSTRUCTION_SEARCHCODE_MOVE(struct INSTRUCTION_Normal_params_t* normalParam)
{
	if(0 == IsAllMotorEnable())
	{
		SetErrorToQue((ErrorTypeEnum) ERROR_MotorDisable_RejectOrder,ErrorC);
		LogPrintf("=ERROR= motor disable\n");
		return -1;
	}
	
	EndianConvert(&normalParam->targetX,sizeof(normalParam->targetX));
	if(normalParam->targetX>200)
	{
		normalParam->targetX =200;
	}
	SetAppActionState(AGV_MOTION_SEARCHCODE_MOVE);
	struct ChassisSearchCodeTskParams_t searchCodeParams;
	searchCodeParams.DeltaDis = normalParam->targetX*0.001;
	searchCodeParams.curDis = 0;
	searchCodeParams.tickTimeOut = 2000;
	searchCodeParams.IsFindDmStop = 1;
	SerchCodeMove_tsk(&searchCodeParams);
	//CompatiableFun(Param);
	WaitLastCmdComplete();
	 return 0;
}
//#define DO_INSTRUCTION_JOYSTICK_MOVE_POS_MODE 56
FH_ERROR_CODE DO_INSTRUCTION_JOYSTICK_MOVE_POS_MODE(struct INSTRUCTION_Normal_params_t* normalParam)
{
	if(0 == IsAllMotorEnable())
	{
		SetErrorToQue((ErrorTypeEnum) ERROR_MotorDisable_RejectOrder,ErrorC);
		LogPrintf("=ERROR= motor disable\n");
		return -1;
	}
	
	EndianConvert(&normalParam->targetX,sizeof(normalParam->targetX));
	
	SetAppActionState(AGV_MOTION_SEARCHCODE_MOVE);
	struct ChassisSearchCodeTskParams_t searchCodeParams;
	searchCodeParams.DeltaDis = normalParam->targetX*0.001;
	searchCodeParams.curDis = 0;
	searchCodeParams.tickTimeOut = 2000;
	searchCodeParams.IsFindDmStop = 0;
	SerchCodeMove_tsk(&searchCodeParams);
	//CompatiableFun(Param);
	WaitLastCmdComplete();
	 return 0;
}

//#define DO_INSTRUCTION_SEARCHCODE_TURNING 34
FH_ERROR_CODE DO_INSTRUCTION_SEARCHCODE_TURNING(struct INSTRUCTION_Normal_params_t* normalParam)
{
	EndianConvert(&normalParam->targetHeading,sizeof(normalParam->targetHeading));
	Uint16 waitCnt = 1000;
	if (IsCurPosInChargePoint())
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_MoveTargetPointInChargeZone,ErrorC);
		return -1;
	}

	if (IsTrayLiftUpState())
	{
		SetAppActionState(AGV_MOTION_POD_ZERO);
		FH_Float target=0;
		GetNominalHeading(Agv.service.tray.Heading2Ground,&target);
		ConvertAngleNearest(&target,Agv.service.tray.Heading2Ground);
		struct TrayTskParams_t trayTskParams;
		GetTrayTskParam(&trayTskParams,target,Tray_RotateToGround);
		TrayRotate_tsk(&trayTskParams);
		WaitMoveDone_tsk(&waitCnt);
		//�ջ�����
		int isCloseLoop = 1;
		TrayCloseLoop_tsk(&isCloseLoop);
	}
	SetAppActionState(AGV_MOTION_SEARCHCODE_TURNING);
	struct ChassisTskParams_t chassisTskparams;
	FH_Float targetHeading = ((double)normalParam->targetHeading)*0.01 * PI /180.0;
	LogPrintf("searchCode Heading:%f\n",targetHeading);
	//ConvertAngleNearest(&targetHeading,Agv.service.chassis.location.curPosGnd.heading);
	FH_Float curX = Agv.service.chassis.location.XRoundToQR;
	FH_Float curY = Agv.service.chassis.location.YRoundToQR;
	GetChissisTskParam(&chassisTskparams,curX ,curY,targetHeading,Motion_TURN);
	chassisTskparams.pg.maxVel = 0.2;
	chassisTskparams.pg.targetType = TargetType_Offset;
	chassisTskparams.esParams.stopConditionType = StopCondition_IsFindDM;
	chassisTskparams.esParams.stopTarget = 1;
	chassisTskparams.esParams.IsEmc = 0;
	ChassisTurning_tsk(&chassisTskparams);
	//CompatiableFun(Param);
	WaitLastCmdComplete();

	return 0;
}


//��ָ���Ѳ�ʹ��
//#define DO_INSTRUCTION_MEASUREDIA_MOVE 35
FH_ERROR_CODE DO_INSTRUCTION_MEASUREDIA_MOVE(int* Param)
{
	/*Instruction.Type = INSTRUCTION_MEASUREDIA_MOVE;
	FH_ERROR_CODE rtn = DO_INSTRUCTION_MOVE();
	if(rtn)
	{
		return rtn;
	}
	WaitLastCmdComplete();*/
	return 0;
}

/*----------------------------------------------------------------------------*/
FH_ERROR_CODE DnCameraFindTargetAngle(struct GuideToPointTskParams_t* guideParams,FH_Float guideX,FH_Float guideY)
{

  //����x��yλ�þ���С���ƶ��Ķ���
  if((guideX <= 0) && (guideY <= 0))
  {
	  guideParams->DeltaDisX = guideY;
	  guideParams->DeltaHeading = PI/2;
	  guideParams->DeltaDisY = -guideX;
  }
  else if((guideX >= 0) && (guideY <= 0))
  {
	  guideParams->DeltaDisX = guideY;
	  guideParams->DeltaHeading = -PI/2;
	  guideParams->DeltaDisY = guideX;
  }
  else if((guideX <= 0) && (guideY >= 0))
  {
	  guideParams->DeltaDisX = guideY;
	  guideParams->DeltaHeading = PI/2;
	  guideParams->DeltaDisY = -guideX;
  }
  else if((guideX >= 0) && (guideY >= 0))
  {
	  guideParams->DeltaDisX = guideY;
	  guideParams->DeltaHeading = -PI/2;
	  guideParams->DeltaDisY =guideX;
  }
  else
  {
	  guideParams->DeltaDisX = 0;
	  guideParams->DeltaHeading = 0;
	  guideParams->DeltaDisY = 0;
	  
  }

 if((fabs(guideX) > 0.08) || (fabs(guideY) > 0.08))
 {
	 LogPrintf("down guid x:%f,y:%f\n",guideX,guideY);
	 SetErrorToQue((ErrorTypeEnum)ERROR_MOVE_RANGE,ErrorC);
	 return -1;
 }
 return 0;
}

//������ͷ����
//#define DO_INSTRUCTION_GUIDE_DOWN_CAM 78
FH_ERROR_CODE DO_INSTRUCTION_GUIDE_DOWN_CAM(int* Param)
{
	if(0 == IsAllMotorEnable())
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_MotorDisable_RejectOrder,ErrorC);
		return -1;
	}
	#if 1
	SetAppActionState(AGV_MOTION_GUIDE_DOWNCAM);
	//1,switch down guide mode
	QRCodeSetMode(&(Agv.device.qrcode), QRCODEMODE_DOWN_GUIDE);
	//2,wait 
	FH_Float dis_limit = 0.015;
	int sucessConfirmCnt = 0;
	int failConfirmCnt = 0;
	int waitCnt = 5000;//��ʱ100s = 50ms*waitCnt;
	int moveCnt = 5;//�˶���������5����
	struct GuideToPointTskParams_t guideParams;
	struct DownGuideAppData_t dnGuideData;
	do
	{
		GetDownGuideAppData(&dnGuideData);
		if((CAMERADATA_GUIDE_DN_OK == dnGuideData.DownGuideState)
                && (fabs(dnGuideData.X) < dis_limit)
                && (fabs(dnGuideData.Y) < dis_limit))
        {
			if(++sucessConfirmCnt>=3)
			{
				LogPrintf("down guid ok x:%f,y:%f\n",dnGuideData.X,dnGuideData.Y);
				break;
			}
        }
		else if(CAMERADATA_GUIDE_DN_FAILED == dnGuideData.DownGuideState)
		{
			sucessConfirmCnt = 0;
			failConfirmCnt++;
			if(failConfirmCnt>=100)
			{
				LogPrintf("down guid time out\n");
				SetErrorToQue((ErrorTypeEnum) ERROR_WAIT_DN642_TIME_OUT,WarningA);
				break;
			}
			else if(0 == (failConfirmCnt%20))
			{
				LogPrintf("down guid set mode\n");
				QRCodeSetMode(&(Agv.device.qrcode), QRCODEMODE_DOWN_GUIDE);
			}
		}
		else
		{
			sucessConfirmCnt = 0;
			LogPrintf("down guid state:%d,x:%f,y:%f\n",dnGuideData.DownGuideState,dnGuideData.X,dnGuideData.Y);			
			if(DnCameraFindTargetAngle(&guideParams,dnGuideData.X,dnGuideData.Y))
			{
				break;
			}
			guideParams.GuideOmg = 0.2;
			guideParams.GuideVel = 0.02;
			guideParams.pointMoveState = POINT_IDLE;
			guideParams.tickTimeOut= 20000;
			MoveToPoint_tsk(&guideParams);
			WaitLastCmdComplete_fun();
			if(--moveCnt<=0)
			{
				LogPrintf("moveCnt out\n");
				SetErrorToQue((ErrorTypeEnum) ERROR_GUIDE_TIME_OUT,WarningA);
				break;
			}
		}
		ThreadSleep(20);
		//1s quit
		waitCnt--;
	}while(waitCnt>0);
	return SwitchToDownMode();
	//
		
	#else
	/*
	BeginDeploy;
	Instruction.Type = INSTRUCTION_GUIDE_DOWN_CAM;
	Deploy_guide_downcam_move();
	EndDeploy;*/
	#endif
}


