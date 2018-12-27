/*
 * AgvAppSafeManager.c
 *
 *  Created on: 2018��7��10��
 *      Author: hwei
 */

#include "AgvApp.h"
#include "basefunction.h"




/*------------------------------------------------------------------------------
*��.....�룺NULL
*��.....����NULL
*��.....�أ�0-��ʾ����ִ�гɹ�,1-��ʾ����ִ��ʧ��(ȫ�ֱ���)
*Ŀ.....�ģ� 1.�����ɹ����¼��ǰλ�ò���Senddata���ϱ�
            2.����ʧ���򱨲�������
*���� ������NULL
*ǰ�� ����: NULL
*���� ����: NULL
------------------------------------------------------------------------------*/
FH_ERROR_CODE LocationNoReadDM(struct Location_t* location)
{
    int rtn = 1;
    double MoveDirectionGap = 0.0;
    static double xPos = 0.0,yPos = 0.0,LastCorrectHeading = 0.0;
    double CodeDistance = 0;
    double err_x = 0,err_y = 0;
    double LateralErr = 0.0,VerticalErr = 0.0;
//    double CodeGapX = CODE_GAP_X;//AgvParams.deviceParams.qrcodeParams.CODE_GAP_X;
//    double CodeGapY = CODE_GAP_Y;//AgvParams.deviceParams.qrcodeParams.CODE_GAP_Y;
    double CodeGapX = GetCurCodeGap(AGV.X,DirX);
	double CodeGapY = GetCurCodeGap(AGV.Y,DirY);
	static int cnt = 0;
	int logPrint = 0;
	cnt++;
	if(cnt==100)
	{
		logPrint = 1;
		cnt =0;
	}
//	struct QueueNode_t *head = Agv.board.errorQueue.errorQue.head;
//	while (head != NULL)
//	{
//		struct Error_t* errorHead = (struct Error_t*)head;
//		if (errorHead->priority>=3)//&&(0 == IsAllMotorEnable())
//			return 1;
//		head = head->next;
//	}
    //(��һ�β������ж�,��2�����ɹ�,�ٽ����ж�)
    if(location->checkOnDM.IsSearchCode == 1)
    {
        //�����ת��Stopʱ,����1������3����
        if((Agv.service.chassis.base.state == ChassisTurning)
            ||(AGV.MotionStatus == AGV_MOTION_TURNING))//(AGV.MotionStatus == AGV_MOTION_TURNING)
        {
            xPos = AGV.X;
            yPos = AGV.Y;
        }

        if((Agv.service.chassis.base.state != ChassisTurning)
            ||(AGV.MotionStatus != AGV_MOTION_TURNING))//�˴�����turning,Ӱ�����
        {
            //����1:δ�ƶ���
            LocationMoveDirCodegap(&MoveDirectionGap);
			if(logPrint == 1)
			LogPrintf("stopDm_1:err_x=%f,err_y=%f\n",fabs(AGV.X - xPos),fabs(AGV.Y- yPos));
            if(fabs(AGV.X - xPos)<0.01 && fabs(AGV.Y- yPos)<0.01)
            {
                rtn &= 1;
            }
            else
            {
                location->checkOnDM.errCode = ERROR_LOCATION_NEVER_MOVE;
                location->checkOnDM.err1 = fabs(AGV.X - xPos);
                location->checkOnDM.err2 = fabs(AGV.Y- yPos);
//              LogPrintf("err_x=%f,err_y=%f\n",location->checkOnDM.err1,location->checkOnDM.err2);
                rtn &= 0;
            }


            //����3:������ȡ�����λ��֮ǰ�Ķ�ά�룬�ҹ������ƫ������С��1cm
            CodeDistance = sqrt((AGV.X - AGV.LastCorrectX)*(AGV.X - AGV.LastCorrectX) + (AGV.Y - AGV.LastCorrectY)*(AGV.Y - AGV.LastCorrectY));
//			  LateralErr = fabs(AGV.LastCorrectX -ROUND_POSITION_MULUNIT(AGV.LastCorrectX,CodeGapX,CameraReadCodeIDActived))*sin(LastCorrectHeading)
//						 + fabs(AGV.LastCorrectY -ROUND_POSITION_MULUNIT(AGV.LastCorrectY,CodeGapY,CameraReadCodeIDActived))*cos(LastCorrectHeading);
		  	LateralErr = fabs(AGV.LastCorrectX -RoundAgvToCurQRPos(AGV.LastCorrectX,DirX,CameraReadCodeIDActived))*sin(LastCorrectHeading)
					 + fabs(AGV.LastCorrectY -RoundAgvToCurQRPos(AGV.LastCorrectY,DirY,CameraReadCodeIDActived))*cos(LastCorrectHeading);
			if(logPrint == 1)
			LogPrintf("stopDm_3:Dis=%f,err_L=%f\n",CodeDistance-1.5*MoveDirectionGap,fabs(LateralErr));
			if((CodeDistance<1.5*MoveDirectionGap)&&(fabs(LateralErr) < 0.015))
            {
                rtn &= 1;
            }
            else
            {
                location->checkOnDM.errCode = ERROR_LOCATION_LASETCODE;
                location->checkOnDM.err1 = 1.5*MoveDirectionGap - CodeDistance;
                location->checkOnDM.err2 = 0.015 - fabs(LateralErr);//������0�Ų�����
//              LogPrintf("LastCorrectX=%f,LastCorrectY=%f,LastCorrectHeading=%f\n",AGV.LastCorrectX,AGV.LastCorrectY,LastCorrectHeading);
//              LogPrintf("nn=%f,1.5MoveDirectionGap=%f\n",CodeDistance,1.5*MoveDirectionGap);
                rtn &= 0;
            }
        }
    }
    //����2:����λ�úͶ�ά��λ�õ�����ƫ��С��ͼ��������Ұһ�룬����ƫ��С��1cm
//	  err_x =AGV.X - ROUND_POSITION_MULUNIT(AGV.X,CodeGapX,CameraReadCodeIDActived);
//	  err_y =AGV.Y - ROUND_POSITION_MULUNIT(AGV.Y,CodeGapY,CameraReadCodeIDActived);
	err_x =AGV.X - RoundAgvToCurQRPos(AGV.X,DirX,CameraReadCodeIDActived);
	err_y =AGV.Y - RoundAgvToCurQRPos(AGV.Y,DirY,CameraReadCodeIDActived);
    VerticalErr = err_x*cos(Agv.service.chassis.TargetHeading)+err_y*sin(Agv.service.chassis.TargetHeading);
    LateralErr = err_x*sin(Agv.service.chassis.TargetHeading)+err_y*cos(Agv.service.chassis.TargetHeading);
	if(logPrint == 1)
		LogPrintf("stopDm_2:VerticalErr=%f,LateralErr=%f\n",VerticalErr,LateralErr);
	if((fabs(VerticalErr) < 0.02)
        &&(fabs(LateralErr)< 0.02))//ע,ԭ����0.01���ܵ��ƶ���������ǰ0.015��Ӱ��ĵ�0.02,�����ת90�ȵ�����
    {
        rtn &= 1;
    }
    else
    {
        location->checkOnDM.errCode = ERROR_LOCATION_FieldVFiew;
        location->checkOnDM.err1 = fabs(VerticalErr);
        location->checkOnDM.err2 = fabs(LateralErr);
        //LogPrintf("VerticalErr=%f,LateralErr=%f\n",VerticalErr,LateralErr);
        rtn &= 0;
    }
    if(rtn)
    {
//      if((location->checkOnDM.IsSearchCode == 0)&&(1 == IsAllMotorEnable()))
        if(location->checkOnDM.IsSearchCode == 0)
        {
            //��ƺ�����ϴ���־
//          FeedBack.FloorPoorFlag = 1;
            ReportFloorPoorEvent();
            xPos = AGV.X;
            yPos = AGV.Y;
            GetNominalHeading(Agv.service.chassis.TargetHeading,&LastCorrectHeading);//
            //����ͣ�����ϵ�ǰ���Ƕ�����������
            if(AGV.RdDMXYCnt <2)
            {
                location->checkOnDM.errCode = ERROR_LOCATION_LowSpeed;
                return 1;
            }
            else
                AGV.RdDMXYCnt =1;
        }
        location->checkOnDM.IsSearchCode = 1;
        location->checkOnDM.moveDistance = 0.0;
//		  location->checkOnDM.OnX = ROUND_POSITION_MULUNIT(AGV.X,CODE_GAP_X,CameraReadCodeIDActived);
//		  location->checkOnDM.OnY= ROUND_POSITION_MULUNIT(AGV.Y,CODE_GAP_Y,CameraReadCodeIDActived);
		location->checkOnDM.OnX = RoundAgvToCurQRPos(AGV.X,DirX,CameraReadCodeIDActived);
		location->checkOnDM.OnY= RoundAgvToCurQRPos(AGV.Y,DirY,CameraReadCodeIDActived);

        return 0;
    }
    else
    {
        location->checkOnDM.IsSearchCode = 0;
        return 1;
    }
}
/*------------------------------------------------------------------------------
*��.....�룺isSearchCode -����Fun1ִ�н��
*��.....����NULL
*��.....�أ�NULL.
*Ŀ.....�ģ� �ú����������ж���,��ص����ƶ�����.
1. fun1FlagΪ1,���ص���ģʽ�ĵ�һ����
            2. fun1FlagΪ0,��ֱ�ӷ���
*���� ������NULL
*ǰ�� ����: NULL
*���� ����: NULL
------------------------------------------------------------------------------*/
FH_ERROR_CODE LocationSearchFirstDM(struct Location_t* location)
{
    double MoveDirectionGap = 0.0;
    extern unsigned char errortable[MAX_ERROR_NUM];
    extern unsigned int ForceCameraDecode;

    if(0 == location->checkOnDM.IsSearchCode)
    {
        return 0;
    }
    if(1 == location->checkOnDM.IsSearchCode)
    {
        /*����������ִ�г���180����תָ��*/
        if((location->chassis->base.state == ChassisTurning)&&
            fabs(location->chassis->TargetHeading - location->curPosGnd.heading)>1.1*PI)
        {
            location->checkOnDM.errCode = ERROR_LOCATION_Larger_Rotate;
            SetError(ERROR_StopOnDM,3);
            location->checkOnDM.IsSearchCode = 0;
			LogPrintf("stopDm_4:TargetHeading=%f,heading=%f\n",location->chassis->TargetHeading,location->curPosGnd.heading);
            return 0;
        }
        /*�ƶ�������*/
        if(AGV.MotionStatus != AGV_MOTION_TURNING)
        {
            //��Ұ��Χ��,�Ҷ������������
            LocationMoveDirCodegap(&MoveDirectionGap);
            location->checkOnDM.moveDistance += AGV.V*AgvSamplePeriod;
            if(location->checkOnDM.moveDistance > 1.1*MoveDirectionGap)
            {
                location->checkOnDM.errCode = ERROR_LOCATION_Larger_Move;
                SetError(ERROR_StopOnDM,3);
                location->checkOnDM.IsSearchCode = 0;
				LogPrintf("stopDm_5:move no code\n");
                return 0;
            }
            else
            {
                if((AGV.FindDM == 1)&&(location->checkOnDM.moveDistance > 0.5*MoveDirectionGap))
                {
                    //�л�������ģʽ
                    AGV.RdDMXYCnt = 2;
                    CameraReadCodeIDActived = 0;
                    ForceCameraDecode = 0;
                    location->checkOnDM.IsSearchCode = 0;
                    return 0;
                }
            }
        }

    }
    return 0;
}

//StopOnDM���,���������߳�
FH_ERROR_CODE LocationCheckOnDM(struct Location_t* location)
{
    static int logcount = 0;
    if(2 == (int)Param.AgvParam[PARAM_StopOnDM_NoErr])
    {
        if(AGV.FindDM == 0
        &&(AGV.MotionStatus == AGV_MOTION_STOP
        ||(Agv.service.chassis.base.state == ChassisTurning)//AGV.MotionStatus == AGV_MOTION_TURNING
        ||AGV.MotionStatus == AGV_MOTION_LIFTING
        ||AGV.MotionStatus == AGV_MOTION_SETDOWN
        ||AGV.MotionStatus == AGV_MOTION_PODCHANGE)
        &&((Agv.service.powerManager.dormancyState == DORMANCY_WAKEUP)
        &&(Agv.service.powerManager.entryDormancy == 0))
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_BEFOREPUTDOWN_OPEN_UPCAMERA_CHECKSHEL)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_GUIDE_DOWN_CAM)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_CONVEYOR_CONFIRM)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_CONVEYOR_LOAD)
        && (AGV.MotionStatus != AGV_MOTION_SHELF_GUIDE)
        &&(0 == IsAgvMovingState()))
        {
            LocationSearchFirstDM(location);
            if(LocationNoReadDM(location))
            {
                SetError(ERROR_StopOnDM,3);
                AGV.RdDMXYCnt =1;//HR,add,20170210
            }
        }
        else
        {
            logcount = 0;
//          location->checkOnDM.IsSearchCode = 0;
            location->checkOnDM.errCode = ERROR_LOCATION_IDLE;
            location->checkOnDM.OnX = 0.0;
            location->checkOnDM.OnY = 0.0;
        }
        if(location->checkOnDM.errCode != ERROR_LOCATION_IDLE)
        {
            //��ƺ�����ϴ���־
            if(++logcount>=100)
            {
                logcount = 0;
//              FeedBack.FloorPoorFlag = 1;
                ReportFloorPoorEvent();
//              LogPrintf("errCode=%x\n",location->checkOnDM.errCode);
//              LogPrintf("x=%f,y=%f\n",location->checkOnDM.OnX,location->checkOnDM.OnY);
            }
        }

    }
    else if((0 == (int)Param.AgvParam[PARAM_StopOnDM_NoErr])
	||(1 == (int)Param.AgvParam[PARAM_StopOnDM_NoErr]))
    {
        if(AGV.FindDM == 0
        &&(AGV.MotionStatus == AGV_MOTION_STOP
        ||(Agv.service.chassis.base.state == ChassisTurning)//AGV.MotionStatus == AGV_MOTION_TURNING
        ||AGV.MotionStatus == AGV_MOTION_LIFTING
        ||AGV.MotionStatus == AGV_MOTION_SETDOWN
        ||AGV.MotionStatus == AGV_MOTION_PODCHANGE)
        &&((Agv.service.powerManager.dormancyState == DORMANCY_WAKEUP)
        &&(Agv.service.powerManager.entryDormancy == 0))
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_AFTERLIFT_OPEN_UPCAMERA_CHECKSHELF)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_BEFOREPUTDOWN_OPEN_UPCAMERA_CHECKSHEL)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_GUIDE_DOWN_CAM)
        && (AGV.MotionStatus != AGV_MOTION_SHELF_GUIDE)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_CONVEYOR_CONFIRM)
        && (Agv.app->AppTask.CurTypeId != INSTRUCTION_CONVEYOR_LOAD)
        &&(0 == IsAgvMovingState()))
        {
            SetError(ERROR_StopOnDM,3);
            AGV.RdDMXYCnt =1;//HR,add,20170210
        }
    }
    return 0;
}

int lowPowerConfir = 0;
int lowPowerFlag = 1;

void PowerCheck()
{
	if(Agv.device.battery.batteryBase.QuantityElectric < Param.AgvParam[PARAM_BATTERY_LOW])//HR,add,20170327
	{
		if (lowPowerConfir<100)
		{
			lowPowerConfir++;
		}
		else
		{
			lowPowerFlag =1;
			lowPowerConfir =0;
		}
	}
	else if (Agv.device.battery.batteryBase.QuantityElectric >=(Param.AgvParam[PARAM_BATTERY_LOW]))
	{
		lowPowerFlag =0;
	}

	if (lowPowerFlag == 1)
    {
//      SetError(ERROR_LowBattery,1);
//	  SetSpeedControlMode(MOVE_SPEED_MODE_LOW);
      lowPowerConfir =0;
    }
}

void TempCheck(struct TempCheckParam_t* tempType,TempId tempId)
{
	//�¶ȼ��
	if((tempType->errtemp<0.01)
	||(tempType->warntemp<0.01))
		return;
	if(tempType->temp >= tempType->errtemp)
	{
		tempType->tmpErrCnt++;
		if(tempType->tmpErrCnt*0.01>tempType->ContinueTime)
		{
			LogPrintf("tempId:%d,temp:%f,errtemp:%f\n",tempId,tempType->temp,tempType->errtemp);
			SetError(ERROR_TempErr,ErrorC);
			if(tempType->cutWaitCnt<=0)
			{
				tempType->cutWaitCnt = 200;
			}
		}
	}
	else if(tempType->temp>=tempType->warntemp)
	{
		tempType->tmpWarnCnt ++;
		tempType->tmpErrCnt = 0;
		if(tempType->tmpWarnCnt *0.01>tempType->ContinueTime)
		{
			tempType->tmpWarnCnt  = 0;
			LogPrintf("tempId:%d,temp:%f,warnTemp:%f\n",tempId,tempType->temp,tempType->warntemp);
			SetError(ERROR_TempWarn,2);
		}
	}
	else
	{
		tempType->tmpWarnCnt = 0;
		tempType->tmpErrCnt = 0;
	}
	
	if(tempType->cutWaitCnt>0)
	{
		tempType->cutWaitCnt --;
		if(tempType->cutWaitCnt<=0)
		{
			tempType->cutWaitCnt = 200;
			SetError(ERROR_TempErr,5);
			if('3' == AgvParams.HardwareVersion[3])//С��ȸ���
			{
				SetChargeRelay(0);
				SetSRPowerControl((struct BatterySR_t*)&Agv.device.battery.batteryBase,Close48VandChargeIn);	
			}
			else
			{
				CloseBatteryPower(1);//�жϳ��
				CloseBatteryPower(1);//�жϳ��
				CloseBatteryPower(0);//�жϹ���
				CloseBatteryPower(0);//�жϹ���
			}
				
		}
	}
}

void InitAllTemp(struct TempCheckParam_t * tempType)
{
	tempType->tmpWarnCnt = 0;
    tempType->tmpErrCnt =0;
    tempType->temp = 0;
	tempType->cutWaitCnt =0;
}
void AppCurrentCheck()
{
	/***********************************��ص����쳣����start******************************************/
	if(Agv.base.state>=FHAGV_WaitParamSet)
	{
		struct BatterySR_t *batterySR = &Agv.device.battery.batterySR;
		struct BatterySRParams_t *batterySRParams = (struct BatterySRParams_t *)batterySR->battery.base.params;
		if((AGV.MotionStatus == AGV_MOTION_STOP)
		&&(1 == IsAllMotorEnable())
		&&(0 == IsTrayLiftUpState())//�Ƕ���״̬
		&&(Agv.app->AppTask.CurTypeId != INSTRUCTION_CHARGEMOVE)//�ǳ��
		&&(Agv.app->AppTask.CurTypeId != INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF)
		&&(batterySR->battery.ChargeState != 1)
		&&(batterySRParams->ctrl48VCoeff>0.1)
		&&(batterySR->battery.QuantityElectric > 40.0)
		&&(batterySR->battery.BatteryPackCurrent>0.01)
		&&(batterySR->battery.PowerOff48V == 0))
		{
			if(0 == App.appSafeManager.errFlag)
			{
				App.appSafeManager.delayCount++;
				if(App.appSafeManager.delayCount ==500)
				{
					App.appSafeManager.power = fabs(batterySR->battery.BatteryPackCurrent)*batterySR->battery.QuantityElectric;
					LogPrintf("48VCtrlIN:Cur=%f,Voltage=%f,Power=%f\n",batterySR->battery.BatteryPackCurrent,batterySR->battery.QuantityElectric,App.appSafeManager.power);
				}
				else if(App.appSafeManager.delayCount >=1000)
				{
					App.appSafeManager.delayCount = 1000;

					if(fabs(batterySR->battery.BatteryPackCurrent)*(batterySR->battery.QuantityElectric)
						>= batterySRParams->ctrl48VCoeff*43.68)
					{
						SetError(ERROR_Power_Off,3);
						App.appSafeManager.delayCount= 0;
						App.appSafeManager.errFlag = 1;
						//�����쳣���ж�48V��Դ
						SetSRPowerControl(batterySR,Close48VandChargeIn);
						LogPrintf("48VCtrlOUT:Cur=%f,Voltage=%f,Power=%f\n",batterySR->battery.BatteryPackCurrent,batterySR->battery.QuantityElectric,App.appSafeManager.power);
					}
				}
			}
		}
		else
		{
			App.appSafeManager.delayCount = 0;
			App.appSafeManager.errFlag = 0;
		}
	}
}
void AppSafeCheck()
{
	//��ⲻ������
    LocationCheckOnDM(&Agv.service.chassis.location);
	//���͵���
	PowerCheck();
	//��֪platform����
	SafeManagerSetChargePointFlag(IsCurPosInChargePoint());
	//ͣ�������µ������
	AppCurrentCheck();
	if(Gyro_ZeroBiasReady != Agv.device.gyro.base.state)//||(0 != UpdateBatteryFW))
	{
		TempId tempId = MIN_TEMP_ID;
		for(tempId=MIN_TEMP_ID;tempId<MAX_TEMP_ID;tempId++)
			InitAllTemp(&tempCheck[tempId]);
		return;
	}
#if TEMP_TEST
	//�¶ȼ��
	tempCheck[TEMP_Gyro].temp = Agv.device.gyro.Temperature;
	TempCheck(&tempCheck[TEMP_Gyro],TEMP_Gyro);
	tempCheck[TEMP_Battery].temp = Agv.device.battery.batteryBase.Temperature;
	TempCheck(&tempCheck[TEMP_Battery],TEMP_Battery);
	tempCheck[TEMP_MotorLeft].temp = Agv.device.motorLeft.md.TempValue;
	TempCheck(&tempCheck[TEMP_MotorLeft],TEMP_MotorLeft);
	tempCheck[TEMP_MotorRight].temp = Agv.device.motorRight.md.TempValue;
	TempCheck(&tempCheck[TEMP_MotorRight],TEMP_MotorRight);
	tempCheck[TEMP_MotorTrayLift].temp = Agv.device.motorTrayLift.md.TempValue;
	TempCheck(&tempCheck[TEMP_MotorTrayLift],TEMP_MotorTrayLift);
	tempCheck[TEMP_MotorTrayRotate].temp = Agv.device.motorTrayRotate.md.TempValue;
	TempCheck(&tempCheck[TEMP_MotorTrayRotate],TEMP_MotorTrayLift);
	
//	tempCheck[TEMP_ChargeInterfacePin].temp = Agv.device.chargeInterface.pinTemperature;
//	TempCheck(&tempCheck[TEMP_ChargeInterfacePin],TEMP_ChargeInterfacePin);
	tempCheck[TEMP_ChargeInterface].temp = Agv.device.chargeInterface.temperature;
	TempCheck(&tempCheck[TEMP_ChargeInterface],TEMP_ChargeInterface);
	tempCheck[TEMP_FrontPanel].temp = Agv.device.ledAgv.leftLed.led.temperature;//
	TempCheck(&tempCheck[TEMP_FrontPanel],TEMP_FrontPanel);
	tempCheck[TEMP_BackPanel].temp = Agv.device.ledAgv.rightLed.led.temperature;//
	TempCheck(&tempCheck[TEMP_BackPanel],TEMP_BackPanel);
	tempCheck[TEMP_Belt].temp = Agv.device.singleBeltIO.temperature;
	TempCheck(&tempCheck[TEMP_Belt],TEMP_Belt);
	tempCheck[TEMP_Laser].temp = Agv.device.laser.temperature;//
	TempCheck(&tempCheck[TEMP_Laser],TEMP_Laser);
	/*��ص������*/
	//Ĭ��20A
	if((Agv.device.battery.batteryBase.BatteryPackCurrent >= CurrentProtectParams.MotionThreshold)
	&&(Gyro_ZeroBiasReady == Agv.device.gyro.base.state))
	{
		Agv.device.battery.batteryBase.batteryMotionOverCnt++;
		if(Agv.device.battery.batteryBase.batteryMotionOverCnt >= (100*(CurrentProtectParams.ContinueTime+3)+3))
		{
			Agv.device.battery.batteryBase.batteryMotionOverCnt = 100*(CurrentProtectParams.ContinueTime+3)+3;
		}
		else if(Agv.device.battery.batteryBase.batteryMotionOverCnt >= 100*(CurrentProtectParams.ContinueTime+2))
		{
			Agv.device.battery.batteryBase.batteryMotionOverCnt=0;
			SetSRPowerControl((struct BatterySR_t*)&Agv.device.battery.batteryBase,Close48VandChargeIn);
			LogPrintf("batteryMotionOverCnt=%d,ContinueTime=%d\n",Agv.device.battery.batteryBase.batteryMotionOverCnt,CurrentProtectParams.ContinueTime);
		}
		else if(Agv.device.battery.batteryBase.batteryOverCnt*0.01 >= (CurrentProtectParams.ContinueTime))
		{
			SetError(ERROR_BatteryCurrentOverLarge,3);
		}

	}
	else
	{
		Agv.device.battery.batteryBase.batteryMotionOverCnt = 0;
	}
	//Ĭ��3A
	if((Agv.device.battery.batteryBase.BatteryPackCurrent >= CurrentProtectParams.StaticThreshold)&&
		(AGV_MOTION_STOP == AGV.MotionStatus)&&
		(Gyro_ZeroBiasReady == Agv.device.gyro.base.state))
	{
		Agv.device.battery.batteryBase.batteryOverCnt++;
		if(Agv.device.battery.batteryBase.batteryOverCnt >= (100*(CurrentProtectParams.ContinueTime+3)+3))
		{
			Agv.device.battery.batteryBase.batteryOverCnt = 100*(CurrentProtectParams.ContinueTime+3)+3;
		}
		else if(Agv.device.battery.batteryBase.batteryOverCnt >= 100*(CurrentProtectParams.ContinueTime+2))
		{
			SetSRPowerControl((struct BatterySR_t*)&Agv.device.battery.batteryBase,Close48VandChargeIn);
			LogPrintf("batteryOverCnt=%d,ContinueTime=%d\n",Agv.device.battery.batteryBase.batteryOverCnt,CurrentProtectParams.ContinueTime);
		}
		else if(Agv.device.battery.batteryBase.batteryOverCnt >= 100*(CurrentProtectParams.ContinueTime))
		{
			SetError(ERROR_BatteryCurrentOverLarge,3);
		}
	}
	else
	{
		Agv.device.battery.batteryBase.batteryOverCnt=0;
	}
	//Ĭ��-15A
	if(Agv.device.battery.batteryBase.BatLoopCurrent <= CurrentProtectParams.ChargeThreshold)
	{
		Agv.device.battery.batteryBase.batteryChargeOverCnt++;

		if(Agv.device.battery.batteryBase.batteryChargeOverCnt >= (100*(CurrentProtectParams.ContinueTime+3)+3))
		{
			Agv.device.battery.batteryBase.batteryChargeOverCnt = 100*(CurrentProtectParams.ContinueTime+3)+3;
		}
		else if(Agv.device.battery.batteryBase.batteryChargeOverCnt*0.01 >=(CurrentProtectParams.ContinueTime+2))
		{
			Agv.device.battery.batteryBase.batteryChargeOverCnt = 0;
			SetChargeRelay(0);
			LogPrintf("batteryChargeOverCnt=%d,ContinueTime=%d\n",Agv.device.battery.batteryBase.batteryChargeOverCnt,CurrentProtectParams.ContinueTime);
		}
		else if(Agv.device.battery.batteryBase.batteryChargeOverCnt*0.01 >=CurrentProtectParams.ContinueTime)
		{
			SetError(ERROR_BatteryCurrentOverLarge,3);
		}
	}
	else
	{
		Agv.device.battery.batteryBase.batteryChargeOverCnt=0;
	}
#endif

}

