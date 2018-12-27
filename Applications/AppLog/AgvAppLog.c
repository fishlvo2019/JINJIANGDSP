/*
 * AgvAppLog.c
 *
 *  Created on: 2017年12月8日
 *      Author: hwei
 */
#include "AgvApp.h"
#include "basefunction.h"
#include "MD5_calc.h"
#include "calibration.h"
/*========================内存分配========================*/
#pragma DATA_SECTION(SpecialFrameArg,"ZONE6DATA");
/*========================全局变量========================*/
extern double feedback_deltaH;
extern enum AGVModeType slam_mode;
extern double g_left_cur;
extern double g_right_cur;
int testErr = 246;
int IsUIDisplayXY = 0;
struct SpecialFrameEventArg_t SpecialFrameArg;


//====================================================================================
//注意没有做字节对齐。
FH_ERROR_CODE SendSampleData(struct SendDataNormal_t* sendDataBuf)
{
//协议48字节，0xAA头，0x55尾，checksum;
	unsigned int* buf = (unsigned int*)sendDataBuf;
	sendDataBuf->head = 0xAA;
	//当前指令执行状态
	if((FHMAINTASK_DOING == App.appBase.AppTask.base.state)||
		(FHMAINTASK_WAITING == App.appBase.AppTask.base.state)||
		(GetTaskCnt(&Agv.service.mainControlTask))||
		(IsAgvMotionState()))
	{
		sendDataBuf->OrderExecutionStatus = 1;
		if(IsAgvMotionState()&&(Agv.service.safeManager.ErrorState>=WarningA))
		{
		    sendDataBuf->OrderExecutionStatus = 3;
		}
	}
	else if((FHMAINTASK_ERROR == App.appBase.AppTask.base.state)||
		(	(Agv.service.safeManager.ErrorState>=WarningA)&&
			(INSTRUCTION_SETPARAM !=GetCurInstructionTypeId())&&
			(INSTRUCTION_RESET !=GetCurInstructionTypeId())&&
			(INSTRUCTION_BEFORELIFT_OPEN_UPCAMERA_CHECKSHELF !=GetCurInstructionTypeId())&&
			(0 !=GetCurInstructionTypeId())&&
			(INSTRUCTION_MAP_INFORMATION !=GetCurInstructionTypeId())))
	{
		sendDataBuf->OrderExecutionStatus = 3;
	}
    else
	{
		sendDataBuf->OrderExecutionStatus = 0;
	}
	if(FHAGV_Initialized != Agv.base.state)
	{
		//sendDataBuf->OrderExecutionStatus = 1;
	}
	sendDataBuf->PodLS = (Agv.service.tray.base.state & Tray_ZeroIdentified)? 1: 0;//bit2 托盘零位确认
	sendDataBuf->GYROZero = (Gyro_ZeroBiasReady == Agv.device.gyro.base.state)? 1 : 0;//bit3 惯导已复位，零偏确认
	sendDataBuf->Shelf = IsTrayLiftUpState();//bit4:5是否带货架
	sendDataBuf->FindDM = AGV.FindDM;//CheckQROnCode();	   //bit6  是否在码上
	sendDataBuf->MotoStatus = IsAllMotorEnable();	  //bit7电机使能
	sendDataBuf->ID = GetCurInstructionId();		 // bit3:0  当前正在执行指令ID号
	sendDataBuf->Frame = 0;   // 1 特殊帧标志
	if(Agv.service.powerManager.dormancyState == DORMANCY_WAKEUP)
		sendDataBuf->Dormancy = 0;
	else
		sendDataBuf->Dormancy = 1;
	//sendDataBuf->Dormancy = FeedBack.ORDERSTATUS.bit.Dormancy;		 // 1休眠标志
	if(ID_CAMERA)
	{
		IsUIDisplayXY = ID_CAMERA;
	}
	if(IsUIDisplayXY)
	{
		sendDataBuf->PowerupCameraData = 1;// 1 dm642已上电，读到正确dm642数据,但是不清零
	}
	//sendDataBuf->PowerupCameraData = ID_CAMERA ? 1:0;
	sendDataBuf->DebugMode = (AGV.DebugMode)? 1: 0; //debug模式
//	sendDataBuf->X = (long)(Agv.service.chassis.location.curPosGnd.x/AgvParams.deviceParams.qrcodeParams.CODE_GAP_X*1000.0);
	sendDataBuf->X = GetCurQRCsysPos(Agv.service.chassis.location.curPosGnd.x,DirX);
	EndianConvert(&sendDataBuf->X,sizeof(int32));
//	sendDataBuf->Y = (long)(Agv.service.chassis.location.curPosGnd.y/AgvParams.deviceParams.qrcodeParams.CODE_GAP_Y*1000.0);
	sendDataBuf->Y = GetCurQRCsysPos(Agv.service.chassis.location.curPosGnd.y,DirY);
	EndianConvert(&sendDataBuf->Y,sizeof(int32));
	sendDataBuf->Heading = (int)(WrapToPiDe(Agv.service.chassis.location.curPosGnd.heading) * 180.0/PI * 100.0);
	EndianConvert(&sendDataBuf->Heading,sizeof(int));
	sendDataBuf->MotionStatus = (unsigned char)AGV.MotionStatus;//Agv.service.motionCtrller.base.state;//13
	//MoveMemByte(buf,4,11,3);//长度11byte，从4移位到3
	int tmpOffsetStart =2* OffsetOf(struct SendDataNormal_t,X);
	int tmpOffsetEnd = 2*OffsetOf(struct SendDataNormal_t,MotionStatus);
	MoveMemByte(buf,tmpOffsetStart,tmpOffsetEnd-tmpOffsetStart+2,tmpOffsetStart-1);
	if(Agv.service.beltManager.BeltType == NONE_BELT)
	{
		if(IsTrayLiftUpState())
		{
			sendDataBuf->PodHeight = (int)(Agv.service.tray.Heading2AGV*180.0/PI*10+18000);//(int)(Agv.service.tray.Heading2AGV*180.0/PI*10);//14-15 托盘角度
		}
		else
		{
			sendDataBuf->PodHeight = (int)(Agv.service.tray.Heading2AGV*180.0/PI*10);//(int)(Agv.service.tray.Heading2AGV*180.0/PI*10);//14-15 托盘角度
		}
	}
	else
	{
		if((Agv.service.beltManager.BeltInfo.FullBoxInfo.Flag == 1)
		&&(Agv.service.beltManager.BeltInfo.FullBoxInfo.Info == 1))
		{
			sendDataBuf->PodHeight = *(Agv.service.beltManager.TargetBeltIOLaser)+
			Agv.service.beltManager.BeltInfo.Uploadinfo.Info*1000;
		}
		else
		{
			sendDataBuf->PodHeight = *(Agv.service.beltManager.TargetBeltIOLaser);
		}

	}
//	if(AgvParams.deviceParams.batteryParams.batteryParamsBase.Type == BatteryQuickCharge_3Gen)
//		sendDataBuf->PodHeight = (int)(Agv.device.battery.Bat12Current*10);//(int)(Agv.service.tray.Heading2AGV*180.0/PI*10);//14-15 托盘角度

	EndianConvert(&sendDataBuf->PodHeight,sizeof(int));
	sendDataBuf->CMDSpeed = Agv.device.gyro.CurOmg*1000;//16-17
	EndianConvert(&sendDataBuf->CMDSpeed,sizeof(int));

	sendDataBuf->leftcmdspeed = (int)(Agv.device.motorLeft.md.RPMSet * AgvParams.serviceParams.chassisParams.locationParams.motor2WheelFactor*1000);//18-19
	EndianConvert(&sendDataBuf->leftcmdspeed,sizeof(int));

	sendDataBuf->rightcmdspeed = (int)(Agv.device.motorRight.md.RPMSet * AgvParams.serviceParams.chassisParams.locationParams.motor2WheelFactor*1000);//20-21
	EndianConvert(&sendDataBuf->rightcmdspeed,sizeof(int));

	sendDataBuf->leftspeed = (int)(Agv.service.chassis.location.leftSpeed*1000) ;//22-23
	EndianConvert(&sendDataBuf->leftspeed,sizeof(int));

	sendDataBuf->rightspeed = (int)(Agv.service.chassis.location.rightSpeed*1000);//24-25
	EndianConvert(&sendDataBuf->rightspeed,sizeof(int));

	struct PosGeneratorParams_t* pgParams = (struct PosGeneratorParams_t*)Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg.base.params;
	sendDataBuf->offsetx = (int)((pgParams->targetPos-Agv.service.chassis.multiAxis.pgMultiAxial.pg_u.pg.curPos)*1000);//26-27
	EndianConvert(&sendDataBuf->offsetx,sizeof(int));

	sendDataBuf->offsety = (int)(Agv.service.chassis.multiAxis.AxisY.pidCtrller.ctrlError*10000);//28-29
	EndianConvert(&sendDataBuf->offsety,sizeof(int));
	//sendDataBuf->offsetHeading = 0;//30-31
	sendDataBuf->offsetHeading = (((Uint16)(Agv.device.laser.NearestBarrierAngle) << 8) | ((Uint16)(Agv.device.laser.BarrierDis*10.0)));
	EndianConvert(&sendDataBuf->offsetHeading,sizeof(int));

	//sendDataBuf->IOSTATUS = 0;//32;
	sendDataBuf->FrontSensorFar = Agv.service.avoidObstacle.FarSensor;
	sendDataBuf->FrontSensorNear= Agv.service.avoidObstacle.NearSensor;
	if(IsBatteryCharging())
	{
		sendDataBuf->Discharge = 0;//FeedBack.IOSTATUS.bit.Discharge;
	}
	else
	{
		sendDataBuf->Discharge = 1;
	}
	sendDataBuf->RearSensorFar= FeedBack.IOSTATUS.bit.RearSensorFar;
	sendDataBuf->PodSensor= FeedBack.IOSTATUS.bit.PodSensor; // 1
	sendDataBuf->FrontShelfSensor= FeedBack.IOSTATUS.bit.FrontShelfSensor;
	sendDataBuf->CLEANER_DIRTY= IsCleanerDirty();//FeedBack.IOSTATUS.bit.CLEANER_DIRTY; // 1
	sendDataBuf->FanStatus= IsCleanerFanON();
	FeedBack.IOSTATUS.all = 0;
//#ifdef SingleIO
//	sendDataBuf->LeftCurrent = ((int)(Agv.device.singleBeltIO.LeftLaserDis)&0xFF);//((int)(Agv.device.motorLeft.md.Current*5.0)&0xFF);//33
//	sendDataBuf->RightCurrent = ((int)(Agv.device.singleBeltIO.RightLaserDis)&0xFF);//((int)(Agv.device.motorRight.md.Current*5.0)&0xFF);//34

//#else
	sendDataBuf->LeftCurrent = ((int)(Agv.device.motorLeft.md.Current*5.0)&0xFF);//33
	sendDataBuf->RightCurrent = ((int)(Agv.device.motorRight.md.Current*5.0)&0xFF);//34

//#endif
	if(Agv.service.beltManager.BeltType == NONE_BELT)
		sendDataBuf->LiftCurrent = ((int)(Agv.device.motorTrayLift.md.Current*5.0)&0xFF);//35
	else
	{
		if(Agv.service.beltManager.BeltInfo.FullBoxInfo.Flag != 0)
		{
	 		Agv.service.beltManager.BeltInfo.Uploadinfo.Flag = Agv.service.beltManager.BeltInfo.FullBoxInfo.Flag;
	 		Agv.service.beltManager.BeltInfo.Uploadinfo.Info = Agv.service.beltManager.BeltInfo.FullBoxInfo.Info;
	 		Agv.service.beltManager.BeltInfo.Uploadinfo.InfoID = Agv.service.beltManager.BeltInfo.FullBoxInfo.InfoID;
			Agv.service.beltManager.BeltInfo.Uploadinfo.Flag = 0x04;
			Agv.service.beltManager.BeltInfo.FullBoxInfo.Flag = 0x00;
			Agv.service.beltManager.IfFullLoad = 0;
			//如果满料,则蜂鸣器响三次
			if(Agv.service.beltManager.BeltInfo.Uploadinfo.Info == 1)
				Agv.service.beltManager.BeltInfo.FullInfoBeep = 1;
		}
		sendDataBuf->LiftCurrent = ((int)(Agv.service.beltManager.BeltInfo.Uploadinfo.Info<<4 | Agv.service.beltManager.BeltInfo.Uploadinfo.Flag)&0xFF);//35

	}

	int tmpBLC = (int)(Agv.device.battery.batteryBase.BatLoopCurrent*10);
	sendDataBuf->BatteryLoopCurrent = (((tmpBLC>>4)&0xFF)|((tmpBLC<<8)&0xF00));
	__byte((&sendDataBuf->BarrierDis)-1,1) = 0;//空余字节

	if(Agv.service.beltManager.BeltType != NONE_BELT)
		FHCharWrite((int*)buf, 38, Agv.service.beltManager.BeltStateSelf);
	//MoveMemByte(buf,16,19,14);
	tmpOffsetStart =2* OffsetOf(struct SendDataNormal_t,PodHeight);
	tmpOffsetEnd = 2*OffsetOf(struct SendDataNormal_t,BarrierDis);
	MoveMemByte(buf,tmpOffsetStart,tmpOffsetEnd-tmpOffsetStart,tmpOffsetStart-2);
	//sendDataBuf->BarrierDis = 0;//39-40
	sendDataBuf->BarrierDis = (int)(Agv.device.laser.BarrierDis*100.0);
	EndianConvert(&sendDataBuf->BarrierDis,sizeof(int));

	if(Agv.service.beltManager.BeltType == NONE_BELT)
		sendDataBuf->liftcmdspeed = (int)(Agv.service.tray.axisTrayRotate.ctrlOut*1000);//41-42
	else
		sendDataBuf->liftcmdspeed = (int)(((int)(fabs(Agv.service.leftBelt.BeltSpeed)*100)&0x00FF) | (int)(((int)(Agv.service.leftBelt.CmdBeltSpeed*100)&0x00FF)<<8));
	EndianConvert(&sendDataBuf->liftcmdspeed,sizeof(int));

	if(Agv.service.beltManager.BeltType == NONE_BELT)
		sendDataBuf->liftspeed = (int)(Agv.device.gyro.CurOmg*1000);//43-44
	else
		sendDataBuf->liftspeed = (int)(((int)(Agv.service.rightBelt.BeltSpeed*100)&0x00FF) | (int)(((int)(Agv.service.rightBelt.CmdBeltSpeed*100)&0x00FF)<<8));
	EndianConvert(&sendDataBuf->liftspeed,sizeof(int));
	//int i =0;
	 AGV.ErrorType = 0;

	struct Error_t * curErr = PeekErrorQueue();
	if(NULL != curErr)
	{
		//error 在最高位置0，区分error和warning
		if((curErr->priority<ErrorC)&&
			(curErr->type>255))
		{
			AGV.ErrorType = (curErr->type | 0x8000);
		}
		else
		{
			AGV.ErrorType = (curErr->type & 0x7FFF);
		}
	}


	if(AGV.ErrorType>255)
	{
		sendDataBuf->ErrorType = TranslateErrorCode(AGV.ErrorType);
	}
	else
	{
		sendDataBuf->ErrorType = AGV.ErrorType;//45
	}

	//MoveMemByte(buf,36,13,33);
	tmpOffsetStart =2* OffsetOf(struct SendDataNormal_t,BarrierDis);
	tmpOffsetEnd = 2*OffsetOf(struct SendDataNormal_t,ErrorType);
	MoveMemByte(buf,tmpOffsetStart,tmpOffsetEnd-tmpOffsetStart+2,tmpOffsetStart-1);

	if(testErr==((buf[22]>>8)&0xFF))
    {
        //int a=0;
    }
    return 0;
}
FH_ERROR_CODE SendSlamData(int* sendDataBuf)
{
	FHCharWrite(sendDataBuf,0,0xAA);
	int tmpInt;
	FHBitReSet((Uint16*)&tmpInt,0);
	FHBitReSet((Uint16*)&tmpInt,1);
	if(Agv.service.tray.base.state & Tray_ZeroIdentified)
		FHBitSet((Uint16*)&tmpInt,2);
	else
		FHBitReSet((Uint16*)&tmpInt,2);
	if(Gyro_ZeroBiasReady == Agv.device.gyro.base.state)
		FHBitSet((Uint16*)&tmpInt,3);
	else
		FHBitReSet((Uint16*)&tmpInt,3);

	if(IsTrayLiftUpState())
		FHBitSet((Uint16*)&tmpInt,4);
	else
	{
		FHBitReSet((Uint16*)&tmpInt,4);
		FHBitReSet((Uint16*)&tmpInt,5);
	}

	if(IsOnQRDown())
		FHBitSet((Uint16*)&tmpInt,6);
	else
		FHBitReSet((Uint16*)&tmpInt,6);
	if(IsAllMotorEnable())
		FHBitSet((Uint16*)&tmpInt,7);
	else
		FHBitReSet((Uint16*)&tmpInt,7);
	FHCharWrite(sendDataBuf,1,tmpInt);
	int tmpId = GetCurInstructionId();
	tmpInt = 0x40|(tmpId&0xF);
	FHCharWrite(sendDataBuf,2,tmpInt);

	//0-9
	int i=0;
	for(i=0;i<10;i++)
	{
		FHCharWrite(sendDataBuf,i+3,resultr[i]);
	}
	FHCharWrite(sendDataBuf,13,0x10);//特殊帧类别,slavepc
	for(i=10;i<20;i++)
	{
		FHCharWrite(sendDataBuf,i+4,resultr[i]);
	}
	tmpInt = slam_mode;
	FHCharWrite(sendDataBuf,24,(tmpInt>>8)&0xFF);
	FHCharWrite(sendDataBuf,25,tmpInt&0xFF);
	for(i=20;i<33;i++)
	{
		FHCharWrite(sendDataBuf,i+6,resultr[i]);
	}
	FHCharWrite(sendDataBuf,39,0);
	FHCharWrite(sendDataBuf,40,AGV_CHECK_MD5);

	return 0;
}

FH_ERROR_CODE ChangeSlamMode(struct AppLog_t* appLog,int isSlamMode)
{
	if(isSlamMode)
	{
		RegisterSendDataCallback((ProcessBeforeSendData)SendSlamData);
	}
	else
	{
		RegisterSendDataCallback((ProcessBeforeSendData)SendSampleData);
	}
	return 0;
}
//特殊帧事件
FH_ERROR_CODE ReportParamsEvent(int paramIndex,FH_Float paramValue)
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,1);//特殊帧类别
	Uint16* tmpPtr = (Uint16*)&paramValue;
	FHCharWrite(specialFrameArg.buf,3,(tmpPtr[1]>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,4,tmpPtr[1]&0xFF);
	FHCharWrite(specialFrameArg.buf,5,(tmpPtr[0]>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,6,tmpPtr[0]&0xFF);
	//FHCharWrite(specialFrameArg.buf,7,1);
	FHCharWrite(specialFrameArg.buf,11,(paramIndex>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,12,paramIndex&0xFF);
	SetReportEvent(eventArg);
	return 0;
}
//5s
FH_ERROR_CODE ReportPowerEvent(FH_Float QuantityElectric,FH_Float RealBatVoltage,FH_Float RealBatCapacity,
								FH_Float temGyro,FH_Float temBattery,FH_Float temMdLeft,FH_Float temMdRight,FH_Float temMdTrayRotate,FH_Float temMdTrayLift)
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,0x4);//特殊帧类别
	
	int tmpInt = (int)(QuantityElectric* 100);
	FHCharWrite(specialFrameArg.buf,14,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,15,tmpInt&0xFF);
	
	tmpInt = (int)(RealBatVoltage* 100);
	FHCharWrite(specialFrameArg.buf,16,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,17,tmpInt&0xFF);

	tmpInt = (int)(RealBatCapacity* 1000);
	FHCharWrite(specialFrameArg.buf,18,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,19,tmpInt&0xFF);
	
	tmpInt = (int)(temGyro);
	FHCharWrite(specialFrameArg.buf,20,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,21,tmpInt&0xFF);

	tmpInt = (int)(temBattery);
	FHCharWrite(specialFrameArg.buf,22,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,23,tmpInt&0xFF);

	tmpInt = (int)(temMdLeft);
	FHCharWrite(specialFrameArg.buf,24,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,25,tmpInt&0xFF);

	tmpInt = (int)(temMdRight);
	FHCharWrite(specialFrameArg.buf,26,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,27,tmpInt&0xFF);

	tmpInt = (int)(temMdTrayRotate);
	FHCharWrite(specialFrameArg.buf,28,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,29,tmpInt&0xFF);

	tmpInt = (int)(temMdTrayLift);
	FHCharWrite(specialFrameArg.buf,30,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,31,tmpInt&0xFF);
	
	tmpInt = (int)(Agv.device.battery.batterySR.tempreture);
	FHCharWrite(specialFrameArg.buf,32,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,33,tmpInt&0xFF);

	tmpInt = (int)(Agv.device.chargeInterface.temperature);
	FHCharWrite(specialFrameArg.buf,34,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,35,tmpInt&0xFF);
	
	SetReportEvent(eventArg);
	return 0;
}

FH_ERROR_CODE ReportQRCodeEvent()
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,0x2);//特殊帧类别

	Uint16 tmpInt = UpCamera.DataType;
	FHCharWrite(specialFrameArg.buf,16,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,17,tmpInt&0xFF);

//	tmpInt = (Uint16)(((int)(CorrectPosition.PreX*2.0/CODE_GAP_X + 0.5))*0.5*100.0);
	FH_Float curCodeGap = GetCurCodeGap(CorrectPosition.PreX,DirX);
	tmpInt = (Uint16)(((int)(CorrectPosition.PreX*2.0/curCodeGap + 0.5))*0.5*100.0);
	FHCharWrite(specialFrameArg.buf,18,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,19,tmpInt&0xFF);


//	tmpInt = (Uint16)(((int)(CorrectPosition.PreY*2.0/CODE_GAP_Y + 0.5))*0.5*100.0);
	curCodeGap = GetCurCodeGap(CorrectPosition.PreY,DirY);
	tmpInt = (Uint16)(((int)(CorrectPosition.PreY*2.0/curCodeGap + 0.5))*0.5*100.0);
	FHCharWrite(specialFrameArg.buf,20,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,21,tmpInt&0xFF);

//	tmpInt = (Uint16)(((int)(CorrectPosition.X*2.0/CODE_GAP_X + 0.5))*0.5*100.0);//HR,modified,20151117
	curCodeGap = GetCurCodeGap(CorrectPosition.X,DirX);
	tmpInt = (Uint16)(((int)(CorrectPosition.X*2.0/curCodeGap + 0.5))*0.5*100.0);
	FHCharWrite(specialFrameArg.buf,22,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,23,tmpInt&0xFF);

//	tmpInt = (Uint16)(((int)(CorrectPosition.Y*2.0/CODE_GAP_Y + 0.5))*0.5*100.0);//HR,modified,20151117
	curCodeGap = GetCurCodeGap(CorrectPosition.Y,DirY);
	tmpInt = (Uint16)(((int)(CorrectPosition.Y*2.0/curCodeGap + 0.5))*0.5*100.0);
	FHCharWrite(specialFrameArg.buf,24,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,25,tmpInt&0xFF);
	int tmpInt1 , tmpInt2; 

	if (CorrectPosition.CodeIDValid==1)//HR,add,20161012
	{
		tmpInt1 =(int)(CorrectPosition.CodeDeltaX*1000);
		tmpInt2 =(int)(CorrectPosition.CodeDeltaY*1000);
	}
	else
	{
		tmpInt1 =(int)(CorrectPosition.CodeDeltaX*1000+1000);
		tmpInt2 =(int)(CorrectPosition.CodeDeltaY*1000+1000);
	}
	
	FHCharWrite(specialFrameArg.buf,26,(tmpInt1>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,27,tmpInt1&0xFF);
	FHCharWrite(specialFrameArg.buf,28,(tmpInt2>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,29,tmpInt2&0xFF);
	
	tmpInt1 = (int)((CorrectPosition.CodeDeltaHeading+feedback_deltaH)*180.0/PI*100.0);
	if (FeedBack.DMCodeDataErrorFlag==1)
	{
		FeedBack.DMCodeDataErrorFlag =0;
		if (tmpInt1>0)
		tmpInt1 += 10000;
		else
		tmpInt1-=10000;					
	}
	
	FHCharWrite(specialFrameArg.buf,30,(tmpInt1>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,31,tmpInt1&0xFF);
	Calibration_estimate_struct *calibration_estimate_ptr;
	//自适应轮半径和摄像头角度上传 xingll_20170811
	Calibration_b_get_var_data_ptr(&calibration_estimate_ptr);
	tmpInt1 = (int)(calibration_estimate_ptr->calibrate_angle*1000);

	FHCharWrite(specialFrameArg.buf,33,(tmpInt1>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,34,tmpInt1&0xFF);
	
	tmpInt1 = (int)(Param.AgvParam[PARAM_WheelDia]*10000);
	FHCharWrite(specialFrameArg.buf,35,(tmpInt1>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,36,tmpInt1&0xFF);
	SetReportEvent(eventArg);
	return 0;
}
FH_ERROR_CODE ReportShelfCodeEvent(int32 upCameraX,int32 upCameraY,int16 upCameraH,unsigned char Status)
{
	
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,0x8);//特殊帧类别
	
	
	int16 tmpInt = (int16)(upCameraX & 0x00FFFF);
	FHCharWrite(specialFrameArg.buf,16,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,17,tmpInt&0xFF);
	tmpInt = (int16)((((upCameraX & 0x00FF0000)>>8)>>8) | ((upCameraY & 0x000000FF) << 8));
	FHCharWrite(specialFrameArg.buf,18,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,19,tmpInt&0xFF);
	tmpInt = (int16)((upCameraY & 0x00FFFF00)>>8);
	FHCharWrite(specialFrameArg.buf,20,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,21,tmpInt&0xFF);
	tmpInt = (int16)(upCameraH);
	FHCharWrite(specialFrameArg.buf,22,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,23,tmpInt&0xFF);

	FHCharWrite(specialFrameArg.buf,32,Status&0xFF);
	SetReportEvent(eventArg);
	return 0;
}

FH_ERROR_CODE ReportVersionEvent(int HardwareFlag)
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,0x10);//特殊帧类别
	Uint16 tmpInt = SPC_SUB_VERSION_NO;//
	FHCharWrite(specialFrameArg.buf,16,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,17,tmpInt&0xFF);
	tmpInt = SPC_DEV_VERSION_NO;//GetPowerBoardVersion();//Agv.device.laser.VersionID;
	FHCharWrite(specialFrameArg.buf,18,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,19,tmpInt&0xFF);

	tmpInt = Agv.device.qrcode.dm642->version;
	FHCharWrite(specialFrameArg.buf,20,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,21,tmpInt&0xFF);
	tmpInt = SPC_VERSION_NO;
	FHCharWrite(specialFrameArg.buf,22,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,23,tmpInt&0xFF);
	tmpInt = slam_mode;
	FHCharWrite(specialFrameArg.buf,24,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,25,tmpInt&0xFF);
	//if(slam_mode == AGV_CHECK_MD5)//LHZ add 20170516
		//slam_mode_VeriryFlag = 1;

	tmpInt = HardwareFlag+1;
	FHCharWrite(specialFrameArg.buf,14,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,15,tmpInt&0xFF);
	if(HardwareFlag == 1)
	{
		FeedBack.offsetx = Agv.device.battery.batteryBase.BatSupplyer[0]*256 + Agv.device.battery.batteryBase.BatSupplyer[1];
		FeedBack.offsety = Agv.device.battery.batteryBase.BatHardVision[0]*256 + Agv.device.battery.batteryBase.BatHardVision[1];
		FeedBack.offsetheading = Agv.device.battery.batteryBase.BatHardVision[2]*256 + Agv.device.battery.batteryBase.BatHardVision[3];
		//FeedBack.IOSTATUS.all = Agv.device.laser.LaserSupplyer;
		FeedBack.LeftCurrent = Agv.device.laser.LaserHardVision[0]*256 + Agv.device.laser.LaserHardVision[1];
		FeedBack.RightCurrent = Agv.device.laser.LaserHardVision[2]*256 + Agv.device.laser.LaserHardVision[3];
		FeedBack.LiftCurrent = Agv.device.MotorSupplyer[0]*256 + Agv.device.MotorSupplyer[1];
		FeedBack.BarrierDis = Agv.device.laser.LaserSupplyer;
	}
	else if(HardwareFlag == 2)
	{
		FeedBack.offsetx = Agv.device.motorLeft.md.MotorHardVision[0]*256 + Agv.device.motorLeft.md.MotorHardVision[1];
		FeedBack.offsety = Agv.device.motorLeft.md.MotorHardVision[2]*256 + Agv.device.motorLeft.md.MotorHardVision[3];
		FeedBack.LeftCurrent = Agv.device.motorRight.md.MotorHardVision[0]*256 + Agv.device.motorRight.md.MotorHardVision[1];
		FeedBack.RightCurrent = Agv.device.motorRight.md.MotorHardVision[2]*256 + Agv.device.motorRight.md.MotorHardVision[3];
		FeedBack.LiftCurrent = Agv.device.motorTrayLift.md.MotorHardVision[0]*256 + Agv.device.motorTrayLift.md.MotorHardVision[1];
		FeedBack.BarrierDis = Agv.device.motorTrayLift.md.MotorHardVision[2]*256 + Agv.device.motorTrayLift.md.MotorHardVision[3];
		FeedBack.liftcmdspeed = Agv.device.motorTrayRotate.md.MotorHardVision[0]*256 + Agv.device.motorTrayRotate.md.MotorHardVision[1];
		FeedBack.liftspeed = Agv.device.motorTrayRotate.md.MotorHardVision[2]*256 + Agv.device.motorTrayRotate.md.MotorHardVision[3];
	}
	else if(HardwareFlag == 3)
	{
		FeedBack.offsetx = Agv.device.motorLeft.md.DriverHardVision.DriverType[0]*256 + Agv.device.motorLeft.md.DriverHardVision.DriverType[1];
		FeedBack.offsety = Agv.device.motorLeft.md.DriverHardVision.DriverCode;
		FeedBack.LeftCurrent = Agv.device.motorRight.md.DriverHardVision.DriverType[0]*256 + Agv.device.motorRight.md.DriverHardVision.DriverType[1];
		FeedBack.RightCurrent = Agv.device.motorRight.md.DriverHardVision.DriverCode;
		FeedBack.LiftCurrent = Agv.device.motorTrayLift.md.DriverHardVision.DriverType[0]*256 + Agv.device.motorTrayLift.md.DriverHardVision.DriverType[1];
		FeedBack.BarrierDis = Agv.device.motorTrayLift.md.DriverHardVision.DriverCode;
		FeedBack.liftcmdspeed = Agv.device.motorTrayRotate.md.DriverHardVision.DriverType[0]*256 + Agv.device.motorTrayRotate.md.DriverHardVision.DriverType[1];
		FeedBack.liftspeed = Agv.device.motorTrayRotate.md.DriverHardVision.DriverCode;
	}
	else if(HardwareFlag == 4)
	{
		FeedBack.offsetx = Agv.device.motorLeft.md.DriverSoftVision.Year;
		FeedBack.offsety = Agv.device.motorLeft.md.DriverSoftVision.Day*256 + Agv.device.motorLeft.md.DriverSoftVision.Month;
		FeedBack.LeftCurrent = Agv.device.motorRight.md.DriverSoftVision.Year;
		FeedBack.RightCurrent = Agv.device.motorRight.md.DriverSoftVision.Day*256 + Agv.device.motorRight.md.DriverSoftVision.Month;
		FeedBack.LiftCurrent = Agv.device.motorTrayLift.md.DriverSoftVision.Year;
		FeedBack.BarrierDis = Agv.device.motorTrayLift.md.DriverSoftVision.Day*256 + Agv.device.motorTrayLift.md.DriverSoftVision.Month;
		FeedBack.liftcmdspeed = Agv.device.motorTrayRotate.md.DriverSoftVision.Year;
		FeedBack.liftspeed = Agv.device.motorTrayRotate.md.DriverSoftVision.Day*256 + Agv.device.motorTrayRotate.md.DriverSoftVision.Month;
	}
	FHCharWrite(specialFrameArg.buf,26,(FeedBack.offsetx>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,27,FeedBack.offsetx&0xFF);
	FHCharWrite(specialFrameArg.buf,28,(FeedBack.offsety>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,29,FeedBack.offsety&0xFF);
	FHCharWrite(specialFrameArg.buf,33,(FeedBack.LeftCurrent>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,34,FeedBack.LeftCurrent&0xFF);
	FHCharWrite(specialFrameArg.buf,35,(FeedBack.RightCurrent>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,36,FeedBack.RightCurrent&0xFF);
	FHCharWrite(specialFrameArg.buf,37,(FeedBack.LiftCurrent>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,38,FeedBack.LiftCurrent&0xFF);
	FHCharWrite(specialFrameArg.buf,39,(FeedBack.BarrierDis>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,40,FeedBack.BarrierDis&0xFF);
	FHCharWrite(specialFrameArg.buf,41,(FeedBack.liftcmdspeed>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,42,FeedBack.liftcmdspeed&0xFF);
	FHCharWrite(specialFrameArg.buf,43,(FeedBack.liftspeed>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,44,FeedBack.liftspeed&0xFF);
	
	SetReportEvent(eventArg);

	return 0;
}
FH_ERROR_CODE ReportMileageEvent()
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,0x20);//特殊帧类别
	
	
	Uint32 tmpInt = Agv.service.mileageManager.MileageIIC.LoadMileage;
	FHCharWrite(specialFrameArg.buf,16,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,17,tmpInt&0xFF);
	FHCharWrite(specialFrameArg.buf,18,(tmpInt>>24)&0xFF);
	FHCharWrite(specialFrameArg.buf,19,(tmpInt>>16)&0xFF);
	
	tmpInt = Agv.service.mileageManager.MileageIIC.NoLoadMileage;
	FHCharWrite(specialFrameArg.buf,20,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,21,tmpInt&0xFF);
	FHCharWrite(specialFrameArg.buf,22,(tmpInt>>24)&0xFF);
	FHCharWrite(specialFrameArg.buf,23,(tmpInt>>16)&0xFF);
	tmpInt = Agv.service.mileageManager.MileageIIC.TotalMileage;
	FHCharWrite(specialFrameArg.buf,24,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,25,tmpInt&0xFF);
	FHCharWrite(specialFrameArg.buf,26,(tmpInt>>24)&0xFF);
	FHCharWrite(specialFrameArg.buf,27,(tmpInt>>16)&0xFF);
	tmpInt = Agv.service.mileageManager.MileageIIC.LiftTime;
	FHCharWrite(specialFrameArg.buf,28,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,29,tmpInt&0xFF);
	FHCharWrite(specialFrameArg.buf,30,(tmpInt>>24)&0xFF);
	FHCharWrite(specialFrameArg.buf,31,(tmpInt>>16)&0xFF);
	
	SetReportEvent(eventArg);
	return 0;
}
FH_ERROR_CODE ReportFloorPoorEvent()
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,0x40);//特殊帧类别
	
//	int tmpInt = (int) (FeedBack.agvX_onPoorFloor *100.0);
	int tmpInt = (int) (Agv.service.chassis.location.checkOnDM.OnX *100.0);
	FHCharWrite(specialFrameArg.buf,14,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,15,tmpInt&0xFF);
//	tmpInt = (int) (FeedBack.agvY_onPoorFloor *100.0);
	tmpInt = (int) (Agv.service.chassis.location.checkOnDM.OnY *100.0);
	FHCharWrite(specialFrameArg.buf,16,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,17,tmpInt&0xFF);

	tmpInt = (int) (FeedBack.agvHeaidng_onPoorFloor *180/PI*10.0);
	FHCharWrite(specialFrameArg.buf,18,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,19,tmpInt&0xFF);

//	tmpInt = FeedBack.motion_onPoorFloor;
	tmpInt = Agv.service.chassis.location.checkOnDM.errCode;
	FHCharWrite(specialFrameArg.buf,20,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,21,tmpInt&0xFF);
   
   tmpInt = FeedBack.poorFloor_info;
   FHCharWrite(specialFrameArg.buf,22,(tmpInt>>8)&0xFF);
   FHCharWrite(specialFrameArg.buf,23,tmpInt&0xFF);
   
	tmpInt = (int) (Agv.service.chassis.location.checkOnDM.err1 *1000.0);
	FHCharWrite(specialFrameArg.buf,24,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,25,tmpInt&0xFF);

	tmpInt = (int) (Agv.service.chassis.location.checkOnDM.err2 *1000.0);
	FHCharWrite(specialFrameArg.buf,26,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,27,tmpInt&0xFF);
	
	SetReportEvent(eventArg);

	return 0;
}
//1s一个
FH_ERROR_CODE ReportLoadEvent()
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,0x80);//特殊帧类别

	int16 tmpInt = Agv.service.HMI.InitlizeMode;//LHZ add 20170512
	FHCharWrite(specialFrameArg.buf,24,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,25,tmpInt&0xFF);

	FeedBack.LeftLoadRate =(int)(fabs(Agv.device.motorLeft.md.Current) * Param.AgvParam[PARAM_DriverMotorTorqueCoefficient]/Param.AgvParam[PARAM_DriverMotorTorqueCapacity] * 100.0);
	FeedBack.RightLoadRate =(int)(fabs(Agv.device.motorRight.md.Current) * Param.AgvParam[PARAM_DriverMotorTorqueCoefficient]/Param.AgvParam[PARAM_DriverMotorTorqueCapacity] * 100.0);
	FeedBack.LiftLoadRate =(int)(fabs(Agv.device.motorTrayRotate.md.Current) * Param.AgvParam[PARAM_LiftMotorTorqueCoefficient]/Param.AgvParam[PARAM_LiftMotorTorqueCapacity] * 100.0);

	tmpInt = FeedBack.LeftLoadRate;
	FHCharWrite(specialFrameArg.buf,33,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,34,tmpInt&0xFF);
	tmpInt = FeedBack.RightLoadRate;
	FHCharWrite(specialFrameArg.buf,35,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,36,tmpInt&0xFF);
	tmpInt = FeedBack.LiftLoadRate;
	FHCharWrite(specialFrameArg.buf,37,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,38,tmpInt&0xFF);
	SetReportEvent(eventArg);
	return 0;
}
FH_ERROR_CODE ReportMD5Event()
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位
	FHCharWrite(specialFrameArg.buf,13,0x10);//特殊帧类别,slavepc

	//0-9
	int i=0;
    for(i=0;i<10;i++)
    {
		FHCharWrite(specialFrameArg.buf,i+3,resultr[i]);
    }
	for(i=10;i<20;i++)
    {
        FHCharWrite(specialFrameArg.buf,i+4,resultr[i]);
    }
	int tmpInt = AGV_CHECK_MD5;
	FHCharWrite(specialFrameArg.buf,24,(tmpInt>>8)&0xFF);
	FHCharWrite(specialFrameArg.buf,25,tmpInt&0xFF);
	for(i=20;i<33;i++)
    {
        FHCharWrite(specialFrameArg.buf,i+6,resultr[i]);
    }
	FHCharWrite(specialFrameArg.buf,39,0);
	FHCharWrite(specialFrameArg.buf,40,AGV_CHECK_MD5);
	SetReportEvent(eventArg);
	return 0;
}
FH_ERROR_CODE ReportDymMap()//3
{
	struct SpecialFrameEventArg_t specialFrameArg;
	struct EventArg_t* eventArg = (struct EventArg_t*)&specialFrameArg;
	eventArg->eventType = Event_Report;
	eventArg->length = 24;
	SendSampleData((struct SendDataNormal_t*)specialFrameArg.buf);
	FHBitSet((Uint16*)&specialFrameArg.buf[1],4);//Frame 标志位	
	FHCharWrite(specialFrameArg.buf,13,0x3);//特殊帧类别,地图特殊帧

	return 0;
}
//函数初始化
FH_ERROR_CODE AppLogInit(struct AppLog_t* appLog)
{
	//系统注册发送log数据函数，每15周期(150ms)发送一次
	RegisterSendDataCallback((ProcessBeforeSendData)SendSampleData);
	return 0;
}

