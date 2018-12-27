/*
 * AgvAppLog.h
 *
 *  Created on: 2017Äê12ÔÂ8ÈÕ
 *      Author: hwei
 */

#ifndef AGVAPPLOG_H_
#define AGVAPPLOG_H_

struct SpecialFrameEventArg_t
{
	struct EventArg_t event;
	int buf[26];
};

struct AppLog_t
{
	int revd;
};
struct SendDataNormal_t
{
	unsigned char head :8;
	//agv state;
	Uint16 OrderExecutionStatus:2;		   // 1:0			
	Uint16 PodLS:1;   // 1	   
	Uint16 GYROZero:1;		 // 1	  
	Uint16 Shelf:2; 	  // 5:4  
	Uint16 FindDM:1;	   //6
	Uint16 MotoStatus:1;	  //7	
	//order state
	Uint16 ID:4;		 // 3:0 		  
	Uint16 Frame:1;   // 1	   
	Uint16 Dormancy:1;		 // 1	  
	Uint16 PowerupCameraData:1; 	  // 1					
	Uint16 DebugMode:1;
	Uint16 filler:8;
	int32 X;//3-6
	int32 Y;//7-10
	int Heading;//11-12
	unsigned char MotionStatus;//13
	int PodHeight;//14-15
	int CMDSpeed;//16-17
	int leftcmdspeed;//18-19
	int rightcmdspeed;//20-21
	int leftspeed;//22-23
	int rightspeed;//24-25
	int offsetx;//26-27
	int offsety;//28-29
	int offsetHeading;//30-31
	//IOSTATUS;//32;
	Uint16 FrontSensorFar:1;         // 1           
	Uint16 FrontSensorNear:1;   // 1     
	Uint16 Discharge:1;       // 1 add sxp 20160921
	Uint16 RearSensorFar:1;       // 1       
	Uint16 PodSensor:1;          // 1    
	Uint16 FrontShelfSensor:1;        // 1     
	Uint16 CLEANER_DIRTY:1;      // 1
	Uint16 FanStatus:1;//HR,add,20160122
	int LeftCurrent : 8;//33
	int RightCurrent : 8;//34
	int LiftCurrent : 8;//35
	int BatteryLoopCurrent : 12;//36-37
	int rsvd1 : 4;//37
	//int rsvd2 : 8;//38
	int BarrierDis;//39-40
	int liftcmdspeed;//41-42
	int liftspeed;//43-44
	int ErrorType;//45
};



FH_ERROR_CODE ReportParamsEvent(int paramIndex,FH_Float paramValue);
FH_ERROR_CODE ReportPowerEvent(FH_Float QuantityElectric,FH_Float RealBatVoltage,FH_Float RealBatCapacity,
								FH_Float temGyro,FH_Float temBattery,FH_Float temMdLeft,FH_Float temMdRight,
								FH_Float temMdTrayRotate,FH_Float temMdTrayLift);
FH_ERROR_CODE ReportQRCodeEvent();
FH_ERROR_CODE ReportShelfCodeEvent(int32 upCameraX,int32 upCameraY,int16 upCameraH,unsigned char Status);

FH_ERROR_CODE ReportVersionEvent(int HardwareFlag);
FH_ERROR_CODE ReportMileageEvent();

FH_ERROR_CODE ReportFloorPoorEvent();
FH_ERROR_CODE ReportLoadEvent();
FH_ERROR_CODE ReportMD5Event();

FH_ERROR_CODE AppLogInit(struct AppLog_t* appLog);
FH_ERROR_CODE ChangeSlamMode(struct AppLog_t* appLog,int isSlamMode);

#endif /* AGVAPPLOG_H_ */
