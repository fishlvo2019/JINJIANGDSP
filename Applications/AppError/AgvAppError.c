/*
 * AgvAppError.c
 *
 *  Created on: 2017Äê11ÔÂ22ÈÕ
 *      Author: hwei
 */
#include"AgvApp.h"
FH_ERROR_CODE SetAppErrorToQue(AppErrorType type,ErrorPriorityEnum priority)
{
	return SetErrorToQue((ErrorTypeEnum)type,priority);
}

unsigned char TranslateErrorCode(Uint16 systemError)
{
	unsigned char appErrorCode = ERROR_Platform_System;
	if(systemError&0x8000)
	{
		appErrorCode = WARN_Platform_System;
	}
	systemError = systemError&0x7FFF;
	if((systemError>=SYSTEM_ERROR_Motor)&&(systemError<SYSTEM_ERROR_Laser))
	{
		switch(systemError&0xFFF0)
		{
			case SYSTEM_ERROR_Motor:
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftClrError;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightClrError;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftClrError;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownClrError;
				}
				else
				{
					appErrorCode = ERROR_LeftMotorTargetSpeedSendTimeOut;
				}
				break;
			case SYSTEM_ERROR_Motor_Communication:
				break;
			case SYSTEM_ERROR_Motor_TimeOut:
				
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftSpdTimeOut;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightSpdTimeOut;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftPosTimeOut;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownSpdTimeOut;
				}
				break;
			case SYSTEM_ERROR_Motor_VelCtrlErr:
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftSpeedControl;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightSpeedControl;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftSpeedControl;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftSpeedControl;
				}
				break;
			case SYSTEM_ERROR_Motor_EncoderABNSign:
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftEncoderABNSign;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightEncoderABNSign;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftEncoderABNSign;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownEncoderABNSign;
				}
				break;
			case SYSTEM_ERROR_Motor_EncoderUVWSign:
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftEncoderUVWSign;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightEncoderUVWSign;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftEncoderUVWSign;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownEncoderUVWSign;
				}
				break;
			case SYSTEM_ERROR_Motor_OverPressure:
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftDriverBrakeResistorAbnormal;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightDriverBrakeResistorAbnormal;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftDriverBrakeResistorAbnormal;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownDriverBrakeResistorAbnormal;
				}
				break;
			case SYSTEM_ERROR_Motor_OverCurrent:
			case SYSTEM_ERROR_Motor_StaticErr:
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftMotoOverCurrent;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightMotoOverCurrent;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftMotoOverCurrent;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownMotoOverCurrent;
				}
				break;
			case SYSTEM_ERROR_Motor_SpeedOut:
				
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftEncoderCount;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightEncoderCount;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftEncoderCount;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownEncoderCount;
				}
				break;
			case SYSTEM_ERROR_Motor_AB:
				
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftClrCMD;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightClrCMD;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftClrCMD;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownClrCMD;
				}
				break;
			case SYSTEM_ERROR_Motor_State:
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftI2T;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightI2T;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_LiftDriverI2T;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_UpDownI2T;
				}
				
				break;
			case SYSTEM_ERROR_Motor_RPMLimitWarn:
				if(0x1 == (systemError&0xF))
				{
					appErrorCode = ERROR_LeftMotorCmdSpeedOutrange;
				}
				else if(0x2 == (systemError&0xF))
				{
					appErrorCode = ERROR_RightMotorCmdSpeedOutrange;
				}
				else if(0x4 == (systemError&0xF))
				{
					appErrorCode = ERROR_TurnMotorCmdSpeedOutrange;
				}
				else if(0x5 == (systemError&0xF))
				{
					appErrorCode = ERROR_TurnMotorCmdSpeedOutrange;
				}
				break;
			case SYSTEM_ERROR_Motor_Disable:
				appErrorCode = ERROR_MotorDisableAbnormal_2;
				break;
		}
	}
	else
	{
		switch(systemError)
		{
			case SYSTEM_ERROR_PC_Communication_SAMEID:
				appErrorCode = ERROR_SAMEIDCMD;
				break;
			case	SYSTEM_ERROR_Tray_UDLimit:
				appErrorCode = ERROR_Pod_UP_DOWN_LS_Both_Valid;
				break;
			case SYSTEM_ERROR_DM642_BadFrame:
				appErrorCode = ERROR_BadFrame;
				break;
			case SYSTEM_ERROR_DM642_TimeOut:
				appErrorCode = ERROR_DM642CommTimeOut;
				break;
			case SYSTEM_ERROR_DM642_CheckSum:
				appErrorCode = ERROR_InsCheckSum;
				break;
			case SYSTEM_ERROR_Laser_OpenFail:
			case SYSTEM_ERROR_Laser_TimeOut:
			case SYSTEM_ERROR_Laser_NoDataTimeOut:
				appErrorCode = ERROR_LaserOpenWithoutData;
				break;
			case SYSTEM_ERROR_Safe_Collide:
				appErrorCode = ERROR_Collide;
				break;
			case SYSTEM_ERROR_Safe_EMC:
				appErrorCode = ERROR_EMCEvent;
				break;
			case SYSTEM_ERROR_Safe_CHARGE_POSITION:
				appErrorCode = ERROR_ChargeInterfaceDis;
				break;
			case SYSTEM_ERROR_PC_Communication_NULLTypeID:
			case SYSTEM_ERROR_PC_Communication_TypeIDMapWrong:
				appErrorCode = ERROR_OperateCode;
				break;
			case SYSTEM_ERROR_Tray_UPLimit:
				appErrorCode = ERROR_UP_LS_LOST;
				break;
			case SYSTEM_ERROR_Tray_DownLimit:
				appErrorCode = ERROR_DOWN_LS_LOST;
				break;
			case	SYSTEM_ERROR_SPG_PARAMSERROR_InitVel:
				appErrorCode = ERROR_CurvePlan_TargetOppositeVelocity;
			//case	SYSTEM_ERROR_PowerBoard_Com:
			//	appErrorCode = ;
				break;
			case	APP_ERROR_ParamValid_Fail_WhenUpdateSystemParams:
			case	APP_ERROR_ParamValid_Fail_WhenParamsProtect:
			case	APP_ERROR_ParamValid_Fail_WhenKincoCheck:
			case	APP_ERROR_ParamValid_Fail_WhenKincoBigTCheck:
			case	APP_ERROR_ParamValid_Fail_WhenHLSCheck:
			case	APP_ERROR_ParamValid_Fail_WhenHLSBigTCheck:
				appErrorCode = ERROR_ParamValid_Fail;
				break;
			case SYSTEM_ERROR_Platform_MainThreadTimeOut:	
				appErrorCode = ERROR_MainLoopTimeOut;
				break;
			case SYSTEM_ERROR_Tray_BiasErr:	
				appErrorCode = ERROR_PodHeaddingBias;
				break;
			case SYSTEM_ERROR_Chassis_PositionBias:	
				appErrorCode = ERROR_VehiclePositionBias;
				break;
			case SYSTEM_ERROR_Chassis_HeadingBias:	
				appErrorCode = ERROR_VehicleHeadingBias;
				break;
			case	APP_ERROR_INSTRUCTION_Move_Target:
				appErrorCode = ERROR_TargetPos;
				break;
			case SYSTEM_ERROR_Battery_WaitPowerTimeOut:
				appErrorCode = ERROR_Wait_Power_Time_Out;
				break;
			case 	SYSTEM_ERROR_Chassis_Slip:
				appErrorCode = ERROR_Slip;
				break;
			case	SYSTEM_ERROR_PowerManger_RecoverFail:
				appErrorCode = ERROR_RELAY_3TIMES;
				break;
			case	SYSTEM_ERROR_PowerBoard_QueryEnergyTimeOut:
				appErrorCode =ERROR_QueryEnergyTimeOut;
				break;
			case	SYSTEM_ERROR_Board_i2c_ReadTimeOut:
				appErrorCode =ERROR_IICTimeOut;
				break;
			case SYSTEM_ERROR_AxisCtrlErrOverLarge:
                appErrorCode = ERROR_Offset_Stop;
                break;
            case SYSTEM_ERROR_AxisCtrlErrOverLarge+0x1:
                appErrorCode = ERROR_Offset;
                break;
            case SYSTEM_ERROR_AxisCtrlErrOverLarge+0x4:
                appErrorCode = ERROR_PodHeadingOutTol;
                break;
			case SYSTEM_ERROR_AxisEmcStop:
					appErrorCode = ERROR_EStop;
				break;
			case SYSTEM_ERROR_Config_MDSupplyWrong:
				appErrorCode = ERROR_HLSMOTOR_SetParamsFaild;
				break;
			case SYSTEM_ERROR_GYRO:
			case SYSTEM_ERROR_GYRO_ReadErr:
			case SYSTEM_ERROR_GYRO_ValueException:
			case SYSTEM_ERROR_GYRO_ZeroBiasCalValueReadErr:
				appErrorCode = ERROR_ReadGYRO;
				break;
			case SYSTEM_ERROR_GYRO_ZeroBiasCal:
				appErrorCode = ERROR_GYROZeroBiasVarLarge;
				break;
			case SYSTEM_ERROR_GYRO_TimeOut:
				appErrorCode = ERROR_GyroTimeOut;
				break;
			case SYSTEM_ERROR_MotionControl_TimeOut:
				appErrorCode = ERROR_ActionTimeOut;
				break;
			case SYSTEM_ERROR_Event:	
			case	SYSTEM_ERROR_Event_CntOverLarge:
			case	SYSTEM_ERROR_Event_ParamsOverLarge:
			case	SYSTEM_ERROR_Event_FifoResource:
			case	SYSTEM_ERROR_Event_GetLen:
			case	SYSTEM_ERROR_Event_Type:
			case	SYSTEM_ERROR_Event_RegisterHandler:
				appErrorCode = ERROR_SpecialFrame;
				break;
			case SYSTEM_ERROR_Printf:
				appErrorCode = ERROR_PrintfFull;
				break;	
			case	SYSTEM_ERROR_AvoidObstacle_FarSenser:
				appErrorCode = ERROR_FARSENSOR;
				break;
			case	SYSTEM_ERROR_AvoidObstacle_NearSenser:
				appErrorCode = ERROR_NEARSENSOR;
				break;
			case	SYSTEM_ERROR_Chassis_CmdSpeedJump:
				appErrorCode = ERROR_CMDSpeed_Jump;
				break;
			case	SYSTEM_ERROR_Laser_ZeroAdjust:
				appErrorCode = ERROR_LASER_ADJUST_FAIL;
				break;
			default:
				//appErrorCode = ERROR_Platform_System;
				break;
		}
	}
	
	return appErrorCode;
}

