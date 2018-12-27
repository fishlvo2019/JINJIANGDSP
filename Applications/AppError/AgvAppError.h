/*
 * AgvAppError.h
 *
 *  Created on: 2017年10月25日
 *      Author: hwei
 */

#ifndef AGVAPPERROR_H_
#define AGVAPPERROR_H_

#define MAX_ERROR_NUM 256
#define ERROR_Free 0
#define ERROR_LeftClrCMD 1
#define ERROR_LeftClrError 2
#define ERROR_LeftModeError 3//未使用
#define ERROR_LeftDisError  4//未使用
#define ERROR_LeftEnableError   5//未使用
#define ERROR_RightClrCMD   6
#define ERROR_RightClrError 7
#define ERROR_RightModeError 8//未使用
#define ERROR_RightDisError 9//未使用
#define ERROR_RightEnableError  10//未使用
#define ERROR_LiftClrCMD    11
#define ERROR_LiftClrError  12
#define ERROR_LiftModeError 13//未使用
#define ERROR_LiftDisError  14//未使用
#define ERROR_LiftEnableError   15//未使用
#define ERROR_SpecialFrame   16//ERROR_PDO未使用
#define ERROR_PrintfFull    17//未使用//ERROR_CMDSpd
#define ERROR_CMDLift   18//未使用
#define ERROR_InitialPodPos 19//未使用
#define ERROR_ReadGYRO  20
#define ERROR_LeftSpdTimeOut    21
#define ERROR_RightSpdTimeOut   22
#define ERROR_LiftPosTimeOut    23
#define ERROR_GyroTimeOut       24//未使用
#define ERROR_InsCheckSum 25
#define ERROR_OperateCode 26
#define Error_NoLS 27
#define ERROR_MainLoopTimeOut 28
#define ERROR_ONLINE_FAIL 29//LHZ modified 20170516  级别3
#define ERROR_MoveDirection 30
#define ERROR_TargetPos 31
#define ERROR_ActionTimeOut 32 //指令执行超时
#define ERROR_CMDProcess 33//未使用
#define ERROR_PosUnstable 34
#define ERROR_LostDM 35
#define ERROR_CleanerInstruction 36  //LHZ 0610 ?????????,?????
#define ERROR_Offset 37
#define ERROR_PodHeadingOutTol 38
#define ERROR_NewCMDWithRuning 39
#define ERROR_HeadingUnstable 40
#define ERROR_BadPodPosition 41//未使用
#define ERROR_Offset_Turning 42//未使用
#define ERROR_Offset_Stop 43
#define ERROR_LSTouchWhenShelfOn 44
#define ERROR_SAMEIDCMD 45
#define ERROR_LeftSpeedControl 46
#define ERROR_RightSpeedControl 47
#define ERROR_LiftSpeedControl 48
#define ERROR_StopOnDM 49
#define ERROR_ChangeCodeGap 50//未使用
#define ERROR_ChargeMoveParam 51
#define ERROR_BadFrame 52
#define ERROR_HeadingSpan 53
//#define ERROR_AutoSwith 54
#define ERROR_LASER_ADJUST_FAIL 54 //HR,modified,20171121,雷达中心位校正失败
#define ERROR_NEARSENSOR 55
#define ERROR_FARSENSOR 56
#define ERROR_Slip 57
#define ERROR_SlipS 58
#define ERROR_UpDownClrError 59
#define ERROR_MotorDisable_RejectOrder 60
#define ERROR_SetParam_NoComplete 61
#define ERROR_ParamGiven 62
#define ERROR_MoveSpan 63
#define ERROR_PodCollision_OutSpeed 64
#define ERROR_PodCollision_OutOffset 65
#define ERROR_RejectedKeyEvent_WhenMoving 66
#define ERROR_PodTopIllegalOperation 67
#define ERROR_ReverseMotionInstruction 68
#define ERROR_SoftStop 69
#define ERROR_EStop 70
#define ERROR_LaserTimeOut 71
#define ERROR_HeadingLightUnstable 72  //warning
#define ERROR_PosLightUnstable 73 //warning
#define ERROR_LaserOpen 74
#define ERROR_LostShelf 75
#define ERROR_LowBattery 76
#define ERROR_Run_PodBadPostion 77
#define ERROR_LabelError_Middle 78 //error
#define ERROR_Camera_SpaceCoordinateConversionFailed 79 //warning
#define ERROR_LabelError_High 80 //
    //#define ERROR_Camera_SpaceCoordinateConversionFailed  81 //
    /*#define ERROR_DownCameraIntialFailed 82
#define ERROR_UpCameraIntialFailed 83
#define ERROR_Camera_Vp0OpenFailed 84
#define ERROR_Camera_Vp1OpenFailed 85
#define ERROR_Camera_LanPortConnectFailed 86*///不用的错误码重新设计应用hwei
    //
// 和利时驱动器参数设置失败
#define   ERROR_HLSMOTOR_SetParamsFaild 81
//lift驱动器制动故障（过压）
#define   ERROR_LiftDriverBrakeResistorAbnormal  82
//lift电机编码器ABN信号错误
#define   ERROR_LiftEncoderABNSign 83
//lift电机编码器UVW信号错误
#define   ERROR_LiftEncoderUVWSign 84
//lift驱动器输出短路（过流、过载）
#define   ERROR_LiftDriverI2T  85
//lift驱动器编码器计数错误（和利时失速错误）
#define   ERROR_LiftEncoderCount  86

#define ERROR_AfterLift_ShelfBadPosition 87//顶升后货架位置偏差过大
#define ERROR_CheckUpCameraActionTimeOut 88
#define ERROR_ReadUpCameraFailed 89
#define ERROR_GYROZeroBiasVarLarge 90
#define ERROR_DEBUGMODE_RejectRunOrders 91
#define ERROR_DM642CommTimeOut 92
#define ERROR_TargetOutBorder 93
#define ERROR_SoftStop_RejectOrders 94
#define ERROR_NearSensor_EStop 95
#define ERROR_Moving_RejectNearEndTarget 96
#define ERROR_ExitChargingStation_RejectForward 97
#define ERROR_PodTurning_LRT 98
#define ERROR_ChargeZone_RejectOrders 99
#define ERROR_GetUpCameraDataWithoutOrders 100
#define ERROR_IICTimeOut 101
#define ERROR_RightMotoOverCurrent 102
#define ERROR_LeftMotoOverCurrent 103
#define ERROR_LiftMotoOverCurrent 104
#define ERROR_MapCompensate 105
#define ERROR_KeepPaceTimeOut 106
#define ERROR_ChargeFailed 107
#define ERROR_LaserOpenWithoutData 108
#define ERROR_SameDMOffsetHeading 109
#define ERROR_SecondDMOffsetHeading 110
#define ERROR_LoadPosPoint 111
#define ERROR_LostOneDM 112
#define ERROR_SecondDMOffset 113
#define ERROR_TargetDMOffset 114
#define ERROR_ReadDMDataTimeoutBeforeMove 115//HR,add,20150912
#define ERROR_ReadFrontCameraTimeOut 116//HR，add，20150912
#define ERROR_ReadFloorDMFailure 117//HR,add,20150917
#define ERROR_ReadErrorFloorDM 118//HR,add,20150921
#define ERROR_MotorDisableAbnormal_1 119//HR,ad,20151008
#define ERROR_MotorDisableAbnormal_2 120//HR,ad,20151008
#define ERROR_MotorDisableAbnormal_3 121//HR,ad,20151008
#define ERROR_MotorDisableAbnormal_4 122//HR,ad,20151008
#define ERROR_MotorDisableAbnormal_5 123//HR,ad,20151008
#define ERROR_LeftMotorTargetSpeedSendTimeOut 124//HR,add,20151121
#define ERROR_RightMotorTargetSpeedSendTimeOut 125//HR,add,20151121
#define ERROR_LiftMotorTargetSpeedSendTimeOut 126//HR,add,20151121
#define ERROR_LaserClose 127//HR,add,20151204
#define ERROR_CurvePlan_DecelerateExceed 128//HR,add,20151221（减加速度超过允许值，但没有超过极限值）
#define ERROR_CurvePlan_DecelerateIllegal 129//HR,add,20151221（减加速度超过极限值，无法完成曲线规划）
#define ERROR_CurvePlan_AAIllegal 130//HR，add,20151221（减加加速度超过允许值，无法完成曲线规划）
#define ERROR_CurvePlan_TargetOppositeVelocity 131//HR,add,20151221(目标停车点与当前行驶方向相反，无法完成曲线规划)
#define ERROR_ChargeTouchedNotInChargeZone 132//HR,add,20151230(在非充电区域接收到正在充电信号)
#define ERROR_ChargeRelayNotReadyBeforeMove 133//HR,add,20160106(在开始充电移动时测电压继电器没有就绪)
#define ERROR_PodTouchLS_TimeOut 134 //HR,add,20160125,托盘限位信号确认超时:1s
#define ERROR_LoadedDisapproveChargedMove 135 //HR,add,20160201,带载情况下不允许进充电站
#define ERROR_EndActionBeforeMoveExist 136 //HR,add,20170414,移动结束前的动作已经存在
#define ERROR_Pod_HeadingOffset_AjustTimeOut 137 //HR,add,20160227,托盘Heading调整超时
#define ERROR_PodHeightTouchUpLS_TimeOut 138 //HR,add,20160228,托盘高度限位上开关点信号确认超时:1s
#define ERROR_PodHeightTouchDownLS_TimeOut 139 //HR,add,20160228,托盘高度限位下开关点信号确认超时:1s
#define ERROR_PositionMismatchDM 140 //HR,add,20160321,车辆位置和图像标识位置不匹配
#define ERROR_MapBorder_MinX 141 //HR,add,20160321,地图边界参数错误：X最小值
#define ERROR_MapBorder_MinY 142 //HR,add,20160321,地图边界参数错误：Y最小值
#define ERROR_MapBorder_MaxX 143 //HR,add,20160321,地图边界参数错误：X最大值
#define ERROR_MapBorder_MaxY 144 //HR,add,20160321,地图边界参数错误：Y最大值
#define ERROR_MapDataNotSet 145  //HR,add,20160321,地图数据未设定
#define ERROR_CamerModeTransfer 146 //HR,add,20160323,图像模式切换失败:warning
#define ERROR_LoadConditionID 147 //HR,add,20160323,工况类型设定错误:warning
#define ERROR_StopStatusConfirmTimeOut 148 //HR,add,20160405,车辆停车状态确认超时,错误等级 5
#define ERROR_UpDownSpdTimeOut 149 //HR,add,20160407,升降电机通讯超时
#define ERROR_UpDownClrCMD 150 //HR,add,20160407,升降电机驱动器故障
#define ERROR_UpDownDisError 151 //HR,add,20160407,升降电机关闭失败
#define ERROR_UpDownEnableError 152 //HR,add,20160407,升降电机打开失败
#define ERROR_UpDownMotoOverCurrent 153 //HR,add,20160409,升降电机过流
#define ERROR_SwitchDownCameraTimeOut 154 //HR,add,20160412,切换至下摄像头超时
#define ERROR_LeftDriverSpeedBias 155 //HR,add,20160413,左侧驱动器速度发生漂移,priority=5
#define ERROR_RightDriverSpeedBias 156 //HR,add,20160420,右侧驱动器速度发生漂移,priority=5
#define ERROR_CameraParameterNotSet 157 //HR,add,20160419,图像单元参数未设置完成,priority=3
#define ERROR_UpDownMotorTargetSpeedSendTimeOut 158//HR,add,20160423,升降电机目标速度发送超时
#define ERROR_VehiclePositionBias 159 //HR,add,20160429,停车工况下，车辆位置出现漂移
#define ERROR_PodHeaddingBias 160 //HR,add,20160504,停车工况下，托盘高度出现漂移
#define ERROR_UpDownModeError 161 //HR,add,20160602,升降电机模式设定失败
#define ERROR_LeftMotorCmdSpeedOutrange 162 //HR,add,20160612,左侧电机指令转速超限
#define ERROR_RightMotorCmdSpeedOutrange 163 //HR,add,20160612,右侧电机指令转速超限
#define ERROR_TurnMotorCmdSpeedOutrange 164 //HR,add,20160612,随动转向电机指令转速超限
#define ERROR_VehicleHeadingBias 165 //HR,add,20160614,停车工况下，车辆方向角出现漂移,priority=5
#define ERROR_UP_LS_LOST 166 //HR,add,20160621,顶升限位信号丢失,priority=3
#define ERROR_DOWN_LS_LOST 167 //HR,add,20160621,顶升限位信号丢失,priority=3
    //panj-add
    //减速停车过程速度规划出现反向
#define ERROR_Fdec_Fail 168
    //不符合导引前置条件
#define ERROR_PlanVel_Fail 169
    //充电运动超时
#define ERROR_Wait_Power_Time_Out 170


#define ERROR_TerribleSlipsInMoveing 171 //移动过程中严重打滑，错误等级：3软停
#define ERROR_MoveTargetPointInChargeZone 172 //移动指令发送的目标点是充电位置

    //退出充电区超时
#define ERROR_Wait_Exit_Time_Out 173
    //顶升位置不能导引小车
#define ERROR_Liftpos_RejectOrder 174
    //充电运动补偿距离参数不合适
#define ERROR_Charge_Params 175
    //小车导引过程中转动超时
#define ERROR_Turntimes_Overflow 176
    //小车导引过程中运动超时
#define ERROR_Movetimes_Overflow 177
    //顶升后货架号没解出
#define ERROR_Shelf_Number 178
    //寻码过程，旋转的时候检测到障碍物
#define ERROR_Barrier_Det 179
    //上下限位同时读取到
#define ERROR_Pod_UP_DOWN_LS_Both_Valid 180
#define ERROR_NotInStop_RejectPowerRestart 181 //不在静止状态下，拒绝硬重启设备(警告)
    //重启摄像头超时
#define ERROR_RestartCameraTimeOut 182
    //重启工控机超时
#define ERROR_RestartIPCTimeOut 183
    //重启DSP超时
#define ERROR_RestartDSPTimeOut 184
    //查询电量超时
#define ERROR_QueryEnergyTimeOut 185
    //强制减速运动的目标停车位不符合要求
#define ERROR_STOPPOS_DANGER 186
    //强制减速起始条件不符合
#define ERROR_INPUT_DANGER 187
    //导引完成后切换下摄像头超时
#define ERROR_WAIT_DN642_TIME_OUT 188
    //导引超时，运动次数过多
#define ERROR_GUIDE_TIME_OUT 189
    //运动中收到充电指令(在回退充电的时候有效)
#define ERROR_CHARGECMD_IN_MOVING 190
    //导引过程中642告知失败
#define ERROR_DM642_FAIL 191
    //进入导引模式失败，642超时
#define ERROR_WAIT_UP642_TIME_OUT 192
    //导引模式中642类型错误
#define ERROR_642_MODE 193
    //导引时，642告知的偏移距离太大
#define ERROR_MOVE_RANGE 194
#define ERROR_LeftEncoderABNSign 195    //左侧电机编码器ABN信号错误
#define ERROR_LeftEncoderUVWSign 196    //左侧电机编码器UVW信号错误
#define ERROR_LeftDriverOutputShort 197  //左侧驱动器输出短路
#define ERROR_LeftDriverBrakeResistorAbnormal 198   //左侧驱动器制动电阻异常
#define ERROR_LeftI2T 199     //左侧I2*T故障
#define ERROR_RightEncoderABNSign 200    //右侧电机编码器ABN信号错误
#define ERROR_RightEncoderUVWSign 201    //右侧电机编码器UVW信号错误
#define ERROR_RightDriverOutputShort 202  //右侧驱动器输出短路
#define ERROR_RightDriverBrakeResistorAbnormal 203   //右侧驱动器制动电阻异常
#define ERROR_RightI2T 204    //右侧I2*T故障
#define ERROR_UpDownEncoderABNSign 205    //升降电机编码器ABN信号错误
#define ERROR_UpDownEncoderUVWSign 206    //升降电机编码器UVW信号错误
#define ERROR_UpDownDriverOutputShort 207  //升降驱动器输出短路
#define ERROR_UpDownDriverBrakeResistorAbnormal 208   //升降驱动器制动电阻异常
#define ERROR_UpDownI2T 209    //升降I2*T故障
#define ERROR_LiftUpPod 210    //错误状态升托盘
#define ERROR_SetDownPod 211   //错误状态降托盘
#define ERROR_UpCameraData 212  //上摄像头解码数据类型错误（没有解出码，但告之解码成功,priority=1）
    //上图像中心与下图像中心偏差过大
#define ERROR_DeltaCam 213
    //货架码位置偏差太大
#define ERROR_ShelfPos 214
    //电机编码器计数错误
#define ERROR_LeftEncoderCount 215
#define ERROR_RightEncoderCount 216
#define ERROR_UpDownEncoderCount 217
    //电源接口通讯超时
#define ERROR_PowerHubTimeOut 218
    //电池通讯超时
#define ERROR_BatteryTimeOut 219
    //升降电机实际位置超过目标位置
#define ERROR_UpDownMotorPositionAbnormal 220  //add-sxp 20161222 升降电机实际位置与目标位置不符
    //地图参数点数量超限
#define ERROR_MapPointNum_Exceed 221
    //校正角度过大>45
#define ERROR_Angle_too_large 222
    //做过角度校正
#define ERROR_Angle_adjusted 223
    //没有货架无法复位
#define ERROR_Downpos_RejectOrder 224
    //货架复位前检测上摄像头偏差过大
#define ERROR_Guide_ShelfBadPosition 225
    //关断48V
#define ERROR_Power_Off 226
    //运动参数不合适
#define ERROR_MoveParam_Wrong 227
    //非停车状态下发改运动参数指令
#define ERROR_NonStop_ParamChg 228
    //需要下发参数
    //#define ERROR_Param_NeedSet 229
    //参数区判别失败
    //#define ERROR_ParamZone_JudgeFail 230
    //减速阶段轨迹规划
#define ERROR_DecPlan 229
    //上曝光调整失败
#define ERROR_AdustCameraExposure 230
    //参数保存过程出错
#define ERROR_ParamSave_Fail 231
    //参数读取过程出错
#define ERROR_ParamRead_Fail 232
    //参数范围不正确
#define ERROR_ParamValid_Fail 233
    //地图参数保存过程出错
#define ERROR_MapSave_Fail 234
    //地图参数读取过程出错
#define ERROR_MapRead_Fail 235
    //地图参数校验过程出错
    //#define ERROR_MapValid_Fail 236
    //地图参数数量计数值保存出错
#define ERROR_MapPtValid_Fail 237
    //点数读取过程出错
#define ERROR_MapPtRead_Fail 238
    //校验和保存过程出错
#define ERROR_ChecksumSave_Fail 239
    //摄像头-轮径标定报错
#define ERROR_Calibration_Angle 240
#define ERROR_Calibration_Radius 241
    //充电接触器开关三次
#define ERROR_RELAY_3TIMES 242
    //CMDSpeed跳变保护,按上一次CMDSpeed做斜坡减速
#define ERROR_CMDSpeed_Jump 243

#define ERROR_EMCEvent 244
#define ERROR_Collide 245	//触边
#define ERROR_BatteryCurrentOverLarge 246 //电池过流

#define ERROR_ChargeInterfaceDis 247
#define ERROR_Laser_Leg 248
#define ERROR_TempWarn 251	//温度报警
#define ERROR_TempErr 252	//温度严重报错



    //货架通过限宽门失败
#define ERROR_ShelfDanger 255
#define ERROR_FindNewCode 55

#define ERROR_Platform_System 254
#define WARN_Platform_System 253

typedef enum
{
	APP_ERROR_PARAM = 0x200,
	APP_ERROR_ParamValid_Fail_WhenUpdateSystemParams,
	APP_ERROR_ParamValid_Fail_WhenParamsProtect,
	APP_ERROR_ParamValid_Fail_WhenKincoCheck,
	APP_ERROR_ParamValid_Fail_WhenKincoBigTCheck,
	APP_ERROR_ParamValid_Fail_WhenHLSCheck,
	APP_ERROR_ParamValid_Fail_WhenHLSBigTCheck,
	APP_ERROR_MAP = 0x300,
	
	APP_ERROR_MAP_CHARGEPOINT = APP_ERROR_MAP+0x10,
	APP_ERROR_MAP_XY = APP_ERROR_MAP+0x20,
	APP_ERROR_MAP_ReadNumFail,
	APP_ERROR_MAP_SaveNumFail,
	APP_ERROR_MAP_ReadDataFail,
	APP_ERROR_MAP_SaveDataFail,
	APP_ERROR_INSTRUCTION = 0x400,
	APP_ERROR_INSTRUCTION_Move_Target,
	APP_ERROR_INSTRUCTION_Move_Target_Equal,
	APP_ERROR_INSTRUCTION_Move_Target_Small,
	APP_ERROR_INSTRUCTION_Move_Extra_Task,
	APP_ERROR_INSTRUCTION_Move_Angle,
	APP_ERROR_INSTRUCTION_SHELF = 0x500,
	APP_ERROR_INSTRUCTION_SHELF_ANGLE,
	APP_ERROR_LOG = 0x600,
	AppErrorMax = 0xFFF
}AppErrorType;
FH_ERROR_CODE SetAppErrorToQue(AppErrorType type,ErrorPriorityEnum priority);
unsigned char TranslateErrorCode(Uint16 systemError);


#endif /* AGVAPPERROR_H_ */
