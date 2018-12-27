/*
 * AgvAppError.h
 *
 *  Created on: 2017��10��25��
 *      Author: hwei
 */

#ifndef AGVAPPERROR_H_
#define AGVAPPERROR_H_

#define MAX_ERROR_NUM 256
#define ERROR_Free 0
#define ERROR_LeftClrCMD 1
#define ERROR_LeftClrError 2
#define ERROR_LeftModeError 3//δʹ��
#define ERROR_LeftDisError  4//δʹ��
#define ERROR_LeftEnableError   5//δʹ��
#define ERROR_RightClrCMD   6
#define ERROR_RightClrError 7
#define ERROR_RightModeError 8//δʹ��
#define ERROR_RightDisError 9//δʹ��
#define ERROR_RightEnableError  10//δʹ��
#define ERROR_LiftClrCMD    11
#define ERROR_LiftClrError  12
#define ERROR_LiftModeError 13//δʹ��
#define ERROR_LiftDisError  14//δʹ��
#define ERROR_LiftEnableError   15//δʹ��
#define ERROR_SpecialFrame   16//ERROR_PDOδʹ��
#define ERROR_PrintfFull    17//δʹ��//ERROR_CMDSpd
#define ERROR_CMDLift   18//δʹ��
#define ERROR_InitialPodPos 19//δʹ��
#define ERROR_ReadGYRO  20
#define ERROR_LeftSpdTimeOut    21
#define ERROR_RightSpdTimeOut   22
#define ERROR_LiftPosTimeOut    23
#define ERROR_GyroTimeOut       24//δʹ��
#define ERROR_InsCheckSum 25
#define ERROR_OperateCode 26
#define Error_NoLS 27
#define ERROR_MainLoopTimeOut 28
#define ERROR_ONLINE_FAIL 29//LHZ modified 20170516  ����3
#define ERROR_MoveDirection 30
#define ERROR_TargetPos 31
#define ERROR_ActionTimeOut 32 //ָ��ִ�г�ʱ
#define ERROR_CMDProcess 33//δʹ��
#define ERROR_PosUnstable 34
#define ERROR_LostDM 35
#define ERROR_CleanerInstruction 36  //LHZ 0610 ?????????,?????
#define ERROR_Offset 37
#define ERROR_PodHeadingOutTol 38
#define ERROR_NewCMDWithRuning 39
#define ERROR_HeadingUnstable 40
#define ERROR_BadPodPosition 41//δʹ��
#define ERROR_Offset_Turning 42//δʹ��
#define ERROR_Offset_Stop 43
#define ERROR_LSTouchWhenShelfOn 44
#define ERROR_SAMEIDCMD 45
#define ERROR_LeftSpeedControl 46
#define ERROR_RightSpeedControl 47
#define ERROR_LiftSpeedControl 48
#define ERROR_StopOnDM 49
#define ERROR_ChangeCodeGap 50//δʹ��
#define ERROR_ChargeMoveParam 51
#define ERROR_BadFrame 52
#define ERROR_HeadingSpan 53
//#define ERROR_AutoSwith 54
#define ERROR_LASER_ADJUST_FAIL 54 //HR,modified,20171121,�״�����λУ��ʧ��
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
#define ERROR_Camera_LanPortConnectFailed 86*///���õĴ������������Ӧ��hwei
    //
// ����ʱ��������������ʧ��
#define   ERROR_HLSMOTOR_SetParamsFaild 81
//lift�������ƶ����ϣ���ѹ��
#define   ERROR_LiftDriverBrakeResistorAbnormal  82
//lift���������ABN�źŴ���
#define   ERROR_LiftEncoderABNSign 83
//lift���������UVW�źŴ���
#define   ERROR_LiftEncoderUVWSign 84
//lift�����������·�����������أ�
#define   ERROR_LiftDriverI2T  85
//lift�������������������󣨺���ʱʧ�ٴ���
#define   ERROR_LiftEncoderCount  86

#define ERROR_AfterLift_ShelfBadPosition 87//���������λ��ƫ�����
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
#define ERROR_ReadFrontCameraTimeOut 116//HR��add��20150912
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
#define ERROR_CurvePlan_DecelerateExceed 128//HR,add,20151221�������ٶȳ�������ֵ����û�г�������ֵ��
#define ERROR_CurvePlan_DecelerateIllegal 129//HR,add,20151221�������ٶȳ�������ֵ���޷�������߹滮��
#define ERROR_CurvePlan_AAIllegal 130//HR��add,20151221�����Ӽ��ٶȳ�������ֵ���޷�������߹滮��
#define ERROR_CurvePlan_TargetOppositeVelocity 131//HR,add,20151221(Ŀ��ͣ�����뵱ǰ��ʻ�����෴���޷�������߹滮)
#define ERROR_ChargeTouchedNotInChargeZone 132//HR,add,20151230(�ڷǳ��������յ����ڳ���ź�)
#define ERROR_ChargeRelayNotReadyBeforeMove 133//HR,add,20160106(�ڿ�ʼ����ƶ�ʱ���ѹ�̵���û�о���)
#define ERROR_PodTouchLS_TimeOut 134 //HR,add,20160125,������λ�ź�ȷ�ϳ�ʱ:1s
#define ERROR_LoadedDisapproveChargedMove 135 //HR,add,20160201,��������²���������վ
#define ERROR_EndActionBeforeMoveExist 136 //HR,add,20170414,�ƶ�����ǰ�Ķ����Ѿ�����
#define ERROR_Pod_HeadingOffset_AjustTimeOut 137 //HR,add,20160227,����Heading������ʱ
#define ERROR_PodHeightTouchUpLS_TimeOut 138 //HR,add,20160228,���̸߶���λ�Ͽ��ص��ź�ȷ�ϳ�ʱ:1s
#define ERROR_PodHeightTouchDownLS_TimeOut 139 //HR,add,20160228,���̸߶���λ�¿��ص��ź�ȷ�ϳ�ʱ:1s
#define ERROR_PositionMismatchDM 140 //HR,add,20160321,����λ�ú�ͼ���ʶλ�ò�ƥ��
#define ERROR_MapBorder_MinX 141 //HR,add,20160321,��ͼ�߽��������X��Сֵ
#define ERROR_MapBorder_MinY 142 //HR,add,20160321,��ͼ�߽��������Y��Сֵ
#define ERROR_MapBorder_MaxX 143 //HR,add,20160321,��ͼ�߽��������X���ֵ
#define ERROR_MapBorder_MaxY 144 //HR,add,20160321,��ͼ�߽��������Y���ֵ
#define ERROR_MapDataNotSet 145  //HR,add,20160321,��ͼ����δ�趨
#define ERROR_CamerModeTransfer 146 //HR,add,20160323,ͼ��ģʽ�л�ʧ��:warning
#define ERROR_LoadConditionID 147 //HR,add,20160323,���������趨����:warning
#define ERROR_StopStatusConfirmTimeOut 148 //HR,add,20160405,����ͣ��״̬ȷ�ϳ�ʱ,����ȼ� 5
#define ERROR_UpDownSpdTimeOut 149 //HR,add,20160407,�������ͨѶ��ʱ
#define ERROR_UpDownClrCMD 150 //HR,add,20160407,�����������������
#define ERROR_UpDownDisError 151 //HR,add,20160407,��������ر�ʧ��
#define ERROR_UpDownEnableError 152 //HR,add,20160407,���������ʧ��
#define ERROR_UpDownMotoOverCurrent 153 //HR,add,20160409,�����������
#define ERROR_SwitchDownCameraTimeOut 154 //HR,add,20160412,�л���������ͷ��ʱ
#define ERROR_LeftDriverSpeedBias 155 //HR,add,20160413,����������ٶȷ���Ư��,priority=5
#define ERROR_RightDriverSpeedBias 156 //HR,add,20160420,�Ҳ��������ٶȷ���Ư��,priority=5
#define ERROR_CameraParameterNotSet 157 //HR,add,20160419,ͼ��Ԫ����δ�������,priority=3
#define ERROR_UpDownMotorTargetSpeedSendTimeOut 158//HR,add,20160423,�������Ŀ���ٶȷ��ͳ�ʱ
#define ERROR_VehiclePositionBias 159 //HR,add,20160429,ͣ�������£�����λ�ó���Ư��
#define ERROR_PodHeaddingBias 160 //HR,add,20160504,ͣ�������£����̸߶ȳ���Ư��
#define ERROR_UpDownModeError 161 //HR,add,20160602,�������ģʽ�趨ʧ��
#define ERROR_LeftMotorCmdSpeedOutrange 162 //HR,add,20160612,�����ָ��ת�ٳ���
#define ERROR_RightMotorCmdSpeedOutrange 163 //HR,add,20160612,�Ҳ���ָ��ת�ٳ���
#define ERROR_TurnMotorCmdSpeedOutrange 164 //HR,add,20160612,�涯ת����ָ��ת�ٳ���
#define ERROR_VehicleHeadingBias 165 //HR,add,20160614,ͣ�������£���������ǳ���Ư��,priority=5
#define ERROR_UP_LS_LOST 166 //HR,add,20160621,������λ�źŶ�ʧ,priority=3
#define ERROR_DOWN_LS_LOST 167 //HR,add,20160621,������λ�źŶ�ʧ,priority=3
    //panj-add
    //����ͣ�������ٶȹ滮���ַ���
#define ERROR_Fdec_Fail 168
    //�����ϵ���ǰ������
#define ERROR_PlanVel_Fail 169
    //����˶���ʱ
#define ERROR_Wait_Power_Time_Out 170


#define ERROR_TerribleSlipsInMoveing 171 //�ƶ����������ش򻬣�����ȼ���3��ͣ
#define ERROR_MoveTargetPointInChargeZone 172 //�ƶ�ָ��͵�Ŀ����ǳ��λ��

    //�˳��������ʱ
#define ERROR_Wait_Exit_Time_Out 173
    //����λ�ò��ܵ���С��
#define ERROR_Liftpos_RejectOrder 174
    //����˶������������������
#define ERROR_Charge_Params 175
    //С������������ת����ʱ
#define ERROR_Turntimes_Overflow 176
    //С�������������˶���ʱ
#define ERROR_Movetimes_Overflow 177
    //��������ܺ�û���
#define ERROR_Shelf_Number 178
    //Ѱ����̣���ת��ʱ���⵽�ϰ���
#define ERROR_Barrier_Det 179
    //������λͬʱ��ȡ��
#define ERROR_Pod_UP_DOWN_LS_Both_Valid 180
#define ERROR_NotInStop_RejectPowerRestart 181 //���ھ�ֹ״̬�£��ܾ�Ӳ�����豸(����)
    //��������ͷ��ʱ
#define ERROR_RestartCameraTimeOut 182
    //�������ػ���ʱ
#define ERROR_RestartIPCTimeOut 183
    //����DSP��ʱ
#define ERROR_RestartDSPTimeOut 184
    //��ѯ������ʱ
#define ERROR_QueryEnergyTimeOut 185
    //ǿ�Ƽ����˶���Ŀ��ͣ��λ������Ҫ��
#define ERROR_STOPPOS_DANGER 186
    //ǿ�Ƽ�����ʼ����������
#define ERROR_INPUT_DANGER 187
    //������ɺ��л�������ͷ��ʱ
#define ERROR_WAIT_DN642_TIME_OUT 188
    //������ʱ���˶���������
#define ERROR_GUIDE_TIME_OUT 189
    //�˶����յ����ָ��(�ڻ��˳���ʱ����Ч)
#define ERROR_CHARGECMD_IN_MOVING 190
    //����������642��֪ʧ��
#define ERROR_DM642_FAIL 191
    //���뵼��ģʽʧ�ܣ�642��ʱ
#define ERROR_WAIT_UP642_TIME_OUT 192
    //����ģʽ��642���ʹ���
#define ERROR_642_MODE 193
    //����ʱ��642��֪��ƫ�ƾ���̫��
#define ERROR_MOVE_RANGE 194
#define ERROR_LeftEncoderABNSign 195    //�����������ABN�źŴ���
#define ERROR_LeftEncoderUVWSign 196    //�����������UVW�źŴ���
#define ERROR_LeftDriverOutputShort 197  //��������������·
#define ERROR_LeftDriverBrakeResistorAbnormal 198   //����������ƶ������쳣
#define ERROR_LeftI2T 199     //���I2*T����
#define ERROR_RightEncoderABNSign 200    //�Ҳ���������ABN�źŴ���
#define ERROR_RightEncoderUVWSign 201    //�Ҳ���������UVW�źŴ���
#define ERROR_RightDriverOutputShort 202  //�Ҳ������������·
#define ERROR_RightDriverBrakeResistorAbnormal 203   //�Ҳ��������ƶ������쳣
#define ERROR_RightI2T 204    //�Ҳ�I2*T����
#define ERROR_UpDownEncoderABNSign 205    //�������������ABN�źŴ���
#define ERROR_UpDownEncoderUVWSign 206    //�������������UVW�źŴ���
#define ERROR_UpDownDriverOutputShort 207  //���������������·
#define ERROR_UpDownDriverBrakeResistorAbnormal 208   //�����������ƶ������쳣
#define ERROR_UpDownI2T 209    //����I2*T����
#define ERROR_LiftUpPod 210    //����״̬������
#define ERROR_SetDownPod 211   //����״̬������
#define ERROR_UpCameraData 212  //������ͷ�����������ʹ���û�н���룬����֮����ɹ�,priority=1��
    //��ͼ����������ͼ������ƫ�����
#define ERROR_DeltaCam 213
    //������λ��ƫ��̫��
#define ERROR_ShelfPos 214
    //�����������������
#define ERROR_LeftEncoderCount 215
#define ERROR_RightEncoderCount 216
#define ERROR_UpDownEncoderCount 217
    //��Դ�ӿ�ͨѶ��ʱ
#define ERROR_PowerHubTimeOut 218
    //���ͨѶ��ʱ
#define ERROR_BatteryTimeOut 219
    //�������ʵ��λ�ó���Ŀ��λ��
#define ERROR_UpDownMotorPositionAbnormal 220  //add-sxp 20161222 �������ʵ��λ����Ŀ��λ�ò���
    //��ͼ��������������
#define ERROR_MapPointNum_Exceed 221
    //У���Ƕȹ���>45
#define ERROR_Angle_too_large 222
    //�����Ƕ�У��
#define ERROR_Angle_adjusted 223
    //û�л����޷���λ
#define ERROR_Downpos_RejectOrder 224
    //���ܸ�λǰ���������ͷƫ�����
#define ERROR_Guide_ShelfBadPosition 225
    //�ض�48V
#define ERROR_Power_Off 226
    //�˶�����������
#define ERROR_MoveParam_Wrong 227
    //��ͣ��״̬�·����˶�����ָ��
#define ERROR_NonStop_ParamChg 228
    //��Ҫ�·�����
    //#define ERROR_Param_NeedSet 229
    //�������б�ʧ��
    //#define ERROR_ParamZone_JudgeFail 230
    //���ٽ׶ι켣�滮
#define ERROR_DecPlan 229
    //���ع����ʧ��
#define ERROR_AdustCameraExposure 230
    //����������̳���
#define ERROR_ParamSave_Fail 231
    //������ȡ���̳���
#define ERROR_ParamRead_Fail 232
    //������Χ����ȷ
#define ERROR_ParamValid_Fail 233
    //��ͼ����������̳���
#define ERROR_MapSave_Fail 234
    //��ͼ������ȡ���̳���
#define ERROR_MapRead_Fail 235
    //��ͼ����У����̳���
    //#define ERROR_MapValid_Fail 236
    //��ͼ������������ֵ�������
#define ERROR_MapPtValid_Fail 237
    //������ȡ���̳���
#define ERROR_MapPtRead_Fail 238
    //У��ͱ�����̳���
#define ERROR_ChecksumSave_Fail 239
    //����ͷ-�־��궨����
#define ERROR_Calibration_Angle 240
#define ERROR_Calibration_Radius 241
    //���Ӵ�����������
#define ERROR_RELAY_3TIMES 242
    //CMDSpeed���䱣��,����һ��CMDSpeed��б�¼���
#define ERROR_CMDSpeed_Jump 243

#define ERROR_EMCEvent 244
#define ERROR_Collide 245	//����
#define ERROR_BatteryCurrentOverLarge 246 //��ع���

#define ERROR_ChargeInterfaceDis 247
#define ERROR_Laser_Leg 248
#define ERROR_TempWarn 251	//�¶ȱ���
#define ERROR_TempErr 252	//�¶����ر���



    //����ͨ���޿���ʧ��
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
