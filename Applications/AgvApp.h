/*
 * FHAgvApp.h
 *
 *  Created on: 2017年9月28日
 *      Author: hwei
 */

#ifndef FHAGVAPP_H_
#define FHAGVAPP_H_

#include "AgvDataType.h"

#include "FHAGV.h"
#include "./AppError/AgvAppError.h"
#include "./AppParams/AgvAppParams.h"
#include "./AppMap/AgvAppMap.h"

#include "./AppInstruction/AgvAppInstruction.h"
#include "./AppLog/AgvAppLog.h"
#include "./AppActionState/AgvAppActionState.h"
#include "./AppWifiCan/AgvWifiManual.h"
#include "./AppEventHandler/AgvAppEventHandler.h"
#include "./AppSafeManager/AgvAppSafeManager.h"
#define MapStartAddress 0x600//len 120*14;
#define MapPointNumAddress  0xE00
#define AppParamsAddress 0x1000//len = 4*200;
#define ChargePointNum 0x2000
#define ChargePointStartAddress 0x2080
#define MaxAppRomAddress 0x5000//限制应用存放区域不能超过0x5000，0x5000以后为系统平台使用

struct AppParams_t
{
	Uint16 AppVersion;
	struct AppBaseParams_t appBaseParams;
	struct MapParams_t mapParams;
};

struct AgvApp_t
{
	struct AppBase_t appBase;
	void* params;
	struct Map_t map;
	struct AgvAppParamsManager_t appParamsManager;
	struct AppInstruction_t appInstruction;
	struct AppLog_t appLog;
	struct AppActionStateManager_t appActionStateManager;
	struct AgvAppSafeManager_t appSafeManager;
};


extern struct AgvApp_t App;
extern struct AppParams_t AppParams;
int AgvAppInit(struct AgvApp_t* app,struct AppParams_t* appParams);

extern struct AgvApp_t App;
int GetCurInstructionId();
int GetCurInstructionTypeId();
int GetPreInstructionTypeId();

#endif /* FHAGVAPP_H_ */
