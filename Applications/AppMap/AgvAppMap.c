/******************************************************************************
* 版权信息：   	QuickTron
* 文 件 名：     	Map.c
* 作    者：		xingll
* 功    能：		地图数据
* 修订记录:    版本		 		修订内容			修订者				修改日期
v1.0		创建				新增				xingll			20171019
*******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "AgvApp.h"


#pragma DATA_SECTION(MapBuf,"ZONE6DATA");
#pragma DATA_SECTION(ChargePointBuf,"ZONE6DATA");

struct MapPointType_t MapBuf[MAP_POINT_NUM_MAX];
struct ChargePoint_t ChargePointBuf[CHARGE_POINT_NUM_MAX];

//Uint16 testPoint =0;
//int16 testPointNum = 0;
/*-----------------------------------------------------------------------------	
*输.....入：mapParams-地图参数
*输.....出：无
*输入/输出：map-地图数据
*返.....回：无
*目.....的：初始化地图数据
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
FH_ERROR_CODE AppMapInfoInit(struct Map_t* map,struct MapParams_t* mapParams)
{
	MemSet(map,0x0,sizeof(struct Map_t));
	MemSet(MapBuf,0x0,sizeof(struct MapPointType_t));
	MemSet(ChargePointBuf,0x0,sizeof(struct ChargePoint_t));
	map->mapPointType = MapBuf;
	map->chargePointInfo = ChargePointBuf;
	return 0;
}
/*------------------------------------------------------------------------------	
*输.....入：mapParams-地图参数
*输.....出：无
*输入/输出：map-地图数据
*返.....回：无
*目.....的：保存数据到IIC
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
FH_ERROR_CODE SetMapPointNUM(struct Map_t* map)
{
	int i = 0;
	int rel_val = 0;
//	Uint16* tmpAdd = (Uint16*)(&map->mapPointType);
	map->MapPointNUM = map->MapPointNUM&0xFF;

	//保存地图数据个数到iic
	rel_val = EEPRomWrite(&Agv.device.eepRom,MapPointNumAddress,(Uint16*)&map->MapPointNUM,1);
//	rel_val = IICRead(&Agv.board.i2c,RomAddress,MapPointNumAddress,(Uint16*)(&testPoint),1);
	if(rel_val) 
	{
		LogPrintf("=ERROR= WriteMapData cnt:%d\n",i);
//		SetError(ERROR_MAP_SaveNumFail,3);
	}
	ThreadSleep(10);
	return 0;
}
int SetMapInfo4Single(struct Map_t* map,int MapIndex)
{
	if(MapIndex>=MAP_POINT_NUM_MAX)
	{
		SetErrorToQue((ErrorTypeEnum)ERROR_MapPointNum_Exceed,ErrorC);
	}
	int rel_val = 0;
	rel_val = EEPRomWrite(&Agv.device.eepRom,MapStartAddress + SizeOfByte(struct MapPointType_t)*MapIndex,(Uint16*)&map->mapPointType[MapIndex],sizeof(struct MapPointType_t));
	if(rel_val) 
	{
		LogPrintf("=ERROR= WriteSingleParamData\n");
//		SetError(ERROR_MAP_SaveNumFail,3);
	}
	return 0;
}
FH_ERROR_CODE SetChargePointInfo4Single(struct Map_t* map,int x,int y)
{
	int rel_val = 0;
	int i=0;
	for(i=0;i < map->ChargePointNUM;i++)
	{
		if ((x == map->chargePointInfo[i].baseX)
		&& (y == map->chargePointInfo[i].baseY))
		{//接收到已经存在的地图点，则更新该地图点数据
			return 0;
		}
	}
	if (i>= map->ChargePointNUM)//接收到新的地图点,保存该地图点数据
	{
		if (map->ChargePointNUM >= CHARGE_POINT_NUM_MAX)
		{
			SetErrorToQue((ErrorTypeEnum)ERROR_MapPointNum_Exceed,ErrorC);
			return -1;
		}
		else
		{

			map->chargePointInfo[i].baseX =x;
			map->chargePointInfo[i].baseY =y;
			map->ChargePointNUM ++;
			
			rel_val = EEPRomWrite(&Agv.device.eepRom,ChargePointStartAddress + SizeOfByte(struct ChargePoint_t)*i,(Uint16*)&map->chargePointInfo[i],sizeof(struct ChargePoint_t));
			ThreadSleep(10);
			if(rel_val) 
			{
				LogPrintf("=ERROR= WriteSingleParamData\n");
				SetErrorToQue((ErrorTypeEnum)APP_ERROR_MAP_SaveNumFail,ErrorC);
				return -2;
			}
		}
	}
	return 0;
}
FH_ERROR_CODE SetChargePointNUM(struct Map_t* map)
{
	//保存地图数据个数到iic
	if(EEPRomWrite(&Agv.device.eepRom,ChargePointNum,(Uint16*)&map->ChargePointNUM,1))
	{
		LogPrintf("=ERROR= SetChargePointNUM\n" );
		SetErrorToQue((ErrorTypeEnum)APP_ERROR_MAP_CHARGEPOINT,ErrorC);
		return -1;
	}
	ThreadSleep(10);
return 0;
}

FH_ERROR_CODE GetChargePointInfo(struct Map_t* map)
{
	FH_ERROR_CODE rtn =0;
	if(EEPRomRead(&Agv.device.eepRom,ChargePointNum,(Uint16*)(&map->ChargePointNUM),1))
	{
		LogPrintf("=ERROR= GetChargePointInfo\n");
		SetErrorToQue((ErrorTypeEnum)APP_ERROR_MAP_CHARGEPOINT,ErrorC);
		rtn= -1;
	}
	if(map->ChargePointNUM > CHARGE_POINT_NUM_MAX)
	{
		map->ChargePointNUM = 0;
		rtn= -2;
	}
	
	if(EEPRomRead(&Agv.device.eepRom,ChargePointStartAddress,(Uint16*)(map->chargePointInfo),map->ChargePointNUM*sizeof(struct ChargePoint_t)))
	{
		LogPrintf("=ERROR= GetChargePointInfo\n");
		//SetErrorToQue(ErrorTypeEnum type,ErrorPriorityEnum priority);
		rtn= -3;
	}
	LogPrintf("GetChargePointInfo res:%d\n",rtn);
	return rtn;
}

/*------------------------------------------------------------------------------	
*输.....入：mapParams-地图参数
*输.....出：无
*输入/输出：map-地图数据
*返.....回：无
*目.....的：保存数据到IIC
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
FH_ERROR_CODE SetMapInfo(struct Map_t* map)
{
	int i = 0;
	int rel_val = 0;
	//保存地图数据到iic
	for(i = 0;i< MAP_POINT_NUM_MAX*sizeof(struct MapPointType_t);i+=8)
	{
		rel_val = EEPRomWrite(&Agv.device.eepRom,MapStartAddress+i*2,(Uint16*)&map->mapPointType+i,8);
		if(rel_val)
		{
			LogPrintf("=ERROR= WriteMapData cnt:%d\n",i);
			SetErrorToQue((ErrorTypeEnum)APP_ERROR_MAP_XY,ErrorC);
		}
		ThreadSleep(10);
	}
	return 0;
}

/*------------------------------------------------------------------------------	
*输.....入：mapParams-地图参数
*输.....出：无
*输入/输出：map-地图数据
*返.....回：无
*目.....的：读取IIC中的地图数据到本地,并进行校验
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
FH_ERROR_CODE GetMapInfo(struct Map_t* map)
{

	long errorCode = 0;
	int tmpIndex = 0;
	
	//读取IIC中地图个数
	errorCode = EEPRomRead(&Agv.device.eepRom,MapPointNumAddress,(Uint16*)(&map->MapPointNUM),1);
//	errorCode = IICRead(&Agv.board.i2c,RomAddress,MapPointNumAddress,(Uint16*)(&testPointNum),1);
	if(0 != errorCode)
	{
		//SetErrorToQue(ERROR_MAP_ReadNumFail,ErrorC);
		map->MapPointNUM = 0;
	}
	map->MapPointNUM = map->MapPointNUM&0xFF;
	if(map->MapPointNUM>=MAP_POINT_NUM_MAX)
	{
		map->readOk = 0;
		map->MapPointNUM = 0;
		return -1;
	}
		
	//读取IIC中的地图数据
	errorCode = EEPRomRead(&Agv.device.eepRom,MapStartAddress,(Uint16*)(map->mapPointType),MAP_POINT_NUM_MAX*sizeof(struct MapPointType_t));
	if(0 != errorCode)
	{
		//SetErrorToQue(ERROR_MAP_ReadDataFail,ErrorC);
		map->readOk = 0;
	}
	else
	{
		map->readOk = 1;
	}
	//数据校验
	map->checkSumEeprom = 0;
	for(tmpIndex=0;tmpIndex < map->MapPointNUM * sizeof(struct MapPointType_t);tmpIndex++)
	{
		//
	}
	map->validOk = 1;
	LogPrintf("GetMapInfo res:%d\n",errorCode);
	return errorCode;
}
void GetMapCompensate(double x,double y,double heading,double tx,double ty,double *deltax,double *deltay)
{
//    Uint32 nxy;
    Uint16 ramxy=0,i=0,nominalx = 0,nominaly = 0;
    //HR,add,20160106
//    if (Param.AgvParam[PARAM_MapBorderXmax]>Param.AgvParam[PARAM_MapBorderYmax])
//    {
//    	nxy =(Uint32)(Param.AgvParam[PARAM_MapBorderXmax]*x + y)*6.0;
//    }
//    else
//    {
//    	nxy = (x + Param.AgvParam[PARAM_MapBorderYmax]*y)*6.0;
//    }
//    
//    if(heading == 0 && tx > x)
//        memcpy(&ramxy,mapdata + nxy + 1,1);
//    else if(heading == PI/2.0 && ty > y)
//        memcpy(&ramxy,mapdata + nxy + 3,1);
//    else if(heading == PI && tx < x)
//        memcpy(&ramxy,mapdata + nxy + 2,1);
//    else if(heading == -PI/2.0 && ty < y)
//        memcpy(&ramxy,mapdata + nxy + 4,1);
//    else 
//        ramxy = 0;
        
    nominalx =(Uint16)(x*100);
    nominaly =(Uint16)(y*100);
	struct Map_t* map= &App.map;
    for(i=0;i < map->MapPointNUM;i++)
    {
    	if (nominalx==map->mapPointType[i].baseX && nominaly == map->mapPointType[i].baseY)
    	{
    		if(heading == 0 && tx > x)
    		    ramxy =map->mapPointType[i].offsetXp;
			else if(heading == PI/2.0 && ty > y)
			    ramxy =map->mapPointType[i].offsetYp;
			else if(heading == PI && tx < x)
			    ramxy =map->mapPointType[i].offsetXn;
			else if(heading == -PI/2.0 && ty < y)
			    ramxy =map->mapPointType[i].offsetYn;
			else 
			    ramxy = 0;
    		break;
    	}
		else
		{
		    ramxy = 0;
		}
    }
    *deltax = ((double)((int16)(ramxy & 0xFF00)/256))*0.001;
    *deltay = ((double)((int16)((ramxy & 0x00FF)<<8)/256))*0.001;    
}

/*------------------------------------------------------------------------------	
*输.....入：x,y-当前位置
*输.....出：locatedChargedPoint-是否为充点电
*输入/输出：map-地图数据
*返.....回：无
*目.....的：检查当前是否为充点电
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
FH_ERROR_CODE CheckMapChargePoint(double x,double y,unsigned int *locatedChargedPoint)
{
	struct Map_t* map= &App.map;
    Uint16 nominalx = 0,nominaly = 0,chargepointFlag=0,i=0;
	nominalx = (Uint16)((FH_Float)GetCurCodeID(x,DirX)/1000+0.5);
	nominaly = (Uint16)((FH_Float)GetCurCodeID(y,DirY)/1000+0.5);
	for(i=0;i < map->ChargePointNUM;i++)
    {
    	if (nominalx==map->chargePointInfo[i].baseX && nominaly == map->chargePointInfo[i].baseY)
    	{
    		chargepointFlag = 1;
    		break;
    	}
    }
    *locatedChargedPoint = chargepointFlag;
	return 0;
}
int IsChargePoint(FH_Float x,FH_Float y)
{
	unsigned int locatedChargedPoint = 0;
	CheckMapChargePoint(x,y,&locatedChargedPoint);
	return locatedChargedPoint;
}
int IsCurPosInChargePoint()
{
	return IsChargePoint(Agv.service.chassis.location.curPosGnd.x,Agv.service.chassis.location.curPosGnd.y);
}

void ClearMapNum()
{
	App.map.MapPointNUM = 0;
	return ;
}
void ClearChargePointNum()
{
	App.map.ChargePointNUM = 0;
	return ;
}

#if MODULE_DEBUG
//IIC测试函数
void TestWriteMap(struct Map_t* map)
{
	map->MapPointNUM = 0x0A0B;
	map->mapPointType[0].baseX = 0x0102;
	map->mapPointType[0].baseY = 0x0304;
	map->mapPointType[MAP_POINT_NUM_MAX-1].baseX = 0x0102;
	map->mapPointType[MAP_POINT_NUM_MAX-1].baseY = 0x0304;
}
#endif
