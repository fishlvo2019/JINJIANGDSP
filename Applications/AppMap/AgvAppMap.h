/******************************************************************************
* 版权信息：   	QuickTron
* 文 件 名：     	Map.c
* 作    者：		xingll
* 功    能：		地图数据
* 修订记录:    版本		 		修订内容			修订者				修改日期
v1.0		创建				新增				xingll			20171019
*******************************************************************************/
#ifndef _FHMAP_H_
#define _FHMAP_H_


#define FHMAP_VALID 1
#define IIC_NEW 1

#define MAP_POINT_NUM_MAX 120
#define CHARGE_POINT_NUM_MAX 500


struct MapParams_t
{
	int reserved1;
};

struct MapInfoType_t
{
	Uint16 X;
    Uint16 Y;
    Uint16 EXRamXY;
    int16 Xxp;
    int16 Yxp;
    Uint16 EXRamXYxp;
    int16 Xxn;
    int16 Yxn;
    Uint16 EXRamXYxn;
    int16 Xyp;
    int16 Yyp;
    Uint16 EXRamXYyp;
    int16 Xyn;
    int16 Yyn;
    Uint16 EXRamXYyn;
    Uint32 nxy;
    Uint16 chargePoint;//充电点标识(0:not chargePoint,otherwise),HR,add,20160106
};

struct MapPointType_t
{
	Uint16 baseX;//地图点X坐标:mm
	Uint16 baseY;//地图点X坐标:mm
	Uint16 offsetXp;//X轴正方向的偏差数据:mm(高8位x坐标偏差数据,低8位y坐标偏差数据)
	Uint16 offsetXn;//X轴负方向的偏差数据:mm(高8位x坐标偏差数据,低8位y坐标偏差数据)
	Uint16 offsetYp;//Y轴正方向的偏差数据:mm(高8位x坐标偏差数据,低8位y坐标偏差数据)
	Uint16 offsetYn;//Y轴负方向的偏差数据:mm(高8位x坐标偏差数据,低8位y坐标偏差数据)
	Uint16 chargePoint;//充电点标识:1充电点
};
struct ChargePoint_t
{
	Uint16 baseX;//地图点X坐标:M
	Uint16 baseY;//地图点X坐标:M
};
//地图本地数据
struct Map_t 
{
    unsigned char MapPointNUM;										//保存在IIC
	struct MapPointType_t* mapPointType;	//保存在IIC
	Uint16 ChargePointNUM;
	struct ChargePoint_t* chargePointInfo;
	Uint16 saveOk;
    Uint16 readOk;
    Uint16 validOk;
	Uint32 checkSumEeprom;
	Uint32 checkSumUart;
};

extern double CODE_GAP_X;
extern double CODE_GAP_Y;
extern unsigned int CameraReadCodeIDActived;
/*------------------------------------------------------------------------------	
*输.....入：mapParams-地图参数
*输.....出：无
*输入/输出：map-地图数据
*返.....回：无
*目.....的：初始化地图数据
*调用 函数：
*前置 条件: 无
*后置 条件: 无
------------------------------------------------------------------------------*/
FH_ERROR_CODE AppMapInfoInit(struct Map_t* map,struct MapParams_t* mapParams);

int SetMapInfo4Single(struct Map_t* map,int MapIndex);

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
FH_ERROR_CODE SetMapInfo(struct Map_t* map);

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
FH_ERROR_CODE GetMapInfo(struct Map_t* map);
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
FH_ERROR_CODE CheckMapChargePoint(double x,double y,unsigned int *locatedChargedPoint);
FH_ERROR_CODE GetChargePointInfo(struct Map_t* map);
FH_ERROR_CODE SetChargePointNUM(struct Map_t* map);

FH_ERROR_CODE SetMapPointNUM(struct Map_t* map);
void ClearMapNum();
void ClearChargePointNum();
FH_ERROR_CODE SetChargePointInfo4Single(struct Map_t* map,int x,int y);
int IsChargePoint(FH_Float x,FH_Float y);
int IsCurPosInChargePoint();

#if MODULE_DEBUG
void TestWriteMap(struct Map_t* map);
#endif

#endif
