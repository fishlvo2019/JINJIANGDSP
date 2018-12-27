/******************************************************************************
* ��Ȩ��Ϣ��   	QuickTron
* �� �� ����     	Map.c
* ��    �ߣ�		xingll
* ��    �ܣ�		��ͼ����
* �޶���¼:    �汾		 		�޶�����			�޶���				�޸�����
v1.0		����				����				xingll			20171019
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
    Uint16 chargePoint;//�����ʶ(0:not chargePoint,otherwise),HR,add,20160106
};

struct MapPointType_t
{
	Uint16 baseX;//��ͼ��X����:mm
	Uint16 baseY;//��ͼ��X����:mm
	Uint16 offsetXp;//X���������ƫ������:mm(��8λx����ƫ������,��8λy����ƫ������)
	Uint16 offsetXn;//X�Ḻ�����ƫ������:mm(��8λx����ƫ������,��8λy����ƫ������)
	Uint16 offsetYp;//Y���������ƫ������:mm(��8λx����ƫ������,��8λy����ƫ������)
	Uint16 offsetYn;//Y�Ḻ�����ƫ������:mm(��8λx����ƫ������,��8λy����ƫ������)
	Uint16 chargePoint;//�����ʶ:1����
};
struct ChargePoint_t
{
	Uint16 baseX;//��ͼ��X����:M
	Uint16 baseY;//��ͼ��X����:M
};
//��ͼ��������
struct Map_t 
{
    unsigned char MapPointNUM;										//������IIC
	struct MapPointType_t* mapPointType;	//������IIC
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
*��.....�룺mapParams-��ͼ����
*��.....������
*����/�����map-��ͼ����
*��.....�أ���
*Ŀ.....�ģ���ʼ����ͼ����
*���� ������
*ǰ�� ����: ��
*���� ����: ��
------------------------------------------------------------------------------*/
FH_ERROR_CODE AppMapInfoInit(struct Map_t* map,struct MapParams_t* mapParams);

int SetMapInfo4Single(struct Map_t* map,int MapIndex);

/*------------------------------------------------------------------------------	
*��.....�룺mapParams-��ͼ����
*��.....������
*����/�����map-��ͼ����
*��.....�أ���
*Ŀ.....�ģ��������ݵ�IIC
*���� ������
*ǰ�� ����: ��
*���� ����: ��
------------------------------------------------------------------------------*/
FH_ERROR_CODE SetMapInfo(struct Map_t* map);

/*------------------------------------------------------------------------------	
*��.....�룺mapParams-��ͼ����
*��.....������
*����/�����map-��ͼ����
*��.....�أ���
*Ŀ.....�ģ���ȡIIC�еĵ�ͼ���ݵ�����,������У��
*���� ������
*ǰ�� ����: ��
*���� ����: ��
------------------------------------------------------------------------------*/
FH_ERROR_CODE GetMapInfo(struct Map_t* map);
/*------------------------------------------------------------------------------	
*��.....�룺x,y-��ǰλ��
*��.....����locatedChargedPoint-�Ƿ�Ϊ����
*����/�����map-��ͼ����
*��.....�أ���
*Ŀ.....�ģ���鵱ǰ�Ƿ�Ϊ����
*���� ������
*ǰ�� ����: ��
*���� ����: ��
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
