#ifndef  __MyPM01__H__
#define  __MyPM01__H__
#include "RM_stm32fxxx_hal.h"
typedef struct
{
	float voltage;
	float ampere;
	float power;
	float limpower;
}MyPM01_t;
//解析
void MyPM01Parse(CAN_RxHeaderTypeDef RxHeader,uint8_t RxHeaderData[]);
//发送远程帧查看
void MyPM01SendRemote(uint32_t StdId);
//发送数据帧
void MyPM01SendData(uint32_t StdId,uint16_t Data);
//发送函数
void MyPM01SendFun();
//初始化
void MyPM01Init();
#endif
