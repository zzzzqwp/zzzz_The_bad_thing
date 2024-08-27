#include "MyPM01.h"
#include "RM_StaticTime.h"//静态定时器
//时间
RM_StaticTime MyPM01Times = { 0 };
//数据
MyPM01_t MyPM01 = { 0 };
//解析
void MyPM01Parse(CAN_RxHeaderTypeDef RxHeader,uint8_t RxHeaderData[])
{
	if(!(RxHeader.StdId == 0x611))return;
	MyPM01.power = (RxHeaderData[0] << 8 | RxHeaderData[1]) / 100.0f;
	MyPM01.voltage = (RxHeaderData[2] << 8 | RxHeaderData[3]) / 100.0f;
	MyPM01.ampere = (RxHeaderData[4] << 8 | RxHeaderData[5]) / 100.0f;
}
//发送远程帧查看
void MyPM01SendRemote(uint32_t StdId)
{
	CAN_TxHeaderTypeDef  TxHeader;
	TxHeader.DLC = 8;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_REMOTE;
	TxHeader.StdId = StdId;
	TxHeader.TransmitGlobalTime = DISABLE;
	HAL_CAN_AddTxMessage(&hcan2,&TxHeader,0,(uint32_t*)CAN_TX_MAILBOX0);
}
//发送数据帧
void MyPM01SendData(uint32_t StdId,uint16_t Data)
{
	CAN_TxHeaderTypeDef  TxHeader;
	TxHeader.DLC = 8;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = StdId;
	TxHeader.TransmitGlobalTime = DISABLE;
	
	uint8_t SendData[8] = { 0 };
	SendData[0] = Data >> 8;
	SendData[1] = Data;
	
	HAL_CAN_AddTxMessage(&hcan2,&TxHeader,SendData,(uint32_t*)CAN_TX_MAILBOX0);
}
//初始化
void MyPM01Init()
{
	MyStaticTimsInit(&MyPM01Times,10);
	MyPM01.limpower = 2;
}
//发送函数
void MyPM01SendFun()
{
	if(!MyStaticTimsIs(&MyPM01Times))return;
	static char sendflag = 0;
	switch(sendflag++)
	{
		case 0:MyPM01SendData(0x600,2);break;
		case 1:MyPM01SendData(0x601,MyPM01.limpower);break;
		case 2:MyPM01SendRemote(0x611);break;
		case 3:MyPM01SendRemote(0x612);break;
		default:sendflag = 0;break;
	}
	MyStaticTimsUpDate(&MyPM01Times);
}
