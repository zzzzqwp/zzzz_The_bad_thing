#pragma once

#include "usart.h"

#define ch_gyro_232_MpDL 82//最大数组长度
#define ch_gyro_232_uart_chassis huart8//串口
#define ch_gyro_232_MDL 76//数据域长度

typedef struct 
{
	uint8_t find_pData;//寻找一字节
	uint8_t find_last_pData;//保存上一次的寻找结果
	uint8_t pData[ch_gyro_232_MpDL];//全部数据
	uint16_t crc;//crc校验
	struct
	{
		uint8_t frame_header;//帧头
		uint8_t frame_type;//帧类型
		uint16_t data_len;//数据域长度
		uint16_t crc;//CRC校验
	}frame_format;//帧格式
	struct
	{
		uint8_t label;//标签
		float x,y,z;//角度
		float AcSx,AcSy,AcSz;//加速度
		float ASx,ASy,ASz;//角速度
		float z_360;//360度z
		float last_z_360;//上一次角度
		float Add_z;//累计角度
		bool InitFlag;//初始化标记
		float InitData;//初始化角度
	}data;
}ch_gyro_232_t;

ch_gyro_232_t ch_gyro_232_data_global;//陀螺仪数据

/*
  currectCrc:previous crc value,set 0 if it's first section
  src:source stream data
  engthInBytes:length
*/
static void voidcrc16_update(uint16_t* currectCrc,const uint8_t* src,uint32_t lengthInBytes);//crc校验

void ch_gyro_232_parse(UART_HandleTypeDef* huart,UART_HandleTypeDef* bind_huart,ch_gyro_232_t* ch_gyro_232_data);//数据解析

void ch_gyro_232_Init(UART_HandleTypeDef* huart,ch_gyro_232_t* ch_gyro_232_data);//初始化

void ch_gyro_232_ore(UART_HandleTypeDef* huart,ch_gyro_232_t* ch_gyro_232_data);//解决ore问题

void ch_gyro_232_ore(UART_HandleTypeDef* huart,ch_gyro_232_t* ch_gyro_232_data)//解决ore问题
{
	if( __HAL_UART_GET_FLAG( huart, UART_FLAG_ORE ) != RESET )
	{
		__HAL_UART_CLEAR_OREFLAG( huart );
		ch_gyro_232_Init(huart,ch_gyro_232_data);//地盘陀螺仪初始化
	}
}

void ch_gyro_232_parse(UART_HandleTypeDef* huart,UART_HandleTypeDef* bind_huart,ch_gyro_232_t* ch_gyro_232_data)
{
	if(huart == bind_huart)
	{
		if(ch_gyro_232_data->find_last_pData == 0x5A && ch_gyro_232_data->find_pData == 0xA5)
		{
			HAL_UART_Receive_DMA(huart,ch_gyro_232_data->pData,ch_gyro_232_MpDL);//开启dma接收
		}
		else
		{
			ch_gyro_232_data->find_last_pData = ch_gyro_232_data->find_pData;
			HAL_UART_Receive_IT(huart,&ch_gyro_232_data->find_pData,1);//先使用中断接收到帧头
		}

		
		*((uint16_t*)&ch_gyro_232_data->frame_format.frame_header) = *((uint16_t*)&ch_gyro_232_data->pData[ch_gyro_232_MpDL - 2]);//末端两位
		memcpy((((uint8_t*)&ch_gyro_232_data->frame_format) + 2),ch_gyro_232_data->pData,sizeof(ch_gyro_232_data->frame_format) - 2);//解算数据帧格式
//		voidcrc16_update(&ch_gyro_232_data->crc,ch_gyro_232_data->pData,4);//crc校验前四位
//		voidcrc16_update(&ch_gyro_232_data->crc,ch_gyro_232_data->pData + 6,ch_gyro_232_data->frame_format.data_len);//crc校验剩余位
//		if(ch_gyro_232_data->crc == ch_gyro_232_data->frame_format.crc)
//		{
      uint8_t star_idx = ch_gyro_232_MpDL - ch_gyro_232_MDL - 2;//数据域起始地址，帧头偏移了俩位
			ch_gyro_232_data->data.label = ch_gyro_232_data->pData[star_idx];//标签
			//加速度
			memcpy(&ch_gyro_232_data->data.AcSx,&ch_gyro_232_data->pData[star_idx + 12],4); 
			memcpy(&ch_gyro_232_data->data.AcSy,&ch_gyro_232_data->pData[star_idx + 16],4);
			memcpy(&ch_gyro_232_data->data.AcSz,&ch_gyro_232_data->pData[star_idx + 20],4);
			//角速度
			memcpy(&ch_gyro_232_data->data.ASx,&ch_gyro_232_data->pData[star_idx + 24],4); 
			memcpy(&ch_gyro_232_data->data.ASy,&ch_gyro_232_data->pData[star_idx + 28],4);
			memcpy(&ch_gyro_232_data->data.ASz,&ch_gyro_232_data->pData[star_idx + 32],4);
			//顺序：俯仰角pitch (x),航向角yaw (y),横滚角roll (z)                      
      memcpy(&ch_gyro_232_data->data.x,&ch_gyro_232_data->pData[star_idx + 48],4); 
			memcpy(&ch_gyro_232_data->data.y,&ch_gyro_232_data->pData[star_idx + 52],4);
			memcpy(&ch_gyro_232_data->data.z,&ch_gyro_232_data->pData[star_idx + 56],4);
			ch_gyro_232_data->data.z_360 = ch_gyro_232_data->data.z;
			if(ch_gyro_232_data->data.z < 0)ch_gyro_232_data->data.z_360 += 360;
			//数据累加
			if(ch_gyro_232_data->data.last_z_360 != ch_gyro_232_data->data.z_360 && 
				 ch_gyro_232_data->data.last_z_360 != -1)
			{
					float lastData = ch_gyro_232_data->data.last_z_360;
					float Data = ch_gyro_232_data->data.z_360;
					if(Data - lastData < -(360*0.5))//正转
						ch_gyro_232_data->data.Add_z += (360 - lastData + Data);
					else if(Data - lastData > (360*0.5))//反转
						ch_gyro_232_data->data.Add_z += -(360 - Data + lastData);
					else 
						ch_gyro_232_data->data.Add_z += (Data - lastData);
			}
			//数据上一次更新
			//数据解析
			ch_gyro_232_data->data.last_z_360 = ch_gyro_232_data->data.z_360;
			//初始化数据
			if(ch_gyro_232_data->data.InitFlag == 0)
			{
				ch_gyro_232_data->data.InitData = ch_gyro_232_data->data.z_360;
				ch_gyro_232_data->data.InitFlag = 1;
			}
//		}
	}
}

void ch_gyro_232_Init(UART_HandleTypeDef* huart,ch_gyro_232_t* ch_gyro_232_data)
{
	HAL_UART_Receive_IT(huart,&ch_gyro_232_data->find_pData,1);//先使用中断接收到帧头
	ch_gyro_232_data->data.Add_z = -1;
	ch_gyro_232_data->data.InitFlag = 0;
}

/*
  currectCrc:previous crc value,set 0 if it's first section
  src:source stream data
  engthInBytes:length
*/
static void voidcrc16_update(uint16_t* currectCrc,const uint8_t* src,uint32_t lengthInBytes)
{
  uint32_t crc =* currectCrc;
  uint32_t j;
  for(j = 0;j < lengthInBytes;++j)
  {
    uint32_t i;
    uint32_t byte = src[j];
    crc ^= byte << 8;
    for(i = 0;i < 8;++i)
    {
      uint32_t temp = crc << 1;
      if(crc & 0x8000)
      {
        temp ^= 0x1021;
      }
      crc = temp;
    }
  }
  *currectCrc = crc;
}
