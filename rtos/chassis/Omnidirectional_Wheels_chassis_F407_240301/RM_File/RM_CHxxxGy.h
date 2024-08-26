#pragma once
#include "RM_stm32fxxx_hal.h"
#include "RM_StaticTime.h"
typedef struct 
{	
	float x;
	float y;
	float z;
  //加速度
	int16_t AcSx;
	int16_t AcSy;
	int16_t AcSz;
	//角速度
	float ASx;
	float ASy;
	float ASz;
}GyAngle_t;

class RM_CHxxxGy
{
public:
  GyAngle_t gy;//陀螺仪
  RM_StaticTime dirTime;//死亡时间
  bool dir;//死亡标志
  void Parse(CAN_RxHeaderTypeDef RxHeader,uint8_t RxHeaderData[]);//解析
  bool GetDir();//断连
};

inline void RM_CHxxxGy::Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
{
  if(!(RxHeader.StdId == 0x188) & !(RxHeader.StdId == 0x288) & !(RxHeader.StdId == 0x388))return;
  if(RxHeader.StdId == 0x188)
	{
		this->gy.AcSx = ((int16_t)((RxHeaderData[1] << 8) | RxHeaderData[0]));
		this->gy.AcSy = ((int16_t)((RxHeaderData[3] << 8) | RxHeaderData[2]));
		this->gy.AcSz = ((int16_t)((RxHeaderData[5] << 8) | RxHeaderData[4]));
	}
	if(RxHeader.StdId == 0x288)
	{
		this->gy.ASx = (float)((int16_t)((RxHeaderData[1] << 8) | RxHeaderData[0])) / 100;
		this->gy.ASy = (float)((int16_t)((RxHeaderData[3] << 8) | RxHeaderData[2])) / 100;
		this->gy.ASz = (float)((int16_t)((RxHeaderData[5] << 8) | RxHeaderData[4])) / 100;
	}
	if(RxHeader.StdId == 0x388)
	{
		this->gy.y = (float)((int16_t)((RxHeaderData[1] << 8) | RxHeaderData[0])) / 100;
		this->gy.x = (float)((int16_t)((RxHeaderData[3] << 8) | RxHeaderData[2])) / 100;
		this->gy.z = (float)((int16_t)((RxHeaderData[5] << 8) | RxHeaderData[4])) / 100;
		if(this->gy.z < 0)this->gy.z += 360;
	}
  dirTime.UpLastTime();
}

inline bool RM_CHxxxGy::GetDir()
{
  this->dir = this->dirTime.ISDir(20);
  return this->dir;
}
