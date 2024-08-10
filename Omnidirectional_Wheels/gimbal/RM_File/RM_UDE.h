#pragma once
#include "RM_stdxxx.h"


//#define ke_6020 0.716f //6020���綯�Ƴ���ke


struct RM_UDE_t
{
	float T,B;//UDE�ĵ���
	float uo,uo_max;//u�Ļ��ֺ��޷�
	
	float out,out_max;//UDE��������޷�
	
	float jjj;
	
	float feedback_torque;//��������
	float out_torque;//�������
	
	
	float UDE_fe_torque(int16_t u)//��������
	{
		return (u/5461.333)*0.741;
	}
	
	float UDE_torque(int16_t u)//�������
	{
		return ((u/1000)/0.9)*0.741;
	}
	
	RM_UDE_t(float t = 0,float b = 0,float uo_max = 300,float out = 0)
	:T(t),B(b),uo_max(uo_max),out(out)
	{}
	
	
	void UDE_u0(float u)
	{
//		this->out_torque = UDE_torque(u);
//		this->uo+=this->out_torque*this->jjj;

		this->uo+=u*this->B;
		if(this->uo > this->uo_max)
		{
			this->uo = this->uo_max;
		}
		if(this->uo < -this->uo_max)
		{
			this->uo = -this->uo_max;
		}
		
	}
	void UDE_calc(float xn)
	{
//		this->feedback_torque = UDE_fe_torque(xn);
		
		if(this->out_torque > this->uo_max)
		{
			this->out_torque = this->uo_max;
		}
		if(this->out_torque < -this->uo_max)
		{
			this->out_torque = -this->uo_max;
		}
		
		this->feedback_torque = xn;
		this->out = ((this->feedback_torque-this->out_torque)*this->T)/this->B;
	}
};

