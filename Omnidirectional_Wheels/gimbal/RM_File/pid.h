#pragma once
#include "RM_stdxxx.h"
#include "RM_Filter.h"
#include "ladrc.h"



#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
struct pid_Feedforward_t
{
	float last_in;
	float T;
	float out;
	float k1;
	float max;
	
	pid_Feedforward_t(float t= 0.001)
		:T(t)
	{}
	void calc(float in)
	{
		this->out = (in-this->last_in)*k1;
		if(this->out > this->max) this->out = this->max;
		if(this->out < -this->max) this->out = -this->max;
		this->last_in = in;
	}
};
struct kzz_pid_t
{
	float Kp,Ki,Kd;
	float max_out,max_iout,iso_K,iso_Aout,iso_Bout,iso_AB,Dbuf[3],error[3];
	bool i_isolation,i_isolationAB,Parallel_bool;
	float feedback,parallel_feedback,mb,last_mb,lastmb_out,lastmb_k,feed_k,fee_out;
	float Pout,Iout,Dout,out;
	kzz_pid_t(float ikp = 0,float iki = 0,float ikd = 0,
		float iso_K = 0.0,
		float imax_out = 25000,float imax_iout = 15000,
		float ipout = 0,float iiout = 0,float idout = 0,float iout = 0)
		:Kp(ikp),Ki(iki),Kd(ikd),
		max_out(imax_out),max_iout(imax_iout),Pout(ipout),Iout(iiout),Dout(idout),out(iout)
	{}
	void pid_mb_feedforward(float zz)
	{
		lastmb_k = zz;
	}
	void pid_feedback_feedforward(float zz)
	{
		feed_k = zz;
	}
	void pid_Parallel(bool zz)
	{
		this->Parallel_bool = zz;
	}
	
	void pid_Parallel_feedback(float zz)
	{
		this->parallel_feedback = zz;
	}
		
	//λ��ʽpid
	void pid_iso_no(float iso_out)
	{
		this->i_isolation = true;
		this->i_isolationAB = false;
		this->iso_K = iso_out;
	}
	void pid_ABiso(float iso_Bout,float iso_Aout)
	{
		this->i_isolationAB = true;
		this->i_isolation = false;
		this->iso_Aout = iso_Aout;
		this->iso_Bout = iso_Bout;
	}
	void pid_calc(float ref, float set)
	{
		this->error[2] = this->error[1];
    this->error[1] = this->error[0];
		
		this->mb = set;
		this->feedback = ref;
		
		this->error[0] = this->mb - this->feedback;
		this->lastmb_out = (this->mb-this->last_mb)*this->lastmb_k;
		this->fee_out = this->feedback*this->feed_k;
		this->Pout = this->Kp * this->error[0];
		if(this->i_isolation == true)
		{
			if(fabs(this->error[0]) < this->iso_K)
			{
				this->Iout += this->Ki * this->error[0]*0.001;
			}
			else if(fabs(this->error[0]) > this->iso_K)
			{
				this->Iout = 0;
			}
		}
		else if(this->i_isolationAB == true)
		{
			if(fabs(this->error[0]) < this->iso_Bout)
			{
				iso_AB = this->error[0]*0.001;
			}
			else if(fabs(this->error[0]) <= this->iso_Aout + this->iso_Bout && this->iso_Bout < fabs(this->error[0]))
			{
				iso_AB = ((iso_Aout-fabs(this->error[0])+this->iso_Bout)/this->iso_Aout)*this->error[0]*0.001;
			}
			else if(fabs(this->error[0]) > this->iso_Aout + this->iso_Bout)
			{
				iso_AB = 0.0;
			}
			this->Iout += this->Ki * iso_AB;
			if(fabs(this->error[0]) > this->iso_Aout + this->iso_Bout)
			{
				this->Iout = 0.0;
			}
			
		}
		else
		{
			this->Iout += this->Ki * this->error[0]*0.001;
		}
		if(this->Parallel_bool == false)
		{
			this->Dbuf[2] = this->Dbuf[1];
			this->Dbuf[1] = this->Dbuf[0];
		
			this->Dbuf[0] = (this->error[1] - this->error[0]);
		
			this->Dout = this->Kd * this->Dbuf[0];
		}
		else if(this->Parallel_bool == true)
		{
			this->Dout = this->Kd *this->parallel_feedback;
			
		}
		
		LimitMax(this->Iout, this->max_iout);
		
		this->out = this->Pout + this->Iout + this->Dout+this->lastmb_out+this->fee_out;
		this->last_mb = this->mb;
		
		LimitMax(this->out, this->max_out);
	}



	//����ʽpid
	void pid_calc01(float ref, float set)
	{
		this->error[2] = this->error[1];
    this->error[1] = this->error[0];
		
		this->mb = set;
		this->feedback = ref;
		
		this->error[0] = set - ref;
		
		this->Pout = this->Kp * (this->error[0] - this->error[1]);
		
		this->Iout = this->Ki * this->error[0];
		this->Dbuf[2] = this->Dbuf[1];
    this->Dbuf[1] = this->Dbuf[0];
		
		this->Dbuf[0] = (this->error[0] - 2.0f * this->error[1] + this->error[2]);
		
		this->Dout = this->Kd * this->Dbuf[0];
		
		this->out += this->Pout + this->Iout + this->Dout;
    LimitMax(this->out, this->max_out);
	}
};




