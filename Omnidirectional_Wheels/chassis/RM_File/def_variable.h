#pragma once

#include "RM_StaticTime.h"//静态定时器
#include "RM_Clicker.h"//遥控器
#include "ch_gyro_232.h"//陀螺仪
#include "RM_PID.h"
#include "ladrc.h"
#include "RM_Motor.h"//RM电机
#include "RM_Can.h"
#include "RM_stm32fxxx_hal.h"
#include "stm32f4xx_hal_can.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include "RM_CHxxxGy.h"
#include "RM_Servos.h"
#include <stdlib.h>
#include "RM_Pid.h"
#include "Pid.h"
#include "RM_UDE.h"


//调试串口 0关 1开
#define Debug_OFF 0
#define Debug_ON 1
#define Debug_UART_SW Debug_ON


/*
														操作控制策划														

x代表无效
y代表有效
数字代表值

左手                                     右手
ch3:前后			ch2:左右			s1开关132			ch1:前后			ch0:左右			s2开关132
y             x            1             x             y            1             底盘跟随云台（1）
x             x            1             y             y            3             云台正方向（2）
x							x						 1						 x						 x					  2							小陀螺（3）

y             x            3             y             y            1             遥控器,底盘跟随云台（1）
y             x            3             y             y            3             遥控器,云台正方向（2）
x             x            3             x             x            2             遥控器,小陀螺（3）

x							x						 2						 x						 x						1							待定（7）
x							x						 2						 x						 x						3							待定（8）
x							x						 2						 x						 x						2							急停（9）
*/

/*遥控器信号源切换*/
/*
	CONTROL_SIG 0 遥控器
	CONTROL_SIG 1 上下板
*/
#define GY_CAN 1//设置陀螺仪的can
#define GY_232 2//设置陀螺仪的232
#define GY_GET_SIG GY_CAN//设置can信号的陀螺仪

#define PITCH_ENCODER 1//pitch选择用编码器控制
#define PITCH_GY 2//pitch选择用陀螺仪控制
#define PITCH__GET_SIG PITCH_ENCODER//设置can信号的陀螺仪

#define CONTROL_SIG 0
#if CONTROL_SIG == 0	
	#define Clicker_chassis_follow_gimbal (RM_Clicker::RC_Ctl.rc.s1 == 1 && RM_Clicker::RC_Ctl.rc.s2 == 1)//（1）
	#define Clicker_gimbal_host_chassis (RM_Clicker::RC_Ctl.rc.s1 == 1 && RM_Clicker::RC_Ctl.rc.s2 == 3)//（2）
	#define Clicker_chassis_gyro (RM_Clicker::RC_Ctl.rc.s1 == 1 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（3）
	#define MOUSE_chassis_follow_gimbal (RM_Clicker::RC_Ctl.rc.s1 == 3 && RM_Clicker::RC_Ctl.rc.s2 == 1)//（1）
	#define MOUSE_gimbal_host_chassis (RM_Clicker::RC_Ctl.rc.s1 == 3 && RM_Clicker::RC_Ctl.rc.s2 == 3)//（2）
	#define MOUSE_chassis_gyro (RM_Clicker::RC_Ctl.rc.s1 == 3 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（3）
	#define stop_mode (RM_Clicker::RC_Ctl.rc.s1 == 2)//（9）
#elif CONTROL_SIG == 1
	#define stop_mode (Gimbal_to_Chassis_Data.stop)//（9）
#endif

#define Clicker_MODE (RM_Clicker::RC_Ctl.rc.s1 == 1)//遥控器模式
#define MOUSE_MODE (RM_Clicker::RC_Ctl.rc.s1 == 3)//遥控器模式
#define STPO_MODE (RM_Clicker::RC_Ctl.rc.s1 == 2)//遥控器模式
#define CM_MODE ((STPO_MODE) & (RM_Clicker::RC_Ctl.rc.s2 == 1))//遥控器开关仓门模式
#define Clicker_S2_Dial_up ((MOUSE_MODE) & (RM_Clicker::RC_Ctl.rc.s2 == 1))//遥控器模式，上拨4次开启摩擦轮，关闭同理
#define Clicker_S2_Dial_down ((MOUSE_MODE) & (RM_Clicker::RC_Ctl.rc.s2 == 2))//遥控器模式，下拨4次开启拨盘，关闭同理


#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
		#define piath_init_encoder_angle 1260 //编码器绝对值初始化
		#define piath_init_gy_angle -0.8 * 22.7527777//pitch初始角度转编码器的值
		#define PITCH_On_limit    (435)//(5800.0 - piath_init_encoder_angle) //上极限 测量编码器-初始化角度piath_init_encoder_angle
		#define PITCH_Lower_limit (-435)//(4800.0 - piath_init_encoder_angle) //下极限 测量编码器-初始化角度piath_init_encoder_angle
#elif PITCH__GET_SIG == PITCH_GY
		#define piath_init_encoder_angle 0- //陀螺仪值初始化
		#define PITCH_On_limit    (-30.0 - piath_init_encoder_angle) //上极限 测量编码器-初始化角度piath_init_encoder_angle
		#define PITCH_Lower_limit (22.0 - piath_init_encoder_angle) //下极限 测量编码器-初始化角度piath_init_encoder_angle
#endif

#define MOUSE_X_K -0.0005//鼠标x移动系数
#define MOUSE_Y_K -0.013//鼠标y移动系数

/***************************变量声明*********************************/

//串口打印
#define Send_Usart_Data_Huart huart8
//云台到底盘
#define Send_Gimbal_to_Chassis_Huart huart7
#define Send_Gimbal_to_Chassis_Huart_LEN 12
//视觉串口
#define vision_Huart huart3

//通信挂掉标记
bool dir = false;

//陀螺仪挂掉标记
bool GY_dir = false;

/***************************底盘*********************************/

RM_StaticTime Total_tasks_staticTime;//控制时间x	

//陀螺仪获取端
#if GY_GET_SIG == GY_CAN
	RM_CHxxxGy chGy_chassis;//底盘陀螺仪
#elif GY_GET_SIG == GY_232
	ch_gyro_232_t chGy_chassis = { 0 };//底盘陀螺仪
#endif

/***************************云台到底盘数据*********************************/
#define YAW_MOTOR_ID 0x205
#define PITCH_MOTOR_ID 0x206

//摩擦轮
#define Friction_MOTOR_L_ID 0x201
#define Friction_MOTOR_R_ID 0x202
#define Friction_MAX_Speed 7200

//拨盘
#define Dial_MOTOR_R_ID 0x203
#define Dial_MAX_Speed 2500


float YAW_Init_Angle;//初始化角度
float yaw_target_add_angle;//增加的角度
float yaw_target_angle;//期望角度
float yaw_target_angle_temp;//期望角度temp
int yaw_encoder_angle_e = 0;//当前编码器与不同位置的误差
int yaw_encod = 0;//当前编码器与初始位置的误差
float pitch_target_angle = piath_init_encoder_angle;//pitch目标角度
float vision_pitch = 0;//视觉pitch目标角度
float pitch_mb = 0;
float Friction_Speed;//摩擦轮期望速度
float Dial_Speed;//拨盘期望速度
uint8_t send_motor_size;//用于3508与6020错开发送

#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
		LADRC_quadratic ladrc_pitch(TD_quadratic(50),10,0,50,30000);//ptich
		float pitch_offsets_gy = 0;//补偿角度
		bool is_open_pitch_offsets_gy;//是否开启补偿
		TD_quadratic td_gy_y(400);//y轴td滤波		
#elif PITCH__GET_SIG == PITCH_GY
		LADRC_quadratic ladrc_pitch;//ptich
		RM_PID pitch_pid;
		Kpid_t pitch_kpid(-1200,-4000,-500);
#endif

LADRC_quadratic ladrc_friction_l(TD_quadratic(10),500,0,120,16384,0.001);//左摩擦轮

LADRC_quadratic ladrc_friction_r(TD_quadratic(10),500,0,120,16384,0.001);//右摩擦轮

LADRC_quadratic ladrc_dial(TD_quadratic(10),1200,0,60,16384,0.001);//拨盘


RM_PID YAW_DEMO_PID;
Kpid_t PID_Init;




float aabb[1000];
float aacc[1000];
RM_PID yaw_pid;
kzz_pid_t yaw_zz_pid_s(1,1,2,40);//yaw
kzz_pid_t yaw_zz_pid_v(2000,0,0);//yaw
RM_UDE_t RM_yaw_UDE(3,13,3000);//yaw
pid_Feedforward_t yaw_VW_Feedforward(0.001,-0.2);//yaw
pid_Feedforward_t yaw_Feedforward(0.001,4000);//yaw

kzz_pid_t pitch_zz_pid_s(0.16,0.7,0);//pitch
kzz_pid_t pitch_zz_pid_v(390,0,0);//pitch
TD_quadratic td_pitch_cin(500);//pitch


kzz_pid_t bp_zz_pid_s(0.1,0,0);//拨盘
kzz_pid_t bp_zz_pid_v(3.5,0,0);//拨盘
TD_quadratic td_bp_cin(150);//拨盘


TD_quadratic td_yaw_cin(200);//对输入滤波
TD_quadratic td_yaw_set(17);//对目标滤波

TD_quadratic td_yaw_ii(17);

TD_quadratic td_yaw_current(70);//对反馈电流滤波


float zz_send_data = 0;


#if GY_GET_SIG == GY_CAN
		Kpid_t yaw_kpid(2000,3000,2000);
		TD_quadratic td_yaw_i(800);//对积分求导
		float yaw_i_kd = -0.05;//积分的导数的系数kd
		LADRC_quadratic ladrc_yaw_angle(TD_quadratic(20),0.3,0,0,30000,0.001);
		float disturbance_test = 0,t,hz = 0,rx = 7;//扰动测试
		float ff_add_angle_kp = 20000;//给予前馈补偿
		float ff_Clicker_add_angle_kp = 20000;//给予前馈补偿
		float ff_Mouse_add_angle_kp = 1000;//给予前馈补偿
		RM_FeedForward ff_add_angle;//前馈
		RM_FeedForward ff_Clicker_add_angle;//前馈
		RM_FeedForward ff_Mouse_add_angle;//前馈
#elif GY_GET_SIG == GY_232
		Kpid_t yaw_kpid(2500,10000,217);
		TD_quadratic td_yaw_i(800);//对积分求导
		float yaw_i_kd = -0.07;//积分的导数的系数kd
#endif

float yaw_send_data = 0;//融合所有数据之后要发送的数据

RM_Key Q_Key;//q键触发
uint8_t Q_Key_size;//q键触发次数,单数小陀螺,双数不小陀螺

RM_Key V_Key;
uint8_t V_Key_size;//v

RM_Key B_Key;//B键触发，开摩擦轮

RM_Key G_Key;//G键触发，关摩擦轮

RM_Key Shift_Key;//Shift键触发
uint8_t Shift_Key_size;//Shift键触发次数,跟随并且加速

RM_Key R_Key;//R键触发-底盘转速
int8_t R_Key_size;//ER计算器

RM_Key E_Key;//E键触发超电
int8_t E_Key_size;//E计算器

RM_Key F_Key;//F键触发超电
int8_t F_Key_size;//F计算器

RM_Key Z_Key;//Z键触发+底盘移速
RM_Key X_Key;//X键触发-底盘移速
int8_t ZX_Key_size ;//ZX计算器


RM_Key Clicker_Friction_Key;//摩擦轮开启
uint8_t Clicker_Friction_Key_size;//摩擦轮开始条件，右手上拨4次启动
RM_Key Clicker_Dial_Key;//拨盘开启
uint8_t Clicker_Dial_Key_size;//拨盘开始条件，右手下拨4次启动

RM_Key C_Key;//C键触发开关仓门舵机
uint8_t C_Key_size;//C键触发开关仓门舵机键触发次数,单数，双数不

RM_Servos servos;//仓门舵机
float servos_angle = 180;//舵机角度



//拟合函数

float x = 0;
float x_out = 0;


float a0  = 17.3335    ;
float a1  = -112.3397  ;
float b1  = 6.0245e+03 ;
float a2  = -119.4970  ;
float b2  = -1.7451e+03;
float a3  = 107.5574   ;
float b3  = 1.3477e+03 ;
float a4  = 283.2110   ;
float b4  = -552.8037  ;
float a5  = 200.3956   ;
float b5  = 402.2970   ;
float a6  = 79.5243    ;
float b6  = 326.5555   ;
float w   = 0.0467     ;





