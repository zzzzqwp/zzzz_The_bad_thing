#pragma once

#include "def_variable.h"

struct SEND_GRAPHIC_QUEUE;//发送数据队列
extern SEND_GRAPHIC_QUEUE send_graphic_queue;

/***************************函数声明*********************************/

//主跑初始化
void Total_tasks_Init();

//主跑函数
void Total_tasks_Run();

//发送3508数据
void Send_3508_CAN();

//急停模式控制所有pid
void control_pid_0_speed();

//清空数据数据
void Clear_ALL_Data();

//获取云台到底盘数据初始化
void Get_Gimbal_to_Chassis_Init();

//获取云台到底盘数据
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart);

//发送3508数据
void Send_3508_CAN()
{
	//发送
	RM_FDorCAN_Send(&hcan1,SEND_MOTOR_ID_3508,msd_3508_2006.Data,CAN_TX_MAILBOX1);
}

//急停模式控制所有pid
void control_pid_0_speed()
{

}

//清空数据数据
void Clear_ALL_Data()
{
	
}

//主跑初始化
void Total_tasks_Init()
{
	RM_FDorCAN_Init();//can配置初始化
	
	rmClicker.Init();//遥控器串口配置初始化
	
	Get_Gimbal_to_Chassis_Init();//串口初始化
	
	RM_RefereeSystem::RM_RefereeSystemInit();//串口初始化
		
	darw_graphic_static_ui_init();
	
	HAL_Delay(10);
	
	//记录上一次时间
	uint64_t time_adrc = HAL_GetTick();
	//adrc收敛期
	while(1)
	{

		if(HAL_GetTick() - time_adrc > 500)
		{
			break;
		}
	}
}
RM_PID lsd;
float now_vxy[4],num_vx,num_vy,now_angle,e_vx,e_vy,k_e_vxy = 0.001,expectations_vx,expectations_vy,vxy_zero_s;
//主跑函数
void Total_tasks_Run()
{
	int dir = rmClicker.ISDir();//遥控器断连处理

	if(Total_tasks_staticTime.ISOne(2))//控制地盘can发送频率
	{
		if(!dir && RM_Clicker::RC_Ctl.rc.s2 != 2)
		{					
			/*
			//车体速度解算（未完成）差个w自转速度
			now_vxy[0] =  wheel_td_left_1 .x1;
			now_vxy[1] =  wheel_td_left_2 .x1;
			now_vxy[2] =  wheel_td_right_1.x1;
			now_vxy[3] =  wheel_td_right_2.x1;
			//求不为零的轮数
			vxy_zero_s = 0;
			for(int i = 0;i < 4;i++){if(fabs(now_vxy[i]) > 50)vxy_zero_s++;}
			if(vxy_zero_s == 0)vxy_zero_s = 1;
			//计算车体速度
			num_vx = -(now_vxy[0] - now_vxy[1] + now_vxy[2] - now_vxy[3]) / vxy_zero_s;
			num_vy =  (now_vxy[0] + now_vxy[1] + now_vxy[2] + now_vxy[3]) / vxy_zero_s;
			//计算车体差速积分
			e_vx += (RC_LX * 24.8242 * 0.1 - num_vx) * k_e_vxy;
			e_vy += (RC_LY * 24.8242 * 0.1 - num_vy) * k_e_vxy;
			//限幅
			e_vx = fmod(e_vx,1000);
			e_vy = fmod(e_vx,1000);			
			//遥控器线速度
			expectations_vx = RC_LX * 24.8242 * 0.1; 
			expectations_vy = RC_LY * 24.8242 * 0.1; 
			wheel_e.UpData(e_vx,e_vy,0,660.0 * 24.8242 * 0.1);

			Send_Usart_Data("%f,%f\r\n",(float)(e_vx),(float)(e_vy));
			*/
			wheel.UpData(RC_LX * 24.8242 * 0.1,RC_LY * 24.8242 * 0.1,RC_RX * 8,660.0 * 24.8242 * 0.1);		
			
//			Send_Usart_Data("%f,%f,%f\r\n",(float)(wheel_ladrc_left_1.feedback),(float)(wheel_ladrc_left_1.z1),(float)(wheel_ladrc_left_1.td.x1));
		}		
		else
		{
			e_vx = e_vy = 0;
			memset(&wheel,0,sizeof(wheel));
		}
		
		wheel_ladrc_left_1.up_data ( (wheel.speed[3] + wheel_e.speed[3]),Motor3508.GetMotorDataSpeed(0x201),1);
		wheel_ladrc_left_2.up_data (-(wheel.speed[2] + wheel_e.speed[2]),Motor3508.GetMotorDataSpeed(0x202),1);
		wheel_ladrc_right_1.up_data(-(wheel.speed[1] + wheel_e.speed[1]),Motor3508.GetMotorDataSpeed(0x203),1);
		wheel_ladrc_right_2.up_data( (wheel.speed[0] + wheel_e.speed[0]),Motor3508.GetMotorDataSpeed(0x204),1);
		
		wheel_td_left_1 .td_quadratic(-Motor3508.GetMotorDataSpeed(0x201));
		wheel_td_left_2 .td_quadratic(-Motor3508.GetMotorDataSpeed(0x202));
		wheel_td_right_1.td_quadratic( Motor3508.GetMotorDataSpeed(0x203));
		wheel_td_right_2.td_quadratic( Motor3508.GetMotorDataSpeed(0x204));
		
		setMSD(&msd_3508_2006,wheel_ladrc_left_1.u ,1);
		setMSD(&msd_3508_2006,wheel_ladrc_left_2.u ,2);
		setMSD(&msd_3508_2006,wheel_ladrc_right_1.u,3);
		setMSD(&msd_3508_2006,wheel_ladrc_right_2.u,4);
		//发送数据
		Send_3508_CAN();
	}
}

//can_filo0中断接收
CAN_RxHeaderTypeDef RxHeader;	//can接收数据
uint8_t RxHeaderData[8] = { 0 };
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//接受信息 
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxHeaderData);
	if(hcan == &hcan1)
	{
		Motor3508.Parse(RxHeader,RxHeaderData);
	}
	if(hcan == &hcan2)
	{
		pm01.PM01Parse(RxHeader,RxHeaderData);
	}
}

//UART空闲中断接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	rmClicker.Parse(huart,Size);//遥控器解析
}

//UART中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Get_Gimbal_to_Chassis(huart);
	RM_RefereeSystem::RM_RefereeSystemParse(huart);
}


//获取云台到底盘数据初始化
void Get_Gimbal_to_Chassis_Init()
{
	HAL_UART_Receive_IT(&Send_Gimbal_to_Chassis_Huart,&Gimbal_to_Chassis_Data.head,1);//先使用中断接收到帧头
}
uint8_t wshu = 0;
uint8_t FP_num = 0;
//获取云台到底盘数据
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart)
{
	if(huart == &Send_Gimbal_to_Chassis_Huart)
	{
		
		//获取帧头
		if(Gimbal_to_Chassis_Data.head == 0xAA)
		{
			HAL_UART_Receive_DMA(&Send_Gimbal_to_Chassis_Huart,Gimbal_to_Chassis_Data.pData,Send_Gimbal_to_Chassis_Huart_LEN);//开启dma接收
			
			//修正帧尾
			if(Gimbal_to_Chassis_Data.pData[Send_Gimbal_to_Chassis_Huart_LEN - 1] != 0xAA)
			{
				Gimbal_to_Chassis_Data.head = 0x00;
				Get_Gimbal_to_Chassis_Init();
			}
			else
			{
				Gimbal_to_Chassis_Data.notation_RC_LY = Gimbal_to_Chassis_Data.pData[0] >> 7;//符号位
				Gimbal_to_Chassis_Data._RC_LY = Gimbal_to_Chassis_Data.pData[0] & 0x7f;//数据
				Gimbal_to_Chassis_Data.int16_RC_LY  = Gimbal_to_Chassis_Data._RC_LY * 6;//_RC_LY解算
				if(Gimbal_to_Chassis_Data.notation_RC_LY == 0)Gimbal_to_Chassis_Data.int16_RC_LY *= -1;//符号位解算
				
				
				Gimbal_to_Chassis_Data.notation_RC_LX = Gimbal_to_Chassis_Data.pData[1] >> 7;//符号位
				Gimbal_to_Chassis_Data._RC_LX = Gimbal_to_Chassis_Data.pData[1] & 0x7f;//数据
				Gimbal_to_Chassis_Data.int16_RC_LX  = Gimbal_to_Chassis_Data._RC_LX * 6;//_RC_LX解算
				if(Gimbal_to_Chassis_Data.notation_RC_LX == 0)Gimbal_to_Chassis_Data.int16_RC_LX *= -1;//符号位解算
				
				Gimbal_to_Chassis_Data.yaw_encoder_angle_e = *((int16_t*)&Gimbal_to_Chassis_Data.pData[2]);
				Gimbal_to_Chassis_Data.yaw_encoder_e = *((int16_t*)&Gimbal_to_Chassis_Data.pData[9]);
				
				Gimbal_to_Chassis_Data.stop = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x80);//停止
				Gimbal_to_Chassis_Data.chassis_follow_gimbal = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x40);//底盘跟随云台
				Gimbal_to_Chassis_Data.gimbal_host_chassis = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x20);//主云台
				Gimbal_to_Chassis_Data.chassis_gyro = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x08);//小陀螺	
				Gimbal_to_Chassis_Data.chassis_top = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x10);//遥控器小陀螺
				Gimbal_to_Chassis_Data.up_ui = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x04);//刷新ui						
				Gimbal_to_Chassis_Data.MCL_of = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x02);//摩擦轮
				Gimbal_to_Chassis_Data.CM_of = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x01);//仓门

				Gimbal_to_Chassis_Data.notation_ER = Gimbal_to_Chassis_Data.pData[5] >> 2;//符号位
				Gimbal_to_Chassis_Data.int8_ER = Gimbal_to_Chassis_Data._ER = (Gimbal_to_Chassis_Data.pData[5] & 0x02) | (Gimbal_to_Chassis_Data.pData[5] & 0x01);//数据
				if(Gimbal_to_Chassis_Data.notation_ER == 1)Gimbal_to_Chassis_Data.int8_ER *= -1;//符号位解算
				Gimbal_to_Chassis_Data.bp_of = (bool)(Gimbal_to_Chassis_Data.pData[8] & 0x01);
				
				Gimbal_to_Chassis_Data.notation_pitch_cai = Gimbal_to_Chassis_Data.pData[6] >> 4;//符号位
				Gimbal_to_Chassis_Data._pitch_cai = Gimbal_to_Chassis_Data.pData[6] & 0x0f;//数据
				Gimbal_to_Chassis_Data.int8_pitch_cai  = Gimbal_to_Chassis_Data._pitch_cai * 1.56;//pitch_cai解算
				if(Gimbal_to_Chassis_Data.notation_pitch_cai == 1)Gimbal_to_Chassis_Data.int8_pitch_cai *= -1;//符号位解算
				
//				Gimbal_to_Chassis_Data.notation_ZX = Gimbal_to_Chassis_Data.pData[7] >> 2;//符号位
//				Gimbal_to_Chassis_Data.int8_ZX = Gimbal_to_Chassis_Data._ER = (Gimbal_to_Chassis_Data.pData[7] & 0x02) | (Gimbal_to_Chassis_Data.pData[7] & 0x01);//数据
//				if(Gimbal_to_Chassis_Data.notation_ZX == 1)Gimbal_to_Chassis_Data.int8_ZX *= -1;//符号位解算
				Gimbal_to_Chassis_Data.int8_ZX= Gimbal_to_Chassis_Data.pData[7];
				
				Ctrl_Key.UpKey(Gimbal_to_Chassis_Data.up_ui);//刷新ui
				if(Ctrl_Key.RisingEdge)
				{
					set_send_graphic_queue_is_Delete_all();//删除全部图层
					darw_graphic_static_ui_init();//静态ui初始化
				}
				
//				last_er_v = now_er_v;//转速获取
//				now_er_v = Gimbal_to_Chassis_Data.int8_ER;
//				if(now_er_v != last_er_v)
//				{
					//set_gy_v(RM_RefereeSystem::OperateRevise,W_V_Init + now_er_v * GY_V_SET);//修改转速
//				}
				
				last_zx_v = Gimbal_to_Chassis_Data.int8_ZX;//速度获取
//				now_zx_v = Gimbal_to_Chassis_Data.int8_ZX;
				if(now_zx_v != last_zx_v)
				{
					set_yd_v(RM_RefereeSystem::OperateRevise,(YD_V_Init+(now_zx_v * YD_V_SET)));//修改速度
					
					set_gy_v(RM_RefereeSystem::OperateRevise,(YD_V_Init+(now_zx_v * YD_V_SET)-0.2));//修改转速
				}
				
				
				MCL_Key.UpKey(Gimbal_to_Chassis_Data.MCL_of);//刷新摩擦轮开关
				if(MCL_Key.RisingEdge | MCL_Key.FallingEdge)
				{
					set_mcl_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.MCL_of);//摩擦轮开关
				}
				
				CM_Key.UpKey(Gimbal_to_Chassis_Data.CM_of);//刷新仓门开关
				if(CM_Key.RisingEdge | CM_Key.FallingEdge)
				{
					set_cm_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.CM_of);//仓门开关
				}
				
				FP_Key.UpKey(Gimbal_to_Chassis_Data.int8_ER);
				if(FP_Key.RisingEdge | FP_Key.FallingEdge)
				{
					set_fp_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.int8_ER);
				}
//				BP_Key.UpKey(Gimbal_to_Chassis_Data.bp_of);
//				if(BP_Key.RisingEdge | BP_Key.FallingEdge)
//				{
//					set_bp_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.bp_of);//拨盘开关
//				}
				//set_cd_of(RM_RefereeSystem::OperateRevise,pm01.cin_voltage);
				//ZM_Key.UpKey(Gimbal_to_Chassis_Data.v_of);
				//wwww = ZM_Key.NowKey;
//				if(ZM_Key.RisingEdge | ZM_Key.FallingEdge)
//				{
//					set_zm_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.v_of);//仓门开关
//				}
//				if(ZM_Key.RisingEdge | ZM_Key.FallingEdge)
//				{
//					set_100w_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.v_of);//100w开关
//				}
			}				
		}
		else
		{
			HAL_UART_Receive_IT(&Send_Gimbal_to_Chassis_Huart,&Gimbal_to_Chassis_Data.head,1);//先使用中断接收到帧头
		}
		Gimbal_to_Chassis_Data.dir_time.UpLastTime();//串口更新时间
	}
}
float t,xm,lk;
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)//回调函数
{
	if(htim == &htim6)
	{					
		if(dir == false)
		{
			
			vx = vy = 0;
			yaw_e_xita = -YAW_E * 0.043950;//θ误差计算
			yaw_e_radian = (yaw_e_xita + 90.0) * 0.017453;//转弧度,+90是因为修正原本的角度
			cos_xita = cosf(yaw_e_radian);//计算cos
			sin_xita = sinf(yaw_e_radian);//计算sin
			if(CHASSIS_FOLLOW_GIMBAL == true/*底盘跟随云台*/ || 
				 GIMBAL_HOST_CHASSIS == true/*主云台模式*/ || 
				 CHASSIS_GYRO == true/*小陀螺*/ || CHASSIS_TOP == true/*遥控器小陀螺*/)
			{
				if(CHASSIS_FOLLOW_GIMBAL == true/*底盘跟随云台*/)
				{					
					vw = Gimbal_to_Chassis_Data.yaw_encoder_angle_e * kvw*-1;	//vw计算
					td_vw.td_quadratic(vw,false);//vw跟踪微方器
				}
				else if(GIMBAL_HOST_CHASSIS == true/*主云台模式*/)
				{
//					vw = 0;//旋转量
//					td_vw.td_quadratic(vw);//vw跟踪微方器
				}
				if(CHASSIS_GYRO == true/*小陀螺*/)
				{
					vw += last_zx_v*0.01;//旋转量
					td_vw.td_quadratic(vw);//vw跟踪微方器
					if(vw>=7000)
					{
						vw = 7000;
					}
					else if(vw<=-7000)
					{
						vw = -7000;
					}
				}
				else if(CHASSIS_TOP == true/*遥控器小陀螺*/)
				{
//					vw = int(Gimbal_to_Chassis_Data.int16_RC_LX*1.3);
//					td_vw.td_quadratic(vw);//vw跟踪微方器
				}
				else if(CHASSIS_FOLLOW_GIMBAL == true /*底盘跟随云台，遥控器或者键鼠shift*/)
				{
					if(Gimbal_to_Chassis_Data.int8_ER==1)
					{
						shift_vxy_zoom=1.65;
					}
					else
					{
						shift_vxy_zoom = YD_V_Init+(0.8 * YD_V_SET)+0.2;//修改速度正比
//					td_vx.r = td_vy.r = 10;//修改速度跟踪
					}
				}
				else
				{
					
					shift_vxy_zoom = YD_V_Init+(0.8 * YD_V_SET_01)-0.3;//修改速度正比
//					td_vx.r = td_vy.r = 10;//修改速度跟踪
				}
				if(CHASSIS_TOP == false/*遥控器小陀螺*/)
				{
					td_vx.td_quadratic(chassis_RC_LX,true);//chassis_RC_LX跟踪微方器
					td_vy.td_quadratic(chassis_RC_LY,true);//chassis_RC_LY跟踪微方器
				
					//旋转矩阵
					vx = (td_vx.x1 * cos_xita - td_vy.x1 * sin_xita) * shift_vxy_zoom;
					vy = (td_vx.x1 * sin_xita + td_vy.x1 * cos_xita) * shift_vxy_zoom;
				}
			}
			if(abs(chassis_RC_LX) > 15 || abs(chassis_RC_LY) > 15)
			{
				if(CHASSIS_GYRO==true)
				{
					vxy_kvw = 1.3;//保底盘功率
				}
				else
				{
					vxy_kvw = 2.6;
				}
			}
			else
			{
				if(CHASSIS_GYRO==true)
				{
					vxy_kvw = 1.3;//保底盘功率
				}
				else
				{
					vxy_kvw = 1.2;
				}
			}
				wheel.UpData(vx * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,vy * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,td_vw.x1 * CHASSIS_SPEED_ZOOM_VW * vxy_kvw,MAX_CHASSIS_SPEED);//底盘解算
		}
		if((dir == true) | (stop_mode == true))
		{
			wheel.speed[0] = wheel.speed[1] = wheel.speed[2] = wheel.speed[3] = 0;
			vw = 0;
		}
		
		wheel_ladrc_left_1.up_data ( (wheel.speed[3] + wheel_e.speed[3]),Motor3508.GetMotorDataSpeed(0x201),1);
		wheel_ladrc_left_2.up_data (-(wheel.speed[2] + wheel_e.speed[2]),Motor3508.GetMotorDataSpeed(0x202),1);
		wheel_ladrc_right_1.up_data(-(wheel.speed[1] + wheel_e.speed[1]),Motor3508.GetMotorDataSpeed(0x203),1);
		wheel_ladrc_right_2.up_data( (wheel.speed[0] + wheel_e.speed[0]),Motor3508.GetMotorDataSpeed(0x204),1);
		
		wheel_td_left_1 .td_quadratic(-Motor3508.GetMotorDataSpeed(0x201));
		wheel_td_left_2 .td_quadratic(-Motor3508.GetMotorDataSpeed(0x202));
		wheel_td_right_1.td_quadratic( Motor3508.GetMotorDataSpeed(0x203));
		wheel_td_right_2.td_quadratic( Motor3508.GetMotorDataSpeed(0x204));
				
		setMSD(&msd_3508_2006,wheel_ladrc_left_1.u ,1);
		setMSD(&msd_3508_2006,wheel_ladrc_left_2.u ,2);
		setMSD(&msd_3508_2006,wheel_ladrc_right_1.u,3);
		setMSD(&msd_3508_2006,wheel_ladrc_right_2.u,4);
	}
	if(htim == &htim5)
	{
		//发送数据
		Send_3508_CAN();
		
	}
}

RM_StaticTime darw_graphic_ui_time;//绘制ui定时器10hz
struct SEND_GRAPHIC_QUEUE//发送数据队列
{
	RM_RefereeSystem::graphic_data_struct_t graphic_data_struct[50];//图层数据
	//图案
	int8_t size;
	int8_t send_graphic_data_struct_size;
	//优先把所有文字显示出来
	RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character[20];//文字数据
	//文字
	int8_t wz_size;
	//刷新图层
	bool is_Delete_all;
	void add(RM_RefereeSystem::graphic_data_struct_t graphic_data_struct_temp)
	{		
		if(size >= 49)return;
		memcpy((void*)&graphic_data_struct[size],(void*)&graphic_data_struct_temp,sizeof(RM_RefereeSystem::graphic_data_struct_t));
		size++;
	}
	void add_wz(RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character_temp)
	{		
		if(wz_size >= 19)return;
		memcpy((void*)&ext_client_custom_character[wz_size],(void*)&ext_client_custom_character_temp,sizeof(RM_RefereeSystem::ext_client_custom_character_t));
		wz_size++;
	}
	bool send()
	{
		if(is_Delete_all == true)return false;		
		if(wz_size != 0)return false;
		if(size == 0)return true;
		if(!darw_graphic_ui_time.ISOne(100))return false;		
		if(size >= 7)send_graphic_data_struct_size = 7;
		else if(size > 2)send_graphic_data_struct_size = 5;
		else if(size == 2)send_graphic_data_struct_size = 2;
		else if(size == 1)send_graphic_data_struct_size = 1;
		else send_graphic_data_struct_size = 0;
		if(send_graphic_data_struct_size != 0)
		{
			RM_RefereeSystem::RM_RefereeSystemSendDataN(graphic_data_struct,send_graphic_data_struct_size);
			size -= send_graphic_data_struct_size;	
		}
		if(size < 0)size = 0;
		memcpy(graphic_data_struct,(void*)&graphic_data_struct[send_graphic_data_struct_size],sizeof(RM_RefereeSystem::graphic_data_struct_t) * (size));	
		memset((void*)&graphic_data_struct[size],0,sizeof(RM_RefereeSystem::graphic_data_struct_t) * (send_graphic_data_struct_size));	
		return true;
	}
	bool send_wz()
	{
		if(is_Delete_all == true)return false;		
		if(wz_size == 0)return true;
		if(!darw_graphic_ui_time.ISOne(100))return false;				
		if(wz_size != 0)
		{
			RM_RefereeSystem::RM_RefereeSystemSendStr(ext_client_custom_character[wz_size - 1]);
			wz_size--;	
		}
		if(wz_size < 0)wz_size = 0;
		return true;
	}
	bool send_delet_all()
	{
		if(is_Delete_all == false)return true;
		if(!darw_graphic_ui_time.ISOne(100))return false;	
		if(is_Delete_all == true)
		{
			RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::DeleteAll);
			RM_RefereeSystem::RM_RefereeSystemDelete(RM_RefereeSystem::DeleteAll,0);
			RM_RefereeSystem::RM_RefereeSystemClsToop();
			is_Delete_all = false;
		}
		return true;
	}
}send_graphic_queue;

//超电位置
#define CD_X 1260
#define CD_Y 240
#define CD_W 200
#define CD_H 20
#define CD_WZ_X 1000 //文字
#define CD_WZ_Y 250
//超电线
#define CDL_X 1260
#define CDL_Y 240
#define CDL_H 50
//底盘位置
#define CHASSIS_X 760
#define CHASSIS_Y 180
#define CHASSIS_W 50
#define CHASSIS_H 50
//底盘碰撞位置
#define collide_1 550
#define collide_2 1370
#define collide_magnify 3 //放大倍率
//下面的瞄准线
#define aim_x 700
#define aim_y 465


//GY_V小陀螺转速
#define GY_V_X 20
#define GY_V_Y 840
//yd_V小陀螺速速
#define YD_V_X 20
#define YD_V_Y 790
//摩擦轮on_off
#define MCL_of_X 20
#define MCL_of_Y 740
//仓门on_off
#define CM_of_X 20
#define CM_of_Y 690
//拨盘on_off
#define BP_of_X 20
#define BP_of_Y 640
//飞坡on_off
#define FP_of_X 20
#define FP_of_Y 640
//超电on_off
#define CD_of_X 20
#define CD_of_Y 340
//视觉自瞄
#define ZM_of_X 20
#define ZM_of_Y 560
//设置删除全部图层
void set_send_graphic_queue_is_Delete_all()
{
	send_graphic_queue.is_Delete_all = true;
}

//修改转速字符串
void set_gy_v(int Operate,float v)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	char str[10] = { 0 };
	sprintf(str,"XTL:%d",int(W_V_Init*v));
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("gyv",1,str,GY_V_X,GY_V_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//修改移速字符串
void set_yd_v(int Operate,float v)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	char str[10] = { 0 };
	sprintf(str,"YD:%d",int(4000*v));
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("rpm",1,str,YD_V_X,YD_V_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//修改摩擦开关
void set_mcl_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mcl",1,"MCL:ON",MCL_of_X,MCL_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mcl",1,"MCL:OFF",MCL_of_X,MCL_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}

//修改仓门开关
void set_cm_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cm",1,"CM:ON",CM_of_X,CM_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cm",1,"CM:OFF",CM_of_X,CM_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//修改拨盘开关
void set_bp_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("bp",1,"BP:ON",BP_of_X,BP_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("bp",1,"BP:OFF",BP_of_X,BP_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//修改飞坡开关
void set_fp_of(int Operate,uint8_t is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == 1)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("fp",1,"FP:ON",FP_of_X,FP_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("fp",1,"FP:OFF",FP_of_X,FP_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}

//修改超电开关
void set_cd_of(int Operate,float cd)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	char str[10] = { 0 };
	sprintf(str,"CD:%f",cd);
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cd_num",1,str,CD_of_X,CD_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}



//修改自瞄开关
void set_zm_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(40);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("zm",1,"ZM:ON",ZM_of_X,ZM_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("zm",1,"ZM:OFF",ZM_of_X,ZM_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
////修改100w开关
//void set_100w_of(int Operate,bool is)
//{
//	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
//	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
//	RM_RefereeSystem::RM_RefereeSystemSetStringSize(40);
//	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
//	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("100w",1,"100W:ON",ZM_of_X,ZM_of_Y));
//	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("100w",1,"100W:OFF",ZM_of_X,ZM_of_Y));
//	RM_RefereeSystem::RM_RefereeSystemClsToop();
//}
void darw_graphic_static_ui_init()//静态ui初始化
{
	is_up_ui = false;//上锁
	send_graphic_queue.size = 0;//复位
//	//规则
//	//0号图层给静态图层使用，用于绘制静态不改图形
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
	//瞄准线
	//send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("w_l",0,500,Win_H * 0.5,1000,Win_H * 0.5));
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("w4", 0,930,540,950,540));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("w5", 0,970,540,990,540));
	
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("w6", 0,960,570,960,550));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("w7", 0,960,510,960,530));
	
	
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorAmaranth);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
	
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("quuu",1,965,481,12));
	
	
	
//	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("w5", 0,1260,370,1260,710));
	
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz1", 0,collide_1,0,collide_1+100,600));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz2", 0,collide_2,0,collide_2-100,600));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz3", 0,collide_1,collide_1,collide_2,collide_2));
	
	//小陀螺转速显示
	set_gy_v(RM_RefereeSystem::OperateAdd,(YD_V_Init+(now_zx_v * YD_V_SET)));
	//平移速度显示
	set_yd_v(RM_RefereeSystem::OperateAdd,(YD_V_Init+(now_zx_v * YD_V_SET_01)-0.3));
	//摩擦轮开启关闭显示
	set_mcl_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.MCL_of);
	//仓门开启关闭显示
	set_cm_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.CM_of);
	//拨盘开启关闭显示
//	set_bp_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.int8_ER);
	//飞坡开启关闭显示
	set_fp_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.bp_of);
	//超电当前电压
	//set_cd_of(RM_RefereeSystem::OperateRevise,pm01.cin_voltage);
	//视觉开启关闭显示
//	set_zm_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.v_of);
	//100w开启关闭显示
//	set_100w_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.v_of);
	//设置超电文字
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(20);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cdw",0,"CD:",CD_WZ_X,CD_WZ_Y));
	
	//绘制超电矩形框
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorBlack);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("cd",0,CD_X - CD_W,CD_Y - CD_H,CD_X + CD_W,CD_Y + CD_H));
	//绘制超电能量调
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("cdc",1,CD_X - CD_W,CD_Y - CD_H + 10,CD_X + CD_W + 1,CD_Y + CD_H - 10));
	//绘制超电15v一下的线
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("cdl",0,CDL_X - CD_W + 250,CDL_Y - CDL_H,CDL_X - CD_W + 250,CDL_Y + CDL_H));
	//绘制小陀螺旋转方向	
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("dp",0,CHASSIS_X,CHASSIS_Y,CHASSIS_X,CHASSIS_Y + 50));
	//绘制小陀螺四边线
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorBlack);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);		
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp1",2,CHASSIS_X - CHASSIS_W,CHASSIS_Y + CHASSIS_H,CHASSIS_X + CHASSIS_W,CHASSIS_Y + CHASSIS_H));//上线	
//	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp2",2,CHASSIS_X + CHASSIS_W,CHASSIS_Y - CHASSIS_H,CHASSIS_X + CHASSIS_W,CHASSIS_Y + CHASSIS_H));//右线
//	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp3",2,CHASSIS_X - CHASSIS_W,CHASSIS_Y - CHASSIS_H,CHASSIS_X + CHASSIS_W,CHASSIS_Y - CHASSIS_H));//下线
//	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp4",2,CHASSIS_X - CHASSIS_W,CHASSIS_Y - CHASSIS_H,CHASSIS_X - CHASSIS_W,CHASSIS_Y + CHASSIS_H));//左线
	
	is_up_ui = true;
}
float angle_ly,ly;//角度，长度
void darw_graphic_ui()//绘制ui
{
	if(send_graphic_queue.send_delet_all() == true && is_up_ui == true && send_graphic_queue.send_wz() == true && send_graphic_queue.send() == true)
	{
		angle_ly = 31.466 + Gimbal_to_Chassis_Data.int8_pitch_cai;
		ly = (asin(angle_ly / 180.0 * 3.1415926) * 43) * collide_magnify;
		
		yaw_e_radian = (Gimbal_to_Chassis_Data.yaw_encoder_angle_e * 0.043950) * 0.017453;//转弧度,+90是因为修正原本的角度
		float cos_xita = cosf(yaw_e_radian);//计算cos
		float sin_xita = sinf(yaw_e_radian);//计算sin
		
		RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);
		


		RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz1", 0,collide_1,0,collide_1+100,ly));
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz2", 0,collide_2,0,collide_2-100,ly));
		RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorAmaranth);
		
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz3", 0,collide_1+100,ly,collide_2-100,ly));
		
		
		
		
		//绘制小陀螺四边线旋转
		RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
		RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp1",2,CHASSIS_X + ((-CHASSIS_W * cos_xita) - (+CHASSIS_H * sin_xita)),
																																						 CHASSIS_Y + ((-CHASSIS_W * sin_xita) + (+CHASSIS_H * cos_xita)),
																																						 CHASSIS_X + ((+CHASSIS_W * cos_xita) - (+CHASSIS_H * sin_xita)),
																																						 CHASSIS_Y + ((+CHASSIS_H * sin_xita) + (+CHASSIS_H * cos_xita))));//上线	
		
//		RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorBlack);
//		RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
//		
//		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp2",2,CHASSIS_X + ((+CHASSIS_W * cos_xita) - (-CHASSIS_H * sin_xita)),
//																																						 CHASSIS_Y + ((+CHASSIS_W * sin_xita) + (-CHASSIS_H * cos_xita)),
//																																						 CHASSIS_X + ((+CHASSIS_W * cos_xita) - (+CHASSIS_H * sin_xita)),
//																																						 CHASSIS_Y + ((+CHASSIS_H * sin_xita) + (+CHASSIS_H * cos_xita))));//右线
//		
//		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp3",2,CHASSIS_X + ((-CHASSIS_W * cos_xita) - (-CHASSIS_H * sin_xita)),
//																																						 CHASSIS_Y + ((-CHASSIS_W * sin_xita) + (-CHASSIS_H * cos_xita)),
//																																						 CHASSIS_X + ((+CHASSIS_W * cos_xita) - (-CHASSIS_H * sin_xita)),
//																																						 CHASSIS_Y + ((+CHASSIS_W * sin_xita) + (-CHASSIS_H * cos_xita))));//下线
//													
//		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp4",2,CHASSIS_X + ((-CHASSIS_W * cos_xita) - (-CHASSIS_H * sin_xita)),
//																																						 CHASSIS_Y + ((-CHASSIS_W * sin_xita) + (-CHASSIS_H * cos_xita)),
//																																						 CHASSIS_X + ((-CHASSIS_W * cos_xita) - (+CHASSIS_H * sin_xita)),
//																																						 CHASSIS_Y + ((-CHASSIS_W * sin_xita) + (+CHASSIS_H * cos_xita))));//左线		
		//绘制超电能量调
		
		
		if(pm01.cout_voltage < 15)
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorOrange);
		}
		else
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
		}
		RM_RefereeSystem::RM_RefereeSystemSetWidth(5);		
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("cdc",1,CD_X - CD_W,CD_Y - CD_H + 10,CD_X - CD_W + pm01.cout_voltage * 16.7,CD_Y + CD_H - 10));
																																								
		}
}


void Gpio_led_init(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();
	
		GPIO_InitTypeDef GPIO_InitStruct = {0};
	
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

