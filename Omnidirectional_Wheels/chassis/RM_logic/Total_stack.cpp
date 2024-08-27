#pragma once

#include "def_variable.h"
#include "Total_stack.h"

struct SEND_GRAPHIC_QUEUE;//�������ݶ���
extern SEND_GRAPHIC_QUEUE send_graphic_queue;

/***************************��������*********************************/

//���ܳ�ʼ��
void Total_tasks_Init();

//���ܺ���
void Total_tasks_Run();

//����3508����
void Send_3508_CAN();

//��ͣģʽ��������pid
void control_pid_0_speed();

//�����������
void Clear_ALL_Data();

//��ȡ��̨���������ݳ�ʼ��
void Get_Gimbal_to_Chassis_Init();

//��ȡ��̨����������
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart);

//����3508����
void Send_3508_CAN()
{
	//����
	RM_FDorCAN_Send(&hcan1,SEND_MOTOR_ID_3508,msd_3508_2006.Data,CAN_TX_MAILBOX1);
}

//��ͣģʽ��������pid
void control_pid_0_speed()
{

}

//�����������
void Clear_ALL_Data()
{
	
}

//���ܳ�ʼ��
void Total_tasks_Init()
{
	RM_FDorCAN_Init();//can���ó�ʼ��
	
	rmClicker.Init();//ң�����������ó�ʼ��
	
	Get_Gimbal_to_Chassis_Init();//���ڳ�ʼ��
	
	RM_RefereeSystem::RM_RefereeSystemInit();//���ڳ�ʼ��
		
	darw_graphic_static_ui_init();
	
	HAL_Delay(10);
	
	//��¼��һ��ʱ��
	uint64_t time_adrc = HAL_GetTick();
	//adrc������
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
//���ܺ���
void Total_tasks_Run()
{
	int dir = rmClicker.ISDir();//ң������������

	Gimbal_to_Chassis_Data.dir = Gimbal_to_Chassis_Data.dir_time.ISDir(50);
	if(Gimbal_to_Chassis_Data.dir)Get_Gimbal_to_Chassis_Init();
	dir = Gimbal_to_Chassis_Data.dir;
	RM_RefereeSystem::RM_RefereeSystemDir();
	
	//���°�ͨ�Źҵ�
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,((GPIO_PinState)!dir));
	
	pm01.PM01SendFun();//���ͳ���
			
	darw_graphic_ui();//����ui
	dianlu = (-_Motor3508_[0].Data[Motor_Data_Torque]+_Motor3508_[1].Data[Motor_Data_Torque]+_Motor3508_[2].Data[Motor_Data_Torque]+(-_Motor3508_[3].Data[Motor_Data_Torque]))*0.25;
	dianlutd.td_quadratic(dianlu / 819.2 * 24);
	dianlupid = (-wheel_ladrc_left_1.u+wheel_ladrc_left_2.u+wheel_ladrc_right_1.u+(-wheel_ladrc_right_2.u))*0.25;
	
//	if(Total_tasks_staticTime.ISOne(2))//���Ƶ���can����Ƶ��
//	{
//		if(!dir && RM_Clicker::RC_Ctl.rc.s2 != 2)
//		{					
//			/*
//			//�����ٶȽ��㣨δ��ɣ����w��ת�ٶ�
//			now_vxy[0] =  wheel_td_left_1 .x1;
//			now_vxy[1] =  wheel_td_left_2 .x1;
//			now_vxy[2] =  wheel_td_right_1.x1;
//			now_vxy[3] =  wheel_td_right_2.x1;
//			//��Ϊ�������
//			vxy_zero_s = 0;
//			for(int i = 0;i < 4;i++){if(fabs(now_vxy[i]) > 50)vxy_zero_s++;}
//			if(vxy_zero_s == 0)vxy_zero_s = 1;
//			//���㳵���ٶ�
//			num_vx = -(now_vxy[0] - now_vxy[1] + now_vxy[2] - now_vxy[3]) / vxy_zero_s;
//			num_vy =  (now_vxy[0] + now_vxy[1] + now_vxy[2] + now_vxy[3]) / vxy_zero_s;
//			//���㳵����ٻ���
//			e_vx += (RC_LX * 24.8242 * 0.1 - num_vx) * k_e_vxy;
//			e_vy += (RC_LY * 24.8242 * 0.1 - num_vy) * k_e_vxy;
//			//�޷�
//			e_vx = fmod(e_vx,1000);
//			e_vy = fmod(e_vx,1000);			
//			//ң�������ٶ�
//			expectations_vx = RC_LX * 24.8242 * 0.1; 
//			expectations_vy = RC_LY * 24.8242 * 0.1; 
//			wheel_e.UpData(e_vx,e_vy,0,660.0 * 24.8242 * 0.1);

//			Send_Usart_Data("%f,%f\r\n",(float)(e_vx),(float)(e_vy));
//			*/
//			wheel.UpData(RC_LX * 24.8242 * 0.1,RC_LY * 24.8242 * 0.1,RC_RX * 8,660.0 * 24.8242 * 0.1);		
//			
////			Send_Usart_Data("%f,%f,%f\r\n",(float)(wheel_ladrc_left_1.feedback),(float)(wheel_ladrc_left_1.z1),(float)(wheel_ladrc_left_1.td.x1));
//		}		
//		else
//		{
//			e_vx = e_vy = 0;
//			memset(&wheel,0,sizeof(wheel));
//		}
//		
//		wheel_ladrc_left_1.up_data ( (wheel.speed[3] + wheel_e.speed[3]),Motor3508.GetMotorDataSpeed(0x201),1);
//		wheel_ladrc_left_2.up_data (-(wheel.speed[2] + wheel_e.speed[2]),Motor3508.GetMotorDataSpeed(0x202),1);
//		wheel_ladrc_right_1.up_data(-(wheel.speed[1] + wheel_e.speed[1]),Motor3508.GetMotorDataSpeed(0x203),1);
//		wheel_ladrc_right_2.up_data( (wheel.speed[0] + wheel_e.speed[0]),Motor3508.GetMotorDataSpeed(0x204),1);
//		
//		wheel_td_left_1 .td_quadratic(-Motor3508.GetMotorDataSpeed(0x201));
//		wheel_td_left_2 .td_quadratic(-Motor3508.GetMotorDataSpeed(0x202));
//		wheel_td_right_1.td_quadratic( Motor3508.GetMotorDataSpeed(0x203));
//		wheel_td_right_2.td_quadratic( Motor3508.GetMotorDataSpeed(0x204));
//		
//		setMSD(&msd_3508_2006,wheel_ladrc_left_1.u ,1);
//		setMSD(&msd_3508_2006,wheel_ladrc_left_2.u ,2);
//		setMSD(&msd_3508_2006,wheel_ladrc_right_1.u,3);
//		setMSD(&msd_3508_2006,wheel_ladrc_right_2.u,4);
//		//��������
//		Send_3508_CAN();
//	}
}

//can_filo0�жϽ���
CAN_RxHeaderTypeDef RxHeader;	//can��������
uint8_t RxHeaderData[8] = { 0 };
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//������Ϣ 
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

//UART�����жϽ���
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	rmClicker.Parse(huart,Size);//ң��������
}

//UART�ж�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Get_Gimbal_to_Chassis(huart);
	RM_RefereeSystem::RM_RefereeSystemParse(huart);
}


//��ȡ��̨���������ݳ�ʼ��
void Get_Gimbal_to_Chassis_Init()
{
	HAL_UART_Receive_IT(&Send_Gimbal_to_Chassis_Huart,&Gimbal_to_Chassis_Data.head,1);//��ʹ���жϽ��յ�֡ͷ
}
uint8_t wshu = 0;
uint8_t FP_num = 0;
//��ȡ��̨����������
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart)
{
	if(huart == &Send_Gimbal_to_Chassis_Huart)
	{
		
		//��ȡ֡ͷ
		if(Gimbal_to_Chassis_Data.head == 0xAA)
		{
			HAL_UART_Receive_DMA(&Send_Gimbal_to_Chassis_Huart,Gimbal_to_Chassis_Data.pData,Send_Gimbal_to_Chassis_Huart_LEN);//����dma����
			
			//����֡β
			if(Gimbal_to_Chassis_Data.pData[Send_Gimbal_to_Chassis_Huart_LEN - 1] != 0xAA)
			{
				Gimbal_to_Chassis_Data.head = 0x00;
				Get_Gimbal_to_Chassis_Init();
			}
			else
			{
				Gimbal_to_Chassis_Data.notation_RC_LY = Gimbal_to_Chassis_Data.pData[0] >> 7;//����λ
				Gimbal_to_Chassis_Data._RC_LY = Gimbal_to_Chassis_Data.pData[0] & 0x7f;//����
				Gimbal_to_Chassis_Data.int16_RC_LY  = Gimbal_to_Chassis_Data._RC_LY * 6;//_RC_LY����
				if(Gimbal_to_Chassis_Data.notation_RC_LY == 0)Gimbal_to_Chassis_Data.int16_RC_LY *= -1;//����λ����
				
				
				Gimbal_to_Chassis_Data.notation_RC_LX = Gimbal_to_Chassis_Data.pData[1] >> 7;//����λ
				Gimbal_to_Chassis_Data._RC_LX = Gimbal_to_Chassis_Data.pData[1] & 0x7f;//����
				Gimbal_to_Chassis_Data.int16_RC_LX  = Gimbal_to_Chassis_Data._RC_LX * 6;//_RC_LX����
				if(Gimbal_to_Chassis_Data.notation_RC_LX == 0)Gimbal_to_Chassis_Data.int16_RC_LX *= -1;//����λ����
				
				Gimbal_to_Chassis_Data.yaw_encoder_angle_e = *((int16_t*)&Gimbal_to_Chassis_Data.pData[2]);
				Gimbal_to_Chassis_Data.yaw_encoder_e = *((int16_t*)&Gimbal_to_Chassis_Data.pData[9]);
				
				Gimbal_to_Chassis_Data.stop = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x80);//ֹͣ
				Gimbal_to_Chassis_Data.chassis_follow_gimbal = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x40);//���̸�����̨
				Gimbal_to_Chassis_Data.gimbal_host_chassis = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x20);//����̨
				Gimbal_to_Chassis_Data.chassis_gyro = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x08);//С����	
				Gimbal_to_Chassis_Data.chassis_top = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x10);//ң����С����
				Gimbal_to_Chassis_Data.up_ui = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x04);//ˢ��ui						
				Gimbal_to_Chassis_Data.MCL_of = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x02);//Ħ����
				Gimbal_to_Chassis_Data.CM_of = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x01);//����

				Gimbal_to_Chassis_Data.notation_R = Gimbal_to_Chassis_Data.pData[5] >> 2;//����λ
				Gimbal_to_Chassis_Data.int8_ER = Gimbal_to_Chassis_Data._R = (Gimbal_to_Chassis_Data.pData[5] & 0x02) | (Gimbal_to_Chassis_Data.pData[5] & 0x01);//����
				if(Gimbal_to_Chassis_Data.notation_R == 1)Gimbal_to_Chassis_Data.int8_ER *= -1;//����λ����
				Gimbal_to_Chassis_Data.sj_of = (bool)(Gimbal_to_Chassis_Data.pData[8] & 0x01);
				
				Gimbal_to_Chassis_Data.notation_pitch_cai = Gimbal_to_Chassis_Data.pData[6] >> 4;//����λ
				Gimbal_to_Chassis_Data._pitch_cai = Gimbal_to_Chassis_Data.pData[6] & 0x0f;//����
				Gimbal_to_Chassis_Data.int8_pitch_cai  = Gimbal_to_Chassis_Data._pitch_cai * 1.56;//pitch_cai����
				if(Gimbal_to_Chassis_Data.notation_pitch_cai == 1)Gimbal_to_Chassis_Data.int8_pitch_cai *= -1;//����λ����
				
//				Gimbal_to_Chassis_Data.notation_ZX = Gimbal_to_Chassis_Data.pData[7] >> 2;//����λ
//				Gimbal_to_Chassis_Data.int8_ZX = Gimbal_to_Chassis_Data._ER = (Gimbal_to_Chassis_Data.pData[7] & 0x02) | (Gimbal_to_Chassis_Data.pData[7] & 0x01);//����
//				if(Gimbal_to_Chassis_Data.notation_ZX == 1)Gimbal_to_Chassis_Data.int8_ZX *= -1;//����λ����
				Gimbal_to_Chassis_Data.int8_ZX= Gimbal_to_Chassis_Data.pData[7];
				
				Ctrl_Key.UpKey(Gimbal_to_Chassis_Data.up_ui);//ˢ��ui
				if(Ctrl_Key.RisingEdge)
				{
					set_send_graphic_queue_is_Delete_all();//ɾ��ȫ��ͼ��
					darw_graphic_static_ui_init();//��̬ui��ʼ��
				}
				
				last_zx_v = now_zx_v;//�ٶȻ�ȡ
				now_zx_v = Gimbal_to_Chassis_Data.int8_ZX;
				
				MCL_Key.UpKey(Gimbal_to_Chassis_Data.MCL_of);//ˢ��Ħ���ֿ���
				if(MCL_Key.RisingEdge | MCL_Key.FallingEdge)
				{
					set_mcl_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.MCL_of);//Ħ���ֿ���
				}
				
				CM_Key.UpKey(Gimbal_to_Chassis_Data.CM_of);//ˢ�²��ſ���
				if(CM_Key.RisingEdge | CM_Key.FallingEdge)
				{
					set_cm_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.CM_of);//���ſ���
				}
//				set_cs_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.int8_ER);
//				BP_Key.UpKey(Gimbal_to_Chassis_Data.bp_of);
//				if(BP_Key.RisingEdge | BP_Key.FallingEdge)
//				{
//					set_bp_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.bp_of);//���̿���
//				}
				//set_cd_of(RM_RefereeSystem::OperateRevise,pm01.cin_voltage);
				//ZM_Key.UpKey(Gimbal_to_Chassis_Data.v_of);
				//wwww = ZM_Key.NowKey;
//				if(ZM_Key.RisingEdge | ZM_Key.FallingEdge)
//				{
//					set_zm_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.v_of);//���ſ���
//				}
//				if(ZM_Key.RisingEdge | ZM_Key.FallingEdge)
//				{
//					set_100w_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.v_of);//100w����
//				}
			}				
		}
		else
		{
			HAL_UART_Receive_IT(&Send_Gimbal_to_Chassis_Huart,&Gimbal_to_Chassis_Data.head,1);//��ʹ���жϽ��յ�֡ͷ
		}
		Gimbal_to_Chassis_Data.dir_time.UpLastTime();//���ڸ���ʱ��
	}
}

float t,xm,lk;
void Chassis_Task()
{
		if(dir == false)
		{
			vx = vy = 0;
			yaw_e_xita = -YAW_E * 0.043950;//��������
			yaw_e_radian = (yaw_e_xita) * 0.017453;//ת����
			
			cos_xita = cosf(yaw_e_radian);//����cos
			sin_xita = sinf(yaw_e_radian);//����sin
			if(CHASSIS_FOLLOW_GIMBAL == true/*���̸�����̨*/ || 
				 GIMBAL_HOST_CHASSIS == true/*����̨ģʽ*/ || 
				 CHASSIS_GYRO == true/*С����*/ || CHASSIS_TOP == true/*ң����С����*/)
			{

				if(CHASSIS_FOLLOW_GIMBAL == true/*���̸�����̨*/)
				{
					
					vw = Gimbal_to_Chassis_Data.yaw_encoder_angle_e *0.8*-1;	//vw����
					td_vw.td_quadratic(vw,false);//vw����΢����
				}
				else if(GIMBAL_HOST_CHASSIS == true/*����̨ģʽ*/)
				{
					if(Gimbal_to_Chassis_Data.int8_ER == 1)
					{
						vw = Gimbal_to_Chassis_Data.yaw_encoder_angle_e * 0.8*-1;	//vw����
					}
					else
					{
						vw = 0;//��ת��
					}
					td_vw.td_quadratic(vw);//vw����΢����
				}
				if(CHASSIS_GYRO == true/*С����*/)
				{
					vw = (now_zx_v * YD_V_GY_SET+0.35)*W_V_Init;
					td_vw.td_quadratic(vw);//vw����΢����
				}
				else if(CHASSIS_TOP == true/*ң����С����*/)
				{
					vw = int(Gimbal_to_Chassis_Data.int16_RC_LX*1.3);
					td_vw.td_quadratic(vw,false);//vw����΢����
				}
				else if(CHASSIS_FOLLOW_GIMBAL == true /*���̸�����̨��ң�������߼���shift*/)
				{
//					if(Gimbal_to_Chassis_Data.sj_of==1)
//					{
//						shift_vxy_zoom=1.65;//1.65
//					}
//					else
//					{
						shift_vxy_zoom = YD_V_Init+(now_zx_v * YD_V_SET)+0.15;//�޸��ٶ�����
//				`	}
				}
				else
				{
					
					shift_vxy_zoom = YD_V_Init+(now_zx_v * YD_V_SET)-0.15;//�޸��ٶ�����
//					td_vx.r = td_vy.r = 10;//�޸��ٶȸ���
				}
				if(CHASSIS_TOP == false/*ң����С����*/)
				{
					td_vx.td_quadratic(chassis_RC_LX,true);//chassis_RC_LX����΢����
					td_vy.td_quadratic(chassis_RC_LY,true);//chassis_RC_LY����΢����
				
					//��ת����
					vx = (td_vx.x1 * cos_xita - td_vy.x1 * sin_xita) * shift_vxy_zoom;
					vy = (td_vx.x1 * sin_xita + td_vy.x1 * cos_xita) * shift_vxy_zoom;
				}
			}
			if(abs(chassis_RC_LX) > 15 || abs(chassis_RC_LY) > 15)
			{
				if(CHASSIS_GYRO==true)
				{
					vxy_kvw = 1.3;//�����̹���
				}
				else
				{
					vxy_kvw = 1.3;
				}
			}
			else
			{
				if(CHASSIS_GYRO==true)
				{
					vxy_kvw = 1.3;//�����̹���
				}
				else
				{
					vxy_kvw = 2.3;
				}
			}
				wheel.UpData(vx * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,vy * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,td_vw.x1 * CHASSIS_SPEED_ZOOM_VW * vxy_kvw,MAX_CHASSIS_SPEED);//���̽���
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



RM_StaticTime darw_graphic_ui_time;//����ui��ʱ��10hz
struct SEND_GRAPHIC_QUEUE//�������ݶ���
{
	RM_RefereeSystem::graphic_data_struct_t graphic_data_struct[50];//ͼ������
	//ͼ��
	int8_t size;
	int8_t send_graphic_data_struct_size;
	//���Ȱ�����������ʾ����
	RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character[20];//��������
	//����
	int8_t wz_size;
	//ˢ��ͼ��
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

//����λ��
#define CD_X 1260
#define CD_Y 240
#define CD_W 200
#define CD_H 20
#define CD_WZ_X 1000 //����
#define CD_WZ_Y 250
//������
#define CDL_X 1260
#define CDL_Y 240
#define CDL_H 50
//����λ��
#define CHASSIS_X 760
#define CHASSIS_Y 180
#define CHASSIS_W 50
#define CHASSIS_H 50
//������ײλ��
#define collide_1 550
#define collide_2 1370
#define collide_magnify 3 //�Ŵ���
//�������׼��
#define aim_x 700
#define aim_y 465



//�ٶȵȼ�
#define ve_init_X 610	
#define ve_init_Y 450
#define ve_X 623

//GY_VС����ת��
#define GY_V_X 20
#define GY_V_Y 840
//yd_VС��������
#define YD_V_X 20
#define YD_V_Y 790
//Ħ����on_off
#define MCL_of_X 20
#define MCL_of_Y 740
//����on_off
#define CM_of_X 20
#define CM_of_Y 690
//С����on_off
#define XTL_of_X 1450
#define XTL_of_Y 775
//����on_off
#define CS_of_X 1450
#define CS_of_Y 710
//С��������
#define XTL_X 1400
#define XTL_Y 700
//����on_off
#define CD_of_X 20
#define CD_of_Y 340
//�Ӿ�����
#define ZM_of_X 20
#define ZM_of_Y 560
//����ɾ��ȫ��ͼ��
void set_send_graphic_queue_is_Delete_all()
{
	send_graphic_queue.is_Delete_all = true;
}

//�޸�Ħ������
void set_mcl_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//�����޸�
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mcl",1,"MCL:ON",MCL_of_X,MCL_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mcl",1,"MCL:OFF",MCL_of_X,MCL_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}

//�޸Ĳ��ſ���
void set_cm_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//�����޸�
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cm",1,"CM:ON",CM_of_X,CM_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cm",1,"CM:OFF",CM_of_X,CM_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//�޸�С���ݿ���
void set_xtl_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//�����޸�
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(20);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("xt",0,"XTL",XTL_of_X,XTL_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//�޸Ĳ�����
void set_cs_of(int Operate,uint8_t is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//�����޸�
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(20);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cs",1,"CS",CS_of_X,CS_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}

//�޸ĳ��翪��
void set_cd_of(int Operate,float cd)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//�����޸�
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	char str[10] = { 0 };
	sprintf(str,"CD:%f",cd);
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cd_num",1,str,CD_of_X,CD_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}



//�޸����鿪��
void set_zm_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//�����޸�
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(40);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("zm",1,"ZM:ON",ZM_of_X,ZM_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("zm",1,"ZM:OFF",ZM_of_X,ZM_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
////�޸�100w����
//void set_100w_of(int Operate,bool is)
//{
//	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//�����޸�
//	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
//	RM_RefereeSystem::RM_RefereeSystemSetStringSize(40);
//	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
//	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("100w",1,"100W:ON",ZM_of_X,ZM_of_Y));
//	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("100w",1,"100W:OFF",ZM_of_X,ZM_of_Y));
//	RM_RefereeSystem::RM_RefereeSystemClsToop();
//}
float angle_ly,ly;//�Ƕȣ�����
void darw_graphic_static_ui_init()//��̬ui��ʼ��
{
	is_up_ui = false;//����
	send_graphic_queue.size = 0;//��λ
//	//����
//	//0��ͼ�����̬ͼ��ʹ�ã����ڻ��ƾ�̬����ͼ��
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
	//��׼Ȧ
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
	
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("quuu",1,960,481,12));
	
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("quTT",0,960,450,7));
	//������
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(2);

	angle_ly = 31.466 + Gimbal_to_Chassis_Data.int8_pitch_cai;
	ly = (asin(angle_ly / 180.0 * 3.1415926) * 43) * collide_magnify;
	
	yaw_e_radian = (Gimbal_to_Chassis_Data.yaw_encoder_angle_e * 0.043950) * 0.017453;//ת����,+90����Ϊ����ԭ���ĽǶ�
	float cos_xita = cosf(yaw_e_radian);//����cos
	float sin_xita = sinf(yaw_e_radian);//����sin
	
	
	

	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz1", 0,collide_1,0,collide_1+100,ly));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz2", 0,collide_2,0,collide_2-100,ly));
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorAmaranth);
	
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mz3", 0,collide_1+100,ly,collide_2-100,ly));
	
	//Ħ���ֿ����ر���ʾ
	set_mcl_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.MCL_of);
	//���ſ����ر���ʾ
	set_cm_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.CM_of);
	//���¿����ر���ʾ
	set_cs_of(RM_RefereeSystem::OperateAdd,0);
	//С���ݿ����ر���ʾ
	set_xtl_of(RM_RefereeSystem::OperateAdd,0);
	//���統ǰ��ѹ
	//set_cd_of(RM_RefereeSystem::OperateRevise,pm01.cin_voltage);
	//�Ӿ������ر���ʾ
//	set_zm_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.v_of);
	//100w�����ر���ʾ
//	set_100w_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.v_of);
	//���ó�������
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(20);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cdw",0,"CD:",CD_WZ_X,CD_WZ_Y));
	
	//�Ӿ������
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorPink);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("vis", 2,1330,750,595,330));
	
	
	//���Ƴ�����ο�
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorBlack);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("cd",0,CD_X - CD_W,CD_Y - CD_H,CD_X + CD_W,CD_Y + CD_H));
	//���Ƴ���������
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("cdc",1,CD_X - CD_W,CD_Y - CD_H + 10,CD_X + CD_W + 1,CD_Y + CD_H - 10));
	//���Ƴ���15vһ�µ���
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("cdl",0,CDL_X - CD_W + 250,CDL_Y - CDL_H,CDL_X - CD_W + 250,CDL_Y + CDL_H));
	
	//�ٶȵȼ���ʾ
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(4);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd", 1,ve_init_X,ve_init_Y,ve_init_X,(ve_init_Y+6*35)));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd2", 1,ve_init_X,(ve_init_Y+0*35),(ve_init_X+10),(ve_init_Y+0*35)));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd3", 1,ve_init_X,(ve_init_Y+1*35),(ve_init_X+10),(ve_init_Y+1*35)));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd4", 1,ve_init_X,(ve_init_Y+2*35),(ve_init_X+10),(ve_init_Y+2*35)));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd5", 1,ve_init_X,(ve_init_Y+3*35),(ve_init_X+10),(ve_init_Y+3*35)));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd6", 1,ve_init_X,(ve_init_Y+4*35),(ve_init_X+10),(ve_init_Y+4*35)));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd7", 1,ve_init_X,(ve_init_Y+5*35),(ve_init_X+10),(ve_init_Y+5*35)));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd8", 1,ve_init_X,(ve_init_Y+6*35),(ve_init_X+10),(ve_init_Y+6*35)));
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(8);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd9",0,ve_X,(ve_init_Y+0*35),ve_X,(ve_init_Y+6*35)));
	
	
	//С���ݵ���ʾ
//	RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("xtld", 0,XTL_X,(XTL_Y+65),13));
	//�������ʾ
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("csd", 0,XTL_X,XTL_Y,13));
	
	is_up_ui = true;
}

void darw_graphic_ui()//����ui
{
	if(send_graphic_queue.send_delet_all() == true && is_up_ui == true && send_graphic_queue.send_wz() == true && send_graphic_queue.send() == true)
	{
		
		RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);
		
		
		if(Gimbal_to_Chassis_Data.sj_of == 1)
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
		}
		else
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorPink);
		}
		RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("vis", 2,1330,750,595,330));
		
		RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
		RM_RefereeSystem::RM_RefereeSystemSetWidth(8);
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("sd9",0,ve_X,(ve_init_Y+0*35),ve_X,(ve_init_Y+now_zx_v*35)));
		//���Ƴ���������
		
		
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
		
		
		
		RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
		//С���ݵ���ʾ
		if(Gimbal_to_Chassis_Data.chassis_gyro==1)
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
		}
		else
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
		}
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("xtld",0,XTL_X,(XTL_Y+65),13));
		//�������ʾ
		if(Gimbal_to_Chassis_Data.int8_ER==1)
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
		}
		else
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
		}
		//�������ʾ
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("csd",0,XTL_X,XTL_Y,13));
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

