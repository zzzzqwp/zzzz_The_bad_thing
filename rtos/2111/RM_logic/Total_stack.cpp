
#include "def_variable.h"
#include "RM_stdxxx.h"
#include "total_stack.h"
#define MIN(i,j) (((i) < (j)) ? (i) : (j))

struct vision
{
	uint8_t vision_TX_data[14];
	uint8_t vision_RX_data[14];
	float vision_yaw;
	float vision_pitah;
	int32_t pitch_360;
	int32_t yaw_360;
	uint8_t vision_ready;
	uint8_t vision_fire;
	bool vision_bool;
	
};




struct vision_t
{
	uint8_t vision_fire_flag;
	uint8_t private_vision_fire_flag;
	uint8_t vision_fire;

	uint32_t fire_time;
	
	
	
};

float vision_yaw_increment_angle;
float vision_yaw_tuoluo_increment_angle;
float vision_pitch_increment_angle;
float vision_pitch_tuoluo_increment_angle;
uint8_t vision_ready;
uint8_t vision_send_buf[3];
uint8_t vision_enable , reset_enable;
uint8_t bullet_speed;
uint8_t vision_rx_buf[23];


vision_t Visual_new_data;


vision vision_data;
/***************************函数声明*********************************/
//视觉函数
void vision_go_Computer();
void vision_to_computer();
void vision_send_Computer();
void vision_recive_task();

//A板led
void Gpio_led_init();
//发送串口数据
void Send_Usart_Data(char* format,...);//发送

//主跑初始化
void Total_tasks_Init();

//主跑函数
void Total_tasks_Run();

//发送6020数据
void Send_6020_CAN();

//发送3508/2006数据
void Send_3508_2006_CAN();

//急停模式控制所有pid
void control_pid_0_speed();

//清空数据数据
void Clear_ALL_Data();

//云台发送数据到底盘
void Send_Gimbal_to_Chassis();

//发送6020数据
void Send_6020_CAN()
{
	//发送
	RM_FDorCAN_Send(&hcan1,SEND_MOTOR_ID_6020,msd_6020.Data,CAN_TX_MAILBOX1);
}
//发送3508/2006数据
void Send_3508_2006_CAN()
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
float angle_quyu;
//主跑初始化
void Total_tasks_Init()
{
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_All,GPIO_PIN_SET);
	
	RM_FDorCAN_Init();//can配置初始化
	
	rmClicker.Init();//遥控器串口配置初始化
	
	servos.Init(&htim2,TIM_CHANNEL_3);//舵机初始化
	servos.SetAngle(servos_angle);
	
	#if GY_GET_SIG == GY_CAN
		
	#elif GY_GET_SIG == GY_232
		ch_gyro_232_Init(&ch_gyro_232_uart_chassis,&chGy_chassis);//地盘陀螺仪初始化
	#endif
	
	HAL_Delay(10);
	
	//设置云台初始角度
	#if GY_GET_SIG == GY_CAN
//		YAW_Init_Angle = chGy_chassis.gy.z_360;
//		yaw_pid.Is_Open_td_e(true);
//		yaw_pid.pid.td_e.r = 500;
//		yaw_pid.pid.td_c_c_d.r = 800;
//		yaw_pid.SetMixI(15000,2,3);//变速积分
		YAW_Init_Angle = chGy_chassis.gy.Add_z;
		ladrc_yaw_angle.set_is_separate_w0(100,8000,35000);
		ladrc_yaw_angle.set_is_separate_wc(2000,500);
	#elif GY_GET_SIG == GY_232
		YAW_Init_Angle = chGy_chassis.data.z_360;
		yaw_pid.Is_Open_td_e(true);
		yaw_pid.pid.td_e.r = 200;
		yaw_pid.pid.td_c_c_d.r = 200;
		yaw_pid.SetMixI(15000,0,5);//变速积分
	#endif
	
	#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
			//pitch设置
			ladrc_pitch.set_is_separate_wc(5000,70);
			ladrc_pitch.set_is_separate_w0(100,2000,20000);
	#elif PITCH__GET_SIG == PITCH_GY
			//pitch
			yaw_pid.Is_Open_td_e(true);
			yaw_pid.pid.td_e.r = 500;
			pitch_pid.pid.td_c_c_d.r = 800;
			pitch_pid.SetMixI(15000,0,10);//变速积分
	#endif

	//拨盘设置
	ladrc_dial.set_is_separate_wc(200,30);
	
	//左摩擦轮设置
	ladrc_friction_l.set_is_separate_wc(8000,400);
	
	//右摩擦轮设置
	ladrc_friction_r.set_is_separate_wc(8000,400);
	
	//记录上一次时间
	uint64_t time_adrc = HAL_GetTick();
	//adrc收敛期
	while(1)
	{
		//yaw收敛
		yaw_zz_pid_s.pid_calc(0,yaw_target_add_angle);	
		yaw_zz_pid_v.pid_calc(td_yaw_cin.x1,(yaw_zz_pid_s.out));
		
		//pithc收敛
		ladrc_pitch.up_data(0,Motor6020.GetMotor(PITCH_MOTOR_ID)->Data[Motor_Data_Angle],2);
		
		//拨盘收敛
		ladrc_dial.up_data(0,Motor3508.GetMotor(Dial_MOTOR_R_ID)->Data[Motor_Data_Speed],2);
		
		//左摩擦轮收敛
		ladrc_friction_l.up_data(0,Motor3508.GetMotor(Friction_MOTOR_L_ID)->Data[Motor_Data_Speed],2);
		
		//左摩擦轮收敛
		ladrc_friction_r.up_data(0,Motor3508.GetMotor(Friction_MOTOR_R_ID)->Data[Motor_Data_Speed],2);

		if(HAL_GetTick() - time_adrc > 500)
		{
			break;
		}
	}
}


uint8_t send_str2[64];
//主跑函数
void Total_tasks_Run()
{	
	servos.SetAngle(servos_angle);//舵机
	if(Total_tasks_staticTime.ISOne(2))//控制地盘can发送频率
	{
		//给底盘发送数据
		Send_Gimbal_to_Chassis();
		
		
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
		//6020电机解析
		Motor6020.Parse(RxHeader,RxHeaderData);
		
		//3508电机解析
		Motor3508.Parse(RxHeader,RxHeaderData);
		
		//2006电机解析
		Motor2006.Parse(RxHeader,RxHeaderData);
		
//		td_pitch_encoder_angle.td_quadratic(Motor6020.GetMotor(PITCH_MOTOR_ID)->Data[Motor_Data_Angle] * -0.04395067);
//		td_pitch_encoder_angle_v.td_quadratic(Motor6020.GetMotor(PITCH_MOTOR_ID)->Data[Motor_Data_Speed] * -0.104719755);
		
	}
	if(hcan == &hcan2)
	{
		#if GY_GET_SIG == GY_CAN
			chGy_chassis.Parse(RxHeader,RxHeaderData);
			td_gy_y.td_quadratic(chGy_chassis.gy.y);
//			td_pitch_gy_angle.td_quadratic(chGy_chassis.gy.y);
//			td_pitch_gy_angle_v.td_quadratic(chGy_chassis.gy.ASx);
		#elif GY_GET_SIG == GY_232
			
		#endif
	}
}
float sinf_temp,cosf_temp;
//UART中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	#if GY_GET_SIG == GY_CAN
		
	#elif GY_GET_SIG == GY_232
		ch_gyro_232_parse(huart,&ch_gyro_232_uart_chassis,&chGy_chassis);
	#endif
	
//	Get_Gimbal_to_Chassis(huart);
	
	
	
}

//UART空闲中断接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	rmClicker.Parse(huart,Size);//遥控器解析
	if(huart == &vision_Huart)
	{
		vision_data.vision_bool = 1;
		HAL_UARTEx_ReceiveToIdle_DMA(&vision_Huart,vision_data.vision_RX_data,13);
	}
//	if(huart == &vision_Huart)
//	{
//		HAL_UARTEx_ReceiveToIdle_IT(&vision_Huart,vision_rx_buf,23);
//	}
}

//设置串口数据
uint8_t send_str[64] = "";//发送缓冲区
extern DMA_HandleTypeDef hdma_usart6_tx;
void Send_Usart_Data(char* format,...)//发送
{
		uint32_t length = 0;
		va_list args;
		
		va_start(args, format);
		
		length = vsnprintf((char*)send_str, 64, (char*)format, args);
		
//		//等待上一次的数据发送完毕
//		while(HAL_DMA_GetState(&hdma_usart6_tx) != HAL_DMA_STATE_READY);
//	
//    //关闭DMA
//    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    //开始发送数据
    HAL_UART_Transmit_DMA(&Send_Usart_Data_Huart, send_str, length);
}

//串口重定向
int std::fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1,10);
//  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&ch, 1);
  return ch;
}

//云台发送数据到底盘
uint8_t format[100] = { 0xAA };
int8_t pitch_cai;
//键位反冲
bool W_recoil = false,A_recoil = false,S_recoil = false,D_recoil = false;
uint8_t W_recoil_frequency = 0,A_recoil_frequency = 0,S_recoil_frequency = 0,D_recoil_frequency = 0;
void Send_Gimbal_to_Chassis()
{
	/*
	帧头 0xAA 8位	
	前进后退符号位 1位 0正 1负
	前进后退 峰值660,压缩到110 倍速6 7位
	左移右移符号位 1位 0正 1负
	左移右移 峰值660,压缩到110 倍速6 7位
	底盘跟随云台误差 32位
	暂停模式 1位 + 底盘跟随云台模式 1位
	*/
	format[1] = format[2] = format[3] = format[5] = 0;//复位
	if(Clicker_MODE == true)//遥控器模式
	{
		format[1] = ((RC_LY) < 0 ? 1 : 0) << 7 | (abs((RC_LY) / 6));
		format[2] = ((RC_LX) < 0 ? 1 : 0) << 7 | (abs((RC_LX) / 6));
		*((int16_t*)&format[3]) = yaw_encoder_angle_e;
		format[5] = ((0) << 7/*保留给停止位*/ | 
			(Clicker_chassis_follow_gimbal == true) << 6 | 
				(Clicker_gimbal_host_chassis == true) << 5 | 
					(Clicker_chassis_gyro == true) << 4 |
						(0) << 3/*保留给键鼠小陀螺*/ | 
					(0) << 2/*刷新ui*/ |
					(0) << 1/*开关摩擦轮*/ | (0) << 0/*仓门开关*/);//停止模式
		*((int16_t*)&format[10]) = yaw_encod;
	}
	else if(MOUSE_MODE == true)//键鼠模式
	{
		if(KEY_PRESSED_OFFSET_W == true)
		{
			format[1] = ((0) << 7 | (660 / 6));
			W_recoil = true;
		}
		//W_反冲
		/*else if(KEY_PRESSED_OFFSET_W == false && W_recoil == true)
		{
			format[1] = ((1) << 7 | (200 / 6));
			W_recoil_frequency++;
			if(W_recoil_frequency==10)
			{
				W_recoil = false;
				W_recoil_frequency=0;
			}
		}*/
		
		else if(KEY_PRESSED_OFFSET_S == true)
		{
			format[1] = ((1) << 7 | (660 / 6));
			S_recoil = true;
		}
		//S_反冲
		/*else if(KEY_PRESSED_OFFSET_S == false && S_recoil == true)
		{
			format[1] = ((0) << 7 | (200 / 6));
			S_recoil_frequency++;
			if(S_recoil_frequency==10)
			{
				S_recoil = false;
				S_recoil_frequency=0;
			}
		}*/
		
		if(KEY_PRESSED_OFFSET_D == true)
		{
			format[2] = ((0) << 7 | (660 / 6));
			D_recoil = true;
		}
		//D_反冲
		/*else if(KEY_PRESSED_OFFSET_D == false && D_recoil == true)
		{
			format[2] = ((1) << 7 | (200 / 6));
			D_recoil_frequency++;
			if(D_recoil_frequency==10)
			{
				D_recoil = false;
				D_recoil_frequency=0;
			}
		}*/
		
		else if(KEY_PRESSED_OFFSET_A == true)
		{
			format[2] = ((1) << 7 | (660 / 6));
			A_recoil=true;
		}
		//A_反冲
		/*else if(KEY_PRESSED_OFFSET_A == false && A_recoil == true)
		{
			format[2] = ((0) << 7 | (200 / 6));
			A_recoil_frequency++;
			if(A_recoil_frequency==10)
			{
				A_recoil = false;
				A_recoil_frequency=0;
			}
		}*/
		
		*((int16_t*)&format[3]) = yaw_encoder_angle_e;
		format[5] = ((0) << 7/*保留给停止位*/ | 
			(Shift_Key_size) << 6/*保留遥控器模式底盘跟随*/ | 
				((MOUSE_gimbal_host_chassis == true) & (Shift_Key_size == false)) << 5 | //ctrl是底盘跟随
				(0) << 4/*保留遥控器模式小陀螺*/ |
				(Q_Key_size) << 3/*键鼠小陀螺*/ | 
				(KEY_PRESSED_OFFSET_CTRL == true) << 2/*刷新ui*/ |
					(Friction_Speed != 0 ? 1 : 0) << 1/*开关摩擦轮*/ | 
				(C_Key_size) << 0/*仓门开关*/);//停止模式
		format[6] = ((R_Key_size) < 0 ? 1 : 0) << 2 | 								
									(bool)(abs(R_Key_size) & 0x02) << 1 | 
									(bool)(abs(R_Key_size) & 0x01) << 0;
									

		format[7] = ((pitch_cai) < 0 ? 1 : 0) << 4 | (int)(abs((pitch_cai)*0.649));

		format[8] = ZX_Key_size;
		format[9] = 0;						
		//format[9] = F_Key_size;
		*((int16_t*)&format[10]) = yaw_encod;
	}
	format[5] |= ((stop_mode == true) | dir) << 7;//停止位
	
	HAL_UART_Transmit_IT(&Send_Gimbal_to_Chassis_Huart, format, Send_Gimbal_to_Chassis_Huart_LEN);
}
int kss = 0,feedfor;
int yaw_init_encoder_angle = 5800;//编码器绝对值初始化


uint16_t aabbi = 0;
uint16_t aabbii = 0;


int yaw_angle_encoder_1,yaw_angle_encoder_2,yaw_angle_encoder_3,yaw_angle_encoder_4,yaw_angle_encoder_5,yaw_angle_encoder_6,yaw_angle_encoder[6] = {3722,5767,7769,1688,2684,6815};
float sintt,angless,sintts,angless01,angless002,bp_feek = 0,last_bp_feek = 0;
int zzz = 0,zzzz= 0,zzout=0,bp_data = 0;


void logic_task(void)
{
		//遥控器通信挂掉
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,((GPIO_PinState)!RM_Clicker::RC_Ctl.Dir));
		//陀螺仪通信挂掉
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,((GPIO_PinState)!chGy_chassis.dir));
		//2006挂掉
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,((GPIO_PinState)!Motor2006.motorData[0].DirFlag));
		//3508挂掉
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,((GPIO_PinState)!(Motor3508.motorData[0].DirFlag | Motor3508.motorData[1].DirFlag)));
		//6020挂掉
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,((GPIO_PinState)!(Motor6020.motorData[0].DirFlag | Motor6020.motorData[1].DirFlag)));

		pitch_cai = (piath_init_encoder_angle - ladrc_pitch.feedback)/22.75555f;
		
		//跟随环
		yaw_angle_encoder_1 = abs(Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle]-Zero_crossing_processing(yaw_angle_encoder[0],Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191));
		yaw_angle_encoder_2 = abs(Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle]-Zero_crossing_processing(yaw_angle_encoder[1],Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191));
		yaw_angle_encoder_3 = abs(Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle]-Zero_crossing_processing(yaw_angle_encoder[2],Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191));
		yaw_angle_encoder_4 = abs(Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle]-Zero_crossing_processing(yaw_angle_encoder[3],Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191));
		yaw_angle_encoder_5 = abs(Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle]-Zero_crossing_processing(yaw_angle_encoder[4],Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191));
		yaw_angle_encoder_6 = abs(Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle]-Zero_crossing_processing(yaw_angle_encoder[5],Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191));
		if(yaw_angle_encoder_1<yaw_angle_encoder_3)yaw_init_encoder_angle = yaw_angle_encoder[0];
		if(yaw_angle_encoder_3<yaw_angle_encoder_1)yaw_init_encoder_angle = yaw_angle_encoder[2];
		if(R_Key_size == 0)
		{
			yaw_encoder_angle_e = Zero_crossing_processing(yaw_init_encoder_angle,Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191) - Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle];//更新底盘误差
		}
		yaw_encod = Zero_crossing_processing(yaw_angle_encoder[2],Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191) - Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle];
		if(R_Key_size == 1)
		{
			if(yaw_angle_encoder_5<yaw_angle_encoder_6)yaw_init_encoder_angle = yaw_angle_encoder[4];
			if(yaw_angle_encoder_6<yaw_angle_encoder_5)yaw_init_encoder_angle = yaw_angle_encoder[5];
			yaw_encoder_angle_e = Zero_crossing_processing(yaw_init_encoder_angle,Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle],8191) - Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle];//更新侧身误差
		}
		
		//遥控器
		if(Clicker_chassis_follow_gimbal == true/*底盘跟随云台*/ || 
			 Clicker_gimbal_host_chassis == true/*主云台模式*/ || 
			 Clicker_chassis_gyro == true/*小陀螺*/)
		{
			if(RC_RX > 15)//移动云台yaw
			{
				yaw_target_add_angle -= 0.15;
				angle_quyu -= 0.1;
			}
			else if(RC_RX < -15)
			{
				yaw_target_add_angle += 0.15;
				angle_quyu += 0.1;
			}
			
//			yaw_target_add_angle = fmod(yaw_target_add_angle,360);//做余数处理
			if(zzzz == 1)yaw_target_add_angle = zzout;
			if(zzzz == 2)yaw_target_add_angle = angless01;
			
			
			if(zzz==2)pitch_target_angle = angless;
			
//			yaw_target_add_angle = fmod(yaw_target_add_angle,360);//做余数处理
			
			#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
					if(RC_RY > 15)//移动云台pitch
					{
						pitch_target_angle += 0.5;
					}
					else if(RC_RY < -15)
					{
						pitch_target_angle -= 0.5;
					}
					//ptich极限判断
					if(pitch_target_angle > (PITCH_On_limit+piath_init_encoder_angle) )
					{
						pitch_target_angle = (PITCH_On_limit+piath_init_encoder_angle);
					}
					else if(pitch_target_angle < (PITCH_Lower_limit+piath_init_encoder_angle))
					{
						pitch_target_angle = (PITCH_Lower_limit+piath_init_encoder_angle);
					}
			#elif PITCH__GET_SIG == PITCH_GY
					//pitch
					if(RC_RY > 15)//移动云台pitch
					{
						pitch_target_angle -= 0.1;
					}
					else if(RC_RY < -15)
					{
						pitch_target_angle += 0.1;
					}
					//ptich极限判断
					if(pitch_target_angle < PITCH_On_limit)
					{
						pitch_target_angle = PITCH_On_limit;
					}
					else if(pitch_target_angle > PITCH_Lower_limit)
					{
						pitch_target_angle = PITCH_Lower_limit;
					}
			#endif			
			
		}
		//键鼠
		else if(MOUSE_chassis_follow_gimbal == true/*底盘跟随云台*/ || 
				    MOUSE_gimbal_host_chassis == true/*主云台模式*/ || 
				    MOUSE_chassis_gyro == true/*小陀螺*/)
		{
			
			
			if(MOUSE_X != 0)//移动云台yaw
			{
				yaw_target_add_angle += MOUSE_X * MOUSE_X_K;
				angle_quyu += MOUSE_X * MOUSE_X_K;
			}
			
//			yaw_target_add_angle = fmod(yaw_target_add_angle,360);//做余数处理
			
			#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
					if(MOUSE_Y != 0 && _Motor6020_[1].Data[3]<85)//移动云台pitch
					{
						pitch_target_angle += MOUSE_Y * MOUSE_Y_K;
					}
					else if(_Motor6020_[1].Data[3]>85)
					{
						pitch_target_angle = piath_init_encoder_angle;
					}
					//ptich极限判断
					if(pitch_target_angle > (PITCH_On_limit+piath_init_encoder_angle) )
					{
						pitch_target_angle = (PITCH_On_limit+piath_init_encoder_angle);
					}
					else if(pitch_target_angle < (PITCH_Lower_limit+piath_init_encoder_angle))
					{
						pitch_target_angle = (PITCH_Lower_limit+piath_init_encoder_angle);
					}
			#elif PITCH__GET_SIG == PITCH_GY
					if(MOUSE_Y != 0)//移动云台pitch
					{
						pitch_target_angle -= MOUSE_Y * MOUSE_Y_K * 0.1;
					}
					//ptich极限判断
					if(pitch_target_angle < PITCH_On_limit)
					{
						pitch_target_angle = PITCH_On_limit;
					}
					else if(pitch_target_angle > PITCH_Lower_limit)
					{
						pitch_target_angle = PITCH_Lower_limit;
					}
			#endif
			
			//V_Key_size +=V_Key.RisingEdge;
			//V_Key_size %= 2;
//			if(V_Key_size == 0)
//			{
				B_Key.UpKey(KEY_PRESSED_OFFSET_B);//b键更新
			
				G_Key.UpKey(KEY_PRESSED_OFFSET_G);//g键更新
				
				if(B_Key.FallingEdge == true)//b键下降沿
				{
					Friction_Speed = 0;
					Clicker_Dial_Key_size = 0;
					Clicker_Friction_Key_size = 0;
				}
				
//			}
	
			Clicker_Friction_Key.UpKey(Clicker_S2_Dial_up);//遥控器控制摩擦轮
			Clicker_Friction_Key_size += Clicker_Friction_Key.FallingEdge;//4次开，4次关
			Clicker_Friction_Key_size %= 2;//遥控器控制摩擦轮更新次数取余			
			
			Clicker_Dial_Key.UpKey(Clicker_S2_Dial_down);//遥控器控制拨盘
			if(Clicker_Dial_Key.FallingEdge==1)
			{
				Clicker_Dial_Key_size=1;
			}
//			Clicker_Dial_Key_size += Clicker_Dial_Key.FallingEdge;//4次开，4次关
//			Clicker_Dial_Key_size %= 2;//遥控器控制拨盘更新次数取余
			
			C_Key.UpKey(KEY_PRESSED_OFFSET_C);//c键更新
			V_Key.UpKey(KEY_PRESSED_OFFSET_V);//v键更新
			if(C_Key.RisingEdge == 1)
			{
				C_Key_size = 1;
				servos_angle = 360;//开仓门
			}
			if(V_Key.RisingEdge == 1)
			{
				C_Key_size = 0;
				servos_angle = 180;//开仓门
			}
			if(KEY_PRESSED_OFFSET_CTRL == true)
			{
				C_Key_size = 0;
				servos_angle = 180;//开仓门
			}
			
			Q_Key.UpKey(KEY_PRESSED_OFFSET_Q);//q键更新
			Q_Key_size += Q_Key.RisingEdge;//q键更新次数
			Q_Key_size %= 2;//q键更新次数取余			
			


			Shift_Key.UpKey(KEY_PRESSED_OFFSET_SHIFT);//Shift键更新
			Shift_Key_size = KEY_PRESSED_OFFSET_SHIFT;//Shift键更新次数
			if(Shift_Key_size==1)
			{
				R_Key_size =0;
			}
//			Shift_Key_size += Shift_Key.RisingEdge;//Shift键更新次数
//			Shift_Key_size %= 2;//f键更新次数取余
			
			if(Q_Key.RisingEdge)Shift_Key_size = false;//q键清除Shift键
			if(Shift_Key.RisingEdge)Q_Key_size = false;//Shift键清除q键
			
			
			E_Key.UpKey(KEY_PRESSED_OFFSET_E);//E键更新	
			if(E_Key.RisingEdge==1)
			{
				yaw_target_add_angle+=180;
				
			}
			F_Key.UpKey(KEY_PRESSED_OFFSET_F);
			F_Key_size +=F_Key.FallingEdge;
			F_Key_size %=2;
//			E_Key.UpKey(KEY_PRESSED_OFFSET_E);//E键更新	
			R_Key.UpKey(KEY_PRESSED_OFFSET_R);//R键更新
			R_Key_size += R_Key.RisingEdge;
			R_Key_size %=2;
//			ER_Key_size -= E_Key.RisingEdge;//er键更新次数
//			ER_Key_size += R_Key.RisingEdge;//er键更新次数
//			if(ER_Key_size >= 3)ER_Key_size = 3;//er键限幅
//			else if(ER_Key_size < -2)ER_Key_size = -2;//er键限幅
//			if(KEY_PRESSED_OFFSET_CTRL == true)ER_Key_size = 0;//摁下ctrl清除er
			
			Z_Key.UpKey(KEY_PRESSED_OFFSET_Z);//Z键更新
			X_Key.UpKey(KEY_PRESSED_OFFSET_X);//X键更新
			ZX_Key_size -=Z_Key.RisingEdge; //ZX键更新次数
			ZX_Key_size += X_Key.RisingEdge;//ZX键更新次数
			if(ZX_Key_size >= 6)ZX_Key_size = 6;//ZX键限幅
			else if(ZX_Key_size <= 0)ZX_Key_size = 0;//ZX键限幅
//			if(KEY_PRESSED_OFFSET_CTRL == true)ZX_Key_size = 0;//摁下ctrl清除ZX
			
			

			
//			if(G_Key.FallingEdge == true || Clicker_Friction_Key_size == true || V_Key_size == 1/*遥控器*/ /*v键*/)//g键下降沿
//			{
//				Friction_Speed = Friction_MAX_Speed;//设置摩擦轮速度
//			}
//			
//			if((MOUSE_LDOWN == true && V_Key_size == 0 && Friction_Speed == Friction_MAX_Speed) || Clicker_Dial_Key_size == true || (vision_data.vision_transmit ==1 && V_Key_size == 1 && MOUSE_RDOWN == true)/*遥控器*//*视觉自动*/)//长按鼠标左键开拨盘，否则关闭
//			{
//				Dial_Speed = Dial_MAX_Speed;
//			}
//			else
//			{
//				Dial_Speed = 0;
//			}
				if(G_Key.FallingEdge == true || Clicker_Friction_Key_size == true/*遥控器*/ /*v键*/)//g键下降沿
			{
				Friction_Speed = Friction_MAX_Speed;//设置摩擦轮速度
			}
//			Friction_MAX_Speed
			if((MOUSE_LDOWN == true) && (Friction_Speed == Friction_MAX_Speed)/*左键*/)//长按鼠标左键开拨盘，否则关闭
			{
				Dial_Speed+=420;
			}
			else if((MOUSE_LDOWN == false) && (MOUSE_RDOWN == true) && (vision_data.vision_fire == 1) && (Friction_Speed == Friction_MAX_Speed))
			{
				Dial_Speed+=32764;
				vision_data.vision_fire = 0;
			}
			else if((Clicker_Dial_Key_size == true)/*遥控器*/)
			{
				if(RC_LX>0)
				{
					Dial_Speed += RC_LX*0.8;
				}
				else if(RC_LX<0)
				{
					Dial_Speed += RC_LX*-0.8;
				}
			}
		}
		else//非运行状态
		{
			if(CM_MODE == true)servos_angle = 360;
			else servos_angle = 180;
			Clicker_Friction_Key_size = Clicker_Dial_Key_size = 0;//复位摩擦轮和拨盘次数
			Friction_Speed  = 0;//复位摩擦轮和拨盘速度
		}

		
		//yaw
		#if GY_GET_SIG == GY_CAN
			yaw_zz_pid_s.pid_iso_no(7);
			yaw_zz_pid_v.pid_feedback_feedforward(30);
		#elif GY_GET_SIG == GY_232
		
		#endif
		
		yaw_target_angle_temp = yaw_target_angle = YAW_Init_Angle + yaw_target_add_angle;//初始角度+增加的角度
		
//		//过零处理
//		#if GY_GET_SIG == GY_CAN
//			yaw_target_angle_temp = Zero_crossing_processing(yaw_target_angle,chGy_chassis.gy.z_360,360);
//			yaw_pid.GetPidPos(yaw_kpid,yaw_target_angle_temp,chGy_chassis.gy.z_360,30000);//角度环
//				
//		#elif GY_GET_SIG == GY_232
//			yaw_target_angle_temp  = Zero_crossing_processing(yaw_target_angle,chGy_chassis.data.z_360,360);
//			yaw_pid.GetPidPos(yaw_kpid,yaw_target_angle_temp,chGy_chassis.data.z_360,30000);//角度环
//		
//		#endif
//		
//		td_yaw_i.td_quadratic(yaw_pid.pid.i);//跟踪积分状态
//			
//		yaw_send_data = yaw_pid.pid.cout - (td_yaw_i.x2) * yaw_i_kd;//减去积分导致的超调

//		yaw_target_add_angle = Zero_crossing_processing(yaw_target_add_angle,chGy_chassis.gy.Add_z,360);
//		td_yaw_current.td_quadratic(_Motor6020_[0].Data[2]);
		//yaw_zz_pid_s.pid_mb_feedforward();
		if(abs(yaw_target_add_angle - chGy_chassis.gy.Add_z)>600)
		{
			yaw_target_add_angle = chGy_chassis.gy.Add_z;
		}
		
		
		if(Q_Key_size==1)
		{
			yaw_VW_Feedforward.calc(1170+(350*ZX_Key_size));
		}
		else
		{
			yaw_VW_Feedforward.out = 0;
		}
		yaw_Feedforward.D_value_calc(yaw_target_add_angle);
		yaw_zz_pid_s.pid_calc(chGy_chassis.gy.Add_z,yaw_target_add_angle);
			
//		x = yaw_zz_pid_s.error[0];	
//		x_out = a0 + a1*cos(x*w) + b1*sin(x*w) + a2*cos(2*x*w) + b2*sin(2*x*w) + a3*cos(3*x*w) + b3*sin(3*x*w) + a4*cos(4*x*w) + b4*sin(4*x*w) + a5*cos(5*x*w) + b5*sin(5*x*w) + a6*cos(6*x*w) + b6*sin(6*x*w);
//			
//			aabbii+=1;
//		if(aabbi<=1000&&aabbii == 10)
//		{
//			aabb[aabbi] = yaw_zz_pid_s.error[0];
//			aacc[aabbi] = yaw_zz_pid_s.out;
//			aabbi+=1;
//			aabbii=0;
//		}
		
		yaw_zz_pid_v.pid_calc(chGy_chassis.gy.ASz,(yaw_zz_pid_s.out));
//		if(yaw_zz_pid_v.out<0)
//		{
//			yaw_Feedforward.out*=-1;
//		}
//		else
//		{
//			yaw_Feedforward.out*=1;
//		}

		yaw_send_data = (yaw_zz_pid_v.out+yaw_Feedforward.out+yaw_VW_Feedforward.out);
		
			
//		RM_yaw_UDE.out_torque+=(((yaw_zz_pid_v.out*0.546133)*0.741)*0.00001)*RM_yaw_UDE.B;
//		RM_yaw_UDE.UDE_calc((td_yaw_cin.x1*9.549296));	
			
			
//		RM_yaw_UDE.out_torque+=yaw_zz_pid_s.out;

		
		
//		ladrc_yaw_angle.up_data(yaw_target_add_angle,chGy_chassis.gy.Add_z,2,true);//yaw的adrc控制
		
		

//		zz_send_data=ladrc_yaw_angle.u;
		
		//yaw_send_data = ladrc_yaw_angle.u + ff_add_angle.feedForward.cout + ff_Clicker_add_angle.feedForward.cout + ff_Mouse_add_angle.feedForward.cout;//adrc+前馈补偿
		
		if(MOUSE_chassis_gyro & Clicker_chassis_gyro)is_open_pitch_offsets_gy = false;//开小陀螺不补偿
		else is_open_pitch_offsets_gy = true;//不开陀螺仪开补偿
		//pitch补偿
		//		pitch_offsets_gy = (td_gy_y.x1 * 22.257777 - piath_init_gy_angle) * is_open_pitch_offsets_gy;//滞后不太可用
//		pitch_offsets_gy = xs * e_angle[(int)(Motor6020.GetMotor(YAW_MOTOR_ID)->Data[Motor_Data_Angle] * 0.04394)] * 22.7577;
		#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
				//pitch
				// ladrc_pitch.up_data(piath_init_encoder_angle + pitch_target_angle + pitch_offsets_gy,Motor6020.GetMotor(PITCH_MOTOR_ID)->Data[Motor_Data_Angle],2,false);
				pitch_zz_pid_s.pid_ABiso(300,40);

				td_pitch_cin.td_quadratic(_Motor6020_[1].Data[1]);
				pitch_zz_pid_s.pid_calc(_Motor6020_[1].Data[0],pitch_target_angle);
				pitch_zz_pid_v.pid_calc(td_pitch_cin.x1,pitch_zz_pid_s.out);

		#elif PITCH__GET_SIG == PITCH_GY


		#endif
				
		//拨盘
		bp_zz_pid_s.pid_mb_feedforward(10);
		td_bp_cin.td_quadratic(_Motor2006_[0].Data[1]);
		if(last_bp_feek != _Motor2006_[0].Data[0] && 
			 last_bp_feek != -1)
		{
				float lastData = last_bp_feek;
				float Data = _Motor2006_[0].Data[0];
				if(Data - lastData < -(8191*0.5))//正转
					bp_feek += (8191 - lastData + Data);
				else if(Data - lastData > (8191*0.5))//反转
					bp_feek += -(8191 - Data + lastData);
				else 
					bp_feek += (Data - lastData);
		}
		last_bp_feek = _Motor2006_[0].Data[0];
		
		bp_zz_pid_s.pid_calc(bp_feek,-Dial_Speed);
		bp_zz_pid_v.pid_calc(td_bp_cin.x1,bp_zz_pid_s.out);
//		ladrc_dial.up_data(-Dial_Speed,bp_feek,2);
		if(abs(_Motor2006_[0].Data[2])>12000)
		{
				bp_data = 1;
		}
		if(bp_data==1)
		{
			bp_zz_pid_v.out = 0;
		}

		//左摩擦轮
		ladrc_friction_l.up_data(Friction_Speed,Motor3508.GetMotor(Friction_MOTOR_L_ID)->Data[Motor_Data_Speed],2);
		
		//右摩擦轮
		ladrc_friction_r.up_data(-Friction_Speed,Motor3508.GetMotor(Friction_MOTOR_R_ID)->Data[Motor_Data_Speed],2);
		
		if((stop_mode == true) | (dir == true))//停止模式
		{
			yaw_pid.clearPID();//清空pid的所有数据
			setMSD(&msd_6020,0,1);
			setMSD(&msd_6020,0,2);
			setMSD(&msd_3508_2006,bp_zz_pid_v.out,3);//拨盘
			setMSD(&msd_3508_2006,ladrc_friction_l.u,1);//左摩擦轮
			setMSD(&msd_3508_2006,ladrc_friction_r.u,2);//右摩擦轮
		}
		else
		{
			//设置发送数据
			setMSD(&msd_6020,yaw_send_data,1);//yaw
			#if PITCH__GET_SIG == PITCH_ENCODER//设置pitch控制信号
					//pitch
					
					setMSD(&msd_6020,pitch_zz_pid_v.out,2);//pitch
			#elif PITCH__GET_SIG == PITCH_GY
					//pitch
//					setMSD(&msd_6020,-ladrc_pitch.u,2);//pitch
				setMSD(&msd_6020,pitch_pid.pid.cout,2);//pitch
			#endif			
			setMSD(&msd_3508_2006,bp_zz_pid_v.out,3);//拨盘
			setMSD(&msd_3508_2006,ladrc_friction_l.u,1);//左摩擦轮
			setMSD(&msd_3508_2006,ladrc_friction_r.u,2);//右摩擦轮
		}
}

void Send_Gimbal_to_Chassis_task(void)
{
//			sintt += 0.001;
//			angless = 500 * sinf(2.0 * 3.1415926 * sintt * sintts * 0.8) * sinf(2.0 * 3.1415926 * sintt * sintts * 0.2) * sinf(2.0 * 3.1415926 * sintt * sintts * 0.5);
//			angless01 = 100*sinf(2.0 * 3.1415926 * sintt * sintts * 0.8) * sinf(2.0 * 3.1415926 * sintt * sintts * 0.2) * sinf(2.0 * 3.1415926 * sintt * sintts * 0.5);
//		angless01 = 100*sinf(2.0*3.1415926*sintt*sintts*0.8);
		//发送数据500 * sin(2.0 * 3.1415926 * x * 1 * 0.8) * sin(2.0 * 3.1415926 * x * 1 * 0.2) * sin(2.0 * 3.1415926 * x * 1 * 0.5)
		if(send_motor_size == 0)
		{
			Send_6020_CAN();
		}
		else
		{
			Send_3508_2006_CAN();
		}
		send_motor_size++;
		send_motor_size %= 2;
		//给底盘发送数据
		Send_Gimbal_to_Chassis();
}


void vision_task(void)
{
		vision_go_Computer();
		if(vision_data.vision_bool ==1)
		{
			vision_to_computer();
			vision_data.vision_bool = 0;
		}
//		vision_send_Computer();
//		vision_recive_task();
		if(__HAL_UART_GET_FLAG(&vision_Huart,UART_FLAG_ORE)!=RESET)
		{
			__HAL_UART_CLEAR_OREFLAG(&vision_Huart);
			HAL_UARTEx_ReceiveToIdle_DMA(&vision_Huart,vision_data.vision_RX_data,13);
		}
}



void vision_go_Computer(void)
{
	vision_data.pitch_360 = _Motor6020_[1].Data[0];
	vision_data.yaw_360 = (chGy_chassis.gy.Add_z*100);
	vision_data.vision_TX_data[0] = 0x39;
	vision_data.vision_TX_data[1] = 0x39;
	vision_data.vision_TX_data[2] = (int32_t)vision_data.pitch_360>>24;
	vision_data.vision_TX_data[3] = (int32_t)vision_data.pitch_360>>16;
	vision_data.vision_TX_data[4] = (int32_t)vision_data.pitch_360>>9;
	vision_data.vision_TX_data[5] = (int32_t)vision_data.pitch_360;
	
	
	vision_data.vision_TX_data[6] = (int32_t)vision_data.yaw_360>>24;
	vision_data.vision_TX_data[7] = (int32_t)vision_data.yaw_360>>16;
	vision_data.vision_TX_data[8] = (int32_t)vision_data.yaw_360>>8;
	vision_data.vision_TX_data[9] = (int32_t)vision_data.yaw_360;
	vision_data.vision_TX_data[10] = 26;
	vision_data.vision_TX_data[11] = 0x42; //0x42红   0X52蓝色
	vision_data.vision_TX_data[12] = 0;
	vision_data.vision_TX_data[13] = 0XFF;
	HAL_UART_Transmit_DMA(&vision_Huart,vision_data.vision_TX_data,14);
}

void vision_to_computer(void)
{
	if(vision_data.vision_RX_data[0]== 0x39 && vision_data.vision_RX_data[1] == 0x39 && vision_data.vision_RX_data[12] == 0xFF)
	{
		vision_data.vision_pitah = vision_data.vision_RX_data[2]<<24 |vision_data.vision_RX_data[3]<<16 |vision_data.vision_RX_data[4]<<8 | vision_data.vision_RX_data[5];
		vision_data.vision_yaw = vision_data.vision_RX_data[6]<<24 |vision_data.vision_RX_data[7]<<16 |vision_data.vision_RX_data[8]<<8 | vision_data.vision_RX_data[9];
		vision_data.vision_ready = vision_data.vision_RX_data[10];
		vision_data.vision_fire = (vision_data.vision_RX_data[11]);
		
	}
	if(MOUSE_RDOWN == true && vision_data.vision_ready==1)
	{
		
//		yaw_target_add_angle = Zero_crossing_processing(((vision_data.vision_yaw/100)+yaw_zz_pid_s.feedback),yaw_zz_pid_s.feedback,360);
			yaw_target_add_angle = ((vision_data.vision_yaw/100)+yaw_zz_pid_s.feedback);
//		yaw_target_add_angle += ((vision_data.vision_yaw)/angle_num)*0.04;
//		yaw_target_add_angle = Zero_crossing_processing((vision_data.vision_yaw/22.75555f),yaw_zz_pid_s.feedback,360);
		pitch_target_angle = _Motor6020_[1].Data[0]+vision_data.vision_pitah;
		
	}
}



uint8_t yaw_angle = 0;



void Gpio_led_init(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();
	
		GPIO_InitTypeDef GPIO_InitStruct = {0};
	
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}


