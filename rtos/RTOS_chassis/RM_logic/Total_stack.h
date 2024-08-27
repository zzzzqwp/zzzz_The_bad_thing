
#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus
	
	//主跑初始化
	void Total_tasks_Init();
	
	//主跑函数
	void Total_tasks_Run();
	
	//发送3508数据
	void Send_3508_CAN();
	
	
	void Chassis_Task();

	
#ifdef __cplusplus
}

#endif  // __cplusplus
