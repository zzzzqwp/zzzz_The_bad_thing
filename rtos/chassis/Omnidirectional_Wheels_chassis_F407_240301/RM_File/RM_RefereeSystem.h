#pragma once
#include "usart.h"
#include "RM_RefereeSystemCRC.h"
#define Win_W (1920)  //x
#define Win_H (1080)  //y
namespace RM_RefereeSystem
{
//操作
enum RM_Operate
{
	//空，添加，修改，删除
	OperateNull,OperateAdd,OperateRevise,OperateDelete
};
//删除操作
enum RM_DeleteOperate
{
	//空，删除，删除全部
	DeleteNull,DeleteOne,DeleteAll
};
//类型
enum RM_Type
{
	//直线，矩形，圆，椭圆，圆弧，浮点，整形，字符
	TypeLine,TypeRectangle,TypeCircle,TypeElliptic,TypeArced,TypeFloat,TypeInt,TypeStr
};
//颜色
enum RM_Color
{
	//红与蓝，黄色，绿色，橙色，紫红色，粉色，青色，黑色，白色
	ColorRedAndBlue,ColorYellow,ColorGreen,ColorOrange,ColorAmaranth,ColorPink,ColorCyan,ColorBlack,ColorWhite
};
//颜色,线粗,字体大小
typedef struct
{
	int width;
	int color;
	int strsize;
	int operate_tpye;
}RM_RefereeSystemToop_t;
typedef __packed struct 
{
	uint8_t SOF;//数据帧起始字节，固定值为 0xA5
	uint16_t data_length;//数据帧中 data 的长度
	uint8_t seq;//包序号
	uint8_t CRC8;//帧头 CRC8 校验
	uint16_t cmd_id;//命令码 ID
	uint8_t data[6+105];//数据
	uint16_t frame_tail;//CRC16，整包校验
}RM_RefereeSystemData_t;
//0x0201 机器人状态数据
typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP; 
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1; 
	uint8_t power_management_shooter_output : 1;
}ext_game_robot_status_t;
//0x0202 实时功率热量数据
typedef __packed struct
{
	uint16_t chassis_volt; 
	uint16_t chassis_current; 
	float chassis_power; 
	uint16_t chassis_power_buffer; 
	uint16_t shooter_id1_17mm_cooling_heat;
	uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
}ext_power_heat_data_t;
//0x0301 机器人间交互数据
typedef __packed struct //交互数据接收信息：0x0301
{ 
  uint16_t data_cmd_id;  
  uint16_t sender_ID;  
  uint16_t receiver_ID; 
}ext_student_interactive_header_data_t;
typedef __packed struct //客户端删除图形 机器人间通信：0x0301
{ 
	uint8_t operate_tpye;  
	uint8_t layer;
}ext_client_custom_graphic_delete_t ;
typedef __packed struct //图形数据
{  
	uint8_t graphic_name[3];  
	uint32_t operate_tpye:3;  
	uint32_t graphic_tpye:3;  
	uint32_t layer:4;  
	uint32_t color:4;  
	uint32_t start_angle:9; 
	uint32_t end_angle:9; 
	uint32_t width:10;  
	uint32_t start_x:11;  
	uint32_t start_y:11;  
	uint32_t radius:10;  
	uint32_t end_x:11;  
	uint32_t end_y:11;  
}graphic_data_struct_t;
typedef __packed struct //客户端绘制一个图形 机器人间通信：0x0301 
{ 
  graphic_data_struct_t  grapic_data_struct; 
} ext_client_custom_graphic_single_t;
typedef __packed struct //客户端绘制字符 机器人间通信：0x0301 
{ 
	graphic_data_struct_t  grapic_data_struct; 
	uint8_t data[30]; 
} ext_client_custom_character_t;
typedef __packed struct//选手端小地图交互数据，选手端触发发送
{
	float target_position_x;
	float target_position_y;
	float target_position_z;
	uint8_t commd_keyboard;
	uint16_t target_robot_id;
}map_command_t;

typedef __packed struct
{
uint16_t target_robot_id;
float target_position_x;
float target_position_y;
}map_robot_data_t;
//初始化
void RM_RefereeSystemInit();
//设置颜色
void RM_RefereeSystemSetColor(int color);
//设置线粗
void RM_RefereeSystemSetWidth(int width);
//设置字体
void RM_RefereeSystemSetStringSize(int size);
//设置图形操作
void RM_RefereeSystemSetOperateTpye(int operate_tpye);
//工具清空
void RM_RefereeSystemClsToop();
//数据解析
void RM_RefereeSystemParseData(uint8_t* RM_pDatas,int size);
//消息获取
void RM_RefereeSystemGetData(uint8_t RM_pData);
//数据发送客户端绘制删除N个图层
void RM_RefereeSystemDelete(const char operate,const char number);
//数据发送客户端绘制1,2,5,7个图形
void RM_RefereeSystemSendDataN(const graphic_data_struct_t graphic_data_struct[],int size);
//数据发送客户端绘制字符串
void RM_RefereeSystemSendStr(const ext_client_custom_character_t ext_client_custom_character);

//机器人获取id
int RM_RefereeSystemGetRobotId();
//绘画直线/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetLine(char* name,uint32_t layer,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
//绘画矩形/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetRectangle(char* name,uint32_t layer,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
//绘画圆/*名字，图层，x1，y1，半径*/
graphic_data_struct_t RM_RefereeSystemSetCircle(char* name,uint32_t layer,uint32_t start_x,uint32_t start_y,uint32_t radius);
//绘画椭圆/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetElliptic(char* name,uint32_t layer,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
//绘画圆弧/*名字，图层，起始角度，终止角度，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetArced(char* name,uint32_t layer,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y);
//绘画浮点数/*名字，图层，浮点数，浮点数长度，x1，y1*/
graphic_data_struct_t RM_RefereeSystemSetFzloat(char* name,uint32_t layer,float x,uint32_t start_x,uint32_t start_y);
//绘画整数/*名字，图层，整数，浮点数长度，x1，y1*/
graphic_data_struct_t RM_RefereeSystemSetInt(char* name,uint32_t layer,int32_t x,uint32_t start_x,uint32_t start_y);
//绘画字符串/*名字，图层，字符串，x1，y1*/
ext_client_custom_character_t RM_RefereeSystemSetStr(char* name,uint32_t layer,char* str,uint32_t start_x,uint32_t start_y);
//断连
bool RM_RefereeSystemDir();
//解析
void RM_RefereeSystemParse(UART_HandleTypeDef *huart);
}
//单字节数据
extern uint8_t RM_RefereeSystemp8Data;
//断连判断
extern bool RM_RefereeSystemDirFlag;
//0x0201 机器人状态数据
extern RM_RefereeSystem::ext_game_robot_status_t ext_power_heat_data_0x0201;
//0x0202 实时功率热量数据
extern RM_RefereeSystem::ext_power_heat_data_t ext_power_heat_data_0x0202;
//0x0301 机器人间交互数据
extern RM_RefereeSystem::graphic_data_struct_t graphic_data_struct_0x0301;
//0x0303 机器人间交互数据
extern RM_RefereeSystem::map_command_t map_command_0x0303;


//发送数据
extern uint8_t tx_buf[128];
extern uint8_t rrrdata[25];
extern uint8_t daohang_tx_buf[128];