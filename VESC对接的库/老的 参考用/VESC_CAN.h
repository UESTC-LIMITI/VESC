
/*
	与VESC CAN通信对接的函数库
	updated:2021-10-28
	希望以后调这个库的人不要骂我
*/

#ifndef __VESC_CAN__
#define __VESC_CAN__

#include "can.h"
#include "stdint.h"
#include "can_bsp.h"

/*所连接的VESC最大id值
id在VESC TOOL里设置
建议设置的小一点*/
#define VESC_MAX_ID 4

#define VESC_SET_DUTY 0
#define VESC_SET_CURRENT 1
#define VESC_SET_CURRENT_BRAKE 2
#define VESC_SET_RPM 3
#define VESC_SET_POSITION 4

#define VESC_GET_INFO 7
//#define VESC_GET_CURRENT (0xffffff & (1 << 7)) //0000 0000 0000 0000 0000 0000 0000 0000
#define VESC_GET_RPM (0xffffff & (1 << 7))     //0000 0000 0000 0000 0000 0000 1000 0000
#define VESC_GET_DUTY (0xffffff & (1 << 6))    //0000 0000 0000 0000 0000 0000 0100 0000
#define VESC_GET_POS (0xffffff & (1 << 16))    //0000 0000 0000 0001 0000 0000 0000 0000

#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

typedef struct
{
	double cur_current;   // 电流
	double cur_rpm;	   // 转速
	double cur_duty;      // 占空比
	double cur_pos;       // 位置

} Motor_INFO;



void VESC_COMMAND_SET(uint32_t cmd, uint8_t id, float set_value);
void VESC_CAN_SENDDATA(CAN_HandleTypeDef *hcan, uint8_t id);
/*CAN接收函数,应将其写入回调函数内*/
void VESC_CAN_DECODE(uint32_t ExtID, uint8_t pData[]);




double VESC_DECODE_RPM(uint8_t pData[]);
double VESC_DECODE_CURRENT(uint8_t pData[]);

extern Motor_INFO VESC_Feedback[VESC_MAX_ID + 1];


#endif
