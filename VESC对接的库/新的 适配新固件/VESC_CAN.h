/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2024-01-28 09:12:06
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2024-01-28 09:42:26
 * @FilePath: \VESC_Code\VESC对接的库\新的 适配新固件\VESC_CAN.h
 * @Description: 
 * 
 * Copyright (c) 2024 by UESTC_LIMITI, All Rights Reserved. 
 */

/*
	与VESC CAN通信对接的函数库
	updated:2021-10-28
	希望以后调这个库的人不要骂我
*/

#ifndef __VESC_CAN_H__
#define __VESC_CAN_H__

#include "can.h"
#include "stdint.h"
#include "can_bsp.h"
#include "main.h"
#include "stdbool.h"
#include "string.h"

#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))
#define ALL_VESC 255
#define MAX_CURRENT 70

#define DUTY_CYCLE_SCALE                    1e5
#define CURRENT_SCALE                       1e3
#define RPM_SCALE                           1e0   
#define POS_SCALE                           1e6

typedef enum {
	CAN_PACKET_SET_DUTY						= 0,
	CAN_PACKET_SET_CURRENT					= 1,
	CAN_PACKET_SET_CURRENT_BRAKE			= 2,
	CAN_PACKET_SET_RPM						= 3,
	CAN_PACKET_SET_POS						= 4,
	CAN_PACKET_STATUS                       = 9,
	CAN_PACKET_STATUS_2						= 14,
	CAN_PACKET_STATUS_3						= 15,
	CAN_PACKET_STATUS_4						= 16,
	CAN_PACKET_SHUTDOWN						= 31,
} CAN_PACKET_ID;

typedef struct
{
	float rpm;	   	  	   // 转速
	float duty_cycle;      // 占空比
	float pos;         	   // 位置
	float mul_pos;         // 多圈位置
	float tot_current_in;  // 总电流
	float tot_voltage;     // 总电压
	float power;

} motor_info_t;

typedef enum {
	CAN_SendData_TimeOut,
	Set_id_Wrong,
}VESC_ErrorCode_t;

extern motor_info_t motor_info[8];
extern uint8_t VESC_Send_Buffer[4];

/*CAN接收函数,应将其写入回调函数内*/
float uchar2float(uint8_t* buffer, uint32_t *index);

bool VESC_CAN_decode(uint32_t ExtID, uint8_t *pData);
bool VESC_CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t id, uint32_t eid);

bool VESC_SetRPM(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetDutyCycle(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetCurrent(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetCurrentBrake(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetPos(float value, uint8_t id, CAN_HandleTypeDef *hcan);

static void VESC_Error_Handler(VESC_ErrorCode_t Code);

#endif
