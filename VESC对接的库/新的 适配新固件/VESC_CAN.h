/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2024-01-28 09:12:06
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2024-01-29 10:40:19
 * @FilePath: \MDK-ARM\RM3508\VESC_CAN.h
 * @Description: 
 * VESC_CAN 1.0 曹总写的库 回传有问题，发送和设置模式分开 10.28.2021
 * VESC_CAN 2.0 GTY改写   可自定义回传，将命令函数分为单独的函数，与RM3508的库类似 1.29.2024
 * 适配新固件使用 60_LIMITI_mk5 v1.1beta3 及以上
 * CAN ID     尽量设置在1~8内，因为初始储存状态的数组只够前八个ID存
 * CAN Status 回传设置如下：
 * Status1：  速度 占空比
 * Status2：  单圈位置 多圈位置（需要编码器）
 * Status3：  总电流 总电压 计算得到总功率
 * 更多Status 可定制
 * Todo：     写一个编码器多圈重置功能
 * Copyright (c) 2024 by UESTC_LIMITI, All Rights Reserved. 
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

#define ALL_VESC 255    //置255是给所有VESC发命令
#define MAX_CURRENT 70  //只是限幅电流命令，可改
//#define FLOAT_TRANSMITTED


#define DUTY_CYCLE_SCALE                    1e3
#define CURRENT_SCALE                       1e3
#define VOLTAGE_SCALE                       1e3
#define RPM_SCALE                           1e0   
#define POS_SCALE                           1e6

typedef enum {                              //VESC里定义的CAN数据包类型
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

typedef struct{                             // VESC回传信息           
	float rpm;	   	  	                    // 转速
	float duty_cycle;                       // 占空比
	float pos;         	                    // 位置
	float mul_pos;                          // 多圈位置
	float tot_current_in;                   // 总电流
	float tot_voltage;                      // 总电压
	float power;                            // 总功率 = V*I
} motor_info_t;

typedef enum {
	CAN_SendData_TimeOut,
	Set_id_Wrong,
}VESC_ErrorCode_t;

extern motor_info_t motor_info[8];
extern uint8_t VESC_Send_Buffer[4];

/*CAN接收函数,应将其写入回调函数内*/
float uchar2float(uint8_t* buffer, uint32_t *index);
int32_t uchar2int32(uint8_t* buffer, uint32_t *index);

bool VESC_CAN_decode(uint32_t ExtID, uint8_t *pData);
bool VESC_CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t id, uint32_t eid);

bool VESC_SetRPM(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetDutyCycle(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetCurrent(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetCurrentBrake(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetPos(float value, uint8_t id, CAN_HandleTypeDef *hcan);

static void VESC_Error_Handler(VESC_ErrorCode_t Code);

#endif
