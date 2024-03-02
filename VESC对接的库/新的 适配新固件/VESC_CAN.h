/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2024-01-28 09:12:06
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2024-02-20 11:04:31
 * @FilePath: \MDK-ARM\Motor\VESC_CAN.h
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
#include "buffer.h"

#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

#define ALL_VESC 255    //置255是给所有VESC发命令
#define MAX_CURRENT 70  //只是限幅电流命令，可改
#define MAX_MULTITURN 2147483 //计算得出当前多圈的scale能够覆盖这么多角度范围
//#define FLOAT_TRANSMITTED
#define BUFFER_MAX_LENTH 8

#define DUTY_CYCLE_SCALE                    1e5
#define CURRENT_SCALE                       1e3
#define VOLTAGE_SCALE                       1e3
#define RPM_SCALE                           1e0   
#define POS_SCALE                           1e6
#define MULTITURN_POS_SCALE                 1e3
#define SUBAREA_PARAMETER_KP_SCALE          1e4 
#define SUBAREA_PARAMETER_DEADBAND_SCALE    1e2 

typedef enum {                              //VESC里定义的CAN数据包类型
	CAN_PACKET_SET_DUTY						= 0,
	CAN_PACKET_SET_CURRENT					= 1,
	CAN_PACKET_SET_CURRENT_BRAKE			= 2,
	CAN_PACKET_SET_RPM						= 3,
	CAN_PACKET_SET_POS						= 4,

	CAN_PACKET_SET_POS_MULTITURN			= 74,  //2.15.2024新增 多圈位置

	CAN_PACKET_STATUS                       = 9,
	CAN_PACKET_STATUS_2						= 14,
	CAN_PACKET_STATUS_3						= 15,
	CAN_PACKET_STATUS_4						= 16,
	CAN_PACKET_SHUTDOWN						= 31,

	CAN_PACKET_GET_SUBAREA_PARA1			= 77,  //2.18.2024新增 分区PID参数读取和设置
	CAN_PACKET_GET_SUBAREA_PARA2			= 78,
	CAN_PACKET_GET_SUBAREA_PARA3			= 79,
	CAN_PACKET_SET_SUBAREA_PARA1			= 80,
	CAN_PACKET_SET_SUBAREA_PARA2			= 81,
	CAN_PACKET_SET_SUBAREA_PARA3			= 82,
	CAN_PACKET_STORE_MC_CONFIGURATION		= 83,
	CAN_PACKET_ENABLE_SUBAREA_PID	        = 84,

												   //2.20.2024 新增 重写一套shoot
	CAN_PACKET_ENABLE_SHOOT	                = 85,  //使能shoot
	CAN_PACKET_EXCUTE_SHOOT	                = 86,  //执行shoot，开始加速
	CAN_PACKET_ENABLE_AUTO_HOMING           = 87,  
	CAN_PACKET_SET_ACCEL_CURRENT			= 63,  
	CAN_PACKET_SET_TARGET_SPEED			    = 65,  
	CAN_PACKET_SET_BRAKE_CURRENT			= 68,
	CAN_PACKET_SET_HOME			            = 75,
	CAN_PACKET_HOMING			            = 76,
	CAN_PACKET_UPDATE_SHOOT_PARAMETER       = 88,  //TODO:把参数更新的CAN_decode加上
	CAN_PACKET_SHOOT_PARAMETER_RECEIVED     = 89,
	CAN_PACKET_SHOOT_GET_ACCEL_CURRENT      = 90,
	CAN_PACKET_SHOOT_GET_BRAKE_CURRENT      = 91,
	CAN_PACKET_SHOOT_GET_TARGET_SPEED       = 92,
	CAN_PACKET_SHOOT_GET_HOME_ANGLE         = 93,

	CAN_PACKET_SHOOT_SET_PTZ_ANGLE          = 64,

	CAN_PACKET_RELEASE_MOTER        = 94,
} CAN_PACKET_ID;

typedef struct {                             // VESC回传信息           
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
	Multiturn_Overflow,
}VESC_error_code_t;

extern motor_info_t motor_info[8];
extern uint8_t VESC_Send_Buffer[BUFFER_MAX_LENTH];

float uchar2float(uint8_t* buffer, int32_t *index);
int32_t uchar2int32(uint8_t* buffer, int32_t *index);

bool VESC_CAN_decode(uint32_t ExtID, uint8_t *pData);
bool VESC_CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t id, uint32_t eid, uint32_t lenth);

bool VESC_SetRPM(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetDutyCycle(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetCurrent(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetCurrentBrake(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetPos(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetMultiturnPos(float value, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SendCommand(CAN_PACKET_ID Cmd, float value, int32_t scale, uint8_t id, CAN_HandleTypeDef *hcan, uint32_t lenth);

/******************************************分区PID控制部分函数开始*******************************************/
bool VESC_PIDParameterRead(uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetSubareaPIDPara1(subarea_PID_parameter_t *para, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetSubareaPIDPara2(subarea_PID_parameter_t *para, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_SetSubareaPIDPara3(subarea_PID_parameter_t *para, uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_GetSubareaPIDPara1Request(uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_GetSubareaPIDPara2Request(uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_GetSubareaPIDPara3Request(uint8_t id, CAN_HandleTypeDef *hcan);
bool VESC_GetSubareaPIDPara1(subarea_PID_parameter_t *para, uint8_t* buffer);
bool VESC_GetSubareaPIDPara2(subarea_PID_parameter_t *para, uint8_t* buffer);
bool VESC_GetSubareaPIDPara3(subarea_PID_parameter_t *para, uint8_t* buffer);
bool VESC_EnableSubareaPIDControl(uint8_t id, CAN_HandleTypeDef *hcan, uint32_t flag);
bool VESC_StoreMcConfiguration(uint8_t id, CAN_HandleTypeDef *hcan);

typedef struct {
	bool para1_received;
	bool para2_received;
	bool para3_received;
}subarea_PID_parameter_communication_t;

typedef struct {
	float subarea_1;
	float subarea_2;
	float kp1;
	float ki1;
	float kd1;
	float kd_proc1;
	float kp2;
	float ki2;
	float kd2;
	float kd_proc2;
	float deadband;
	subarea_PID_parameter_communication_t com_status;
}subarea_PID_parameter_t;

extern subarea_PID_parameter_t subarea_PID_parameter;
/******************************************分区PID控制部分函数结束*******************************************/

/*********************************shoot相关开始****************************************/
#define SHOOT_VESC_ID 1
#define SHOOT_VESC_HCAN hcan1
bool SHOOT_ParameterInit (void);
bool SHOOT_EnableShoot (uint32_t flag);
bool SHOOT_EnableAutoHoming (uint32_t flag);
bool SHOOT_SetAccelCurrent (float acc_cur);
bool SHOOT_SetBrakeCurrent (float brk_cur);
bool SHOOT_SetTargetSpeed (float tar_spd);
bool SHOOT_SetHome (void);
bool SHOOT_ExcuteShoot (uint32_t flag);//加一个flag保险起见
bool SHOOT_ExcuteHoming (void);
bool SHOOT_UpdateParameter (void);
bool SHOOT_ParamterReceivedAcknowledge (void)
typedef struct {
	bool accel_current_received;
	bool brake_current_received;
	bool target_speed_received;
	bool home_angle_received;
	bool init_done;
}shoot_parameter_communication_status_t;

typedef struct {
	float accel_current;
	float target_speed;
	float brake_current;
	float home_angle;
	bool shoot_excute;
	bool homing_excute;
	bool auto_homing;
	shoot_parameter_communication_status_t com_status;
}shoot_parameter_t;

extern shoot_parameter_t shoot_parameter;
/*********************************shoot相关结束****************************************/


static void VESC_Error_Handler(VESC_error_code_t Code);

#endif
