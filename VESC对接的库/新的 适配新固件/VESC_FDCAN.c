/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2024-01-28 09:12:06
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2024-02-16 16:07:16
 * @FilePath: \MDK-ARM\VESC\VESC_CAN.c
 * @Description: 
 * VESC_CAN 1.0 曹总写的库 回传有问题，发送和设置模式分开 10.28.2021
 * VESC_CAN 2.0 GTY改写   可自定义回传，将命令函数分为单独的函数，与RM3508的库类似 1.29.2024
 * 适配新固件使用 60_LIMITI_mk5 v1.1beta3 及以上
 * CAN ID 尽量设置在1~8内，因为初始储存状态的数组只够前八个ID存
 * CAN Status 回传设置如下：
 * Status1：速度 占空比
 * Status2：单圈位置 多圈位置（需要编码器）
 * Status3：总电流 总电压 计算得到总功率
 * 更多Status 可定制
 * Todo：写一个编码器多圈重置功能
 * 2.16 新增多圈位置控制和FDCAN的库
 * Copyright (c) 2024 by UESTC_LIMITI, All Rights Reserved. 
 */
#include "VESC_CAN.h"	

motor_info_t motor_info[8] = {0};  //VESC回传数据储存在这里

/**
 * @description:                    FDCAN发送函数
 * @param {FDCAN_HandleTypeDef} *hfdcan 目标FDCAN
 * @param {uint8_t} id              目标
 * @param {uint32_t} eid            上一级函数计算好的eid
 * @return {*}                      函数执行成功返回 1 不成功返回0
 */
bool VESC_FDCAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint32_t eid) {
	 return FDCAN_SendData_Ext(hfdcan, VESC_Send_Buffer, eid, 4);
}

uint8_t VESC_Send_Buffer[4] = {0};  //发送的buffer

/**
 * @description:                     实现各种功能的函数
 * @param {float} value              目标值
 * @param {uint8_t} id               目标id
 * @param {FDCAN_HandleTypeDef} *hfdcan  目标CAN
 * @return {*}                       成功执行函数返回 1
 */
/*************************************************************************************/
bool VESC_SetRPM(float value, uint8_t id, FDCAN_HandleTypeDef *hfdcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_RPM << 8);
	int temp = (int)(value * RPM_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_FDCAN_SendData(hfdcan, id, eid);
	return true;
}

bool VESC_SetDutyCycle(float value, uint8_t id, FDCAN_HandleTypeDef *hfdcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
	value = CLAMP(value, -0.95, 0.95);
	int temp = (int)(value * DUTY_CYCLE_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_FDCAN_SendData(hfdcan, id, eid);
	return true;
}

bool VESC_SetCurrent(float value, uint8_t id, FDCAN_HandleTypeDef *hfdcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
	value = CLAMP(value, -MAX_CURRENT, MAX_CURRENT);
	int temp = (int)(value * CURRENT_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_FDCAN_SendData(hfdcan, id, eid);
	return true;
}

bool VESC_SetCurrentBrake(float value, uint8_t id, FDCAN_HandleTypeDef *hfdcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
	value = CLAMP(value, -MAX_CURRENT, MAX_CURRENT);
	int temp = (int)(value * CURRENT_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_FDCAN_SendData(hfdcan, id, eid);
	return true;
}

bool VESC_SetPos(float value, uint8_t id, FDCAN_HandleTypeDef *hfdcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_POS << 8);
	int temp = (int)value*POS_SCALE;
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_FDCAN_SendData(hfdcan, id, eid);
	return true;
}

bool VESC_SetMultiturnPos(float value, uint8_t id, FDCAN_HandleTypeDef *hfdcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	if(value > MAX_MULTITURN || value < -MAX_MULTITURN) {
		VESC_Error_Handler(Multiturn_Overflow);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_POS_MULTITURN << 8);
	int temp = (int)value*MULTITURN_POS_SCALE;
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_FDCAN_SendData(hfdcan, id, eid);
	return true;
}
/*************************************************************************************/


/**
 * @description:             类型转换函数
 * @param {uint8_t*} buffer  uint8的数组
 * @param {uint32_t} *index  外部序号，用于长度大于4的数组连续转换，自增
 * @return {*}               返回需要的类型
 */
/*************************************************************************************/
float uchar2float(uint8_t* buffer, uint32_t *index) {
	float temp = 0;
	memcpy(&(temp), (buffer+(*index)), 4);
	(*index) += 4;
	return temp;
}

int32_t uchar2int32(uint8_t* buffer, uint32_t *index) {
	int32_t temp = 0;
	temp |= ((0xff & buffer[(*index)++]) << 24);
	temp |= ((0xff & buffer[(*index)++]) << 16);
	temp |= ((0xff & buffer[(*index)++]) << 8);
	temp |= ((0xff & buffer[(*index)++]) << 0);
	return temp;
}
/*************************************************************************************/

/**
 * @description:            FDCAN消息解码  
 * @param {uint32_t} ExtID  收到的eid
 * @param {uint8_t} pData   接收buffer
 * @return {*}              成功返回 1 不成功返回 0
 */
bool VESC_FDCAN_decode(uint32_t ExtID, uint8_t *pData) {
	uint32_t packet_id = ExtID >> 8;
	uint8_t id = ExtID & 0x00000FF;
	uint32_t index = 0;
	if (packet_id != (uint32_t)CAN_PACKET_STATUS &&
		packet_id != (uint32_t)CAN_PACKET_STATUS_2 &&
		packet_id != (uint32_t)CAN_PACKET_STATUS_3 &&
		packet_id != (uint32_t)CAN_PACKET_STATUS_4 )
		return false;

#if defined(FLOAT_TRANSMITTED)
	switch (packet_id) {
		case (uint32_t)CAN_PACKET_STATUS:
			motor_info[id-1].rpm = uchar2float(pData, &index);
			motor_info[id-1].duty_cycle = uchar2float(pData, &index);
			break;

		case (uint32_t)CAN_PACKET_STATUS_2:
			motor_info[id-1].pos = uchar2float(pData, &index);
			motor_info[id-1].mul_pos = uchar2float(pData, &index);
			
			break;
		case (uint32_t)CAN_PACKET_STATUS_3:
			motor_info[id-1].tot_current_in = uchar2float(pData, &index);
			motor_info[id-1].tot_voltage = uchar2float(pData, &index);
			motor_info[id-1].power = motor_info[id-1].tot_current_in * motor_info[id-1].tot_voltage;
			break;

		case (uint32_t)CAN_PACKET_STATUS_4:
			//目前 do nothing			
			break;

		default:
			return false;
	}
	#else
	switch (packet_id) {
		case (uint32_t)CAN_PACKET_STATUS:
			motor_info[id-1].rpm = (uchar2int32(pData, &index) / RPM_SCALE);
			motor_info[id-1].duty_cycle = (uchar2int32(pData, &index) / DUTY_CYCLE_SCALE);
			break;

		case (uint32_t)CAN_PACKET_STATUS_2:
			motor_info[id-1].pos = (uchar2int32(pData, &index) / POS_SCALE);
			motor_info[id-1].mul_pos = (uchar2int32(pData, &index) / MULTITURN_POS_SCALE);
			
			break;
		case (uint32_t)CAN_PACKET_STATUS_3:
			motor_info[id-1].tot_current_in = (uchar2int32(pData, &index) / CURRENT_SCALE);
			motor_info[id-1].tot_voltage = (uchar2int32(pData, &index) / VOLTAGE_SCALE);
			motor_info[id-1].power = motor_info[id-1].tot_current_in * motor_info[id-1].tot_voltage;
			break;

		case (uint32_t)CAN_PACKET_STATUS_4:
			//目前 do nothing			
			break;

		default:
			return false;
	}
#endif
	return true;
}

/**
 * @description:                   出错啦
 * @param {VESC_ErrorCode_t} Code  错误码
 */
static void VESC_Error_Handler(VESC_ErrorCode_t Code) {
	//call stack 里看 ErrorCode
	Error_Handler();
}
