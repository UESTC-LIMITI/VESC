/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2024-01-28 09:12:06
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2024-01-28 10:48:15
 * @FilePath: \VESC_Code\VESC对接的库\新的 适配新固件\VESC_CAN.c
 * @Description: 
 * 
 * Copyright (c) 2024 by UESTC_LIMITI, All Rights Reserved. 
 */
#include "VESC_CAN.h"

/*********************************************************************************
 *@  name      : VESC_CAN_SENDDATA
 *@  function  : 发送CAN信号给VESC
 *@  input     : CAN线、扩展帧、需要发送的数组指针
 *@  output    : 无
 *********************************************************************************/
bool VESC_CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t id, uint32_t eid)
{
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t TxMailboxX = CAN_TX_MAILBOX0; // CAN发送邮箱
	uint32_t TimeOutCount = 0;

	TxMessage.ExtId = eid;
	TxMessage.DLC = 4;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;

	while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan)) { // 等待空邮箱	
		if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET) // 检查发送邮箱状态
			TxMailboxX = CAN_TX_MAILBOX0;
		else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET)
			TxMailboxX = CAN_TX_MAILBOX1;
		else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET)
			TxMailboxX = CAN_TX_MAILBOX2;
		else {
			TimeOutCount++;
			if (TimeOutCount > 1000) {
				VESC_Error_Handler(CAN_SendData_TimeOut);
			}
		}
	}

	HAL_CAN_AddTxMessage(hcan, &TxMessage, VESC_Send_Buffer, (uint32_t *)TxMailboxX);
	return true;
}


uint8_t VESC_Send_Buffer[4] = {0};

bool VESC_SetRPM(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_RPM << 8);
	value = (int)(value * RPM_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (value >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid);
	return true;
}

bool VESC_SetDutyCycle(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
	value = CLAMP(value, -0.95, 0.95);
	value = (int)(value * DUTY_CYCLE_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (value >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid);
	return true;
}

bool VESC_SetCurrent(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
	value = CLAMP(value, -MAX_CURRENT, MAX_CURRENT);
	value = (int)(value * CURRENT_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (value >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid);
	return true;
}
bool VESC_SetCurrentBrake(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
	value = CLAMP(value, -MAX_CURRENT, MAX_CURRENT);
	value = (int)(value * CURRENT_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (value >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid);
	return true;
}

bool VESC_SetPos(int32_t value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_POS << 8);
	value = CLAMP(value, 0, 360);
	value = (int)value*POS_SCALE;
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (value >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid);
	return true;
}


float uchar2float(uint8_t* buffer, uint32_t *index) {
	float temp = 0;
	memcpy(&(temp), (buffer+(*index)), 4);
	(*index) += 4;
	return temp;
}

/**
 * @description: 
 * @param {uint32_t} ExtID
 * @param {uint8_t} pData
 * @return {*}成功返回 1 不成功返回 0
 */
bool VESC_CAN_decode(uint32_t ExtID, uint8_t *pData) {
	uint32_t packet_id = ExtID >> 8;
	uint8_t id = ExtID & 0x00000FF;
	uint32_t index = 0;
	if (packet_id != (uint32_t)CAN_PACKET_STATUS ||
		packet_id != (uint32_t)CAN_PACKET_STATUS2 ||
		packet_id != (uint32_t)CAN_PACKET_STATUS3 ||
		packet_id != (uint32_t)CAN_PACKET_STATUS4 )
		return false;

	switch (packet_id) {
		case (uint32_t)CAN_PACKET_STATUS:
			motor_info[id-1].rpm = uchar2float(pData, &index);
			motor_info[id-1].duty_cycle = uchar2float(pData, &index);
			break;

		case (uint32_t)CAN_PACKET_STATUS2:
			motor_info[id-1].pos = uchar2float(pData, &index);
			motor_info[id-1].mul_pos = uchar2float(pData, &index);
			
			break;
		case (uint32_t)CAN_PACKET_STATUS3:
			motor_info[id-1].tot_current_in = uchar2float(pData, &index);
			motor_info[id-1].tot_voltage = uchar2float(pData, &index);
			motor_info[id-1].power = motor_info[id-1].tot_current_in * motor_info[id-1].tot_voltage;
			break;

		case (uint32_t)CAN_PACKET_STATUS4:
			//目前 do nothing			
			break;

		default:
			return false;
	}
	return true;
}

static void VESC_Error_Handler(VESC_ErrorCode_t Code) {
	//call stack 里看 ErrorCode
	Error_Handler();
}
