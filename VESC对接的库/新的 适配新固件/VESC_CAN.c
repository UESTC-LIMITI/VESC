/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2024-01-28 09:12:06
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2024-02-21 14:03:00
 * @FilePath: \VESC_Code\VESC对接的库\新的 适配新固件\VESC_CAN.c
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
 * Copyright (c) 2024 by UESTC_LIMITI, All Rights Reserved. 
 */
#include "VESC_CAN.h"

motor_info_t motor_info[8] = {0};  //VESC回传数据储存在这里
subarea_PID_parameter_t subarea_PID_parameter = {0};
shoot_parameter_t shoot_parameter = {0};

/**
 * @description:                    CAN发送函数
 * @param {CAN_HandleTypeDef} *hcan 目标CAN
 * @param {uint8_t} id              目标id
 * @param {uint32_t} eid            上一级函数计算好的eid
 * @return {*}                      函数执行成功返回 1
 */
bool VESC_CAN_SendData(CAN_HandleTypeDef *hcan, uint8_t id, uint32_t eid, uint32_t lenth)
{
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t TxMailboxX = CAN_TX_MAILBOX0; // CAN发送邮箱
	uint32_t TimeOutCount = 0;

	TxMessage.ExtId = eid;
	TxMessage.DLC = lenth;
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

uint8_t VESC_Send_Buffer[BUFFER_MAX_LENTH] = {0};  //发送的buffer

/**
 * @description:                     实现各种功能的函数
 * @param {float} value              目标值
 * @param {uint8_t} id               目标id
 * @param {CAN_HandleTypeDef} *hcan  目标CAN
 * @return {*}                       成功执行函数返回 1
 */
/*************************************************************************************/
bool VESC_SetRPM(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_RPM << 8);
	int temp = (int)(value * RPM_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

bool VESC_SetDutyCycle(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
	value = CLAMP(value, -0.95, 0.95);
	int temp = (int)(value * DUTY_CYCLE_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

bool VESC_SetCurrent(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
	value = CLAMP(value, -MAX_CURRENT, MAX_CURRENT);
	int temp = (int)(value * CURRENT_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

bool VESC_SetCurrentBrake(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
	value = CLAMP(value, -MAX_CURRENT, MAX_CURRENT);
	int temp = (int)(value * CURRENT_SCALE);
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

bool VESC_SetPos(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_POS << 8);
	int temp = (int)value*POS_SCALE;
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

bool VESC_SetMultiturnPos(float value, uint8_t id, CAN_HandleTypeDef *hcan) {
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
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

/******************************************分区PID控制部分函数开始*******************************************/
/**
 * @description:                    初始化从VESC获取参数
 * @param {uint8_t} id              VESC id
 * @param {CAN_HandleTypeDef} *hcan 目标CAN
 * @return {*}
 */
bool VESC_PIDParameterRead(uint8_t id, CAN_HandleTypeDef *hcan) {
//	subarea_PID_parameter_t* para = &subarea_PID_parameter;
	subarea_PID_parameter_communication_t* para_com_status = &(subarea_PID_parameter.com_status);
	para_com_status->para1_received = false;
	para_com_status->para2_received = false;
	para_com_status->para3_received = false;
	while (para_com_status->para1_received == false) {
		VESC_GetSubareaPIDPara1Request(id, hcan);
		HAL_Delay(50);
	}
	while (para_com_status->para2_received == false) {
		VESC_GetSubareaPIDPara2Request(id, hcan);
		HAL_Delay(50);
	}
	while (para_com_status->para3_received == false) {
		VESC_GetSubareaPIDPara3Request(id, hcan);
		HAL_Delay(50);
	}
	return true;
}

/**
 * @description:                          设置分区PID控制参数 1组 2组 3组
 * @param {subarea_PID_parameter_t} *para 储存参数的指针
 * @param {uint8_t} id                    VESC id
 * @param {CAN_HandleTypeDef} *hcan       目标CAN
 * @return {*}
 */
bool VESC_SetSubareaPIDPara1(subarea_PID_parameter_t *para, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	int32_t ind = 0;
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_SUBAREA_PARA1 << 8);
	buffer_append_float16(VESC_Send_Buffer, para->kp1, SUBAREA_PARAMETER_KP_SCALE, &ind);
	buffer_append_float16(VESC_Send_Buffer, para->ki1, SUBAREA_PARAMETER_KP_SCALE, &ind);
	buffer_append_float16(VESC_Send_Buffer, para->kd1, SUBAREA_PARAMETER_KP_SCALE, &ind);
	buffer_append_float16(VESC_Send_Buffer, para->kd_proc1, SUBAREA_PARAMETER_KP_SCALE, &ind);
	VESC_CAN_SendData(hcan, id, eid, 8);
	return true;
}

bool VESC_SetSubareaPIDPara2(subarea_PID_parameter_t *para, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	int32_t ind = 0;
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_SUBAREA_PARA2 << 8);
	buffer_append_float16(VESC_Send_Buffer, para->kp2, SUBAREA_PARAMETER_KP_SCALE, &ind);
	buffer_append_float16(VESC_Send_Buffer, para->ki2, SUBAREA_PARAMETER_KP_SCALE, &ind);
	buffer_append_float16(VESC_Send_Buffer, para->kd2, SUBAREA_PARAMETER_KP_SCALE, &ind);
	buffer_append_float16(VESC_Send_Buffer, para->kd_proc2, SUBAREA_PARAMETER_KP_SCALE, &ind);
	VESC_CAN_SendData(hcan, id, eid, 8);
	return true;
}

bool VESC_SetSubareaPIDPara3(subarea_PID_parameter_t *para, uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	int32_t ind = 0;
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_SUBAREA_PARA3 << 8);
	buffer_append_float16(VESC_Send_Buffer, para->subarea_1, SUBAREA_PARAMETER_DEADBAND_SCALE, &ind);
	buffer_append_float16(VESC_Send_Buffer, para->subarea_2, SUBAREA_PARAMETER_DEADBAND_SCALE, &ind);
	buffer_append_float16(VESC_Send_Buffer, para->deadband, SUBAREA_PARAMETER_DEADBAND_SCALE, &ind);
	VESC_CAN_SendData(hcan, id, eid, 6);
	return true;
}

/**
 * @description:                    获取分区PID控制参数 1组 2组 3组 发送请求
 * @param {uint8_t} id              VESC id
 * @param {CAN_HandleTypeDef} *hcan 目标CAN
 * @return {*}               
 */
bool VESC_GetSubareaPIDPara1Request(uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_GET_SUBAREA_PARA1 << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

bool VESC_GetSubareaPIDPara2Request(uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_GET_SUBAREA_PARA2 << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

bool VESC_GetSubareaPIDPara3Request(uint8_t id, CAN_HandleTypeDef *hcan) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_GET_SUBAREA_PARA3 << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}


/**
 * @description:                          获取分区PID控制参数 1组 2组 3组 的解算函数
 * @param {subarea_PID_parameter_t} *para 储存参数的指针
 * @param {uint8_t*} buffer               接收buffer
 * @return {*}
 */
bool VESC_GetSubareaPIDPara1(subarea_PID_parameter_t *para, uint8_t* buffer) {
	int32_t ind = 0;
	para->kp1 = buffer_get_float16(buffer, SUBAREA_PARAMETER_KP_SCALE, &ind);
	para->ki1 = buffer_get_float16(buffer, SUBAREA_PARAMETER_KP_SCALE, &ind);
	para->kd1 = buffer_get_float16(buffer, SUBAREA_PARAMETER_KP_SCALE, &ind);
	para->kd_proc1 = buffer_get_float16(buffer, SUBAREA_PARAMETER_KP_SCALE, &ind);
	return true;
}

bool VESC_GetSubareaPIDPara2(subarea_PID_parameter_t *para, uint8_t* buffer) {
	int32_t ind = 0;
	para->kp2 = buffer_get_float16(buffer, SUBAREA_PARAMETER_KP_SCALE, &ind);
	para->ki2 = buffer_get_float16(buffer, SUBAREA_PARAMETER_KP_SCALE, &ind);
	para->kd2 = buffer_get_float16(buffer, SUBAREA_PARAMETER_KP_SCALE, &ind);
	para->kd_proc2 = buffer_get_float16(buffer, SUBAREA_PARAMETER_KP_SCALE, &ind);
	return true;
}

bool VESC_GetSubareaPIDPara3(subarea_PID_parameter_t *para, uint8_t* buffer) {
	int32_t ind = 0;
	para->subarea_1 = buffer_get_float16(buffer, SUBAREA_PARAMETER_DEADBAND_SCALE, &ind);
	para->subarea_2 = buffer_get_float16(buffer, SUBAREA_PARAMETER_DEADBAND_SCALE, &ind);
	para->deadband = buffer_get_float16(buffer, SUBAREA_PARAMETER_DEADBAND_SCALE, &ind);
	return true;
}

/**
 * @description:                    使能区PID控制
 * @param {uint8_t} id              VESC id
 * @param {CAN_HandleTypeDef} *hcan 目标CAN
 * @param {uint32_t} flag           置0失能， 大于0使能
 * @return {*}
 */
bool VESC_EnableSubareaPIDControl(uint8_t id, CAN_HandleTypeDef *hcan, uint32_t flag) {
	if (id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	int32_t ind = 0;
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_ENABLE_SUBAREA_PID << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	buffer_append_uint32(VESC_Send_Buffer, flag, &ind);
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

/**
 * @description:                    让VESC把电机设置储存进eeprom
 * @param {uint8_t} id              VESC id
 * @param {CAN_HandleTypeDef} *hcan 目标CAN
 * @return {*}
 */
bool VESC_StoreMcConfiguration(uint8_t id, CAN_HandleTypeDef *hcan) {
	if (id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
//	int32_t ind = 0;
	uint32_t eid = (id & 0xff) | ((uint32_t)CAN_PACKET_STORE_MC_CONFIGURATION << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	VESC_CAN_SendData(hcan, id, eid, 4);
	return true;
}

/******************************************分区PID控制部分函数结束*******************************************/


/*************************************************************************************/

/**
 * @description:                    发送通用指令的函数
 * @param {CAN_PACKET_ID} Cmd       指令
 * @param {float} value             值
 * @param {int32_t} scale           涉及发送数值需要用到的幅值
 * @param {uint8_t} id              VESC id
 * @param {CAN_HandleTypeDef} *hcan 目标CAN
 * @return {*}                      成功返回处
 */
bool VESC_SendCommand(CAN_PACKET_ID Cmd, float value, int32_t scale, uint8_t id, CAN_HandleTypeDef *hcan, uint32_t lenth) {
	if(id > 255 || id <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	uint32_t eid = (id & 0xff) | ((uint32_t)Cmd << 8);
 	int temp = (int)value*scale;
	for (int i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(hcan, id, eid, lenth);
	return true;	
}


/**
 * @description:             类型转换函数  (但是现在直接用buffer.c/h里的了)
 * @param {uint8_t*} buffer  uint8的数组
 * @param {uint32_t} *index  外部序号，用于长度大于4的数组连续转换，自增
 * @return {*}               返回需要的类型
 */
/*************************************************************************************/
float uchar2float(uint8_t* buffer, int32_t *index) {
	float temp = 0;
	memcpy(&(temp), (buffer+(*index)), 4);
	(*index) += 4;
	return temp;
}

int32_t uchar2int32(uint8_t* buffer, int32_t *index) {
	int32_t temp = 0; 
	temp |= ((0xff & buffer[(*index)++]) << 24);
	temp |= ((0xff & buffer[(*index)++]) << 16);
	temp |= ((0xff & buffer[(*index)++]) << 8);
	temp |= ((0xff & buffer[(*index)++]) << 0);
	return temp;
}
/*************************************************************************************/

/*********************************shoot相关开始****************************************/
bool SHOOT_ParameterInit (void) {
	if(SHOOT_VESC_ID > 255 || SHOOT_VESC_ID <= 0) {
		VESC_Error_Handler(Set_id_Wrong);
	}
	shoot_parameter_t* para = &shoot_parameter;
	para->com_status.init_done = false;
	SHOOT_UpdateParameter();
	while (para->com_status.home_angle_received == false);
	while (para->com_status.target_speed_received == false);
	while (para->com_status.brake_current_received == false);
	while (para->com_status.accel_current_received == false);		
	para->auto_homing = false;
	para->homing_excute = false;
	para->shoot_excute = false;
	para->com_status.init_done = true;
	return true;
}

bool SHOOT_EnableShoot (uint32_t flag) {
	int32_t ind = 0;
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_ENABLE_SHOOT << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	buffer_append_uint32(VESC_Send_Buffer, flag, &ind);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_EnableAutoHoming (uint32_t flag) {
	int32_t ind = 0;
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_ENABLE_AUTO_HOMING << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	buffer_append_uint32(VESC_Send_Buffer, flag, &ind);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_SetAccelCurrent (float acc_cur) {
	
	int32_t ind = 0;
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_SET_ACCEL_CURRENT << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	buffer_append_float32(VESC_Send_Buffer, acc_cur, CURRENT_SCALE, &ind);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_SetBrakeCurrent (float brk_cur) {
	
	int32_t ind = 0;
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_SET_BRAKE_CURRENT << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	buffer_append_float32(VESC_Send_Buffer, brk_cur, CURRENT_SCALE, &ind);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_SetTargetSpeed (float tar_spd) {
	
	int32_t ind = 0;
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_SET_TARGET_SPEED << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	buffer_append_float32(VESC_Send_Buffer, tar_spd, RPM_SCALE, &ind);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_SetHome (void) {
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_SET_HOME << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_ExcuteShoot (uint32_t flag) {  //加一个flag保险起见
	int32_t ind = 0;
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_EXCUTE_SHOOT << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	buffer_append_uint32(VESC_Send_Buffer, flag, &ind);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_ExcuteHoming (void) {
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_HOMING << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_UpdateParameter (void) {
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_UPDATE_SHOOT_PARAMETER << 8);
	shoot_parameter_t* shoot_para = &shoot_parameter;
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	shoot_para->com_status.accel_current_received = 0;
	shoot_para->com_status.brake_current_received = 0;
	shoot_para->com_status.target_speed_received = 0;
	shoot_para->com_status.home_angle_received = 0;
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

bool SHOOT_ParamterReceivedAcknowledge (void) {
	uint32_t eid = (SHOOT_VESC_ID & 0xff) | ((uint32_t)CAN_PACKET_SHOOT_PARAMETER_RECEIVED << 8);
	memset(VESC_Send_Buffer, 0, BUFFER_MAX_LENTH);
	VESC_CAN_SendData(&SHOOT_VESC_HCAN, SHOOT_VESC_ID, eid, 4);
	return true;
}

/*********************************shoot相关结束****************************************/


/**
 * @description:            CAN消息解码  
 * @param {uint32_t} ExtID  收到的eid
 * @param {uint8_t} pData   接收buffer
 * @return {*}              成功返回 1 不成功返回 0
 */
bool VESC_CAN_decode (uint32_t ExtID, uint8_t *pData) {
	uint32_t packet_id = ExtID >> 8;
	uint8_t id = ExtID & 0x00000FF;
	int32_t index = 0;
	bool ret = false;
	
	subarea_PID_parameter_t* PID_para = &subarea_PID_parameter;
	shoot_parameter_t* shoot_para = &shoot_parameter;

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
			index = 0;
			motor_info[id-1].rpm = (uchar2int32(pData, &index) / RPM_SCALE);
			motor_info[id-1].duty_cycle = (uchar2int32(pData, &index) / DUTY_CYCLE_SCALE);
			ret = true;
			break;

		case (uint32_t)CAN_PACKET_STATUS_2:
			index = 0;
			motor_info[id-1].pos = (uchar2int32(pData, &index) / POS_SCALE);
			motor_info[id-1].mul_pos = (uchar2int32(pData, &index) / MULTITURN_POS_SCALE);
			ret = true;
			break;
		
		case (uint32_t)CAN_PACKET_STATUS_3:
			index = 0;
			motor_info[id-1].tot_current_in = (uchar2int32(pData, &index) / CURRENT_SCALE);
			motor_info[id-1].tot_voltage = (uchar2int32(pData, &index) / VOLTAGE_SCALE);
			motor_info[id-1].power = motor_info[id-1].tot_current_in * motor_info[id-1].tot_voltage;
			ret = true;
			break;

		case (uint32_t)CAN_PACKET_STATUS_4:
			index = 0;
			//目前 do nothing			
			ret = true;
			break;

		case (uint32_t)CAN_PACKET_GET_SUBAREA_PARA1:
			index = 0;
			VESC_GetSubareaPIDPara1(PIDpara, pData);
			PIDpara->com_status.para1_received = true;
			ret = true;
			break;

		case (uint32_t)CAN_PACKET_GET_SUBAREA_PARA2:
			index = 0;
			VESC_GetSubareaPIDPara2(PIDpara, pData);
			PIDpara->com_status.para2_received = true;
			ret = true;
			break;

		case (uint32_t)CAN_PACKET_GET_SUBAREA_PARA3:
			index = 0;
			VESC_GetSubareaPIDPara3(PIDpara, pData);
			PIDpara->com_status.para3_received = true;
			ret = true;
			break;
		
		//以下是shoot部分
		case (uint32_t)CAN_PACKET_SHOOT_GET_ACCEL_CURRENT:
			index = 0;
			shoot_para->accel_current = buffer_get_float32(pData, CURRENT_SCALE, &index);
			shoot_para->com_status.accel_current_received = true;
			ret = true;
			break;

		case (uint32_t)CAN_PACKET_SHOOT_GET_BRAKE_CURRENT:
			index = 0;
			shoot_para->brake_current = buffer_get_float32(pData, CURRENT_SCALE, &index);
			shoot_para->com_status.brake_current_received = true;
			ret = true;
			break;

		case (uint32_t)CAN_PACKET_SHOOT_GET_TARGET_SPEED:
			index = 0;
			shoot_para->target_speed = buffer_get_float32(pData, RPM_SCALE, &index);
			shoot_para->com_status.target_speed_received = true;
			ret = true;
			break;

		case (uint32_t)CAN_PACKET_SHOOT_GET_HOME_ANGLE:
			index = 0;
			shoot_para->home_angle = buffer_get_float32(pData, CURRENT_SCALE, &index);
			shoot_para->com_status.home_angle_received = true;
			ret = true;
			break;

		default:
			ret = false;
			break;
	}
#endif
	return ret;
}

/**
 * @description:                   出错啦
 * @param {VESC_error_code_t} Code  错误码
 */
static void VESC_Error_Handler(VESC_error_code_t Code) {
	//call stack 里看 ErrorCode
	Error_Handler();
}
