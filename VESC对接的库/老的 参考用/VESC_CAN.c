#include "VESC_CAN.h"

#define MAXID 4

/*电机的返回参数*/
Motor_INFO VESC_Feedback[VESC_MAX_ID + 1];

uint8_t VESC_SendBuf[MAXID][4];
uint8_t VESC_Mode[MAXID];

/*********************************************************************************
 *@  name      : VESC_CAN_SENDDATA
 *@  function  : 发送CAN信号给VESC
 *@  input     : CAN线、扩展帧、需要发送的数组指针
 *@  output    : 无
 *********************************************************************************/
void VESC_CAN_SENDDATA(CAN_HandleTypeDef *hcan, uint8_t id)
{
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t TxMailboxX = CAN_TX_MAILBOX0; // CAN发送邮箱

	TxMessage.ExtId = (VESC_Mode[id - 1] << 8) | ((uint32_t)id);
	TxMessage.DLC = 4;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;

	while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan)) // 等待空邮箱
		;
	if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET) // 检查发送邮箱状态
		TxMailboxX = CAN_TX_MAILBOX0;
	else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET)
		TxMailboxX = CAN_TX_MAILBOX1;
	else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET)
		TxMailboxX = CAN_TX_MAILBOX2;

	HAL_CAN_AddTxMessage(hcan, &TxMessage, VESC_SendBuf[id - 1], (uint32_t *)TxMailboxX);
}

/*********************************************************************************
 *@  name      : VESC_COMMAND_SET
 *@  function  : VESC命令设置
 *@  input     : CAN线、命令、VESC的id值、目标值
 *@  output    : 
 *********************************************************************************/
void VESC_COMMAND_SET(uint32_t cmd, uint8_t id, float set_value)
{
	/*与接收端进行设置同步*/
	uint8_t GET_INFO_FLAG = 0;
	switch (cmd)
	{
	case VESC_SET_DUTY:
		set_value *= 100000;
		break;
	case VESC_SET_CURRENT:
		set_value *= 1000;
		break;
	case VESC_SET_CURRENT_BRAKE:
		set_value *= 1000;
		break;
	case VESC_SET_RPM:
		set_value *= 1;
		break;
	case VESC_SET_POSITION:
		set_value *= 1000000;
		break;
	case VESC_GET_INFO:
		GET_INFO_FLAG = 1;
		switch((int)set_value){
		case VESC_GET_POS:
			VESC_SendBuf[id-1][0] = 0x32;
			VESC_SendBuf[id-1][1] = (0x01 << 0);
			VESC_SendBuf[id-1][2] = 0x0;
			VESC_SendBuf[id-1][3] = 0x0;
			break;

		case VESC_GET_RPM:
			VESC_SendBuf[id-1][0] = 0x32;
			VESC_SendBuf[id-1][1] = 0x0;
			VESC_SendBuf[id-1][2] = 0x0;
			VESC_SendBuf[id-1][3] = (0x1 << 7);
			break;

		case VESC_GET_DUTY:
			VESC_SendBuf[id-1][0] = 0x32;
			VESC_SendBuf[id-1][1] = 0x01;
			VESC_SendBuf[id-1][2] = 0x0;
			VESC_SendBuf[id-1][3] = (0x01 << 6);
			break;
		}
	}

	if(!GET_INFO_FLAG){
		for (int i = 3; i >= 0; i--)
			VESC_SendBuf[id - 1][i] = ((int)set_value >> ((3 - i) * 8)) & 0xFF;
	}
	VESC_Mode[id - 1] = cmd;
	
//	else{
//		*(uint32_t*)VESC_SendBuf[id - 1] = (GET_INFO_CMD | 0x00000032);
//	}
}


/*********************************************************************************
 *@  name      : VESC_CAN_DECODE
 *@  function  : 获取VESC的反馈并存入全局变量VESC_Feedback中
 *@  input     : 扩展帧、数据帧
 *@  output    : 无
 *********************************************************************************/
void VESC_CAN_DECODE(uint32_t ExtID, uint8_t pData[])
{
	/*判断是否为VESC发送来的信息*/
	if (ExtID >> 8 == 0x000009)
	{

		/*id值为扩展帧后8位*/
		uint8_t id = ExtID & 0x00000FF;

		/*将此id的数据解码,得到当前电流和转速*/
		VESC_Feedback[id].cur_rpm = VESC_DECODE_RPM(pData);
		VESC_Feedback[id].cur_current = VESC_DECODE_CURRENT(pData);
//		VESC_Feedback[id].cur_pos = VESC_DECODE_CURRENT(pData);
//		VESC_Feedback[id].cur_duty = VESC_DECODE_CURRENT(pData);
	}
}

/*********************************************************************************
 *@  name      : VESC_DECODE_RPM
 *@  function  : 将获取到的反馈值解码为转速
 *@  input     : 反馈数组指针
 *@  output    : 电机当前转速
 *********************************************************************************/
double VESC_DECODE_RPM(uint8_t pData[])
{
	int ret_rpm = 0;

	ret_rpm = ((uint32_t)pData[0] << 24) |
			  ((uint32_t)pData[1] << 16) |
			  ((uint32_t)pData[2] << 8) |
			  ((uint32_t)pData[3]);

	return ret_rpm;
}

/*********************************************************************************
 *@  name      : VESC_DECODE_CURRENT
 *@  function  : 将获取到的反馈值解码为电流值
 *@  input     : 反馈数组指针
 *@  output    : 电机当前电流值
 *********************************************************************************/
double VESC_DECODE_CURRENT(uint8_t pData[])
{
	int ret_current = ((uint32_t)pData[4] << 8) |
					  ((uint32_t)pData[5]);

	return ret_current / 1e1;
}

/*********************************************************************************
 *@  name      : VESC_DECODE_DUTY
 *@  function  : 将获取到的反馈值解码为占空比值
 *@  input     : 反馈数组指针
 *@  output    : 电机当前占空比
 *********************************************************************************/
double VESC_DECODE_DUTY(uint8_t pData[])
{
	int ret_duty = ((uint32_t)pData[6] << 8) |
					  ((uint32_t)pData[7]);

	return ret_duty / 1e3;
}

/*********************************************************************************
 *@  name      : VESC_DECODE_POS
 *@  function  : 将获取到的反馈值解码为位置
 *@  input     : 反馈数组指针
 *@  output    : 电机当前位置（0~360角度）
 *********************************************************************************/
double VESC_DECODE_POS(uint8_t pData[])
{
	int ret_pos = ((uint32_t)pData[6] << 8) |
					  ((uint32_t)pData[7]);

	return ret_pos / 50;
}
