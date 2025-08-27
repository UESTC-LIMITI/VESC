/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2025-08-27 19:15:53
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2025-08-27 21:27:20
 * @FilePath: \VESC_Code\All_About_FOC\my_foc\MDK-ARM\foc\foc_interface.h
 * @Description: 
 * 
 * Copyright (c) 2025 by xiayuan, All Rights Reserved. 
 */
#ifndef FOC_INTERFACE_H_
#define FOC_INTERFACE_H_

#include "main.h"
#include "datatypes.h"
#include "foc_mcconfig.h"
#include "stm32f4xx_hal.h"
#include "stm32f405xx.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;

extern uint16_t ADC_Value[ADC_CHANNEL_NUM];  //存放DMA传输过来的采样值
extern volatile motor_all_state_t motor;

#endif // FOC_INTERFACE_H_
