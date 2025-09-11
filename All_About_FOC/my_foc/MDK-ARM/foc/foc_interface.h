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
#include "foc_callbacks.h"
#include "utils_math.h"

#ifndef ADC_CHANNEL_NUM
#define ADC_CHANNEL_NUM		    8
#endif

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;

extern volatile uint16_t ADC_Value[ADC_CHANNEL_NUM];  //存放DMA传输过来的采样值

void foc_init(void);
void interface_encoder_routine(void);
void interface_set_duty(float duty);
void interface_set_current(float current);
void interface_set_current_brake(float current);
void interface_set_speed(float speed);
void interface_set_pos(float pos);

void interface_run_pid(void);

#endif // FOC_INTERFACE_H_
