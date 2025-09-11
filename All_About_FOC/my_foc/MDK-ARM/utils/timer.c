/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2025-09-06 15:15:22
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2025-09-07 13:18:39
 * @FilePath: \VESC_Code\All_About_FOC\my_foc\MDK-ARM\utils\timer.c
 * @Description: 
 * 
 * Copyright (c) 2025 by xiayuan, All Rights Reserved. 
 */
#include "timer.h"
#include "main.h"
#include "tim.h"
#include "stm32f405xx.h"

// Settings
#define TIMER_HZ					1.4e7

void timer_init(void) {
	HAL_TIM_Base_Start(&htim5);
}

uint32_t timer_time_now(void) {
	return TIM5->CNT;
}

float timer_seconds_elapsed_since(uint32_t time) {  //一般计时用timer5
	uint32_t diff = TIM5->CNT - time;
	return (float)diff / (float)TIMER_HZ;
}

/**
 * Blocking sleep based on timer. 阻塞式
 *
 * @param seconds
 * Seconds to sleep.
 */
void timer_sleep(float seconds) {
	uint32_t start_t = TIM5->CNT;

	for (;;) {
		if (timer_seconds_elapsed_since(start_t) >= seconds) {
			return;
		}
	}
}