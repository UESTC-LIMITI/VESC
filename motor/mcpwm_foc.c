/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2024-01-25 20:23:49
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2025-05-26 09:38:42
 * @FilePath: \VESC_Code\motor\mcpwm_foc.c
 * @Description: 
 * 
 * Copyright (c) 2025 by xiayuan, All Rights Reserved. 
 */
/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define HW_HAS_3_SHUNTS
// #define HW_HAS_PHASE_SHUNTS  //临时加的
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "digital_filter.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "ledpwm.h"
#include "terminal.h"
#include "encoder/encoder.h"
#include "commands.h"
#include "timeout.h"
#include "timer.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "virtual_motor.h"
#include "foc_math.h"

//#define SHOOT_TEST

// Private variables
static volatile bool m_dccal_done = false;
static volatile float m_last_adc_isr_duration;
static volatile bool m_init_done = false;
static volatile motor_all_state_t m_motor_1;  //定义电机的所有状态？与interface里的不一样
#ifdef HW_HAS_DUAL_MOTORS
static volatile motor_all_state_t m_motor_2;
#endif
static volatile int m_isr_motor = 0;

// Private functions
static void control_current(motor_all_state_t *motor, float dt);
static void update_valpha_vbeta(motor_all_state_t *motor, float mod_alpha, float mod_beta);
static void stop_pwm_hw(motor_all_state_t *motor);
static void start_pwm_hw(motor_all_state_t *motor);
static void terminal_tmp(int argc, const char **argv);
static void terminal_plot_hfi(int argc, const char **argv);
static void timer_update(motor_all_state_t *motor, float dt);
static void input_current_offset_measurement( void );
static void hfi_update(volatile motor_all_state_t *motor, float dt);

// Threads
static THD_WORKING_AREA(timer_thread_wa, 512);
static THD_FUNCTION(timer_thread, arg);
static volatile bool timer_thd_stop;

static THD_WORKING_AREA(hfi_thread_wa, 512);
static THD_FUNCTION(hfi_thread, arg);
static volatile bool hfi_thd_stop;

static THD_WORKING_AREA(pid_thread_wa, 256);
static THD_FUNCTION(pid_thread, arg);
static volatile bool pid_thd_stop;

// Macros
#ifdef HW_HAS_3_SHUNTS
#define TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

#define TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3) \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM8->CCR1 = duty1; \
		TIM8->CCR2 = duty2; \
		TIM8->CCR3 = duty3; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;
#else
#define TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty3; \
		TIM1->CCR3 = duty2; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;
#define TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3) \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM8->CCR1 = duty1; \
		TIM8->CCR2 = duty3; \
		TIM8->CCR3 = duty2; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;
#endif

#define TIMER_UPDATE_SAMP(samp) \
		TIM2->CCR2 = (samp / 2);

#define TIMER_UPDATE_SAMP_TOP_M1(samp, top) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM2->CR1 |= TIM_CR1_UDIS; \
		TIM1->ARR = top; \
		TIM2->CCR2 = samp / 2; \
		TIM1->CR1 &= ~TIM_CR1_UDIS; \
		TIM2->CR1 &= ~TIM_CR1_UDIS;
#define TIMER_UPDATE_SAMP_TOP_M2(samp, top) \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM2->CR1 |= TIM_CR1_UDIS; \
		TIM8->ARR = top; \
		TIM2->CCR2 = samp / 2; \
		TIM8->CR1 &= ~TIM_CR1_UDIS; \
		TIM2->CR1 &= ~TIM_CR1_UDIS;

// #define M_MOTOR: For single motor compilation, expands to &m_motor_1.
// For dual motors, expands to &m_motor_1 or _2, depending on is_second_motor.
#ifdef HW_HAS_DUAL_MOTORS
#define M_MOTOR(is_second_motor) (is_second_motor ? &m_motor_2 : &m_motor_1)
#else
#define M_MOTOR(is_second_motor)  (((void)is_second_motor), &m_motor_1)
#endif

static void update_hfi_samples(foc_hfi_samples samples, volatile motor_all_state_t *motor) {
	utils_sys_lock_cnt();

	memset((void*)&motor->m_hfi, 0, sizeof(motor->m_hfi));
	switch (samples) {
	case HFI_SAMPLES_8:
		motor->m_hfi.samples = 8;
		motor->m_hfi.table_fact = 4;
		motor->m_hfi.fft_bin0_func = utils_fft8_bin0;
		motor->m_hfi.fft_bin1_func = utils_fft8_bin1;
		motor->m_hfi.fft_bin2_func = utils_fft8_bin2;
		break;

	case HFI_SAMPLES_16:
		motor->m_hfi.samples = 16;
		motor->m_hfi.table_fact = 2;
		motor->m_hfi.fft_bin0_func = utils_fft16_bin0;
		motor->m_hfi.fft_bin1_func = utils_fft16_bin1;
		motor->m_hfi.fft_bin2_func = utils_fft16_bin2;
		break;

	case HFI_SAMPLES_32:
		motor->m_hfi.samples = 32;
		motor->m_hfi.table_fact = 1;
		motor->m_hfi.fft_bin0_func = utils_fft32_bin0;
		motor->m_hfi.fft_bin1_func = utils_fft32_bin1;
		motor->m_hfi.fft_bin2_func = utils_fft32_bin2;
		break;
	}

	utils_sys_unlock_cnt();
}

static void timer_reinit(int f_zv) {
	utils_sys_lock_cnt();

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM_DeInit(TIM2);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

	TIM1->CNT = 0;
	TIM2->CNT = 0;
	TIM8->CNT = 0;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / f_zv);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;

#ifndef INVERTED_TOP_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // gpio high = top fets on
#else
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

#ifndef INVERTED_BOTTOM_DRIVER_INPUT
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  // gpio high = bottom fets on
#else
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
#endif
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime =  conf_general_calculate_deadtime(HW_DEAD_TIME_NSEC, SYSTEM_CORE_CLOCK);
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

#ifdef HW_USE_BRK
	// Enable BRK function. Hardware will asynchronously stop any PWM activity upon an
	// external fault signal. PWM outputs remain disabled until MCU is reset.
	// software will catch the BRK flag to report the fault code
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
#else
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
#endif

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM8, ENABLE);
	TIM_ARRPreloadConfig(TIM8, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// TIM2配置为PWM输出, 实际不输出PWM, 所以period拉满, 配置为向上计数作为工具定时器.
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	// 主要是需要这个output compare模式, 这样才能产生CC中断信号.
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 250;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_CCPreloadControl(TIM2, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM2, ENABLE);  
	// 因为TIM2的CCx输出是ADC的触发源, 为了保证ADC一直采样, 需要提前使能

#if defined HW_HAS_DUAL_MOTORS || defined HW_HAS_DUAL_PARALLEL
	// See: https://www.cnblogs.com/shangdawei/p/4758988.html
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Trigger);
	TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Enable);
	TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);
	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR1);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
#else
	// TIM1的TIM_TRGOSource_Update作为重置信号, 即TIM1输出一个PWM波完成后, TIM2计数重置一次
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
#endif

#ifdef HW_HAS_DUAL_MOTORS
	TIM8->CNT = TIM1->ARR;
#else
	TIM8->CNT = 0;
#endif
	TIM1->CNT = 0;
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	// Prevent all low side FETs from switching on
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
#ifdef HW_HAS_DUAL_MOTORS
	stop_pwm_hw((motor_all_state_t*)&m_motor_2);
#endif

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);

	// Enable CC2 interrupt, which will be fired in V0 and V7
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	utils_sys_unlock_cnt();

	nvicEnableVector(TIM2_IRQn, 6);
}

void mcpwm_foc_init(mc_configuration *conf_m1, mc_configuration *conf_m2) {
	utils_sys_lock_cnt();  // 系统锁定, 线程切换, 或者中断. 并不是保护所有的volatile变量.

#ifndef HW_HAS_DUAL_MOTORS
	(void)conf_m2;
#endif

	m_init_done = false;

	memset((void*)&m_motor_1, 0, sizeof(motor_all_state_t));  // 将涉及FOC控制的所有变量都初始化为0
	m_isr_motor = 0;

	m_motor_1.m_conf = conf_m1;  // 一些电机配置可以照搬
	m_motor_1.m_state = MC_STATE_OFF;
	m_motor_1.m_control_mode = CONTROL_MODE_NONE;
	m_motor_1.m_hall_dt_diff_last = 1.0;
	foc_precalc_values((motor_all_state_t*)&m_motor_1);
	update_hfi_samples(m_motor_1.m_conf->foc_hfi_samples, &m_motor_1);

#ifdef HW_HAS_DUAL_MOTORS
	memset((void*)&m_motor_2, 0, sizeof(motor_all_state_t));
	m_motor_2.m_conf = conf_m2;
	m_motor_2.m_state = MC_STATE_OFF;
	m_motor_2.m_control_mode = CONTROL_MODE_NONE;
	m_motor_2.m_hall_dt_diff_last = 1.0;
	foc_precalc_values((motor_all_state_t*)&m_motor_2);
	update_hfi_samples(m_motor_2.m_conf->foc_hfi_samples, &m_motor_2);
#endif

/**
 * 分区PID控制的参数初始化
 * TODO:
 * 1. 定义好参数 done
 * 2. 将单圈和多圈控制的PID函数里都应用上分区控制 done
 * 3. 写一堆CAN协议 考虑如何用单片机设置和提取PID参数 done
 * 4. 写对应的库来调PID done
 * 5. 测试库 done 
 * 6. 考虑如何储存调好的参数 下载？
 **************************************************************************/
/*
	m_motor_1.m_conf->subarea_PID.initialized = false;                     \
	m_motor_1.m_conf->subarea_PID.subarea_1 = SUBAREA1;                    \
	m_motor_1.m_conf->subarea_PID.subarea_2 = SUBAREA2;                    \
	m_motor_1.m_conf->subarea_PID.kp1 = 0.035 * 2;                         \
	m_motor_1.m_conf->subarea_PID.ki1 = 0.0009 * 2;                        \
	m_motor_1.m_conf->subarea_PID.kd1 = 0.0003 * 2;                        \
	m_motor_1.m_conf->subarea_PID.kd_proc1 = 0.0002 * 2;                   \
	m_motor_1.m_conf->subarea_PID.kp2 = 0.035 * 3;                         \
	m_motor_1.m_conf->subarea_PID.ki2 = 0.0009 * 3;                        \
	m_motor_1.m_conf->subarea_PID.kd2 = 0.0003 * 3;                        \
	m_motor_1.m_conf->subarea_PID.kd_proc2 = 0.0002 * 3;                   \
	m_motor_1.m_conf->subarea_PID.deadband = 0.08;                         \
	m_motor_1.m_conf->subarea_PID.initialized = true;                      \
/*
/**************************************************************************/
	m_motor_1.m_conf->subarea_PID.enable_subarea_control = false;          \

	virtual_motor_init(conf_m1);

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM2);  // foc初始化过程中要关闭各个定时器, 特别是TIM2, 里面有foc计算的内容
	TIM_DeInit(TIM8);

	TIM1->CNT = 0;
	TIM2->CNT = 0;
	TIM8->CNT = 0;

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

	dmaStreamAllocate(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)),
					  5,
					  (stm32_dmaisr_t)mcpwm_foc_adc_int_handler,  // 这是绑定了DMA转换完成后的处理函数吗
					  (void *)0);

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value; //在foc初始化时确定DMA的内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);

	DMA_Cmd(DMA2_Stream4, ENABLE);  //使能DMA2的4号通道，从现在启动DMA2的4号通道

	// Note: The half transfer interrupt is used as we already have all current and voltage
	// samples by then and we can start processing them. Entering the interrupt earlier gives
	// more cycles to finish it and update the timer before the next zero vector. This helps
	// at higher f_zv. Only use this if the three first samples are current samples.
	// 半传输中断, 完成一半DMA传输即进入中断, 当ADC前三个值是电流采样值时, 进入中断一定转换完毕, 因此可以开启半传输中断
#if ADC_IND_CURR1 < 3 && ADC_IND_CURR2 < 3 && ADC_IND_CURR3 < 3
	DMA_ITConfig(DMA2_Stream4, DMA_IT_HT, ENABLE);  // 半传输中断
#else
	DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);  // 完全传输中断
#endif

	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;  
	//ADC三重模式，两个从ADC，一个主ADC，主ADC外部触发时, 通过硬件同步, 从ADC自动触发. 保证三个ADC同时采样
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;  // 主ADC外部触发是在下降沿
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;  // 主ADC外部触发是TIM2的CC2通道
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;

	ADC_Init(ADC1, &ADC_InitStructure);  // ADC1使用以上配置初始化是主ADC
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;  // 从ADC禁用外部触发
	ADC_Init(ADC2, &ADC_InitStructure);  // ADC2和ADC3是从ADC
	ADC_Init(ADC3, &ADC_InitStructure);

	ADC_TempSensorVrefintCmd(ENABLE);
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	hw_setup_adc_channels();

	ADC_Cmd(ADC1, ENABLE);  //启动三路的采样
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);

	timer_reinit((int)m_motor_1.m_conf->foc_f_zv);

	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
#ifdef HW_HAS_DUAL_MOTORS
	stop_pwm_hw((motor_all_state_t*)&m_motor_2);
#endif

	utils_sys_unlock_cnt();

	CURRENT_FILTER_ON();  // 目前是空的宏定义
	CURRENT_FILTER_ON_M2();
	ENABLE_GATE();  // 目前是空的宏定义
	DCCAL_OFF();  // 目前是空的宏定义
#ifdef HW_USE_ALTERNATIVE_DC_CAL
	m_dccal_done = true;
#else
	if (m_motor_1.m_conf->foc_offsets_cal_on_boot) {
		systime_t cal_start_time = chVTGetSystemTimeX();
		float cal_start_timeout = 10.0;

		// Wait for input voltage to rise above minimum voltage
		// 初始化还要检查输入电压
		while (mc_interface_get_input_voltage_filtered() < m_motor_1.m_conf->l_min_vin) {
			chThdSleepMilliseconds(1);
			if (UTILS_AGE_S(cal_start_time) >= cal_start_timeout) {
				m_dccal_done = true;  // 超时了就置位, 除了接下来的初始化不再进行, 这个标志位还会影响什么?
				break;
			}
		}

		// Wait for input voltage to settle
		if (!m_dccal_done) {
			float v_in_last = mc_interface_get_input_voltage_filtered();
			systime_t v_in_stable_time = chVTGetSystemTimeX();  // 获取当前时刻
			while (UTILS_AGE_S(v_in_stable_time) < 2.0) {  
				// 计算稳定的时间大于2s才进行下一步, 一定程度上解释为什么VESC开机需要一些时间
				// UTILS_AGE_S(x) 这个宏定义的作用时计算自从x时刻到现在的时间, 返回的是秒数
				
				chThdSleepMilliseconds(1);

				float v_in_now = mc_interface_get_input_voltage_filtered();
				if (fabsf(v_in_now - v_in_last) > 1.5) {
					v_in_last = v_in_now;
					v_in_stable_time = chVTGetSystemTimeX();  // 电源电压不稳定, 更新当前时刻
				}

				if (UTILS_AGE_S(cal_start_time) >= cal_start_timeout) {
					m_dccal_done = true;  // 超时咯, 拜拜
					break;
				}
			}
		}

		// Wait for fault codes to go away
		// 等待一段时间处理故障代码, 别的线程会处理, 如果一直没有消除错误, 那应该是没法正常运行了
		if (!m_dccal_done) {
			while (mc_interface_get_fault() != FAULT_CODE_NONE) {
				chThdSleepMilliseconds(1);
				if (UTILS_AGE_S(cal_start_time) >= cal_start_timeout) {
					m_dccal_done = true;
					break;
				}
			}
		}

		if (!m_dccal_done) {
			m_motor_1.m_conf->foc_offsets_voltage[0] = MCCONF_FOC_OFFSETS_VOLTAGE_0;
			m_motor_1.m_conf->foc_offsets_voltage[1] = MCCONF_FOC_OFFSETS_VOLTAGE_1;
			m_motor_1.m_conf->foc_offsets_voltage[2] = MCCONF_FOC_OFFSETS_VOLTAGE_2;

			m_motor_1.m_conf->foc_offsets_voltage_undriven[0] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_0;
			m_motor_1.m_conf->foc_offsets_voltage_undriven[1] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_1;
			m_motor_1.m_conf->foc_offsets_voltage_undriven[2] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_2;

			m_motor_1.m_conf->foc_offsets_current[0] = MCCONF_FOC_OFFSETS_CURRENT_0;
			m_motor_1.m_conf->foc_offsets_current[1] = MCCONF_FOC_OFFSETS_CURRENT_1;
			m_motor_1.m_conf->foc_offsets_current[2] = MCCONF_FOC_OFFSETS_CURRENT_2;

#ifdef HW_HAS_DUAL_MOTORS
			m_motor_2.m_conf->foc_offsets_voltage[0] = MCCONF_FOC_OFFSETS_VOLTAGE_0;
			m_motor_2.m_conf->foc_offsets_voltage[1] = MCCONF_FOC_OFFSETS_VOLTAGE_1;
			m_motor_2.m_conf->foc_offsets_voltage[2] = MCCONF_FOC_OFFSETS_VOLTAGE_2;

			m_motor_2.m_conf->foc_offsets_voltage_undriven[0] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_0;
			m_motor_2.m_conf->foc_offsets_voltage_undriven[1] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_1;
			m_motor_2.m_conf->foc_offsets_voltage_undriven[2] = MCCONF_FOC_OFFSETS_VOLTAGE_UNDRIVEN_2;

			m_motor_2.m_conf->foc_offsets_current[0] = MCCONF_FOC_OFFSETS_CURRENT_0;
			m_motor_2.m_conf->foc_offsets_current[1] = MCCONF_FOC_OFFSETS_CURRENT_1;
			m_motor_2.m_conf->foc_offsets_current[2] = MCCONF_FOC_OFFSETS_CURRENT_2;
#endif

			mcpwm_foc_dc_cal(false);  // 这里面有5秒以上的延时??
			// 测current和votage的offset的, 为什么要测? 如何测?
			// 开机时候的电流声应该就是它, 可以之后用示波器验证一下
		}
	} else {
		m_dccal_done = true;
	}
#endif
	// Start threads
	// 这三个线程中没有实质上与foc控制有关的代码, 例如svpwm计算, park变换, clark变换
	timer_thd_stop = false;
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);

	hfi_thd_stop = false;
	chThdCreateStatic(hfi_thread_wa, sizeof(hfi_thread_wa), NORMALPRIO, hfi_thread, NULL);

	pid_thd_stop = false;
	chThdCreateStatic(pid_thread_wa, sizeof(pid_thread_wa), NORMALPRIO, pid_thread, NULL);

	// Check if the system has resumed from IWDG reset and generate fault if it has. This can be used to
	// tell if some frozen thread caused a watchdog reset. Note that this also will trigger after running
	// the bootloader and after the reset command.
	// 看门狗错误是这里产生的, 为何一直没法消除掉?
	// 这个错误是因为看门狗一直没有被喂, 可能是某个线程卡死了, 也可能是bootloader的原因
	if (timeout_had_IWDG_reset()) {
		mc_interface_fault_stop(FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET, false, false);
	}

	// 以下是终端命令的绑定, 我如果有想画的图或者想加的命令都可以模仿着加进去
	terminal_register_command_callback(
			"foc_tmp",
			"FOC Test Print",
			0,
			terminal_tmp);  
		// 在终端可以用foc_tmp命令来画图

	terminal_register_command_callback(
			"foc_plot_hfi_en",
			"Enable HFI plotting. 0: off, 1: DFT, 2: Raw",
			"[en]",
			terminal_plot_hfi);
	// 在终端可以用foc_plot_hfi_en命令来画图

	m_init_done = true;
}

void mcpwm_foc_deinit(void) {
	if (!m_init_done) {
		return;
	}

	m_init_done = false;

	timer_thd_stop = true;
	while (timer_thd_stop) {
		chThdSleepMilliseconds(1);
	}

	hfi_thd_stop = true;
	while (hfi_thd_stop) {
		chThdSleepMilliseconds(1);
	}

	pid_thd_stop = true;
	while (pid_thd_stop) {
		chThdSleepMilliseconds(1);
	}

	TIM_DeInit(TIM1);
	TIM_DeInit(TIM2);
	TIM_DeInit(TIM8);
	ADC_DeInit();
	DMA_DeInit(DMA2_Stream4);
	nvicDisableVector(ADC_IRQn);
	dmaStreamRelease(STM32_DMA_STREAM(STM32_DMA_STREAM_ID(2, 4)));
}

static volatile motor_all_state_t *get_motor_now(void) {  //用于返回现在motor的所有参数
#ifdef HW_HAS_DUAL_MOTORS
	return mc_interface_motor_now() == 1 ? &m_motor_1 : &m_motor_2;
#else
	return &m_motor_1;
#endif
}

bool mcpwm_foc_init_done(void) {
	return m_init_done;
}

void mcpwm_foc_set_configuration(mc_configuration *configuration) {
	get_motor_now()->m_conf = configuration;
	foc_precalc_values((motor_all_state_t*)get_motor_now());

	// Below we check if anything in the configuration changed that requires stopping the motor.

	uint32_t top = SYSTEM_CORE_CLOCK / (int)configuration->foc_f_zv;
	if (TIM1->ARR != top) {
#ifdef HW_HAS_DUAL_MOTORS
		m_motor_1.m_control_mode = CONTROL_MODE_NONE;
		m_motor_1.m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)&m_motor_1);

		m_motor_2.m_control_mode = CONTROL_MODE_NONE;
		m_motor_2.m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)&m_motor_2);

		timer_reinit((int)configuration->foc_f_zv);
#else
		get_motor_now()->m_control_mode = CONTROL_MODE_NONE;
		get_motor_now()->m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)get_motor_now());
		TIMER_UPDATE_SAMP_TOP_M1(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);
#ifdef  HW_HAS_DUAL_PARALLEL
		TIMER_UPDATE_SAMP_TOP_M2(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);
#endif
#endif
	}

	if (((1 << get_motor_now()->m_conf->foc_hfi_samples) * 8) != get_motor_now()->m_hfi.samples) {
		get_motor_now()->m_control_mode = CONTROL_MODE_NONE;
		get_motor_now()->m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)get_motor_now());
		update_hfi_samples(get_motor_now()->m_conf->foc_hfi_samples, get_motor_now());
	}

	virtual_motor_set_configuration(configuration);
}

mc_state mcpwm_foc_get_state(void) {
	return get_motor_now()->m_state;
}

mc_control_mode mcpwm_foc_control_mode(void) {
	return get_motor_now()->m_control_mode;
}

bool mcpwm_foc_is_dccal_done(void) {
	return m_dccal_done;
}

/**
 * Get the current motor used in the mcpwm ISR
 *
 * @return
 * 0: Not in ISR
 * 1: Motor 1
 * 2: Motor 2
 */
int mcpwm_foc_isr_motor(void) {
	return m_isr_motor;
}

/**
 * Switch off all FETs.
 */
void mcpwm_foc_stop_pwm(bool is_second_motor) {
	motor_all_state_t *motor = (motor_all_state_t*)M_MOTOR(is_second_motor);

	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw(motor);
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * @param dutyCycle
 * The duty cycle to use
 */
void mcpwm_foc_set_duty(float dutyCycle) {
	get_motor_now()->m_control_mode = CONTROL_MODE_DUTY;
	get_motor_now()->m_duty_cycle_set = dutyCycle;

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will
 * stop the motor.
 *
 * WARNING: This function does not use ramping. A too large step with a large motor
 * can destroy hardware.
 *
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_foc_set_duty_noramp(float dutyCycle) {
	// TODO: Actually do this without ramping
	mcpwm_foc_set_duty(dutyCycle);
}

/**
 * Use PID rpm control. Note that this value has to be multiplied by half of
 * the number of motor poles.
 *
 * @param rpm
 * The electrical RPM goal value to use.
 */
void mcpwm_foc_set_pid_speed(float rpm) {
	volatile motor_all_state_t *motor = get_motor_now();

	if (motor->m_conf->s_pid_ramp_erpms_s > 0.0 ) {
		if (motor->m_control_mode != CONTROL_MODE_SPEED ||
				motor->m_state != MC_STATE_RUNNING) {
			motor->m_speed_pid_set_rpm = mcpwm_foc_get_rpm();
		}

		motor->m_speed_command_rpm = rpm;
	} else {
		motor->m_speed_pid_set_rpm = rpm;
	}

	motor->m_control_mode = CONTROL_MODE_SPEED;

	if (motor->m_state != MC_STATE_RUNNING &&
			fabsf(rpm) >= motor->m_conf->s_pid_min_erpm) {
		motor->m_motor_released = false;
		motor->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Use PID position control. Note that this only works when encoder support
 * is enabled.
 *
 * @param pos
 * The desired position of the motor in degrees.

 */
void mcpwm_foc_set_pid_pos(float pos) {
	get_motor_now()->m_control_mode = CONTROL_MODE_POS;
	get_motor_now()->m_pos_pid_set = pos;

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * 自定义多圈控制的状态更改函数，与上面的单圈控制对应
 * 状态设置好以后PID线程里会执行多圈控制函数
 */
void mcpwm_foc_set_pid_pos_multiturn(float pos) {  
	get_motor_now()->m_control_mode = CONTROL_MODE_POS_MULTITURN;
	get_motor_now()->m_pos_pid_set = pos;  //设置多圈角度

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}


/**
 * Use current control and specify a goal current to use. The sign determines
 * the direction of the torque. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use.
 */
void mcpwm_foc_set_current(float current) {
	get_motor_now()->m_control_mode = CONTROL_MODE_CURRENT;
	get_motor_now()->m_iq_set = current;
	get_motor_now()->m_id_set = 0;
	
	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

void mcpwm_foc_release_motor(void) {
	get_motor_now()->m_control_mode = CONTROL_MODE_CURRENT;
	get_motor_now()->m_iq_set = 0.0;
	get_motor_now()->m_id_set = 0.0;
	get_motor_now()->m_motor_released = true;
}

/**
 * Brake the motor with a desired current. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use. Positive and negative values give the same effect.
 */
void mcpwm_foc_set_brake_current(float current) {
	get_motor_now()->m_control_mode = CONTROL_MODE_CURRENT_BRAKE;
	get_motor_now()->m_iq_set = current;

	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Apply a fixed static current vector in open loop to emulate an electric
 * handbrake.
 *
 * @param current
 * The brake current to use.
 */
void mcpwm_foc_set_handbrake(float current) {
	get_motor_now()->m_control_mode = CONTROL_MODE_HANDBRAKE;
	get_motor_now()->m_iq_set = current;

	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Produce an openloop rotating current.
 *
 * @param current
 * The current to use.
 *
 * @param rpm
 * The RPM to use.
 *
 */
void mcpwm_foc_set_openloop_current(float current, float rpm) {
	utils_truncate_number(&current, -get_motor_now()->m_conf->l_current_max * get_motor_now()->m_conf->l_current_max_scale,
						  get_motor_now()->m_conf->l_current_max * get_motor_now()->m_conf->l_current_max_scale);

	get_motor_now()->m_control_mode = CONTROL_MODE_OPENLOOP;
	get_motor_now()->m_iq_set = current;
	get_motor_now()->m_openloop_speed = RPM2RADPS_f(rpm);

	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Produce an openloop current at a fixed phase.
 *
 * @param current
 * The current to use.
 *
 * @param phase
 * The phase to use in degrees, range [0.0 360.0]
 */
void mcpwm_foc_set_openloop_phase(float current, float phase) {  // 直接电流控制, phase是电流的相位角
	utils_truncate_number(&current, -get_motor_now()->m_conf->l_current_max * get_motor_now()->m_conf->l_current_max_scale,
						  get_motor_now()->m_conf->l_current_max * get_motor_now()->m_conf->l_current_max_scale);

	get_motor_now()->m_control_mode = CONTROL_MODE_OPENLOOP_PHASE;
	get_motor_now()->m_id_set = current;
	get_motor_now()->m_iq_set = 0;

	get_motor_now()->m_openloop_phase = DEG2RAD_f(phase);
	utils_norm_angle_rad((float*)&get_motor_now()->m_openloop_phase);

	if (fabsf(current) < get_motor_now()->m_conf->cc_min_current) {
		return;
	}

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Get current offsets,
 * this is used by the virtual motor to save the current offsets,
 * when it is connected
 */
void mcpwm_foc_get_current_offsets(
		volatile float *curr0_offset,
		volatile float *curr1_offset,
		volatile float *curr2_offset,
		bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	*curr0_offset = motor->m_conf->foc_offsets_current[0];
	*curr1_offset = motor->m_conf->foc_offsets_current[1];
	*curr2_offset = motor->m_conf->foc_offsets_current[2];
}

/**
 * Set current offsets values,
 * this is used by the virtual motor to set the previously saved offsets back,
 * when it is disconnected
 */
void mcpwm_foc_set_current_offsets(volatile float curr0_offset,
								   volatile float curr1_offset,
								   volatile float curr2_offset) {
	get_motor_now()->m_conf->foc_offsets_current[0] = curr0_offset;
	get_motor_now()->m_conf->foc_offsets_current[1] = curr1_offset;
	get_motor_now()->m_conf->foc_offsets_current[2] = curr2_offset;
}

void mcpwm_foc_get_voltage_offsets(
		float *v0_offset,
		float *v1_offset,
		float *v2_offset,
		bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	*v0_offset = motor->m_conf->foc_offsets_voltage[0];
	*v1_offset = motor->m_conf->foc_offsets_voltage[1];
	*v2_offset = motor->m_conf->foc_offsets_voltage[2];
}

void mcpwm_foc_get_voltage_offsets_undriven(
		float *v0_offset,
		float *v1_offset,
		float *v2_offset,
		bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	*v0_offset = motor->m_conf->foc_offsets_voltage_undriven[0];
	*v1_offset = motor->m_conf->foc_offsets_voltage_undriven[1];
	*v2_offset = motor->m_conf->foc_offsets_voltage_undriven[2];
}

void mcpwm_foc_get_currents_adc(
		float *ph0,
		float *ph1,
		float *ph2,
		bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	*ph0 = motor->m_currents_adc[0];
	*ph1 = motor->m_currents_adc[1];
	*ph2 = motor->m_currents_adc[2];
}

/**
 * Produce an openloop rotating voltage.
 *
 * @param dutyCycle
 * The duty cycle to use.
 *
 * @param rpm
 * The RPM to use.
 */
void mcpwm_foc_set_openloop_duty(float dutyCycle, float rpm) {
	get_motor_now()->m_control_mode = CONTROL_MODE_OPENLOOP_DUTY;
	get_motor_now()->m_duty_cycle_set = dutyCycle;
	get_motor_now()->m_openloop_speed = RPM2RADPS_f(rpm);

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

/**
 * Produce an openloop voltage at a fixed phase.
 *
 * @param dutyCycle
 * The duty cycle to use.
 *
 * @param phase
 * The phase to use in degrees, range [0.0 360.0]
 */
void mcpwm_foc_set_openloop_duty_phase(float dutyCycle, float phase) {  // 直接电压(占空比)控制, phase是电压的相位角
	get_motor_now()->m_control_mode = CONTROL_MODE_OPENLOOP_DUTY_PHASE;
	get_motor_now()->m_duty_cycle_set = dutyCycle;
	get_motor_now()->m_openloop_phase = DEG2RAD_f(phase);
	utils_norm_angle_rad((float*)&get_motor_now()->m_openloop_phase);

	if (get_motor_now()->m_state != MC_STATE_RUNNING) {
		get_motor_now()->m_motor_released = false;
		get_motor_now()->m_state = MC_STATE_RUNNING;
	}
}

float mcpwm_foc_get_duty_cycle_set(void) {
	return get_motor_now()->m_duty_cycle_set;
}

float mcpwm_foc_get_duty_cycle_now(void) {
	return get_motor_now()->m_motor_state.duty_now;
}

float mcpwm_foc_get_pid_pos_set(void) {
	return get_motor_now()->m_pos_pid_set;
}

float mcpwm_foc_get_pid_pos_now(void) {
	return get_motor_now()->m_pos_pid_now;
}

/**
 * Get the current switching frequency.
 *
 * @return
 * The switching frequency in Hz.
 */
float mcpwm_foc_get_switching_frequency_now(void) {
	return get_motor_now()->m_conf->foc_f_zv;
}

/**
 * Get the current sampling frequency.
 *
 * @return
 * The sampling frequency in Hz.
 */
float mcpwm_foc_get_sampling_frequency_now(void) {
#ifdef HW_HAS_PHASE_SHUNTS
	if (get_motor_now()->m_conf->foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7) {
		return get_motor_now()->m_conf->foc_f_zv;
	} else {
		return get_motor_now()->m_conf->foc_f_zv / 2.0;
	}
#else
	return get_motor_now()->m_conf->foc_f_zv / 2.0;
#endif
}

/**
 * Returns Ts used for virtual motor sync
 */
float mcpwm_foc_get_ts(void) {
#ifdef HW_HAS_PHASE_SHUNTS
	if (get_motor_now()->m_conf->foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7) {
		return (1.0 / get_motor_now()->m_conf->foc_f_zv) ;
	} else {
		return (1.0 / (get_motor_now()->m_conf->foc_f_zv / 2.0));
	}
#else
	return (1.0 / get_motor_now()->m_conf->foc_f_zv) ;
#endif
}

bool mcpwm_foc_is_using_encoder(void) {
	return get_motor_now()->m_using_encoder;
}

void mcpwm_foc_get_observer_state(float *x1, float *x2) {
	volatile motor_all_state_t *motor = get_motor_now();
	*x1 = motor->m_observer_state.x1;
	*x2 = motor->m_observer_state.x2;
}

/**
 * Set current off delay. Prevent the current controller from switching off modulation
 * for target currents < cc_min_current for this amount of time.
 */
void mcpwm_foc_set_current_off_delay(float delay_sec) {
	if (get_motor_now()->m_current_off_delay < delay_sec) {
		get_motor_now()->m_current_off_delay = delay_sec;
	}
}

float mcpwm_foc_get_tot_current_motor(bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq) * motor->m_motor_state.i_abs;
}

float mcpwm_foc_get_tot_current_filtered_motor(bool is_second_motor) {
	volatile motor_all_state_t *motor = M_MOTOR(is_second_motor);
	return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq_filter) * motor->m_motor_state.i_abs_filter;
}

float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor) {
	return M_MOTOR(is_second_motor)->m_motor_state.i_bus;
}

float mcpwm_foc_get_tot_current_in_filtered_motor(bool is_second_motor) {
	// TODO: Filter current?
	return M_MOTOR(is_second_motor)->m_motor_state.i_bus;
}

float mcpwm_foc_get_abs_motor_current_motor(bool is_second_motor) {
	return M_MOTOR(is_second_motor)->m_motor_state.i_abs;
}

float mcpwm_foc_get_abs_motor_current_filtered_motor(bool is_second_motor) {
	return M_MOTOR(is_second_motor)->m_motor_state.i_abs_filter;
}

mc_state mcpwm_foc_get_state_motor(bool is_second_motor) {
	return M_MOTOR(is_second_motor)->m_state;
}

/**
 * Calculate the current RPM of the motor. This is a signed value and the sign
 * depends on the direction the motor is rotating in. Note that this value has
 * to be divided by half the number of motor poles.
 *
 * @return
 * The RPM value.
 */
float mcpwm_foc_get_rpm(void) {
	return RADPS2RPM_f(get_motor_now()->m_pll_speed);
	//	return get_motor_now()->m_speed_est_fast * RADPS2RPM_f;
}

/**
 * Same as above, but uses the fast and noisier estimator.
 */
float mcpwm_foc_get_rpm_fast(void) {
	return RADPS2RPM_f(get_motor_now()->m_speed_est_fast);
}

/**
 * Same as above, but uses the faster and noisier estimator.
 */
float mcpwm_foc_get_rpm_faster(void) {
	return RADPS2RPM_f(get_motor_now()->m_speed_est_faster);
}

/**
 * Get the motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current. This is the q-axis current which produces torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_foc_get_tot_current(void) {
	volatile motor_all_state_t *motor = get_motor_now();
	return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq) * motor->m_motor_state.i_abs;
}

/**
 * Get the filtered motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current. This is the q-axis current which produces torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_foc_get_tot_current_filtered(void) {
	volatile motor_all_state_t *motor = get_motor_now();
	return SIGN(motor->m_motor_state.vq * motor->m_motor_state.iq_filter) * motor->m_motor_state.i_abs_filter;
}

/**
 * Get the magnitude of the motor current, which includes both the
 * D and Q axis.
 *
 * @return
 * The magnitude of the motor current.
 */
float mcpwm_foc_get_abs_motor_current(void) {
	return get_motor_now()->m_motor_state.i_abs;
}

/**
 * Get the magnitude of the motor current unbalance
 *
 * @return
 * The magnitude of the phase currents unbalance.
 */
float mcpwm_foc_get_abs_motor_current_unbalance(void) {
	return get_motor_now()->m_curr_unbalance * FAC_CURRENT;
}

/**
 * Get the magnitude of the motor voltage.
 *
 * @return
 * The magnitude of the motor voltage.
 */
float mcpwm_foc_get_abs_motor_voltage(void) {
	const float vd_tmp = get_motor_now()->m_motor_state.vd;
	const float vq_tmp = get_motor_now()->m_motor_state.vq;
	return NORM2_f(vd_tmp, vq_tmp);
}

/**
 * Get the filtered magnitude of the motor current, which includes both the
 * D and Q axis.
 *
 * @return
 * The magnitude of the motor current.
 */
float mcpwm_foc_get_abs_motor_current_filtered(void) {
	return get_motor_now()->m_motor_state.i_abs_filter;
}

/**
 * Get the motor current. The sign of this value represents the direction
 * in which the motor generates torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_foc_get_tot_current_directional(void) {
	return get_motor_now()->m_motor_state.iq;
}

/**
 * Get the filtered motor current. The sign of this value represents the
 * direction in which the motor generates torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_foc_get_tot_current_directional_filtered(void) {
	return get_motor_now()->m_motor_state.iq_filter;
}

/**
 * Get the direct axis motor current.
 *
 * @return
 * The D axis current.
 */
float mcpwm_foc_get_id(void) {
	return get_motor_now()->m_motor_state.id;
}

/**
 * Get the quadrature axis motor current.
 *
 * @return
 * The Q axis current.
 */
float mcpwm_foc_get_iq(void) {
	return get_motor_now()->m_motor_state.iq;
}

float mcpwm_foc_get_id_set(void) {
	return get_motor_now()->m_id_set;
}

float mcpwm_foc_get_iq_set(void) {
	return get_motor_now()->m_iq_set;
}

/**
 * Get the filtered direct axis motor current.
 *
 * @return
 * The D axis current.
 */
float mcpwm_foc_get_id_filter(void) {
	return get_motor_now()->m_motor_state.id_filter;
}

/**
 * Get the filtered quadrature axis motor current.
 *
 * @return
 * The Q axis current.
 */
float mcpwm_foc_get_iq_filter(void) {
	return get_motor_now()->m_motor_state.iq_filter;
}

/**
 * Get the input current to the motor controller.
 *
 * @return
 * The input current.
 */
float mcpwm_foc_get_tot_current_in(void) {
	return get_motor_now()->m_motor_state.i_bus;
}

/**
 * Get the filtered input current to the motor controller.
 *
 * @return
 * The filtered input current.
 */
float mcpwm_foc_get_tot_current_in_filtered(void) {
	return get_motor_now()->m_motor_state.i_bus; // TODO: Calculate filtered current?
}

/**
 * Set the number of steps the motor has rotated. This number is signed and
 * becomes a negative when the motor is rotating backwards.
 *
 * @param steps
 * New number of steps will be set after this call.
 *
 * @return
 * The previous tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int mcpwm_foc_set_tachometer_value(int steps) {
	int val = get_motor_now()->m_tachometer;
	get_motor_now()->m_tachometer = steps;
	return val;
}

/**
 * Read the number of steps the motor has rotated. This number is signed and
 * will return a negative number when the motor is rotating backwards.
 *
 * @param reset
 * If true, the tachometer counter will be reset after this call.
 *
 * @return
 * The tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int mcpwm_foc_get_tachometer_value(bool reset) {
	int val = get_motor_now()->m_tachometer;

	if (reset) {
		get_motor_now()->m_tachometer = 0;
	}

	return val;
}

/**
 * Read the absolute number of steps the motor has rotated.
 *
 * @param reset
 * If true, the tachometer counter will be reset after this call.
 *
 * @return
 * The tachometer value in motor steps. The number of motor revolutions will
 * be this number divided by (3 * MOTOR_POLE_NUMBER).
 */
int mcpwm_foc_get_tachometer_abs_value(bool reset) {
	int val = get_motor_now()->m_tachometer_abs;

	if (reset) {
		get_motor_now()->m_tachometer_abs = 0;
	}

	return val;
}

float mcpwm_foc_get_phase(void) {
	float angle = RAD2DEG_f(get_motor_now()->m_motor_state.phase);
	utils_norm_angle(&angle);
	return angle;
}

float mcpwm_foc_get_phase_observer(void) {
	float angle = RAD2DEG_f(get_motor_now()->m_phase_now_observer);
	utils_norm_angle(&angle);
	return angle;
}

float mcpwm_foc_get_phase_encoder(void) {
	float angle = RAD2DEG_f(get_motor_now()->m_phase_now_encoder);
	utils_norm_angle(&angle);
	return angle;
}

float mcpwm_foc_get_phase_hall(void) {
	float angle = RAD2DEG_f(get_motor_now()->m_ang_hall_rate_limited);
	utils_norm_angle(&angle);
	return angle;
}

float mcpwm_foc_get_vd(void) {
	return get_motor_now()->m_motor_state.vd;
}

float mcpwm_foc_get_vq(void) {
	return get_motor_now()->m_motor_state.vq;
}

float mcpwm_foc_get_mod_alpha_raw(void) {
	return get_motor_now()->m_motor_state.mod_alpha_raw;
}

float mcpwm_foc_get_mod_beta_raw(void) {
	return get_motor_now()->m_motor_state.mod_beta_raw;
}

float mcpwm_foc_get_mod_alpha_measured(void) {
	return get_motor_now()->m_motor_state.mod_alpha_measured;
}

float mcpwm_foc_get_mod_beta_measured(void) {
	return get_motor_now()->m_motor_state.mod_beta_measured;
}

float mcpwm_foc_get_est_lambda(void) {
	return get_motor_now()->m_observer_state.lambda_est;
}

float mcpwm_foc_get_est_res(void) {
	return get_motor_now()->m_res_est;
}

// NOTE: Requires the regular HFI sensor mode to run
float mcpwm_foc_get_est_ind(void) {
	float real_bin0, imag_bin0;
	get_motor_now()->m_hfi.fft_bin0_func((float*)get_motor_now()->m_hfi.buffer, &real_bin0, &imag_bin0);
	return real_bin0;
}

/**
 * Measure encoder offset and direction.
 *
 * @param current
 * The locking open loop current for the motor.
 *
 * @param print
 * Controls logging during the detection procedure. Set to true to enable
 * logging.
 *
 * @param offset
 * The detected offset.
 *
 * @param ratio
 * The ratio between electrical and mechanical revolutions
 *
 * @param direction
 * The detected direction.
 *
 * @param inverted
 * Is set to true if the encoder reports an increase in angle in the opposite
 * direction of the motor.
 *
 * @return
 * The fault code
 */
int mcpwm_foc_encoder_detect(float current, bool print, float *offset, float *ratio, bool *inverted) {  //编码器整定函数
	int fault = FAULT_CODE_NONE;
	mc_interface_lock();  // 期间不允许任何其他命令控制电机

	volatile motor_all_state_t *motor = get_motor_now();

	motor->m_phase_override = true;  // 允许手动控制相位, 但是相位写入之后, 如何真正让电机转动到那里?
	motor->m_id_set = current;
	motor->m_iq_set = 0.0;
	motor->m_control_mode = CONTROL_MODE_CURRENT;
	motor->m_motor_released = false;
	motor->m_state = MC_STATE_RUNNING;

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Save configuration
	float offset_old = motor->m_conf->foc_encoder_offset;
	float inverted_old = motor->m_conf->foc_encoder_inverted;
	float ratio_old = motor->m_conf->foc_encoder_ratio;
	float ldiff_old = motor->m_conf->foc_motor_ld_lq_diff;

	motor->m_conf->foc_encoder_offset = 0.0;
	motor->m_conf->foc_encoder_inverted = false;
	motor->m_conf->foc_encoder_ratio = 1.0;
	motor->m_conf->foc_motor_ld_lq_diff = 0.0;

	// Find index
	int cnt = 0;
	while(!encoder_index_found()) {  // 只有ABI编码器有这一步吧
		for (float i = 0.0;i < 2.0 * M_PI;i += (2.0 * M_PI) / 500.0) {
			motor->m_phase_now_override = i;
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_encoder_detect;
			}
			chThdSleepMilliseconds(1);
		}

		cnt++;
		if (cnt > 30) {
			// Give up
			break;
		}
	}

	if (print) {
		commands_printf("Index found");
	}

	// Rotate
	for (float i = 0.0;i < 2.0 * M_PI;i += (2.0 * M_PI) / 500.0) {  // 旋转到电角度为0的位置
		motor->m_phase_now_override = i;
		fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			goto exit_encoder_detect;
		}
		chThdSleepMilliseconds(1);
	}

	if (print) {
		commands_printf("Rotated for sync");
	}

	// Inverted and ratio  // 这一段是测定编码器的方向, 以及电机的电角度和机械角度的比值
	chThdSleepMilliseconds(1000);  // 现实使用的时候真的有等待一秒的环节吗

	const int it_rat = 30;
	float s_sum = 0.0;
	float c_sum = 0.0;
	float first = motor->m_phase_now_encoder;

	for (int i = 0; i < it_rat; i++) {  // 循环30次, 每次旋转2/3PI, 总计20PI, 电角度意义上的10圈?
		float phase_old = motor->m_phase_now_encoder;
		float phase_ovr_tmp = motor->m_phase_now_override;
		for (float j = phase_ovr_tmp; j < phase_ovr_tmp + (2.0 / 3.0) * M_PI;
			 j += (2.0 * M_PI) / 500.0) {  // 总计旋转2/3PI, 每1ms旋转1/250PI, 也就是一次500/3ms
			motor->m_phase_now_override = j;  // for循环的最后一次赋值, 与进入for循环前的phase_ovr_tmp相同, 保证这是连续的旋转
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_encoder_detect;
			}
			chThdSleepMilliseconds(1);
		}

		utils_norm_angle_rad((float*)&motor->m_phase_now_override);  // 为什么要归回到-PI到PI之间?
		chThdSleepMilliseconds(300);  // 这300ms的延时是否造成了有感整定时位置图像中的台阶式变化? 也许是标志一次旋转的短暂结束.
		timeout_reset();
		float diff = utils_angle_difference_rad(motor->m_phase_now_encoder, phase_old);
		// 我们知道电角度变化是2/3PI, 现在来看看编码器角度变化了多少. (返回的是弧度角)
		// 电角度没有变化超过2/3PI, 所以编码器不可能变化超过2/3PI, 不用担心这个函数在归一化的时候, 截断了2pi以上的角度变化.
		// 由于每次旋转固定电角度, 所以编码器角度变化也应该几乎是固定的, 也就是diff是固定的

		float s, c;
		sincosf(diff, &s, &c);  // 为什么要把角度变化转换为正弦余弦值来表示? 为什么不能直接累加diff?
		s_sum += s;
		c_sum += c;  

		if (print) {
			commands_printf("Diff: %.2f", (double)RAD2DEG_f(diff));
		}

		if (i > 3 && fabsf(utils_angle_difference_rad(motor->m_phase_now_encoder, first)) < fabsf(diff / 2.0)) {
			// 这个判断退出的条件, 是判断, 当前编码器角度与开始旋转时编码器角度的差值绝对值小于diff/2, 也就是每次变化量的一半
			// 这个条件保证每次正向旋转, 旋转实际编码器值接近一圈之后, 才会停止旋转.
			// diff/2巧妙在, 每次停止旋转, 最多只旋转多或少一小段, 不超过diff/2, 正转和反转几乎是在同一个位置开始和停止的.
			break;
		}
	}

	first = motor->m_phase_now_encoder;

	for (int i = 0; i < it_rat; i++) {  // 这次循环同上, 只不过是反向旋转.
		float phase_old = motor->m_phase_now_encoder;
		float phase_ovr_tmp = motor->m_phase_now_override;
		for (float j = phase_ovr_tmp; j > phase_ovr_tmp - (2.0 / 3.0) * M_PI; j -= (2.0 * M_PI) / 500.0) {
			motor->m_phase_now_override = j;
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_encoder_detect;
			}
			chThdSleepMilliseconds(1);
		}
		utils_norm_angle_rad((float*)&motor->m_phase_now_override);
		chThdSleepMilliseconds(300);
		timeout_reset();
		float diff = utils_angle_difference_rad(phase_old, motor->m_phase_now_encoder);  
		// 与正转的时候计算方式相反, 也就是正转反转计算的符号是一样的

		float s, c;
		sincosf(diff, &s, &c);
		s_sum += s;
		c_sum += c;

		if (print) {
			commands_printf("Diff: %.2f", (double)RAD2DEG_f(diff));
		}

		if (i > 3 && fabsf(utils_angle_difference_rad(motor->m_phase_now_encoder, first)) < fabsf(diff / 2.0)) {
			break;
		}
	}

	float diff = RAD2DEG_f(atan2f(s_sum, c_sum));
	*inverted = diff < 0.0;  // 新角度-旧角度, 是负数, 说明编码器增量与电机正转方向反向.
	*ratio = roundf(((2.0 / 3.0) * 180.0) / fabsf(diff)); 
	// 电角度变化2/3PI, 编码器变化diff, 所以ratio就是 2/3PI / diff, 一般diff小于2/3PI, 所以ratio大于1.
	// 应该是个整数.

	motor->m_conf->foc_encoder_inverted = *inverted;
	motor->m_conf->foc_encoder_ratio = *ratio;

	if (print) {
		commands_printf("Inversion and ratio detected");
		commands_printf("Ratio: %.2f", (double)*ratio);
	}

	// 以上, inverted 和 ratio 计算结束, 接下来要利用这两个数据来计算offset.

	// Rotate
	for (float i = motor->m_phase_now_override;i < 2.0 * M_PI;i += (2.0 * M_PI) / 500.0) {
		motor->m_phase_now_override = i;
		fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			goto exit_encoder_detect;
		}
		chThdSleepMilliseconds(2);
	}  
	// 旋转到电角度的2PI, 也就是0. 这一步是不是导致整定过程中间图像出现V型变化的原因? 
	// 但是V型变化似乎是正转再反转, 这段代码只有一个反向的旋转.
	// 电角度归零, 应该是方便下一步

	if (print) {
		commands_printf("Rotated for sync");
		commands_printf("Enc: %.2f", (double)encoder_read_deg());
	}

	const int it_ofs = motor->m_conf->foc_encoder_ratio * 3.0;
	s_sum = 0.0;
	c_sum = 0.0;

	// 计算offset的部分, 我的理解:
	// 1. 为什么要分解成正弦余弦值?
	// 将每次旋转的电角度和编码器角度差值转化为正弦值和余弦值, 是以向量形式将角度差表示在平面上.
	// 这个向量在单位圆中, 模长固定为1, 所以合成时我们只需要关注其角度.
	//
	// 2. 为什么对累计的正弦余弦值进行atan2能得到offset?
	// 代码执行是正转一圈再反转一圈, 期间产生了一系列角度差的向量. 
	// 若编码器和电机电角度偏差是0, 那么这些向量合成在一起的结果就是0. 
	// 但是如果存在一个offset, 无论是正转还是反转, 生成的向量都可以被分解为
	// (没有offset时的角度差向量 + 固定offset向量) 两部分, 
	// 其中没有offset的向量, 最后合成为0; 
	// 固定offset向量合成产生一个指向某一角度的向量, 每一次累积几乎不会改变这个向量的方向, 只会增加模长. 
	// 所有旋转结束后, 电机机械角度和电角度回到原点, 此时合成所有向量得到的只有offset的向量, 所以能用atan还原成offset. 
	// 并且由于电机旋转过机械角度上的一圈, 所有电角度与机械角度的组合都被遍历, 产生同一组向量, 所以合成的offset不会有不同.
	//
	// 3. 为什么要分成两次正转和反转?
	// 一次正转一次反转, 没有偏移的部分被抵消, offset角度向量被放大和平均, 理论上累积的次数越多越准确
	
	for (int i = 0;i < it_ofs;i++) {  
		// 执行编码器ratio*3的次数循环, 每次旋转(2/3)*PI的电角度, ((2/3)*PI)/ratio的机械角度, 总计为机械角度的一圈.
		// 也解释了这样的现象: ratio越大, 这一步整定用时越多.
		float step = (2.0 * M_PI * motor->m_conf->foc_encoder_ratio) / ((float)it_ofs);  // 实际上计算出的step一直是2/3PI.
		float override = (float)i * step;

		while (motor->m_phase_now_override != override) {
			utils_step_towards((float*)&motor->m_phase_now_override, override, step / 100.0);  
			// 按照2/300PI步长步进, 手动修改电机相位, 直到达到目标值, 也就是比现在大2/3PI的位置.
			// 循环100次, 用时400ms, 位置变化比较缓和.
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_encoder_detect;
			}
			chThdSleepMilliseconds(4);
		}

		chThdSleepMilliseconds(100);  // 造成台阶型图像.
		timeout_reset();

		float angle_diff = utils_angle_difference_rad(motor->m_phase_now_encoder, motor->m_phase_now_override);
		// 电角度变化2/3PI, 记录编码器角度与电角度差值(使用as5047时, 实际是机械角度与电角度差值)
		float s, c;
		sincosf(angle_diff, &s, &c);
		s_sum += s;
		c_sum += c;

		if (print) {
			commands_printf("Ovr: %.2f/%.2f Diff: %.2f", (double)override, (double)(it_ofs * step), (double)RAD2DEG_f(angle_diff));
		}
	}

	for (int i = it_ofs;i > 0;i--) {  // 反转, 同上
		float step = (2.0 * M_PI * motor->m_conf->foc_encoder_ratio) / ((float)it_ofs);
		float override = (float)i * step;

		while (motor->m_phase_now_override != override) {
			utils_step_towards((float*)&motor->m_phase_now_override, override, step / 100.0);
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_encoder_detect;
			}
			chThdSleepMilliseconds(4);
		}

		chThdSleepMilliseconds(100);
		timeout_reset();

		float angle_diff = utils_angle_difference_rad(motor->m_phase_now_encoder, motor->m_phase_now_override);
		float s, c;
		sincosf(angle_diff, &s, &c);
		s_sum += s;
		c_sum += c;

		if (print) {
			commands_printf("Ovr: %.2f/%.2f Diff: %.2f", (double)override, (double)(it_ofs * step), (double)RAD2DEG_f(angle_diff));
		}
	}

	*offset = RAD2DEG_f(atan2f(s_sum, c_sum));

	if (print) {
		commands_printf("Avg: %.2f", (double)*offset);
	}

	utils_norm_angle(offset);

	if (print) {
		commands_printf("Offset detected");
	}

	exit_encoder_detect:
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_phase_override = false;
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw((motor_all_state_t*)motor);

	// Restore configuration
	motor->m_conf->foc_encoder_inverted = inverted_old;
	motor->m_conf->foc_encoder_offset = offset_old;
	motor->m_conf->foc_encoder_ratio = ratio_old;
	motor->m_conf->foc_motor_ld_lq_diff = ldiff_old;

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);

	mc_interface_unlock();
	return fault;
}

/**
 * Lock the motor with a current and sample the voltage and current to
 * calculate the motor resistance.
 *
 * @param current
 * The locking current.
 *
 * @param samples
 * The number of samples to take.
 *
 * @param stop_after
 * Stop motor after finishing the measurement. Otherwise, the current will
 * still be applied after returning. Setting this to false is useful if you want
 * to run this function again right away, without stopping the motor in between.
 *
 * @param resistance
 * The calculated motor resistance
 *
 * @return
 * The fault code.
 */
int mcpwm_foc_measure_resistance(float current, int samples, bool stop_after, float *resistance) {
	mc_interface_lock();

	volatile motor_all_state_t *motor = get_motor_now();
	int fault = FAULT_CODE_NONE;

	motor->m_phase_override = true;
	motor->m_phase_now_override = 0.0;
	motor->m_id_set = 0.0;
	motor->m_control_mode = CONTROL_MODE_CURRENT;
	motor->m_motor_released = false;
	motor->m_state = MC_STATE_RUNNING;

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Ramp up the current slowly
	while (fabsf(motor->m_iq_set - current) > 0.001) {
		utils_step_towards((float*)&motor->m_iq_set, current, fabsf(current) / 200.0);
		fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			motor->m_id_set = 0.0;
			motor->m_iq_set = 0.0;
			motor->m_phase_override = false;
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw((motor_all_state_t*)motor);

			timeout_configure(tout, tout_c, tout_ksw);
			mc_interface_unlock();

			return fault;
		}
		chThdSleepMilliseconds(1);
	}

	// Wait for the current to rise and the motor to lock.
	chThdSleepMilliseconds(50);

	// Sample
	motor->m_samples.avg_current_tot = 0.0;
	motor->m_samples.avg_voltage_tot = 0.0;
	motor->m_samples.sample_num = 0;

	int cnt = 0;
	while (motor->m_samples.sample_num < samples) {
		chThdSleepMilliseconds(1);
		cnt++;
		// Timeout
		if (cnt > 10000) {
			break;
		}
		fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			motor->m_id_set = 0.0;
			motor->m_iq_set = 0.0;
			motor->m_phase_override = false;
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw((motor_all_state_t*)motor);

			timeout_configure(tout, tout_c, tout_ksw);
			mc_interface_unlock();

			return fault;
		}
	}

	const float current_avg = motor->m_samples.avg_current_tot / (float)motor->m_samples.sample_num;
	const float voltage_avg = motor->m_samples.avg_voltage_tot / (float)motor->m_samples.sample_num;

	// Stop
	if (stop_after) {
		motor->m_id_set = 0.0;
		motor->m_iq_set = 0.0;
		motor->m_phase_override = false;
		motor->m_control_mode = CONTROL_MODE_NONE;
		motor->m_state = MC_STATE_OFF;
		stop_pwm_hw((motor_all_state_t*)motor);
	}

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	*resistance = voltage_avg / current_avg;

	return fault;
}

/**
 * Measure the motor inductance with short voltage pulses.
 *
 * @param duty
 * The duty cycle to use in the pulses.
 *
 * @param samples
 * The number of samples to average over.
 *
 * @param
 * The current that was used for this measurement.
 *
 * @inductance
 * The average d and q axis inductance in uH.
 *
 * @return
 * The fault code
 */
int mcpwm_foc_measure_inductance(float duty, int samples, float *curr, float *ld_lq_diff, float *inductance) {
	volatile motor_all_state_t *motor = get_motor_now();
	int fault = FAULT_CODE_NONE;

	mc_foc_sensor_mode sensor_mode_old = motor->m_conf->foc_sensor_mode;
	float f_zv_old = motor->m_conf->foc_f_zv;
	float hfi_voltage_start_old = motor->m_conf->foc_hfi_voltage_start;
	float hfi_voltage_run_old = motor->m_conf->foc_hfi_voltage_run;
	float hfi_voltage_max_old = motor->m_conf->foc_hfi_voltage_max;
	float sl_erpm_hfi_old = motor->m_conf->foc_sl_erpm_hfi;
	mc_foc_control_sample_mode foc_control_sample_mode_old = motor->m_conf->foc_control_sample_mode;
	foc_hfi_samples samples_old = motor->m_conf->foc_hfi_samples;
	mc_foc_current_sample_mode foc_current_sample_mode_old = motor->m_conf->foc_current_sample_mode;

	mc_interface_lock();
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw((motor_all_state_t*)motor);

	motor->m_conf->foc_sensor_mode = FOC_SENSOR_MODE_HFI;
	motor->m_conf->foc_hfi_voltage_start = duty * mc_interface_get_input_voltage_filtered() * (2.0 / 3.0) * SQRT3_BY_2;
	motor->m_conf->foc_hfi_voltage_run = duty * mc_interface_get_input_voltage_filtered() * (2.0 / 3.0) * SQRT3_BY_2;
	motor->m_conf->foc_hfi_voltage_max = duty * mc_interface_get_input_voltage_filtered() * (2.0 / 3.0) * SQRT3_BY_2;
	motor->m_conf->foc_sl_erpm_hfi = 20000.0;
	motor->m_conf->foc_control_sample_mode = FOC_CONTROL_SAMPLE_MODE_V0;
	motor->m_conf->foc_hfi_samples = HFI_SAMPLES_32;
	motor->m_conf->foc_current_sample_mode = FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO;

	if (motor->m_conf->foc_f_zv > 30.0e3) {
		motor->m_conf->foc_f_zv = 30.0e3;
	}

	mcpwm_foc_set_configuration(motor->m_conf);

	chThdSleepMilliseconds(1);

	timeout_reset();
	mcpwm_foc_set_duty(0.0);
	chThdSleepMilliseconds(1);

	int ready_cnt = 0;
	while (!motor->m_hfi.ready) {
		chThdSleepMilliseconds(1);
		ready_cnt++;
		if (ready_cnt > 100) {
			break;
		}
	}

	if (samples < 10) {
		samples = 10;
	}

	float l_sum = 0.0;
	float ld_lq_diff_sum = 0.0;
	float i_sum = 0.0;
	float iterations = 0.0;

	for (int i = 0;i < (samples / 10);i++) {
		timeout_reset();
		mcpwm_foc_set_duty(0.0);

		fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			motor->m_id_set = 0.0;
			motor->m_iq_set = 0.0;
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw((motor_all_state_t*)motor);

			motor->m_conf->foc_sensor_mode = sensor_mode_old;
			motor->m_conf->foc_f_zv = f_zv_old;
			motor->m_conf->foc_hfi_voltage_start = hfi_voltage_start_old;
			motor->m_conf->foc_hfi_voltage_run = hfi_voltage_run_old;
			motor->m_conf->foc_hfi_voltage_max = hfi_voltage_max_old;
			motor->m_conf->foc_sl_erpm_hfi = sl_erpm_hfi_old;
			motor->m_conf->foc_control_sample_mode = foc_control_sample_mode_old;
			motor->m_conf->foc_hfi_samples = samples_old;
			motor->m_conf->foc_current_sample_mode = foc_current_sample_mode_old;

			mcpwm_foc_set_configuration(motor->m_conf);

			mc_interface_unlock();

			return fault;
		}

		chThdSleepMilliseconds(10);

		float real_bin0, imag_bin0;
		float real_bin2, imag_bin2;
		float real_bin0_i, imag_bin0_i;

		motor->m_hfi.fft_bin0_func((float*)motor->m_hfi.buffer, &real_bin0, &imag_bin0);
		motor->m_hfi.fft_bin2_func((float*)motor->m_hfi.buffer, &real_bin2, &imag_bin2);
		motor->m_hfi.fft_bin0_func((float*)motor->m_hfi.buffer_current, &real_bin0_i, &imag_bin0_i);

		l_sum += real_bin0;
		i_sum += real_bin0_i;

		// See https://vesc-project.com/comment/8338#comment-8338
		ld_lq_diff_sum += 4.0 * NORM2_f(real_bin2, imag_bin2);

		iterations++;
	}

	mcpwm_foc_set_current(0.0);

	motor->m_conf->foc_sensor_mode = sensor_mode_old;
	motor->m_conf->foc_f_zv = f_zv_old;
	motor->m_conf->foc_hfi_voltage_start = hfi_voltage_start_old;
	motor->m_conf->foc_hfi_voltage_run = hfi_voltage_run_old;
	motor->m_conf->foc_hfi_voltage_max = hfi_voltage_max_old;
	motor->m_conf->foc_sl_erpm_hfi = sl_erpm_hfi_old;
	motor->m_conf->foc_control_sample_mode = foc_control_sample_mode_old;
	motor->m_conf->foc_hfi_samples = samples_old;
	motor->m_conf->foc_current_sample_mode = foc_current_sample_mode_old;

	mcpwm_foc_set_configuration(motor->m_conf);

	mc_interface_unlock();

	// The observer is more stable when the inductance is underestimated compared to overestimated,
	// so scale it by 0.8. This helps motors that start to saturate at higher currents and when
	// the hardware has problems measuring the inductance correctly. Another reason for decreasing the
	// measured value is that delays in the hardware and/or a high resistance compared to inductance
	// will cause the value to be overestimated.
	// NOTE: This used to be 0.8, but was changed to 0.9 as that works better with HFIv2 on most motors.
	float ind_scale_factor = 0.9;

	if (curr) {
		*curr = i_sum / iterations;
	}

	if (ld_lq_diff) {
		*ld_lq_diff = (ld_lq_diff_sum / iterations) * 1e6 * ind_scale_factor;
	}

	*inductance = (l_sum / iterations) * 1e6 * ind_scale_factor;
	return fault;
}

/**
 * Measure the motor inductance with short voltage pulses. The difference from the
 * other function is that this one will aim for a specific measurement current. It
 * will also use an appropriate switching frequency.
 *
 * @param curr_goal
 * The measurement current to aim for.
 *
 * @param samples
 * The number of samples to average over.
 *
 * @param *curr
 * The current that was used for this measurement.
 *
 * @inductance
 * The average d and q axis inductance in uH.
 *
 * @return
 * The fault code
 */
int mcpwm_foc_measure_inductance_current(float curr_goal, int samples, float *curr, float *ld_lq_diff, float *inductance) {
	int fault = FAULT_CODE_NONE;
	float duty_last = 0.0;
	for (float i = 0.02;i < 0.5;i *= 1.5) {
		utils_truncate_number_abs(&i, 0.6);
		float i_tmp;
		fault = mcpwm_foc_measure_inductance(i, 10, &i_tmp, 0, 0);
		if (fault != FAULT_CODE_NONE) {
			return fault;
		}

		duty_last = i;
		if (i_tmp >= curr_goal) {
			break;
		}
	}
	fault = mcpwm_foc_measure_inductance(duty_last, samples, curr, ld_lq_diff, inductance);
	return fault;
}

bool mcpwm_foc_beep(float freq, float time, float voltage) {
	volatile motor_all_state_t *motor = get_motor_now();

	mc_foc_sensor_mode sensor_mode_old = motor->m_conf->foc_sensor_mode;
	float f_zv_old = motor->m_conf->foc_f_zv;
	float hfi_voltage_start_old = motor->m_conf->foc_hfi_voltage_start;
	float hfi_voltage_run_old = motor->m_conf->foc_hfi_voltage_run;
	float hfi_voltage_max_old = motor->m_conf->foc_hfi_voltage_max;
	float sl_erpm_hfi_old = motor->m_conf->foc_sl_erpm_hfi;
	mc_foc_control_sample_mode foc_control_sample_mode_old = motor->m_conf->foc_control_sample_mode;
	foc_hfi_samples samples_old = motor->m_conf->foc_hfi_samples;
	uint16_t start_samples_old = motor->m_conf->foc_hfi_start_samples;

	mc_interface_lock();
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw((motor_all_state_t*)motor);

	motor->m_conf->foc_sensor_mode = FOC_SENSOR_MODE_HFI;
	motor->m_conf->foc_hfi_voltage_start = voltage;
	motor->m_conf->foc_hfi_voltage_run = voltage;
	motor->m_conf->foc_hfi_voltage_max = voltage;
	motor->m_conf->foc_sl_erpm_hfi = 20000.0;
	motor->m_conf->foc_control_sample_mode = FOC_CONTROL_SAMPLE_MODE_V0;
	motor->m_conf->foc_hfi_samples = HFI_SAMPLES_8;
	motor->m_conf->foc_hfi_start_samples = 10;

	freq *= 4.0;

	if (freq > 3500) {
		motor->m_conf->foc_sensor_mode = FOC_SENSOR_MODE_HFI_V3;
		freq /= 8.0;
	}

	motor->m_conf->foc_f_zv = freq * 8.0;

	utils_truncate_number(&motor->m_conf->foc_f_zv, 3.0e3, 30.0e3);

	mcpwm_foc_set_configuration(motor->m_conf);

	chThdSleepMilliseconds(1);

	timeout_reset();
	mcpwm_foc_set_duty(0.0);

	int ms_sleep = (time * 1000.0) - 1;
	if (ms_sleep > 0) {
		chThdSleepMilliseconds(ms_sleep);
	}

	mcpwm_foc_set_current(0.0);

	motor->m_conf->foc_sensor_mode = sensor_mode_old;
	motor->m_conf->foc_f_zv = f_zv_old;
	motor->m_conf->foc_hfi_voltage_start = hfi_voltage_start_old;
	motor->m_conf->foc_hfi_voltage_run = hfi_voltage_run_old;
	motor->m_conf->foc_hfi_voltage_max = hfi_voltage_max_old;
	motor->m_conf->foc_sl_erpm_hfi = sl_erpm_hfi_old;
	motor->m_conf->foc_control_sample_mode = foc_control_sample_mode_old;
	motor->m_conf->foc_hfi_samples = samples_old;
	motor->m_conf->foc_hfi_start_samples = start_samples_old;

	mcpwm_foc_set_configuration(motor->m_conf);

	mc_interface_unlock();

	return true;
}

/**
 * Automatically measure the resistance and inductance of the motor with small steps.
 *
 * @param res
 * The measured resistance in ohm.
 *
 * @param ind
 * The measured inductance in microhenry.
 *
 * @param ld_lq_diff
 * The measured difference in D axis and Q axis inductance.
 *
 * @return
 * The fault code
 */
int mcpwm_foc_measure_res_ind(float *res, float *ind, float *ld_lq_diff) {
	volatile motor_all_state_t *motor = get_motor_now();
	int fault = FAULT_CODE_NONE;

	const float kp_old = motor->m_conf->foc_current_kp;
	const float ki_old = motor->m_conf->foc_current_ki;
	const float res_old = motor->m_conf->foc_motor_r;

	motor->m_conf->foc_current_kp = 0.001;
	motor->m_conf->foc_current_ki = 1.0;

	float i_last = 0.0;
	for (float i = 2.0;i < (motor->m_conf->l_current_max / 2.0);i *= 1.5) {
		float r_tmp = 0.0;
		fault = mcpwm_foc_measure_resistance(i, 20, false, &r_tmp);
		if (fault != FAULT_CODE_NONE || r_tmp == 0.0) {
			goto exit_measure_res_ind;
		}
		if (i > (1.0 / r_tmp)) {
			i_last = i;
			break;
		}
	}

	if (i_last < 0.01) {
		i_last = (motor->m_conf->l_current_max / 2.0);
	}

#ifdef HW_AXIOM_FORCE_HIGH_CURRENT_MEASUREMENTS
	i_last = (motor->m_conf->l_current_max / 2.0);
#endif

	fault = mcpwm_foc_measure_resistance(i_last, 200, true, res);
	if (fault == FAULT_CODE_NONE && *res != 0.0) {
		motor->m_conf->foc_motor_r = *res;
		mcpwm_foc_set_current(0.0);
		chThdSleepMilliseconds(10);
		fault = mcpwm_foc_measure_inductance_current(i_last, 200, 0, ld_lq_diff, ind);
	}

	exit_measure_res_ind:
	motor->m_conf->foc_current_kp = kp_old;
	motor->m_conf->foc_current_ki = ki_old;
	motor->m_conf->foc_motor_r = res_old;
	return fault;
}

/**
 * Run the motor in open loop and figure out at which angles the hall sensors are.
 *
 * @param current
 * Current to use.
 *
 * @param hall_table
 * Table to store the result to.
 *
 * @result
 * true: Success
 * false: Something went wrong
 *
 * @return
 * The fault code
 */
int mcpwm_foc_hall_detect(float current, uint8_t *hall_table, bool *result) {
	volatile motor_all_state_t *motor = get_motor_now();
	int fault = FAULT_CODE_NONE;
	mc_interface_lock();

	motor->m_phase_override = true;
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_control_mode = CONTROL_MODE_CURRENT;
	motor->m_motor_released = false;
	motor->m_state = MC_STATE_RUNNING;

	// MTPA overrides id target
	MTPA_MODE mtpa_old = motor->m_conf->foc_mtpa_mode;
	motor->m_conf->foc_mtpa_mode = MTPA_MODE_OFF;

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Lock the motor
	motor->m_phase_now_override = 0;

	*result = false;

	for (int i = 0;i < 1000;i++) {
		motor->m_id_set = (float)i * current / 1000.0;
		fault = mc_interface_get_fault();
		if (fault != FAULT_CODE_NONE) {
			goto exit_hall_detect;
		}
		chThdSleepMilliseconds(1);
	}

	float sin_hall[8];
	float cos_hall[8];
	int hall_iterations[8];
	memset(sin_hall, 0, sizeof(sin_hall));
	memset(cos_hall, 0, sizeof(cos_hall));
	memset(hall_iterations, 0, sizeof(hall_iterations));

	// Forwards
	for (int i = 0;i < 3;i++) {
		for (int j = 0;j < 360;j++) {
			motor->m_phase_now_override = DEG2RAD_f(j);
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_hall_detect;
			}
			chThdSleepMilliseconds(5);

			int hall = utils_read_hall(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
			float s, c;
			sincosf(motor->m_phase_now_override, &s, &c);
			sin_hall[hall] += s;
			cos_hall[hall] += c;
			hall_iterations[hall]++;
		}
	}

	// Reverse
	for (int i = 0;i < 3;i++) {
		for (int j = 360;j >= 0;j--) {
			motor->m_phase_now_override = DEG2RAD_f(j);
			fault = mc_interface_get_fault();
			if (fault != FAULT_CODE_NONE) {
				goto exit_hall_detect;
			}
			chThdSleepMilliseconds(5);

			int hall = utils_read_hall(motor != &m_motor_1, motor->m_conf->m_hall_extra_samples);
			float s, c;
			sincosf(motor->m_phase_now_override, &s, &c);
			sin_hall[hall] += s;
			cos_hall[hall] += c;
			hall_iterations[hall]++;
		}
	}

	int fails = 0;
	for(int i = 0;i < 8;i++) {
		if (hall_iterations[i] > 30) {
			float ang = RAD2DEG_f(atan2f(sin_hall[i], cos_hall[i]));
			utils_norm_angle(&ang);
			hall_table[i] = (uint8_t)(ang * 200.0 / 360.0);
		} else {
			hall_table[i] = 255;
			fails++;
		}
	}
	*result = (fails == 2);

	exit_hall_detect:
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;
	motor->m_phase_override = false;
	motor->m_control_mode = CONTROL_MODE_NONE;
	motor->m_state = MC_STATE_OFF;
	stop_pwm_hw((motor_all_state_t*)motor);
	motor->m_conf->foc_mtpa_mode = mtpa_old;
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	return fault;
}

/**
 * Calibrate voltage and current offsets. For the observer to work at low modulation it
 * is very important to get all current and voltage offsets right. Therefore we store
 * the offsets for when the motor is undriven and when it is driven separately. The
 * motor is driven at 50% modulation on all phases when measuring the driven offset, which
 * corresponds to space-vector modulation with 0 amplitude.
 *
 * cal_undriven:
 * Calibrate undriven voltages too. This requires the motor to stand still.
 *
 * return:
 * -1: Timed out while waiting for fault code to go away.
 * 1: Success
 *
 */
#ifndef HW_USE_ALTERNATIVE_DC_CAL
int mcpwm_foc_dc_cal(bool cal_undriven) {
	// Wait max 5 seconds for DRV-fault to go away
	int cnt = 0;
	while(IS_DRV_FAULT()){
		chThdSleepMilliseconds(1);
		cnt++;
		if (cnt > 5000) {
			return -1;
		}
	};

	chThdSleepMilliseconds(1000);

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Measure driven offsets
	// NOTE: One phase is measured at a time while the others are left
	// floating so that no torque is generated in case the motor is spinning
	// at boot.

	const float samples = 1000.0;
	float current_sum[3] = {0.0, 0.0, 0.0};
	float voltage_sum[3] = {0.0, 0.0, 0.0};

	TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);

	// Start PWM on phase 1
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_MOTORS
	float current_sum_m2[3] = {0.0, 0.0, 0.0};
	float voltage_sum_m2[3] = {0.0, 0.0, 0.0};
	TIMER_UPDATE_DUTY_M2(TIM8->ARR / 2, TIM8->ARR / 2, TIM8->ARR / 2);

	stop_pwm_hw((motor_all_state_t*)&m_motor_2);
	PHASE_FILTER_ON_M2();
	TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
#endif

	chThdSleep(1);

	for (float i = 0;i < samples;i++) {
		current_sum[0] += m_motor_1.m_currents_adc[0];
		voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1);
#ifdef HW_HAS_DUAL_MOTORS
		current_sum_m2[0] += m_motor_2.m_currents_adc[0];
		voltage_sum_m2[0] += ADC_VOLTS(ADC_IND_SENS4);
#endif
		chThdSleep(1);
	}

	// Start PWM on phase 2
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_MOTORS
	stop_pwm_hw((motor_all_state_t*)&m_motor_2);
	PHASE_FILTER_ON_M2();
	TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
#endif

	chThdSleep(1);

	for (float i = 0;i < samples;i++) {
		current_sum[1] += m_motor_1.m_currents_adc[1];
		voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2);
#ifdef HW_HAS_DUAL_MOTORS
		current_sum_m2[1] += m_motor_2.m_currents_adc[1];
		voltage_sum_m2[1] += ADC_VOLTS(ADC_IND_SENS5);
#endif
		chThdSleep(1);
	}

	// Start PWM on phase 3
	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_MOTORS
	stop_pwm_hw((motor_all_state_t*)&m_motor_2);
	PHASE_FILTER_ON_M2();
	TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);
	TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
#endif

	chThdSleep(1);

	for (float i = 0;i < samples;i++) {
		current_sum[2] += m_motor_1.m_currents_adc[2];
		voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3);
#ifdef HW_HAS_DUAL_MOTORS
		current_sum_m2[2] += m_motor_2.m_currents_adc[2];
		voltage_sum_m2[2] += ADC_VOLTS(ADC_IND_SENS6);
#endif
		chThdSleep(1);
	}

	stop_pwm_hw((motor_all_state_t*)&m_motor_1);

	m_motor_1.m_conf->foc_offsets_current[0] = current_sum[0] / samples;
	m_motor_1.m_conf->foc_offsets_current[1] = current_sum[1] / samples;
	m_motor_1.m_conf->foc_offsets_current[2] = current_sum[2] / samples;

	voltage_sum[0] /= samples;
	voltage_sum[1] /= samples;
	voltage_sum[2] /= samples;
	float v_avg = (voltage_sum[0] + voltage_sum[1] + voltage_sum[2]) / 3.0;

	m_motor_1.m_conf->foc_offsets_voltage[0] = voltage_sum[0] - v_avg;
	m_motor_1.m_conf->foc_offsets_voltage[1] = voltage_sum[1] - v_avg;
	m_motor_1.m_conf->foc_offsets_voltage[2] = voltage_sum[2] - v_avg;

#ifdef HW_HAS_DUAL_MOTORS
	stop_pwm_hw((motor_all_state_t*)&m_motor_2);

	m_motor_2.m_conf->foc_offsets_current[0] = current_sum_m2[0] / samples;
	m_motor_2.m_conf->foc_offsets_current[1] = current_sum_m2[1] / samples;
	m_motor_2.m_conf->foc_offsets_current[2] = current_sum_m2[2] / samples;

	voltage_sum_m2[0] /= samples;
	voltage_sum_m2[1] /= samples;
	voltage_sum_m2[2] /= samples;
	v_avg = (voltage_sum_m2[0] + voltage_sum_m2[1] + voltage_sum_m2[2]) / 3.0;

	m_motor_2.m_conf->foc_offsets_voltage[0] = voltage_sum_m2[0] - v_avg;
	m_motor_2.m_conf->foc_offsets_voltage[1] = voltage_sum_m2[1] - v_avg;
	m_motor_2.m_conf->foc_offsets_voltage[2] = voltage_sum_m2[2] - v_avg;
#endif

	// Measure undriven offsets

	if (cal_undriven) {
		chThdSleepMilliseconds(10);

		voltage_sum[0] = 0.0; voltage_sum[1] = 0.0; voltage_sum[2] = 0.0;
#ifdef HW_HAS_DUAL_MOTORS
		voltage_sum_m2[0] = 0.0; voltage_sum_m2[1] = 0.0; voltage_sum_m2[2] = 0.0;
#endif

		for (float i = 0;i < samples;i++) {
			v_avg = (ADC_VOLTS(ADC_IND_SENS1) + ADC_VOLTS(ADC_IND_SENS2) + ADC_VOLTS(ADC_IND_SENS3)) / 3.0;
			voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1) - v_avg;
			voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2) - v_avg;
			voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3) - v_avg;
#ifdef HW_HAS_DUAL_MOTORS
			v_avg = (ADC_VOLTS(ADC_IND_SENS4) + ADC_VOLTS(ADC_IND_SENS5) + ADC_VOLTS(ADC_IND_SENS6)) / 3.0;
			voltage_sum_m2[0] += ADC_VOLTS(ADC_IND_SENS4) - v_avg;
			voltage_sum_m2[1] += ADC_VOLTS(ADC_IND_SENS5) - v_avg;
			voltage_sum_m2[2] += ADC_VOLTS(ADC_IND_SENS6) - v_avg;
#endif
			chThdSleep(1);
		}

		stop_pwm_hw((motor_all_state_t*)&m_motor_1);

		voltage_sum[0] /= samples;
		voltage_sum[1] /= samples;
		voltage_sum[2] /= samples;

		m_motor_1.m_conf->foc_offsets_voltage_undriven[0] = voltage_sum[0];
		m_motor_1.m_conf->foc_offsets_voltage_undriven[1] = voltage_sum[1];
		m_motor_1.m_conf->foc_offsets_voltage_undriven[2] = voltage_sum[2];
#ifdef HW_HAS_DUAL_MOTORS
		stop_pwm_hw((motor_all_state_t*)&m_motor_2);

		voltage_sum_m2[0] /= samples;
		voltage_sum_m2[1] /= samples;
		voltage_sum_m2[2] /= samples;

		m_motor_2.m_conf->foc_offsets_voltage_undriven[0] = voltage_sum_m2[0];
		m_motor_2.m_conf->foc_offsets_voltage_undriven[1] = voltage_sum_m2[1];
		m_motor_2.m_conf->foc_offsets_voltage_undriven[2] = voltage_sum_m2[2];
#endif
	}

	// TODO: Make sure that offsets are no more than e.g. 5%, as larger values indicate hardware problems.

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	m_dccal_done = true;

	return 1;
}
#else
// WARNING: This calibration routine can only be run when the motor is not spinning.
// For low side shunt hardware with high capacitance mosfets this works a lot better
int mcpwm_foc_dc_cal(bool cal_undriven) {
	// Wait max 5 seconds for DRV-fault to go away
	int cnt = 0;
	while(IS_DRV_FAULT()){
		chThdSleepMilliseconds(1);
		cnt++;
		if (cnt > 5000) {
			return -1;
		}
	};

	chThdSleepMilliseconds(1000);

	// Disable timeout
	systime_t tout = timeout_get_timeout_msec();
	float tout_c = timeout_get_brake_current();
	KILL_SW_MODE tout_ksw = timeout_get_kill_sw_mode();
	timeout_reset();
	timeout_configure(60000, 0.0, KILL_SW_MODE_DISABLED);

	// Measure driven offsets
	const float samples = 1000.0;
	float current_sum[3] = {0.0, 0.0, 0.0};
	float voltage_sum[3] = {0.0, 0.0, 0.0};

	TIMER_UPDATE_DUTY_M1(TIM1->ARR / 2, TIM1->ARR / 2, TIM1->ARR / 2);

	stop_pwm_hw((motor_all_state_t*)&m_motor_1);
	PHASE_FILTER_ON();
	
	// Start PWM on all phases at 50% to get a V0 measurement
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
		
	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

	chThdSleep(1);	

	for (float i = 0; i < samples; i++) {
		current_sum[0] += m_motor_1.m_currents_adc[0];
		voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1);
		current_sum[1] += m_motor_1.m_currents_adc[1];
		voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2);
		current_sum[2] += m_motor_1.m_currents_adc[2];
		voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3);
		chThdSleep(1);
	}	

	stop_pwm_hw((motor_all_state_t*)&m_motor_1);

	m_motor_1.m_conf->foc_offsets_current[0] = current_sum[0] / samples;
	m_motor_1.m_conf->foc_offsets_current[1] = current_sum[1] / samples;
	m_motor_1.m_conf->foc_offsets_current[2] = current_sum[2] / samples;

	voltage_sum[0] /= samples;
	voltage_sum[1] /= samples;
	voltage_sum[2] /= samples;
	float v_avg = (voltage_sum[0] + voltage_sum[1] + voltage_sum[2]) / 3.0;

	m_motor_1.m_conf->foc_offsets_voltage[0] = voltage_sum[0] - v_avg;
	m_motor_1.m_conf->foc_offsets_voltage[1] = voltage_sum[1] - v_avg;
	m_motor_1.m_conf->foc_offsets_voltage[2] = voltage_sum[2] - v_avg;

	// Measure undriven offsets

	if (cal_undriven) {
		chThdSleepMilliseconds(10);

		voltage_sum[0] = 0.0; voltage_sum[1] = 0.0; voltage_sum[2] = 0.0;

		for (float i = 0;i < samples;i++) {
			v_avg = (ADC_VOLTS(ADC_IND_SENS1) + ADC_VOLTS(ADC_IND_SENS2) + ADC_VOLTS(ADC_IND_SENS3)) / 3.0;
			voltage_sum[0] += ADC_VOLTS(ADC_IND_SENS1) - v_avg;
			voltage_sum[1] += ADC_VOLTS(ADC_IND_SENS2) - v_avg;
			voltage_sum[2] += ADC_VOLTS(ADC_IND_SENS3) - v_avg;

			chThdSleep(1);
		}

		stop_pwm_hw((motor_all_state_t*)&m_motor_1);

		voltage_sum[0] /= samples;
		voltage_sum[1] /= samples;
		voltage_sum[2] /= samples;

		m_motor_1.m_conf->foc_offsets_voltage_undriven[0] = voltage_sum[0];
		m_motor_1.m_conf->foc_offsets_voltage_undriven[1] = voltage_sum[1];
		m_motor_1.m_conf->foc_offsets_voltage_undriven[2] = voltage_sum[2];
	}

	// TODO: Make sure that offsets are no more than e.g. 5%, as larger values indicate hardware problems.

	// Enable timeout
	timeout_configure(tout, tout_c, tout_ksw);
	mc_interface_unlock();

	m_dccal_done = true;

	return 1;
}
#endif

void mcpwm_foc_print_state(void) {
	commands_printf("Mod d:     %.2f", (double)get_motor_now()->m_motor_state.mod_d);
	commands_printf("Mod q:     %.2f", (double)get_motor_now()->m_motor_state.mod_q);
	commands_printf("Mod q flt: %.2f", (double)get_motor_now()->m_motor_state.mod_q_filter);
	commands_printf("Duty:      %.2f", (double)get_motor_now()->m_motor_state.duty_now);
	commands_printf("Vd:        %.2f", (double)get_motor_now()->m_motor_state.vd);
	commands_printf("Vq:        %.2f", (double)get_motor_now()->m_motor_state.vq);
	commands_printf("Phase:     %.2f", (double)get_motor_now()->m_motor_state.phase);
	commands_printf("V_alpha:   %.2f", (double)get_motor_now()->m_motor_state.v_alpha);
	commands_printf("V_beta:    %.2f", (double)get_motor_now()->m_motor_state.v_beta);
	commands_printf("id:        %.2f", (double)get_motor_now()->m_motor_state.id);
	commands_printf("iq:        %.2f", (double)get_motor_now()->m_motor_state.iq);
	commands_printf("id_filter: %.2f", (double)get_motor_now()->m_motor_state.id_filter);
	commands_printf("iq_filter: %.2f", (double)get_motor_now()->m_motor_state.iq_filter);
	commands_printf("id_target: %.2f", (double)get_motor_now()->m_motor_state.id_target);
	commands_printf("iq_target: %.2f", (double)get_motor_now()->m_motor_state.iq_target);
	commands_printf("i_abs:     %.2f", (double)get_motor_now()->m_motor_state.i_abs);
	commands_printf("i_abs_flt: %.2f", (double)get_motor_now()->m_motor_state.i_abs_filter);
	commands_printf("Obs_x1:    %.2f", (double)get_motor_now()->m_observer_state.x1);
	commands_printf("Obs_x2:    %.2f", (double)get_motor_now()->m_observer_state.x2);
	commands_printf("lambda_est:%.4f", (double)get_motor_now()->m_observer_state.lambda_est);
	commands_printf("vd_int:    %.2f", (double)get_motor_now()->m_motor_state.vd_int);
	commands_printf("vq_int:    %.2f", (double)get_motor_now()->m_motor_state.vq_int);
	commands_printf("off_delay: %.2f", (double)get_motor_now()->m_current_off_delay);
}

float mcpwm_foc_get_last_adc_isr_duration(void) {
	return m_last_adc_isr_duration;
}

void mcpwm_foc_tim_sample_int_handler(void) {
	if (m_init_done) {
		// Generate COM event here for synchronization
		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);

		virtual_motor_int_handler(
				m_motor_1.m_motor_state.v_alpha,
				m_motor_1.m_motor_state.v_beta);
	}
}

void mcpwm_foc_adc_int_handler(void *p, uint32_t flags) {
	(void)p;
	(void)flags;

	static int skip = 0;
	if (++skip == FOC_CONTROL_LOOP_FREQ_DIVIDER) {  // 修改这个可以达到减采样的效果
		skip = 0;
	} else {
		return;
	}

	uint32_t t_start = timer_time_now();

	bool is_v7 = !(TIM1->CR1 & TIM_CR1_DIR);  //用TIM1状态判断是不是“v7”
	int norm_curr_ofs = 0;

#ifdef HW_HAS_DUAL_MOTORS
	bool is_second_motor = is_v7;
	norm_curr_ofs = is_second_motor ? 3 : 0;
	motor_all_state_t *motor_now = is_second_motor ? (motor_all_state_t*)&m_motor_2 : (motor_all_state_t*)&m_motor_1;
	motor_all_state_t *motor_other = is_second_motor ? (motor_all_state_t*)&m_motor_1 : (motor_all_state_t*)&m_motor_2;
	m_isr_motor = is_second_motor ? 2 : 1;
#ifdef HW_HAS_3_SHUNTS
	volatile TIM_TypeDef *tim = is_second_motor ? TIM8 : TIM1;
#endif
#else
	motor_all_state_t *motor_other = (motor_all_state_t*)&m_motor_1;
	motor_all_state_t *motor_now = (motor_all_state_t*)&m_motor_1;;
	m_isr_motor = 1;        //对单motor，用两个状态来描述motor1，isr标志置1
#ifdef HW_HAS_3_SHUNTS
	volatile TIM_TypeDef *tim = TIM1; //三个采样电阻，用TIM1？
#endif
#endif

	mc_configuration *conf_now = motor_now->m_conf;
	mc_configuration *conf_other = motor_other->m_conf;

	bool skip_interpolation = motor_other->m_cc_was_hfi;

// 	mc_interface_selflock();

// #if defined  STALLING_DETECT
 	// mc_interface_max_current_detect();
// #endif

	// Update modulation for V7 and collect current samples. This is used by the HFI.
	if (motor_other->m_duty_next_set) {  // 以下一大堆是HFI专用
		motor_other->m_duty_next_set = false;
		skip_interpolation = true;
#ifdef HW_HAS_DUAL_MOTORS
		float curr0;
		float curr1;

		if (is_second_motor) {
			curr0 = (GET_CURRENT1() - conf_other->foc_offsets_current[0]) * FAC_CURRENT;
			curr1 = (GET_CURRENT2() - conf_other->foc_offsets_current[1]) * FAC_CURRENT;
			TIMER_UPDATE_DUTY_M1(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
		} else {
			curr0 = (GET_CURRENT1_M2() - conf_other->foc_offsets_current[0]) * FAC_CURRENT;
			curr1 = (GET_CURRENT2_M2() - conf_other->foc_offsets_current[1]) * FAC_CURRENT;
			TIMER_UPDATE_DUTY_M2(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
		}
#else
		float curr0 = (GET_CURRENT1() - conf_other->foc_offsets_current[0]) * FAC_CURRENT;
		float curr1 = (GET_CURRENT2() - conf_other->foc_offsets_current[1]) * FAC_CURRENT; //采样了

		TIMER_UPDATE_DUTY_M1(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next); //更新定时器占空比，M1用TIM1控制  // 但是哪来的占空比?
#ifdef HW_HAS_DUAL_PARALLEL
		TIMER_UPDATE_DUTY_M2(motor_other->m_duty1_next, motor_other->m_duty2_next, motor_other->m_duty3_next);
#endif
#endif

		motor_other->m_i_alpha_sample_next = curr0;
		motor_other->m_i_beta_sample_next = ONE_BY_SQRT3 * curr0 + TWO_BY_SQRT3 * curr1; //根号3分之1
		// 用两相电流采样即可进行clark变换, 得到i_alpha和i_beta
	}

	bool do_return = false;

#ifndef HW_HAS_DUAL_MOTORS
#ifdef HW_HAS_PHASE_SHUNTS
	if (conf_now->foc_control_sample_mode != FOC_CONTROL_SAMPLE_MODE_V0_V7 && is_v7) {
		do_return = true;
	}
#else
	if (is_v7) {
		do_return = true;
	}
#endif
#endif

#ifdef HW_HAS_PHASE_SHUNTS
	float dt;
	if (conf_now->foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7) {
		dt = 1.0 / conf_now->foc_f_zv;
	} else {
		dt = 1.0 / (conf_now->foc_f_zv / 2.0);
	}
#else
	float dt = 1.0 / (conf_now->foc_f_zv / 2.0); // 为什么是1/2? dt是干什么的?
#endif

	
	// 采样模式是V0_V7插值, 
	if (conf_other->foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7_INTERPOL && !skip_interpolation) {
		float interpolated_phase = motor_other->m_motor_state.phase + motor_other->m_speed_est_fast * dt * 0.5;  //相位插值计算
		utils_norm_angle_rad(&interpolated_phase);

		float s, c;
		utils_fast_sincos_better(interpolated_phase, &s, &c);

		volatile motor_state_t *state_m = &(motor_other->m_motor_state);
		state_m->phase_sin = s;
		state_m->phase_cos = c;
		state_m->mod_alpha_raw = c * state_m->mod_d - s * state_m->mod_q;
		state_m->mod_beta_raw  = c * state_m->mod_q + s * state_m->mod_d;  //疑似帕克变换  // 这是park逆变换, 小孩子不懂事写着玩的
		// 这里mod_d和mod_q的来源是current_control, 也就是电流环控制器

		uint32_t duty1, duty2, duty3, top;
		top = TIM1->ARR;
		foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw,
				top, &duty1, &duty2, &duty3, (uint32_t*)&state_m->svm_sector);  //用SVPWM计算出三相占空比

#ifdef HW_HAS_DUAL_MOTORS
		if (is_second_motor) {
			TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3);
#ifdef HW_HAS_DUAL_PARALLEL
			TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
#endif
		} else {
#ifndef HW_HAS_DUAL_PARALLEL
			TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
#endif
		}
#else
		TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3);  // 直接应用占空比
#endif
	}

	if (do_return) {
		return;
	}

	// Reset the watchdog
	timeout_feed_WDT(THREAD_MCPWM);

#ifdef AD2S1205_SAMPLE_GPIO
	// force a position sample in the AD2S1205 resolver IC (falling edge)
	palClearPad(AD2S1205_SAMPLE_GPIO, AD2S1205_SAMPLE_PIN);
#endif

#ifdef HW_HAS_DUAL_MOTORS
	float curr0 = 0;
	float curr1 = 0;

	if (is_second_motor) {
		curr0 = GET_CURRENT1_M2();
		curr1 = GET_CURRENT2_M2();
	} else {
		curr0 = GET_CURRENT1();
		curr1 = GET_CURRENT2();
	}
#else
	float curr0 = GET_CURRENT1();
	float curr1 = GET_CURRENT2();
#ifdef HW_HAS_DUAL_PARALLEL
	curr0 += GET_CURRENT1_M2();
	curr1 += GET_CURRENT2_M2();
#endif
#endif

#ifdef HW_HAS_3_SHUNTS
#ifdef HW_HAS_DUAL_MOTORS
	float curr2 = is_second_motor ? GET_CURRENT3_M2() : GET_CURRENT3();
#else
	float curr2 = GET_CURRENT3();  // 我们的电调有三个采样电阻
#ifdef HW_HAS_DUAL_PARALLEL
	curr2 += GET_CURRENT3_M2();
#endif
#endif
#endif

	motor_now->m_currents_adc[0] = curr0;
	motor_now->m_currents_adc[1] = curr1;
#ifdef HW_HAS_3_SHUNTS
	motor_now->m_currents_adc[2] = curr2;
#else
	motor_now->m_currents_adc[2] = 0.0;
#endif

	curr0 -= conf_now->foc_offsets_current[0];  // offset在init的时候计算过了, 但是为什么会有offset的存在呢?
	curr1 -= conf_now->foc_offsets_current[1];
#ifdef HW_HAS_3_SHUNTS
	curr2 -= conf_now->foc_offsets_current[2];
	motor_now->m_curr_unbalance = curr0 + curr1 + curr2;  // 算这个好像没大用
#endif

	ADC_curr_norm_value[0 + norm_curr_ofs] = curr0;  // 加这个norm_curr_ofs是给第二个motor用的, 一般这个值一直是0
	ADC_curr_norm_value[1 + norm_curr_ofs] = curr1;
#ifdef HW_HAS_3_SHUNTS
	ADC_curr_norm_value[2 + norm_curr_ofs] = curr2;
#else
	ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);  //两个采样电阻，KCL得第三相电流
#endif

	// Use the best current samples depending on the modulation state.
#ifdef HW_HAS_3_SHUNTS
	if (conf_now->foc_current_sample_mode == FOC_CURRENT_SAMPLE_MODE_HIGH_CURRENT) {
		// High current sampling mode. Choose the lower currents to derive the highest one
		// in order to be able to measure higher currents.
		// 大电流 = 小电流+小电流, 当大电流超过ADC的量程时, 直接用小电流的和来表示大电流
		// 硬件配置中似乎提到过, 这个ADC采样范围大概是150A以内.
		
		const float i0_abs = fabsf(ADC_curr_norm_value[0 + norm_curr_ofs]);
		const float i1_abs = fabsf(ADC_curr_norm_value[1 + norm_curr_ofs]);
		const float i2_abs = fabsf(ADC_curr_norm_value[2 + norm_curr_ofs]);

		if (i0_abs > i1_abs && i0_abs > i2_abs) {
			ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
		} else if (i1_abs > i0_abs && i1_abs > i2_abs) {
			ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
		} else if (i2_abs > i0_abs && i2_abs > i1_abs) {
			ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
		}
	} else if (conf_now->foc_current_sample_mode == FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO) {  
		// 采样模式默认是FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO
#ifdef HW_HAS_PHASE_SHUNTS
		if (is_v7) {
			if (tim->CCR1 > 500 && tim->CCR2 > 500) {  //前两项占空比较大，用前两相计算第三相
				// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
				// This is especially important when using HFI.
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			} else {  //三相的占空比都不太大，用高两相计算低一相
				if (tim->CCR1 < tim->CCR2 && tim->CCR1 < tim->CCR3) {
					ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR2 < tim->CCR1 && tim->CCR2 < tim->CCR3) {
					ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR3 < tim->CCR1 && tim->CCR3 < tim->CCR2) {
					ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
				}
			}
		} else {
			if (tim->CCR1 < (tim->ARR - 500) && tim->CCR2 < (tim->ARR - 500)) {  //同上
				// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
				// This is especially important when using HFI.
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			} else {
				if (tim->CCR1 > tim->CCR2 && tim->CCR1 > tim->CCR3) {
					ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR2 > tim->CCR1 && tim->CCR2 > tim->CCR3) {
					ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
				} else if (tim->CCR3 > tim->CCR1 && tim->CCR3 > tim->CCR2) {
					ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
				}
			}
		}
#else
		if (tim->CCR1 < (tim->ARR - 500) && tim->CCR2 < (tim->ARR - 500)) {  
			// 通过占空比来判断, 这个条件说的是duty1和duty2都小于ARR-500
			// Use the same 2 shunts on low modulation, as that will avoid jumps in the current reading.
			// This is especially important when using HFI.
			ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
		} else {  
			// 否则利用低占空比(low modulation)来估计高占空比, 并且高占空比的电流值符号一定与两个低占空比的电流值符号相反
			// 有个问题, ADC采样值能是负数吗?
			if (tim->CCR1 > tim->CCR2 && tim->CCR1 > tim->CCR3) {
				ADC_curr_norm_value[0 + norm_curr_ofs] = -(ADC_curr_norm_value[1 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
			} else if (tim->CCR2 > tim->CCR1 && tim->CCR2 > tim->CCR3) {
				ADC_curr_norm_value[1 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[2 + norm_curr_ofs]);
			} else if (tim->CCR3 > tim->CCR1 && tim->CCR3 > tim->CCR2) {
				ADC_curr_norm_value[2 + norm_curr_ofs] = -(ADC_curr_norm_value[0 + norm_curr_ofs] + ADC_curr_norm_value[1 + norm_curr_ofs]);
			}
		}
#endif
	}
#endif

	float ia = ADC_curr_norm_value[0 + norm_curr_ofs] * FAC_CURRENT;
	float ib = ADC_curr_norm_value[1 + norm_curr_ofs] * FAC_CURRENT;
	float ic = ADC_curr_norm_value[2 + norm_curr_ofs] * FAC_CURRENT;  // 之前还是ADC数值, 现在是电流值了

	// This has to be done for the skip function to have any chance at working with the
	// observer and control loops.
	// TODO: Test this.
	dt *= (float)FOC_CONTROL_LOOP_FREQ_DIVIDER;  // 之前修改FOC_CONTROL_LOOP_FREQ_DIVIDER造成了减采样, 这里按照倍数延长dt.

	UTILS_LP_FAST(motor_now->m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1);  

	volatile float enc_ang = 0;
	volatile bool encoder_is_being_used = false;

	if (virtual_motor_is_connected()) {  //“virtual motor”和使用encoder似乎是冲突的？那么“virtual motor”是不是对应无感？  // 并不是, virtual motor是只有模型没有实物的电机, 类似于跑仿真
		if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER ) {
			enc_ang = virtual_motor_get_angle_deg();
			encoder_is_being_used = true;
		}
	} else {
		if (encoder_is_configured()) {
			enc_ang = encoder_read_deg();   //在这里读编码器数据
			encoder_is_being_used = true;
		}
	}

	if (encoder_is_being_used) {  //使用编码器的数据转换
		float phase_tmp = enc_ang;
		if (conf_now->foc_encoder_inverted) {  //编码器翻转的转换
			phase_tmp = 360.0 - phase_tmp;
		}
		phase_tmp *= conf_now->foc_encoder_ratio;
		phase_tmp -= conf_now->foc_encoder_offset;
		utils_norm_angle((float*)&phase_tmp);  //编码器的转换，增益/偏差/多圈变单圈  // 编码器角度转化为电角度
		motor_now->m_phase_now_encoder = DEG2RAD_f(phase_tmp);//角度->弧度
	}

	if (motor_now->m_state == MC_STATE_RUNNING) {//电机运动的情况下的一堆赋值和计算
		if (conf_now->foc_current_sample_mode == FOC_CURRENT_SAMPLE_MODE_ALL_SENSORS) { //使用三个采样电阻  
			// Full Clarke Transform
			motor_now->m_motor_state.i_alpha = (2.0 / 3.0) * ia - (1.0 / 3.0) * ib - (1.0 / 3.0) * ic;
			motor_now->m_motor_state.i_beta = ONE_BY_SQRT3 * ib - ONE_BY_SQRT3 * ic;
		} else {
			// Clarke transform assuming balanced currents
			motor_now->m_motor_state.i_alpha = ia;
			motor_now->m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib; //只用两个  // 一般是这个
		}

		motor_now->m_i_alpha_sample_with_offset = motor_now->m_motor_state.i_alpha;
		motor_now->m_i_beta_sample_with_offset = motor_now->m_motor_state.i_beta;

		if (motor_now->m_i_alpha_beta_has_offset) {  //计算的α和β轴电流有偏差，进行补偿  // 这里只有开启HFI的时候才会进入
			motor_now->m_motor_state.i_alpha = 0.5 * (motor_now->m_motor_state.i_alpha + motor_now->m_i_alpha_sample_next); //相当于容量等于2的均值滤波？
			motor_now->m_motor_state.i_beta = 0.5 * (motor_now->m_motor_state.i_beta + motor_now->m_i_beta_sample_next);
			motor_now->m_i_alpha_beta_has_offset = false;
		}

		const float duty_now = motor_now->m_motor_state.duty_now;
		const float duty_abs = fabsf(duty_now);
		const float vq_now = motor_now->m_motor_state.vq;  // vq是来自current_control的, 通过PI控制器计算出来的
		const float speed_fast_now = motor_now->m_pll_speed;  //相位环锁定（？）得到现在速度

		float id_set_tmp = motor_now->m_id_set;
		float iq_set_tmp = motor_now->m_iq_set;  // 两个set是通过外部过程计算的, 例如整定过程需要的电流, 或速度环PID计算出的电流
		motor_now->m_motor_state.max_duty = conf_now->l_max_duty;

		if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			utils_truncate_number_abs(&iq_set_tmp, -conf_now->lo_current_min);
		}

		UTILS_LP_FAST(motor_now->m_duty_abs_filtered, duty_abs, 0.01);
		utils_truncate_number_abs((float*)&motor_now->m_duty_abs_filtered, 1.0);

		UTILS_LP_FAST(motor_now->m_duty_filtered, duty_now, 0.01);//这一次滤波的数据 -= 上一次滤波后数据与这一次作比较的差值，乘以0.01
		utils_truncate_number_abs((float*)&motor_now->m_duty_filtered, 1.0);//为什么要搞一大堆低通滤波呢？  // 为了不被尖峰触发报错

		float duty_set = motor_now->m_duty_cycle_set;
		bool control_duty = motor_now->m_control_mode == CONTROL_MODE_DUTY ||
				motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY ||
				motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE;
			// VESC特有的占空比控制模式吗

		// Short all phases (duty=0) the moment the direction or modulation changes sign. That will avoid
		// active braking or changing direction. Keep all phases shorted (duty == 0) until the
		// braking current reaches the set or maximum value, then go back to current control
		// mode. Stay in duty=0 for at least 10 cycles to avoid jumping in and out of that mode rapidly
		// around the threshold.
		if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {  // 电流刹车模式, 直接把duty拉到0
			if ((SIGN(speed_fast_now) != SIGN(motor_now->m_br_speed_before) ||
					SIGN(vq_now) != SIGN(motor_now->m_br_vq_before) ||
					fabsf(motor_now->m_duty_filtered) < 0.001 || motor_now->m_br_no_duty_samples < 10) &&
					motor_now->m_motor_state.i_abs_filter < fabsf(iq_set_tmp)) {
				control_duty = true;
				duty_set = 0.0;
				motor_now->m_br_no_duty_samples = 0;
			} else if (motor_now->m_br_no_duty_samples < 10) {
				control_duty = true;
				duty_set = 0.0;
				motor_now->m_br_no_duty_samples++;
			}
		} else {
			motor_now->m_br_no_duty_samples = 100;
		}

		motor_now->m_br_speed_before = speed_fast_now;
		motor_now->m_br_vq_before = vq_now;

		// Brake when set ERPM is below min ERPM
		if (motor_now->m_control_mode == CONTROL_MODE_SPEED &&
				fabsf(motor_now->m_speed_pid_set_rpm) < conf_now->s_pid_min_erpm) {
			control_duty = true;
			duty_set = 0.0;
		}

		// Reset integrator when leaving duty cycle mode, as the windup protection is not too fast. Making
		// better windup protection is probably better, but not easy.
		if (!control_duty && motor_now->m_was_control_duty) {
			motor_now->m_motor_state.vq_int = motor_now->m_motor_state.vq;
			if (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
					conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
				motor_now->m_motor_state.vq_int -= motor_now->m_pll_speed * conf_now->foc_motor_flux_linkage;
			}
		}
		motor_now->m_was_control_duty = control_duty;

		float current_max_for_duty = conf_now->lo_current_max;
		if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			current_max_for_duty = fabsf(conf_now->lo_current_min);
		}
		// 对电流上下限做限制

		if (!control_duty) {
			motor_now->m_duty_i_term = motor_now->m_motor_state.iq / current_max_for_duty;
			motor_now->duty_was_pi = false;
		}

		if (control_duty) {
			// 对于duty control, 测量 > 设定, 计算电流采用PI控制器; 设定 > 测量, 直接使用最大电流, 
			// 这不基本上还是用PI控制器的电流控制吗??
			// Duty cycle control
			if (fabsf(duty_set) < (duty_abs - 0.01) && (!motor_now->duty_was_pi || 
				SIGN(motor_now->duty_pi_duty_last) == SIGN(duty_now))) {
				// 从高占空比变化到低占空比, 直接截断会很危险, 所以使用PI控制器
				// Truncating the duty cycle here would be dangerous, so run a PI controller.

				motor_now->duty_pi_duty_last = duty_now;
				motor_now->duty_was_pi = true;

				// Reset the integrator in duty mode to not increase the duty if the load suddenly changes. In braking
				// mode this would cause a discontinuity, so there we want to keep the value of the integrator.
				if (motor_now->m_control_mode == CONTROL_MODE_DUTY) {
					if (duty_now > 0.0) {
						if (motor_now->m_duty_i_term > 0.0) {
							motor_now->m_duty_i_term = 0.0;
						}
					} else {
						if (motor_now->m_duty_i_term < 0.0) {
							motor_now->m_duty_i_term = 0.0;
						}
					}
				}

				// Compute error
				float error = duty_set - motor_now->m_motor_state.duty_now;

				// Compute parameters
				float scale = 1.0 / motor_now->m_motor_state.v_bus;
				float p_term = error * conf_now->foc_duty_dowmramp_kp * scale;
				motor_now->m_duty_i_term += error * (conf_now->foc_duty_dowmramp_ki * dt) * scale;

				// I-term wind-up protection
				utils_truncate_number((float*)&motor_now->m_duty_i_term, -1.0, 1.0);

				// Calculate output
				float output = p_term + motor_now->m_duty_i_term;
				utils_truncate_number(&output, -1.0, 1.0);
				iq_set_tmp = output * current_max_for_duty;  //整个这一段，占空比从大到小的控制通过PI控制器来完成，最后这几行就是PI计算
				// 没有涉及i_d的计算
			} else {
				// 从低占空比到高占空比, 不需要PI控制器, 直接使用最大电流
				// If the duty cycle is less than or equal to the set duty cycle just limit
				// the modulation and use the maximum allowed current.
				motor_now->m_duty_i_term = motor_now->m_motor_state.iq / current_max_for_duty;
				motor_now->m_motor_state.max_duty = duty_set;
				if (duty_set > 0.0) {
					iq_set_tmp = current_max_for_duty;
				} else {
					iq_set_tmp = -current_max_for_duty;
				}
				motor_now->duty_was_pi = false; //不使用PI控制器，直接加可以加的最大电流来尽快达到更高的占空比
			}
		} else if (motor_now->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			// Braking
			iq_set_tmp = -SIGN(speed_fast_now) * fabsf(iq_set_tmp);  //刹车，直接给反向电流
		}

		// Set motor phase
		{
			if (!motor_now->m_phase_override) {  // m_phase_override是true的情况只有整定时候, 其他时候都是false
				foc_observer_update(motor_now->m_motor_state.v_alpha, motor_now->m_motor_state.v_beta,
						motor_now->m_motor_state.i_alpha, motor_now->m_motor_state.i_beta,
						dt, &(motor_now->m_observer_state), &motor_now->m_phase_now_observer, motor_now);
				// 电机在旋转, 计算的过程中相位会有变化, 这里根据转速做一个估计补偿.
				// Compensate from the phase lag caused by the switching frequency. This is important for motors
				// that run on high ERPM compared to the switching frequency.
				motor_now->m_phase_now_observer += motor_now->m_pll_speed * dt * (0.5 + conf_now->foc_observer_offset);
				utils_norm_angle_rad((float*)&motor_now->m_phase_now_observer);
			}

			switch (conf_now->foc_sensor_mode) {  //根据不同的encoder选择不同的参数设置，开始
			// 主要是计算相位, 目前FOC进程还处于获取相位(电角度)的阶段    // we can never against the stream i know.
			case FOC_SENSOR_MODE_ENCODER:
				if (encoder_index_found() || virtual_motor_is_connected()) {
					motor_now->m_motor_state.phase = foc_correct_encoder(  //可能是检查编码器的值是否可靠
							motor_now->m_phase_now_observer,
							motor_now->m_phase_now_encoder,
							motor_now->m_speed_est_fast,
							conf_now->foc_sl_erpm,
							motor_now);  // 即使是使用编码器, observer也会运行, 将obsever的值和encoder的值做融合
				} else {
					// Rotate the motor in open loop if the index isn't found.
					motor_now->m_motor_state.phase = motor_now->m_phase_now_encoder_no_index; //编码器失效，开环跑
				}

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;
			case FOC_SENSOR_MODE_HALL:
				motor_now->m_motor_state.phase = foc_correct_hall(motor_now->m_phase_now_observer, dt, motor_now,
						utils_read_hall(motor_now != &m_motor_1, conf_now->m_hall_extra_samples));

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;
			case FOC_SENSOR_MODE_SENSORLESS:
				if (motor_now->m_phase_observer_override) {  
					// 无感, observer复写开启, 似乎不是使用observer计算的本来的相位
					// 有时observer的相位需要略微调整才是准确的, 例如电机有速度的时候, 
					// 会用转速*dt来补偿相位, 补偿后的值储存在override中.
					// 电机0速的时候才不用override.
					motor_now->m_motor_state.phase = motor_now->m_phase_now_observer_override;
					motor_now->m_observer_state.x1 = motor_now->m_observer_x1_override;
					motor_now->m_observer_state.x2 = motor_now->m_observer_x2_override;
					iq_set_tmp += conf_now->foc_sl_openloop_boost_q * SIGN(iq_set_tmp);
					// 这个boost_q的默认值是0, 而且没有任何地方修改这个值.
					if (conf_now->foc_sl_openloop_max_q > conf_now->cc_min_current) {
						utils_truncate_number_abs(&iq_set_tmp, conf_now->foc_sl_openloop_max_q);
					}
				} else {  // 直接使用observer计算的相位
					motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				}

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;

			case FOC_SENSOR_MODE_HFI_START:  // START就是把数值清零吗
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;

				if (motor_now->m_phase_observer_override) {
					motor_now->m_hfi.est_done_cnt = 0;
					motor_now->m_hfi.flip_cnt = 0;

					motor_now->m_min_rpm_hyst_timer = 0.0;
					motor_now->m_min_rpm_timer = 0.0;
					motor_now->m_phase_observer_override = false;
				}

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;

			case FOC_SENSOR_MODE_HFI:
			case FOC_SENSOR_MODE_HFI_V2:
			case FOC_SENSOR_MODE_HFI_V3:
			case FOC_SENSOR_MODE_HFI_V4:
			case FOC_SENSOR_MODE_HFI_V5:  //高频注入
				if (fabsf(RADPS2RPM_f(motor_now->m_speed_est_fast)) > conf_now->foc_sl_erpm_hfi) {
					motor_now->m_hfi.observer_zero_time = 0;
				} else {
					motor_now->m_hfi.observer_zero_time += dt;
				}

				if (motor_now->m_hfi.observer_zero_time < conf_now->foc_hfi_obs_ovr_sec) {
					motor_now->m_hfi.angle = motor_now->m_phase_now_observer;
					motor_now->m_hfi.double_integrator = -motor_now->m_speed_est_fast;
				}
				// 下面这个函数, 把hfi的相位当作encoder来和observer的相位做融合.
				motor_now->m_motor_state.phase = foc_correct_encoder(
						motor_now->m_phase_now_observer,
						motor_now->m_hfi.angle,
						motor_now->m_speed_est_fast,
						conf_now->foc_sl_erpm_hfi,
						motor_now);

				if (!motor_now->m_phase_override && motor_now->m_control_mode != CONTROL_MODE_OPENLOOP_PHASE) {
					id_set_tmp = 0.0;
				}
				break;
			}  //根据不同的encoder选择不同的参数设置，结束
			// 通过不同传感器得到初步的相位的部分, 结束, 总体分为四种情况: 编码器/霍尔传感器/无感/高频注入

			if (motor_now->m_control_mode == CONTROL_MODE_HANDBRAKE) {  //根据不同的控制模式选择相位的计算方式，开始
				// Force the phase to 0 in handbrake mode so that the current simply locks the rotor.
				motor_now->m_motor_state.phase = 0.0;
			} else if (motor_now->m_control_mode == CONTROL_MODE_OPENLOOP ||
					motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY) {
				motor_now->m_openloop_angle += dt * motor_now->m_openloop_speed;  //开环，直接暴力预测位置=当前角度+时间*角速度
				utils_norm_angle_rad((float*)&motor_now->m_openloop_angle);
				motor_now->m_motor_state.phase = motor_now->m_openloop_angle;  // 这是真开环, 会用到这个模式吗
			} else if (motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_PHASE ||
					motor_now->m_control_mode == CONTROL_MODE_OPENLOOP_DUTY_PHASE) {
				motor_now->m_motor_state.phase = motor_now->m_openloop_phase;
			}

			if (motor_now->m_phase_override) {
				motor_now->m_motor_state.phase = motor_now->m_phase_now_override;//用obsever来控制，当前相位 = observer的相位 // <- 并非, 小孩子不懂事乱写的
			}

			utils_fast_sincos_better(motor_now->m_motor_state.phase,
					(float*)&motor_now->m_motor_state.phase_sin,
					(float*)&motor_now->m_motor_state.phase_cos);
		}//根据不同的控制模式选择相位的计算方式，结束  // 以上一坨都是phase

		// Apply MTPA. See: https://github.com/vedderb/bldc/pull/179    //应用单位电流最大转矩比值
		const float ld_lq_diff = conf_now->foc_motor_ld_lq_diff;     //ld-lq diff
		if (conf_now->foc_mtpa_mode != MTPA_MODE_OFF && ld_lq_diff != 0.0) {
			const float lambda = conf_now->foc_motor_flux_linkage;

			float iq_ref = iq_set_tmp;
			if (conf_now->foc_mtpa_mode == MTPA_MODE_IQ_MEASURED) {
				iq_ref = utils_min_abs(iq_set_tmp, motor_now->m_motor_state.iq_filter);
				// 使用iq_set_tmp和iq_filter的最小值来计算.
				//
				// iq_filter是在电流控制器中经过iq低通滤波来的, iq是通过i_alpha和i_beta计算来的,
				// i_alpha和i_beta是用采样的三相电流计算得来的, 在同一个函数的上面就算出来了.
				//
				// 但是 iq_filter是下面control_current函数种计算的, 这里用到的iq_filter是上次计算的值.
				// 既然本次已经有采样结果, 为什么不使用最新的iq计算一次iq_filter, 然后投入使用, 
				// 一定要用上次的值吗? MPTA的计算是基于一定基于上次的iq_filter吗?
				//
				// VESC tool中说, 这个MTPA_MODE_IQ_MEASURED是基于测量的iq来计算id, 使用的是上次测量的iq
				// 我还能理解, 但是为什么实际是取了一个设定和测量的最小值? 
			}
			// 根据神秘公式计算id和iq的值, 还需学习MTPA的原理, 详见上面的链接
			id_set_tmp = (lambda - sqrtf(SQ(lambda) + 8.0 * SQ(ld_lq_diff * iq_ref))) / (4.0 * ld_lq_diff);
			iq_set_tmp = SIGN(iq_set_tmp) * sqrtf(SQ(iq_set_tmp) - SQ(id_set_tmp));
		}

		const float mod_q = motor_now->m_motor_state.mod_q_filter;

		// fw是磁场弱化, 这里公式化减去id和iq电流, 但是当最大弱磁电流设置为0时, 这个值为0.
		// 弱磁是一直在跑的(详见foc_math.c中foc_run_fw函数的引用), 最大弱磁电流决定m_i_fw_set
		// Running FW from the 1 khz timer seems fast enough.
		//run_fw(motor_now, dt);
		id_set_tmp -= motor_now->m_i_fw_set;
		iq_set_tmp -= SIGN(mod_q) * motor_now->m_i_fw_set * conf_now->foc_fw_q_current_factor;

		// Apply current limits
		// TODO: Consider D axis current for the input current as well. Currently this is done using
		// l_in_current_map_start in update_override_limits.
		// mod_q是归一化的vq, 是vq与母线电压的比值.
		if (mod_q > 0.001) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_in_current_min / mod_q, conf_now->lo_in_current_max / mod_q);
		} else if (mod_q < -0.001) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_in_current_max / mod_q, conf_now->lo_in_current_min / mod_q);
		}

		if (mod_q > 0.0) {
			utils_truncate_number(&iq_set_tmp, conf_now->lo_current_min, conf_now->lo_current_max);
		} else {
			utils_truncate_number(&iq_set_tmp, -conf_now->lo_current_max, -conf_now->lo_current_min);
		}

		float current_max_abs = fabsf(utils_max_abs(conf_now->lo_current_max, conf_now->lo_current_min));
		utils_truncate_number_abs(&id_set_tmp, current_max_abs);
		utils_truncate_number_abs(&iq_set_tmp, sqrtf(SQ(current_max_abs) - SQ(id_set_tmp)));

		motor_now->m_motor_state.id_target = id_set_tmp;
		motor_now->m_motor_state.iq_target = iq_set_tmp;
		// 终于算完了, 已经得到正确的目标iq和id值, 接下来是输入电流控制器
		// 控制器中涉及: 
		// 1. iq和id到vq和vd的转换, 有vq和vd的解耦补偿
		// 2. vq和vd到v_alpha和v_beta的转换, 使用的是park逆变换
		// 3. 用v_alpha和v_beta计算SVPWM, 得到三相占空比
		// we can never against the stream i know.
		control_current(motor_now, dt);
	} else {  //电机不运动的情况下的一堆计算
		// 电机不运动, 所有目标电压电流都设置为0, 到那时保留测量值的更新
		// 例如调用update_valpha_vbeta函数更新v_alpha和v_beta, 
		// 以及更新observer, 能够一定程度上追踪相位变化.
		// Motor is not running

		// The current is 0 when the motor is undriven
		motor_now->m_motor_state.i_alpha = 0.0;
		motor_now->m_motor_state.i_beta = 0.0;
		motor_now->m_motor_state.id = 0.0;
		motor_now->m_motor_state.iq = 0.0;
		motor_now->m_motor_state.id_filter = 0.0;
		motor_now->m_motor_state.iq_filter = 0.0;
		motor_now->m_duty_i_term = 0.0;
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
		GET_INPUT_CURRENT_OFFSET(); // TODO: should this be done here?
#endif
		motor_now->m_motor_state.i_bus = 0.0;
		motor_now->m_motor_state.i_abs = 0.0;
		motor_now->m_motor_state.i_abs_filter = 0.0;

		// Track back emf
		update_valpha_vbeta(motor_now, 0.0, 0.0);

		// Run observer     //不主动运动的时候也要跑observer吗
		foc_observer_update(motor_now->m_motor_state.v_alpha, motor_now->m_motor_state.v_beta,
						motor_now->m_motor_state.i_alpha, motor_now->m_motor_state.i_beta,
						dt, &(motor_now->m_observer_state), 0, motor_now);
		motor_now->m_phase_now_observer = utils_fast_atan2(motor_now->m_x2_prev + motor_now->m_observer_state.x2,
														   motor_now->m_x1_prev + motor_now->m_observer_state.x1);

		// The observer phase offset has to be added here as well, with 0.5 switching cycles offset
		// compared to when running. Otherwise going from undriven to driven causes a current
		// spike.
		// 根据电机的转速来估计相位的变化, 如果不做这个补偿, 从静止到运动会有电流尖峰.
		motor_now->m_phase_now_observer += motor_now->m_pll_speed * dt * conf_now->foc_observer_offset;
		utils_norm_angle_rad((float*)&motor_now->m_phase_now_observer);

		motor_now->m_x1_prev = motor_now->m_observer_state.x1;
		motor_now->m_x2_prev = motor_now->m_observer_state.x2;

		// Set motor phase
		{  //和上面的 Set motor phase 应该是一样的
			switch (conf_now->foc_sensor_mode) {
			case FOC_SENSOR_MODE_ENCODER:
				motor_now->m_motor_state.phase = foc_correct_encoder(
						motor_now->m_phase_now_observer,
						motor_now->m_phase_now_encoder,
						motor_now->m_speed_est_fast,
						conf_now->foc_sl_erpm,
						motor_now);
				break;
			case FOC_SENSOR_MODE_HALL:
				motor_now->m_motor_state.phase = foc_correct_hall(motor_now->m_phase_now_observer, dt, motor_now,
						utils_read_hall(motor_now != &m_motor_1, conf_now->m_hall_extra_samples));
				break;
			case FOC_SENSOR_MODE_SENSORLESS:
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				break;
			case FOC_SENSOR_MODE_HFI:
			case FOC_SENSOR_MODE_HFI_V2:
			case FOC_SENSOR_MODE_HFI_V3:
			case FOC_SENSOR_MODE_HFI_V4:
			case FOC_SENSOR_MODE_HFI_V5:
			case FOC_SENSOR_MODE_HFI_START:{
				motor_now->m_motor_state.phase = motor_now->m_phase_now_observer;
				if (fabsf(RADPS2RPM_f(motor_now->m_pll_speed)) < (conf_now->foc_sl_erpm_hfi * 1.1)) {
					motor_now->m_hfi.est_done_cnt = 0;
					motor_now->m_hfi.flip_cnt = 0;
				}
			} break;

			}

			utils_fast_sincos_better(motor_now->m_motor_state.phase,
					(float*)&motor_now->m_motor_state.phase_sin,
					(float*)&motor_now->m_motor_state.phase_cos);
		}

		// HFI Restore
#ifdef HW_HAS_DUAL_MOTORS
		if (is_second_motor) {
			CURRENT_FILTER_ON_M2();
		} else {
			CURRENT_FILTER_ON();
		}
#else
		CURRENT_FILTER_ON();
#endif

		motor_now->m_hfi.ind = 0;
		motor_now->m_hfi.ready = false;
		motor_now->m_hfi.double_integrator = 0.0;
		motor_now->m_hfi.is_samp_n = false;
		motor_now->m_hfi.prev_sample = 0.0;
		motor_now->m_hfi.angle = motor_now->m_motor_state.phase;

		float s = motor_now->m_motor_state.phase_sin;
		float c = motor_now->m_motor_state.phase_cos;

		// Park transform
		float vd_tmp = c * motor_now->m_motor_state.v_alpha + s * motor_now->m_motor_state.v_beta;
		float vq_tmp = c * motor_now->m_motor_state.v_beta  - s * motor_now->m_motor_state.v_alpha;

		UTILS_NAN_ZERO(motor_now->m_motor_state.vd);
		UTILS_NAN_ZERO(motor_now->m_motor_state.vq);

		// 测量到的vd和vq, 经过低通滤波, 接下来用于计算电机占空比
		// 由于是不进行主动驱动的状态, 这里的vd和vq是电机的反电动势吗?
		UTILS_LP_FAST(motor_now->m_motor_state.vd, vd_tmp, 0.2);  
		UTILS_LP_FAST(motor_now->m_motor_state.vq, vq_tmp, 0.2);

		// Set the current controller integrator to the BEMF voltage to avoid
		// a current spike when the motor is driven again. Notice that we have
		// to take decoupling into account.
		motor_now->m_motor_state.vd_int = motor_now->m_motor_state.vd;
		motor_now->m_motor_state.vq_int = motor_now->m_motor_state.vq;

		if (conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF || 
				conf_now->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {  //反电动势（BEMF）解耦
			motor_now->m_motor_state.vq_int -= motor_now->m_pll_speed * conf_now->foc_motor_flux_linkage;
			// 也就是对反电动势进行补偿
		}

		// Update corresponding modulation
		/* voltage_normalize = 1/(2/3*V_bus) */
		const float voltage_normalize = 1.5 / motor_now->m_motor_state.v_bus;  //电压归一化，但是为什么呢？

		motor_now->m_motor_state.mod_d = motor_now->m_motor_state.vd * voltage_normalize;
		motor_now->m_motor_state.mod_q = motor_now->m_motor_state.vq * voltage_normalize;
		UTILS_NAN_ZERO(motor_now->m_motor_state.mod_q_filter);
		UTILS_LP_FAST(motor_now->m_motor_state.mod_q_filter, motor_now->m_motor_state.mod_q, 0.2);
		utils_truncate_number_abs((float*)&motor_now->m_motor_state.mod_q_filter, 1.0);  //计算出了vq和vd
	}

	// Calculate duty cycle
	motor_now->m_motor_state.duty_now = SIGN(motor_now->m_motor_state.vq) *
			NORM2_f(motor_now->m_motor_state.mod_d, motor_now->m_motor_state.mod_q) * TWO_BY_SQRT3;  //占空比是这样被计算出来的，似乎和电压有关

	float phase_for_speed_est = 0.0;
	switch (conf_now->foc_speed_soure) {
	case FOC_SPEED_SRC_CORRECTED:
		phase_for_speed_est = motor_now->m_motor_state.phase;
		break;
	case FOC_SPEED_SRC_OBSERVER:
		phase_for_speed_est = motor_now->m_phase_now_observer;
		break;
	};

	// PLL速度估计是什么? 如何实现? 还需学习
	//
	// 原理: 
	// 本质是用相位进行估计(相位和速度一起估计). 使用这个周期的实际相位和上个周期对这个周期的相位估计, 
	// 比较误差, 使用比例来修正相位估计, 积分来修正速度估计, 给出这个周期对下个周期的相位估计(和速度估计).
	// 
	// 目的: 
	// 这样做的主要目的是获得更准确、更实时的电机状态估计（如速度、位置），以便为观测器（如反电动势观测器、
	// 磁链观测器）和高级控制器（如前馈补偿、解耦、速度环、位置环等）提供可靠的输入。
	//
	// Run PLL for speed estimation
	foc_pll_run(phase_for_speed_est, dt, &motor_now->m_pll_phase, &motor_now->m_pll_speed, conf_now);

	// Low latency speed estimation, for e.g. HFI and speed control.
	{
		// 除了pll外, 还进行快读的速度估计
		float diff = utils_angle_difference_rad(phase_for_speed_est, motor_now->m_phase_before_speed_est);
		utils_truncate_number(&diff, -M_PI / 3.0, M_PI / 3.0);

		UTILS_LP_FAST(motor_now->m_speed_est_fast, diff / dt, 0.01);
		UTILS_NAN_ZERO(motor_now->m_speed_est_fast);

		UTILS_LP_FAST(motor_now->m_speed_est_faster, diff / dt, 0.2);
		UTILS_NAN_ZERO(motor_now->m_speed_est_faster);  // 低通滤波系数越大, 响应越快

		float diff_corr = utils_angle_difference_rad(motor_now->m_motor_state.phase, motor_now->m_phase_before_speed_est_corrected);
		utils_truncate_number(&diff_corr, -M_PI / 3.0, M_PI / 3.0);

		UTILS_LP_FAST(motor_now->m_speed_est_fast_corrected, diff_corr / dt, 0.01);
		UTILS_NAN_ZERO(motor_now->m_speed_est_fast_corrected);

		// pll wind-up protection
		utils_truncate_number_abs((float*)&motor_now->m_pll_speed, fabsf(motor_now->m_speed_est_fast) * 3.0);

		motor_now->m_phase_before_speed_est = phase_for_speed_est;
		motor_now->m_phase_before_speed_est_corrected = motor_now->m_motor_state.phase;
	}

	// Update tachometer (resolution = 60 deg as for BLDC)
	// 这个东西好像没用到的样子
	float ph_tmp = motor_now->m_motor_state.phase;
	utils_norm_angle_rad(&ph_tmp);
	int step = (int)floorf((ph_tmp + M_PI) / (2.0 * M_PI) * 6.0);
	utils_truncate_number_int(&step, 0, 5);
	int diff = step - motor_now->m_tacho_step_last;
	motor_now->m_tacho_step_last = step;

	if (diff > 3) {
		diff -= 6;
	} else if (diff < -2) {
		diff += 6;
	}

	motor_now->m_tachometer += diff;
	motor_now->m_tachometer_abs += abs(diff);

	// Track position control angle
	float angle_now = 0.0;
	if (encoder_is_configured()) {  // 在这里进行了某个多圈编码器的更新?
		if (conf_now->m_sensor_port_mode == SENSOR_PORT_MODE_TS5700N8501_MULTITURN) {
			angle_now = encoder_read_deg_multiturn();
		} else {
			angle_now = enc_ang;
		}
	} else {
		angle_now = RAD2DEG_f(motor_now->m_motor_state.phase);
	}

	utils_norm_angle(&angle_now);

	if (conf_now->p_pid_ang_div > 0.98 && conf_now->p_pid_ang_div < 1.02) {
		motor_now->m_pos_pid_now = angle_now;
	} else {
		if (angle_now < 90.0 && motor_now->m_pid_div_angle_last > 270.0) {
			motor_now->m_pid_div_angle_accumulator += 360.0 / conf_now->p_pid_ang_div;
			utils_norm_angle((float*)&motor_now->m_pid_div_angle_accumulator);
		} else if (angle_now > 270.0 && motor_now->m_pid_div_angle_last < 90.0) {
			motor_now->m_pid_div_angle_accumulator -= 360.0 / conf_now->p_pid_ang_div;
			utils_norm_angle((float*)&motor_now->m_pid_div_angle_accumulator);
		}

		motor_now->m_pid_div_angle_last = angle_now;

		motor_now->m_pos_pid_now = motor_now->m_pid_div_angle_accumulator + angle_now / conf_now->p_pid_ang_div;
		utils_norm_angle((float*)&motor_now->m_pos_pid_now);
	}

#ifdef AD2S1205_SAMPLE_GPIO
	// Release sample in the AD2S1205 resolver IC.
	palSetPad(AD2S1205_SAMPLE_GPIO, AD2S1205_SAMPLE_PIN);
#endif

#ifdef HW_HAS_DUAL_MOTORS
	mc_interface_mc_timer_isr(is_second_motor);
#else
	mc_interface_mc_timer_isr(false);
#endif

	m_isr_motor = 0;
	m_last_adc_isr_duration = timer_seconds_elapsed_since(t_start);
	
}

// Private functions

static void timer_update(motor_all_state_t *motor, float dt) {
	foc_run_fw(motor, dt);

	const mc_configuration *conf_now = motor->m_conf;

	// Calculate temperature-compensated parameters here
	if (mc_interface_temp_motor_filtered() > -30.0) {
		float comp_fact = 1.0 + 0.00386 * (mc_interface_temp_motor_filtered() - conf_now->foc_temp_comp_base_temp);
		motor->m_res_temp_comp = conf_now->foc_motor_r * comp_fact;
		motor->m_current_ki_temp_comp = conf_now->foc_current_ki * comp_fact;
	} else {
		motor->m_res_temp_comp = conf_now->foc_motor_r;
		motor->m_current_ki_temp_comp = conf_now->foc_current_ki;
	}

	// Check if it is time to stop the modulation. Notice that modulation is kept on as long as there is
	// field weakening current.
	utils_sys_lock_cnt();
	utils_step_towards((float*)&motor->m_current_off_delay, 0.0, dt);
	if (!motor->m_phase_override && motor->m_state == MC_STATE_RUNNING &&
			(motor->m_control_mode == CONTROL_MODE_CURRENT ||
					motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
					motor->m_control_mode == CONTROL_MODE_HANDBRAKE ||
					motor->m_control_mode == CONTROL_MODE_OPENLOOP ||
					motor->m_control_mode == CONTROL_MODE_OPENLOOP_PHASE)) {

		// This is required to allow releasing the motor when cc_min_current is 0
		float min_current = conf_now->cc_min_current;
		if (min_current < 0.001 && get_motor_now()->m_motor_released) {
			min_current = 0.001;
		}

		if (fabsf(motor->m_iq_set) < min_current &&
				fabsf(motor->m_id_set) < min_current &&
				motor->m_i_fw_set < min_current &&
				motor->m_current_off_delay < dt) {
			motor->m_control_mode = CONTROL_MODE_NONE;
			motor->m_state = MC_STATE_OFF;
			stop_pwm_hw(motor);
		}
	}
	utils_sys_unlock_cnt();

	// Use this to study the openloop timers under experiment plot
#if 0 //画图用的？
	{
		static bool plot_started = false;
		static int plot_div = 0;
		static float plot_int = 0.0;
		static int get_fw_version_cnt = 0;

		if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
			get_fw_version_cnt = commands_get_fw_version_sent_cnt();
			plot_started = false;
		}

		plot_div++;
		if (plot_div >= 10) {
			plot_div = 0;
			if (!plot_started) {
				plot_started = true;
				commands_init_plot("Time", "Val");
				commands_plot_add_graph("m_min_rpm_timer");
				commands_plot_add_graph("m_min_rpm_hyst_timer");
			}

			commands_plot_set_graph(0);
			commands_send_plot_points(plot_int, motor->m_min_rpm_timer);
			commands_plot_set_graph(1);
			commands_send_plot_points(plot_int, motor->m_min_rpm_hyst_timer);
			plot_int++;
		}
	}
#endif

	// Use this to study the observer state in a XY-plot
#if 0
	{
		static bool plot_started = false;
		static int plot_div = 0;
		static int get_fw_version_cnt = 0;

		if (commands_get_fw_version_sent_cnt() != get_fw_version_cnt) {
			get_fw_version_cnt = commands_get_fw_version_sent_cnt();
			plot_started = false;
		}

		plot_div++;
		if (plot_div >= 10) {
			plot_div = 0;
			if (!plot_started) {
				plot_started = true;
				commands_init_plot("X1", "X2");
				commands_plot_add_graph("Observer");
				commands_plot_add_graph("Observer Mag");
			}

			commands_plot_set_graph(0);
			commands_send_plot_points(m_motor_1.m_observer_x1, m_motor_1.m_observer_x2);
			float mag = NORM2_f(m_motor_1.m_observer_x1, m_motor_1.m_observer_x2);
			commands_plot_set_graph(1);
			commands_send_plot_points(0.0, mag);
		}
	}
#endif

	float t_lock = conf_now->foc_sl_openloop_time_lock;
	float t_ramp = conf_now->foc_sl_openloop_time_ramp;
	float t_const = conf_now->foc_sl_openloop_time;

	float openloop_current = fabsf(motor->m_motor_state.iq_filter);
	openloop_current += conf_now->foc_sl_openloop_boost_q;
	if (conf_now->foc_sl_openloop_max_q > 0.0) {
		utils_truncate_number(&openloop_current, 0.0, conf_now->foc_sl_openloop_max_q);
	}

	float openloop_rpm_max = utils_map(openloop_current,
			0.0, conf_now->l_current_max,
			conf_now->foc_openloop_rpm_low * conf_now->foc_openloop_rpm,
			conf_now->foc_openloop_rpm);

	utils_truncate_number_abs(&openloop_rpm_max, conf_now->foc_openloop_rpm);

	float openloop_rpm = openloop_rpm_max;
	if (conf_now->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER) {
		float time_fwd = t_lock + t_ramp + t_const - motor->m_min_rpm_timer;
		if (time_fwd < t_lock) {
			openloop_rpm = 0.0;
		} else if (time_fwd < (t_lock + t_ramp)) {
			openloop_rpm = utils_map(time_fwd, t_lock,
					t_lock + t_ramp, 0.0, openloop_rpm);
		}
	}

	utils_truncate_number_abs(&openloop_rpm, openloop_rpm_max);

	float add_min_speed = 0.0;
	if (motor->m_motor_state.duty_now > 0.0) {
		add_min_speed = RPM2RADPS_f(openloop_rpm) * dt;
	} else {
		add_min_speed = -RPM2RADPS_f(openloop_rpm) * dt;
	}

	// Open loop encoder angle for when the index is not found
	motor->m_phase_now_encoder_no_index += add_min_speed;
	utils_norm_angle_rad((float*)&motor->m_phase_now_encoder_no_index);

	if (fabsf(motor->m_pll_speed) < RPM2RADPS_f(openloop_rpm_max) &&
			motor->m_min_rpm_hyst_timer < conf_now->foc_sl_openloop_hyst) {
		motor->m_min_rpm_hyst_timer += dt;
	} else if (motor->m_min_rpm_hyst_timer > 0.0) {
		motor->m_min_rpm_hyst_timer -= dt;
	}

	// Don't use this in brake mode.
	if (motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE ||
			(motor->m_state == MC_STATE_RUNNING && fabsf(motor->m_motor_state.duty_now) < 0.001)) {
		motor->m_min_rpm_hyst_timer = 0.0;
		motor->m_min_rpm_timer = 0.0;
		motor->m_phase_observer_override = false;
	}

	bool started_now = false;
	if (motor->m_min_rpm_hyst_timer >= conf_now->foc_sl_openloop_hyst &&
			motor->m_min_rpm_timer <= 0.0001) {
		motor->m_min_rpm_timer = t_lock + t_ramp + t_const;
		started_now = true;
	}

	if (motor->m_state != MC_STATE_RUNNING) {
		motor->m_min_rpm_timer = 0.0;
	}

	if (motor->m_min_rpm_timer > 0.0) {
		motor->m_phase_now_observer_override += add_min_speed;  
		// 对observer的相位进行补偿, 这个add_min_speed是根据电机的转速来计算的
		// add_min_speed = rpm * dt

		// When the motor gets stuck it tends to be 90 degrees off, so start the open loop
		// sequence by correcting with 60 degrees.
		if (started_now) {  // 这是什么补偿?
			if (motor->m_motor_state.duty_now > 0.0) {
				motor->m_phase_now_observer_override += M_PI / 3.0;
			} else {
				motor->m_phase_now_observer_override -= M_PI / 3.0;
			}
		}

		utils_norm_angle_rad((float*)&motor->m_phase_now_observer_override);
		motor->m_phase_observer_override = true;
		motor->m_min_rpm_timer -= dt;
		motor->m_min_rpm_hyst_timer = 0.0;

		// Set observer state to help it start tracking when leaving open loop.
		float s, c;
		utils_fast_sincos_better(motor->m_phase_now_observer_override + SIGN(motor->m_motor_state.duty_now) * M_PI / 4.0, &s, &c);
		motor->m_observer_x1_override = c * conf_now->foc_motor_flux_linkage;
		motor->m_observer_x2_override = s * conf_now->foc_motor_flux_linkage;
	} else {
		motor->m_phase_now_observer_override = motor->m_phase_now_observer;
		motor->m_phase_observer_override = false;
	}

	// Samples
	if (motor->m_state == MC_STATE_RUNNING) {
		const volatile float vd_tmp = motor->m_motor_state.vd;
		const volatile float vq_tmp = motor->m_motor_state.vq;
		const volatile float id_tmp = motor->m_motor_state.id;
		const volatile float iq_tmp = motor->m_motor_state.iq;

		motor->m_samples.avg_current_tot += NORM2_f(id_tmp, iq_tmp);
		motor->m_samples.avg_voltage_tot += NORM2_f(vd_tmp, vq_tmp);
		motor->m_samples.sample_num++;
	}

	// Observer gain scaling, based on bus voltage and duty cycle
	float gamma_tmp = utils_map(fabsf(motor->m_motor_state.duty_now),
								0.0, 40.0 / motor->m_motor_state.v_bus,
								0, conf_now->foc_observer_gain);
	if (gamma_tmp < (conf_now->foc_observer_gain_slow * conf_now->foc_observer_gain)) {
		gamma_tmp = conf_now->foc_observer_gain_slow * conf_now->foc_observer_gain;
	}

	// 4.0 scaling is kind of arbitrary, but it should make configs from old VESC Tools more likely to work.
	motor->m_gamma_now = gamma_tmp * 4.0;

	//以下又是一个observer, 电阻观测器
	// Run resistance observer
	// See "An adaptive flux observer for the permanent magnet synchronous motor" 
	// https://doi.org/10.1002/acs.2587
	{
		float res_est_gain = 0.00002;
		float i_abs_sq = SQ(motor->m_motor_state.i_abs);
		motor->m_res_est = motor->m_r_est_state - 0.5 * res_est_gain * conf_now->foc_motor_l * i_abs_sq;
		float res_dot = -res_est_gain * (motor->m_res_est * i_abs_sq + motor->m_speed_est_fast *
				(motor->m_motor_state.i_beta * motor->m_observer_state.x1 - motor->m_motor_state.i_alpha * motor->m_observer_state.x2) -
				(motor->m_motor_state.i_alpha * motor->m_motor_state.v_alpha + motor->m_motor_state.i_beta * motor->m_motor_state.v_beta));
		motor->m_r_est_state += res_dot * dt;

		utils_truncate_number((float*)&motor->m_r_est_state, conf_now->foc_motor_r * 0.25, conf_now->foc_motor_r * 3.0);
	}
}

static void terminal_tmp(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	int top = 1;
	if (argc == 2) {
		float seconds = -1.0;
		sscanf(argv[1], "%f", &seconds);

		if (seconds > 0.0) {
			top = seconds * 2;
		}
	}

	if (top > 1) {
		commands_init_plot("Time", "Temperature");
		commands_plot_add_graph("Temp Measured");
		commands_plot_add_graph("Temp Estimated");
		commands_plot_add_graph("lambda_est");
	}

	for (int i = 0;i < top;i++) {
		float res_est = m_motor_1.m_res_est;
		float t_base = m_motor_1.m_conf->foc_temp_comp_base_temp;
		float res_base = m_motor_1.m_conf->foc_motor_r;
		float t_est = (res_est / res_base - 1) / 0.00386 + t_base;
		float t_meas = mc_interface_temp_motor_filtered();

		if (top > 1) {
			commands_plot_set_graph(0);
			commands_send_plot_points((float)i / 2.0, t_meas);
			commands_plot_set_graph(1);
			commands_send_plot_points((float)i / 2.0, t_est);
			commands_plot_set_graph(2);
			commands_send_plot_points((float)i / 2.0, m_motor_1.m_observer_state.lambda_est);
			commands_printf("Sample %d of %d", i, top);
		}

		commands_printf("R: %.2f, EST: %.2f",
				(double)(res_base * 1000.0), (double)(res_est * 1000.0));
		commands_printf("T: %.2f, T_EST: %.2f\n",
				(double)t_meas, (double)t_est);

		chThdSleepMilliseconds(500);
	}
}

// TODO: This won't work for dual motors
static void input_current_offset_measurement(void) {
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
	static uint16_t delay_current_offset_measurement = 0;

	if (delay_current_offset_measurement < 1000) {
		delay_current_offset_measurement++;
	} else {
		if (delay_current_offset_measurement == 1000) {
			delay_current_offset_measurement++;
			MEASURE_INPUT_CURRENT_OFFSET();
		}
	}
#endif
}

static THD_FUNCTION(timer_thread, arg) {
	(void)arg;

	chRegSetThreadName("foc timer");

	for(;;) {
		const float dt = 0.001;

		if (timer_thd_stop) {
			timer_thd_stop = false;
			return;
		}

		timer_update((motor_all_state_t*)&m_motor_1, dt);
#ifdef HW_HAS_DUAL_MOTORS
		timer_update((motor_all_state_t*)&m_motor_2, dt);
#endif

		input_current_offset_measurement();

		chThdSleepMilliseconds(1);
	}
}

static void hfi_update(volatile motor_all_state_t *motor, float dt) {
	(void)dt;
	float rpm_abs = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

	if (rpm_abs > motor->m_conf->foc_sl_erpm_hfi) {
		motor->m_hfi.angle = motor->m_phase_now_observer;
		motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
	}

	if (motor->m_hfi.ready) {
		if ((motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4 ||
				motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5) &&
				motor->m_hfi.est_done_cnt >= motor->m_conf->foc_hfi_start_samples) {
			// Nothing done here, the update is done in the interrupt.
		} else if ((motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 ||
				motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3) &&
				motor->m_hfi.est_done_cnt >= motor->m_conf->foc_hfi_start_samples) {
			// Nothing done here, the update is done in the interrupt.

			// Enable to set the observer position to the HFI angle for plotting the error in the position plot RT page in VESC Tool. Just
			// remember that enabling this will make the transition to using the observer bad.
#if 0
			float s, c;
			utils_fast_sincos_better(motor->m_hfi.angle, &s, &c);
			motor->m_observer_x1 = c * motor->m_conf->foc_motor_flux_linkage;
			motor->m_observer_x2 = s * motor->m_conf->foc_motor_flux_linkage;
#endif

			// Enable to plot the sample together with encoder position
#if 0

			commands_plot_set_graph(0);
			commands_send_plot_points(motor->m_hfi_plot_sample, ind_a);
			commands_plot_set_graph(1);
			commands_send_plot_points(motor->m_hfi_plot_sample, RAD2DEG_f(motor->m_phase_now_encoder) / 4e6);
			motor->m_hfi_plot_sample++;
#endif
		} else {
			float real_bin1, imag_bin1, real_bin2, imag_bin2;
			motor->m_hfi.fft_bin1_func((float*)motor->m_hfi.buffer, &real_bin1, &imag_bin1);
			motor->m_hfi.fft_bin2_func((float*)motor->m_hfi.buffer, &real_bin2, &imag_bin2);

			float mag_bin_1 = NORM2_f(imag_bin1, real_bin1);
			float angle_bin_1 = -utils_fast_atan2(imag_bin1, real_bin1);

			angle_bin_1 += M_PI / 1.7; // Why 1.7??
			utils_norm_angle_rad(&angle_bin_1);

			float mag_bin_2 = NORM2_f(imag_bin2, real_bin2);
			float angle_bin_2 = -utils_fast_atan2(imag_bin2, real_bin2) / 2.0;

			// Assuming this thread is much faster than it takes to fill the HFI buffer completely,
			// we should lag 1/2 HFI buffer behind in phase. Compensate for that here.
			float dt_sw;
			if (motor->m_conf->foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7) {
				dt_sw = 1.0 / motor->m_conf->foc_f_zv;
			} else {
				dt_sw = 1.0 / (motor->m_conf->foc_f_zv / 2.0);
			}
			angle_bin_2 += motor->m_pll_speed * ((float)motor->m_hfi.samples / 2.0) * dt_sw;

			if (fabsf(utils_angle_difference_rad(angle_bin_2 + M_PI, motor->m_hfi.angle)) <
					fabsf(utils_angle_difference_rad(angle_bin_2, motor->m_hfi.angle))) {
				angle_bin_2 += M_PI;
			}

			if (motor->m_hfi.est_done_cnt < motor->m_conf->foc_hfi_start_samples) {
				motor->m_hfi.est_done_cnt++;

				if (fabsf(utils_angle_difference_rad(angle_bin_2, angle_bin_1)) > (M_PI / 2.0)) {
					motor->m_hfi.flip_cnt++;
				}
			}

			if (motor->m_hfi.est_done_cnt >= motor->m_conf->foc_hfi_start_samples) {
				if (motor->m_hfi.flip_cnt >= (motor->m_conf->foc_hfi_start_samples / 2)) {
					angle_bin_2 += M_PI;
				}
				motor->m_hfi.flip_cnt = 0;

				if (motor->m_conf->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START) {
					float s, c;
					utils_fast_sincos_better(angle_bin_2, &s, &c);
					motor->m_observer_state.x1 = c * motor->m_conf->foc_motor_flux_linkage;
					motor->m_observer_state.x2 = s * motor->m_conf->foc_motor_flux_linkage;
				}
			}

			motor->m_hfi.angle = angle_bin_2;
			utils_norm_angle_rad((float*)&motor->m_hfi.angle);

			// As angle_bin_1 is based on saturation, it is only accurate when the motor current is low. It
			// might be possible to compensate for that, which would allow HFI on non-salient motors.
			//			m_hfi.angle = angle_bin_1;

			if (motor->m_hfi_plot_en == 1) {
				static float hfi_plot_div = 0;
				hfi_plot_div++;

				if (hfi_plot_div >= 8) {
					hfi_plot_div = 0;

					float real_bin0, imag_bin0;
					motor->m_hfi.fft_bin0_func((float*)motor->m_hfi.buffer, &real_bin0, &imag_bin0);

					commands_plot_set_graph(0);
					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_hfi.angle);

					commands_plot_set_graph(1);
					commands_send_plot_points(motor->m_hfi_plot_sample, angle_bin_1);

					commands_plot_set_graph(2);
					commands_send_plot_points(motor->m_hfi_plot_sample, 2.0 * mag_bin_2 * 1e6);

					commands_plot_set_graph(3);
					commands_send_plot_points(motor->m_hfi_plot_sample, 2.0 * mag_bin_1 * 1e6);

					commands_plot_set_graph(4);
					commands_send_plot_points(motor->m_hfi_plot_sample, real_bin0 * 1e6);

//					commands_plot_set_graph(0);
//					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_pll_speed);
//
//					commands_plot_set_graph(1);
//					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_speed_est_fast);

					motor->m_hfi_plot_sample++;
				}
			} else if (motor->m_hfi_plot_en == 2) {
				static float hfi_plot_div = 0;
				hfi_plot_div++;

				if (hfi_plot_div >= 8) {
					hfi_plot_div = 0;

					if (motor->m_hfi_plot_sample >= motor->m_hfi.samples) {
						motor->m_hfi_plot_sample = 0;
					}

					commands_plot_set_graph(0);
					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_hfi.buffer_current[(int)motor->m_hfi_plot_sample]);

					commands_plot_set_graph(1);
					commands_send_plot_points(motor->m_hfi_plot_sample, motor->m_hfi.buffer[(int)motor->m_hfi_plot_sample] * 1e6);

					motor->m_hfi_plot_sample++;
				}
			}
		}
	} else {
		motor->m_hfi.angle = motor->m_phase_now_observer;
		motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
	}
}

static THD_FUNCTION(hfi_thread, arg) {
	(void)arg;

	chRegSetThreadName("foc hfi");

	uint32_t t_last = timer_time_now();

	for(;;) {
		if (hfi_thd_stop) {
			hfi_thd_stop = false;
			return;
		}

		float dt = timer_seconds_elapsed_since(t_last);
		t_last = timer_time_now();

		hfi_update(&m_motor_1, dt);
#ifdef HW_HAS_DUAL_MOTORS
		hfi_update(&m_motor_2, dt);
#endif

		chThdSleepMicroseconds(500);
	}
}

static THD_FUNCTION(pid_thread, arg) {
	(void)arg;

	chRegSetThreadName("foc pid");

	uint32_t last_time = timer_time_now();

	for(;;) {
		if (pid_thd_stop) {
			pid_thd_stop = false;
			return;
		}

		switch (m_motor_1.m_conf->sp_pid_loop_rate) {
		case PID_RATE_25_HZ: chThdSleepMicroseconds(1000000 / 25); break;
		case PID_RATE_50_HZ: chThdSleepMicroseconds(1000000 / 50); break;
		case PID_RATE_100_HZ: chThdSleepMicroseconds(1000000 / 100); break;
		case PID_RATE_250_HZ: chThdSleepMicroseconds(1000000 / 250); break;
		case PID_RATE_500_HZ: chThdSleepMicroseconds(1000000 / 500); break;
		case PID_RATE_1000_HZ: chThdSleepMicroseconds(1000000 / 1000); break;
		case PID_RATE_2500_HZ: chThdSleepMicroseconds(1000000 / 2500); break;
		case PID_RATE_5000_HZ: chThdSleepMicroseconds(1000000 / 5000); break;
		case PID_RATE_10000_HZ: chThdSleepMicroseconds(1000000 / 10000); break;
		}

		float dt = timer_seconds_elapsed_since(last_time);
		last_time = timer_time_now();

		foc_run_pid_control_pos(encoder_index_found(), dt, (motor_all_state_t*)&m_motor_1);
		foc_run_pid_control_speed(dt, (motor_all_state_t*)&m_motor_1);
		foc_run_pid_control_pos_multiturn(encoder_index_found(), dt, (motor_all_state_t*)&m_motor_1);
		// mc_interface_selflock();

#ifdef HW_HAS_DUAL_MOTORS
		foc_run_pid_control_pos(encoder_index_found(), dt, (motor_all_state_t*)&m_motor_2);
		foc_run_pid_control_speed(dt, (motor_all_state_t*)&m_motor_2);
#endif
	}
}

/**
 * Run the current control loop.
 *
 * @param state_m
 * The motor state.
 *
 * Parameters that shall be set before calling this function:
 * id_target
 * iq_target
 * max_duty
 * phase
 * i_alpha
 * i_beta
 * v_bus
 *
 * Parameters that will be updated in this function:
 * i_bus
 * i_abs
 * i_abs_filter
 * v_alpha
 * v_beta
 * mod_d
 * mod_q
 * id
 * iq
 * id_filter
 * iq_filter
 * vd
 * vq
 * vd_int
 * vq_int
 * svm_sector
 *
 * @param dt
 * The time step in seconds.
 */
static void control_current(motor_all_state_t *motor, float dt) {
	volatile motor_state_t *state_m = &motor->m_motor_state;
	volatile mc_configuration *conf_now = motor->m_conf;

	float s = state_m->phase_sin;
	float c = state_m->phase_cos;

	float abs_rpm = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));

	bool do_hfi = (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4 ||
			conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5 ||
			(conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START &&
					motor->m_control_mode != CONTROL_MODE_CURRENT_BRAKE &&
					fabsf(state_m->iq_target) > conf_now->cc_min_current)) &&
							!motor->m_phase_override &&
							abs_rpm < (conf_now->foc_sl_erpm_hfi * (motor->m_cc_was_hfi ? 1.8 : 1.5));

	bool hfi_est_done = motor->m_hfi.est_done_cnt >= conf_now->foc_hfi_start_samples;

	// Only allow Q axis current after the HFI ambiguity is resolved. This causes
	// a short delay when starting.
	if (do_hfi && !hfi_est_done) {
		state_m->iq_target = 0;
	} else if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_START) {
		do_hfi = false;
	}

	motor->m_cc_was_hfi = do_hfi;

	float max_duty = fabsf(state_m->max_duty);
	utils_truncate_number(&max_duty, 0.0, conf_now->l_max_duty);

	// Park transform: transforms the currents from stator to the rotor reference frame
	state_m->id = c * state_m->i_alpha + s * state_m->i_beta;
	state_m->iq = c * state_m->i_beta  - s * state_m->i_alpha;  
	// i_alpha和i_beta是通过采样得到的
	// park变换得到当前id, iq值

	// Low passed currents are used for less time critical parts, not for the feedback
	// 低通滤波, 会引入延迟, 所以是给不太要求实时性的部分用的, 不是给本次FOC反馈计算用的.
	UTILS_LP_FAST(state_m->id_filter, state_m->id, conf_now->foc_current_filter_const);
	UTILS_LP_FAST(state_m->iq_filter, state_m->iq, conf_now->foc_current_filter_const);

	// 以下代码块, 计算d轴电流的PI控制器增益限制, 这种限制主要是在高调制(高占空比)的情况下
	// 一般d轴电流是0, 但是当d轴电流不为0, 高占空比下: 
	// (1)相电压要接近母线电压, 电压资源吃紧, 如果不限制d轴电压, 可能会影响q轴电压的供给, 导致转矩不足等问题;
	// (2)果d轴PI增益过高，容易导致积分环节快速累积，输出电压超出硬件能力，产生饱和、wind up，甚至引发系统振荡。
	float d_gain_scale = 1.0;
	if (conf_now->foc_d_gain_scale_start < 0.99) {
		float max_mod_norm = fabsf(state_m->duty_now / max_duty);
		if (max_duty < 0.01) {
			max_mod_norm = 1.0;
		}
		if (max_mod_norm > conf_now->foc_d_gain_scale_start) {
			d_gain_scale = utils_map(max_mod_norm, conf_now->foc_d_gain_scale_start, 1.0,
					1.0, conf_now->foc_d_gain_scale_max_mod);
			if (d_gain_scale < conf_now->foc_d_gain_scale_max_mod) {
				d_gain_scale = conf_now->foc_d_gain_scale_max_mod;
			}
		}
	}

	float Ierr_d = state_m->id_target - state_m->id;
	float Ierr_q = state_m->iq_target - state_m->iq;  // 计算误差项, 要应用PI控制器算Vd和Vq

	float ki = conf_now->foc_current_ki;
	if (conf_now->foc_temp_comp) {
		ki = motor->m_current_ki_temp_comp;
	}

	// PI控制器部分
	// id_err -> PI控制器 -> 误差补偿 -> vd
	// iq_err -> PI控制器 -> 误差补偿 -> vq
	// 本质上我们只能输出电压，要知道将电压与电流如何对应, 需要对电机建模, 需要用到电机参数. //
	// 忽略耦合因素, 电机系统开环传递函数是tf = 1/(Ls+R), 即电压是输入, 相线电流是输出.    //
	// 那么根据需要的电流, 我们可以反推需要的电压, 但是直接套公式算, 是稳态的情况.          //
	// 电机是要运动的, 是一个动态的场景, 为了实现动态场景下的自动控制, 我们选择一个PI控制器  // 
	// 来满足系统的动态响应性能, 例如响应速度, 稳态误差等(不是二阶系统没有震荡超调问题)      //
	// 系统框图:
	//  ______________________________________________________________________________
	// |                                                                             |
	// | i_set -> (-) -> 得到i_err ->  (PI) -> 得到v_set -> (电机) -> 采样得到i_real   |
	// |     	  /|\													|            |
	// |		   |____________________________________________________|            |
	// |_____________________________________________________________________________|
	// 
	// 这是一个开环系统的闭环反馈控制, 电机开环系统有一个极点在s = -(R/L)处, 加入PI控制器, //
	// 是在系统中加入了一个s = 0处的极点(积分作用), 加入一个s = -(ki/kp)的零点,           //
	// 结合根轨迹法, 修改kp和ki的值, 可以改变闭环系统的极点位置, 进而改变系统的动态响应性能 //
	//
	// VESC中, kp的计算与l成正比, ki的计算与r成正比, 原因如下:
	// 1. 电感L越大, 电流上升越慢, 需要更大的kp来提高系统的响应速度.
	// 2. 电阻R越大, 需要更大的ki来补偿电阻带来的稳态误差.
	state_m->vd_int += Ierr_d * (ki * d_gain_scale * dt);
	state_m->vq_int += Ierr_q * (ki * dt);

	// Feedback (PI controller). No D action needed because the plant is a first order system (tf = 1/(Ls+R))
	state_m->vd = state_m->vd_int + Ierr_d * conf_now->foc_current_kp * d_gain_scale;
	state_m->vq = state_m->vq_int + Ierr_q * conf_now->foc_current_kp;

	// iq和id到vq和vd转换, 考虑到vd和vq有耦合, 通过补偿降低耦合的影响
	// 下面这一步就是在计算耦合项, 然后对上面代码计算出的vd和vq进行补偿
	// Decoupling. Using feedforward this compensates for the fact that the equations of a PMSM
	// are not really decoupled (the d axis current has impact on q axis voltage and visa-versa):
	//      Resistance  Inductance   Cross terms   Back-EMF   (see www.mathworks.com/help/physmod/sps/ref/pmsm.html)
	// vd = Rs*id   +   Ld*did/dt −  ωe*iq*Lq
	// vq = Rs*iq   +   Lq*diq/dt +  ωe*id*Ld     + ωe*ψm
	//                               ^^^^^^^^ 耦合项
	float dec_vd = 0.0;
	float dec_vq = 0.0;
	float dec_bemf = 0.0;

	if (motor->m_control_mode < CONTROL_MODE_HANDBRAKE && conf_now->foc_cc_decoupling != FOC_CC_DECOUPLING_DISABLED) {
		switch (conf_now->foc_cc_decoupling) {
		// 使用低通滤波后的值进行补偿, 补偿不是很精确, 但是可以避免噪声影响总体性能
		case FOC_CC_DECOUPLING_CROSS:
			dec_vd = state_m->iq_filter * motor->m_speed_est_fast * motor->p_lq; // m_speed_est_fast is ωe in [rad/s]
			dec_vq = state_m->id_filter * motor->m_speed_est_fast * motor->p_ld;
			break;

		case FOC_CC_DECOUPLING_BEMF:
			dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
			break;

		case FOC_CC_DECOUPLING_CROSS_BEMF:
			dec_vd = state_m->iq_filter * motor->m_speed_est_fast * motor->p_lq;
			dec_vq = state_m->id_filter * motor->m_speed_est_fast * motor->p_ld;
			dec_bemf = motor->m_speed_est_fast * conf_now->foc_motor_flux_linkage;
			break;

		default:
			break;
		}
	}

	// 计算耦合项结束, 应用耦合项
	state_m->vd -= dec_vd; //Negative sign as in the PMSM equations
	state_m->vq += dec_vq + dec_bemf;

	// Calculate the max length of the voltage space vector without overmodulation.
	// Is simply 1/sqrt(3) * v_bus. See https://microchipdeveloper.com/mct5001:start. Adds margin with max_duty.
	float max_v_mag = ONE_BY_SQRT3 * max_duty * state_m->v_bus;  // 这个是SVPWM的理论最大电压幅值

	// Saturation and anti-windup. Notice that the d-axis has priority as it controls field
	// weakening and the efficiency.
	float vd_presat = state_m->vd;
	utils_truncate_number_abs((float*)&state_m->vd, max_v_mag);
	state_m->vd_int += (state_m->vd - vd_presat);  // 为什么要修改vd的积分项? 这就是anti-windup吗?

	float max_vq = sqrtf(SQ(max_v_mag) - SQ(state_m->vd));  //通过vd计算最大vq(向量合成的方式)
	float vq_presat = state_m->vq;
	utils_truncate_number_abs((float*)&state_m->vq, max_vq);
	state_m->vq_int += (state_m->vq - vq_presat);  // 限幅后-限幅前, 然后在积分项上加上这个差值, 这样做有什么理论依据吗?

	// 最终, 如果饱和/超最大幅度, 对vq和vd进行整体限幅
	utils_saturate_vector_2d((float*)&state_m->vd, (float*)&state_m->vq, max_v_mag);

	// mod_d and mod_q are normalized such that 1 corresponds to the max possible voltage:
	//    voltage_normalize = 1/(2/3*V_bus)
	// This includes overmodulation and therefore cannot be made in any direction.
	// Note that this scaling is different from max_v_mag, which is without over modulation.
	// 以下是一个归一化, 但是乘了一个1.5, 为什么?
	const float voltage_normalize = 1.5 / state_m->v_bus;
	state_m->mod_d = state_m->vd * voltage_normalize;
	state_m->mod_q = state_m->vq * voltage_normalize;
	// 计算归一化的目标dq轴电压, 使用归一化是为了方便计算占空比
	UTILS_NAN_ZERO(state_m->mod_q_filter);
	UTILS_LP_FAST(state_m->mod_q_filter, state_m->mod_q, 0.2);

	// TODO: Have a look at this?
#ifdef HW_HAS_INPUT_CURRENT_SENSOR
	state_m->i_bus = GET_INPUT_CURRENT();
#else
	state_m->i_bus = state_m->mod_alpha_measured * state_m->i_alpha + state_m->mod_beta_measured * state_m->i_beta;
	// TODO: Also calculate motor power based on v_alpha, v_beta, i_alpha and i_beta. This is much more accurate
	// with phase filters than using the modulation and bus current.
#endif
	state_m->i_abs = NORM2_f(state_m->id, state_m->iq);
	state_m->i_abs_filter = NORM2_f(state_m->id_filter, state_m->iq_filter);

	// Inverse Park transform: transforms the (normalized) voltages from the rotor reference frame to the stator frame
	// 目标dq轴电压转换为目标αβ轴电压, 接下来根据SVPWM原理, 可以直接由αβ轴电压计算出各相占空比
	// 利用归一化后的vd和vq计算出一对mod_alpha和mod_beta, raw代表什么?
	state_m->mod_alpha_raw = c * state_m->mod_d - s * state_m->mod_q;
	state_m->mod_beta_raw  = c * state_m->mod_q + s * state_m->mod_d;

	// 这个函数是整个流程中唯一根据本轮采样值更新v_alpha和v_beta的地方，
	// 再次更新v_alpha和v_beta前, 一直用这个数据.
	//
	// 疑问: 为什么不在一开始就更新v_alpha和v_beta呢? 这样会不会更好?
	// 可能的答案: 前面几乎没有用到v_alpha和v_beta, 用的最多的本轮采样值是ia, ib, ic, 
	// 以及根据本轮采样ia, ib, ic计算出来的i_alpha和i_beta, 以及进一步计算出来的计算本轮采样id和iq,
	// 接下来
	update_valpha_vbeta(motor, state_m->mod_alpha_raw, state_m->mod_beta_raw);

	// Dead time compensated values for vd and vq. Note that these are not used to control the switching times.
	// 之前vd, vq一直是目标电压, 现在这个变量被复用为采样到的实际电压, 因为是实际电压, 所以经过了死区时间补偿, 
	// 所以称 Dead time compensated values
	state_m->vd = c * motor->m_motor_state.v_alpha + s * motor->m_motor_state.v_beta;
	state_m->vq = c * motor->m_motor_state.v_beta  - s * motor->m_motor_state.v_alpha;

	// HFI部分开始
	// HFI
	if (do_hfi) {
#ifdef HW_HAS_DUAL_MOTORS
		if (motor == &m_motor_2) {
			CURRENT_FILTER_OFF_M2();
		} else {
			CURRENT_FILTER_OFF();
		}
#else
		CURRENT_FILTER_OFF();
#endif

		float mod_alpha_v7 = state_m->mod_alpha_raw;
		float mod_beta_v7 = state_m->mod_beta_raw;

#ifdef HW_HAS_PHASE_SHUNTS
		float mod_alpha_v0 = state_m->mod_alpha_raw;
		float mod_beta_v0 = state_m->mod_beta_raw;
#endif

		float hfi_voltage;
		if (motor->m_hfi.est_done_cnt < conf_now->foc_hfi_start_samples) {
			hfi_voltage = conf_now->foc_hfi_voltage_start;
		} else {
			hfi_voltage = utils_map(fabsf(state_m->iq), -0.01, conf_now->l_current_max,
					conf_now->foc_hfi_voltage_run, conf_now->foc_hfi_voltage_max);
		}

		utils_truncate_number_abs(&hfi_voltage, state_m->v_bus * (1.0 - fabsf(state_m->duty_now)) * SQRT3_BY_2 * (2.0 / 3.0) * 0.95);

		if ((conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5) && hfi_est_done) {
			if (motor->m_hfi.is_samp_n) {
				float sample_now = c * motor->m_i_beta_sample_with_offset - s * motor->m_i_alpha_sample_with_offset;
				float di = (motor->m_hfi.prev_sample - sample_now);

				if (!motor->m_using_encoder) {
					motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
					motor->m_hfi.angle = motor->m_phase_now_observer;
				} else {
					float hfi_dt = dt * 2.0;
#ifdef HW_HAS_PHASE_SHUNTS
					if (conf_now->foc_control_sample_mode != FOC_CONTROL_SAMPLE_MODE_V0_V7 && conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4) {
						hfi_dt = dt;
					}
#endif
					foc_hfi_adjust_angle(
							(di * conf_now->foc_f_zv) / (hfi_voltage * motor->p_inv_ld_lq),
							motor, hfi_dt
					);
				}

#ifdef HW_HAS_PHASE_SHUNTS
				if (conf_now->foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V5) {
					mod_alpha_v7 -= hfi_voltage * c * voltage_normalize;
					mod_beta_v7 -= hfi_voltage * s * voltage_normalize;
				} else {
					motor->m_hfi.prev_sample = c * motor->m_i_beta_sample_next - s * motor->m_i_alpha_sample_next;

					mod_alpha_v0 -= hfi_voltage * c * voltage_normalize;
					mod_beta_v0 -= hfi_voltage * s * voltage_normalize;

					mod_alpha_v7 += hfi_voltage * c * voltage_normalize;
					mod_beta_v7 += hfi_voltage * s * voltage_normalize;

					motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;
					motor->m_i_alpha_beta_has_offset = true;
				}
#else
				mod_alpha_v7 -= hfi_voltage * c * voltage_normalize;
				mod_beta_v7 -= hfi_voltage * s * voltage_normalize;
#endif
			} else {
				motor->m_hfi.prev_sample = c * state_m->i_beta - s * state_m->i_alpha;
				mod_alpha_v7 += hfi_voltage * c * voltage_normalize;
				mod_beta_v7  += hfi_voltage * s * voltage_normalize;
			}
		} else if ((conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3) && hfi_est_done) {
			if (motor->m_hfi.is_samp_n) {
				if (fabsf(state_m->iq_target) > conf_now->foc_hfi_hyst) {
					motor->m_hfi.sign_last_sample = SIGN(state_m->iq_target);
				}

				float sample_now = motor->m_hfi.cos_last * motor->m_i_alpha_sample_with_offset +
						motor->m_hfi.sin_last * motor->m_i_beta_sample_with_offset;
				float di = (sample_now - motor->m_hfi.prev_sample);

				if (!motor->m_using_encoder) {
					motor->m_hfi.double_integrator = -motor->m_speed_est_fast;
					motor->m_hfi.angle = motor->m_phase_now_observer;
				} else {
					float hfi_dt = dt * 2.0;
#ifdef HW_HAS_PHASE_SHUNTS
					if (conf_now->foc_control_sample_mode != FOC_CONTROL_SAMPLE_MODE_V0_V7 && conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2) {
						hfi_dt = dt;
					}
#endif
					foc_hfi_adjust_angle(
							motor->m_hfi.sign_last_sample * ((conf_now->foc_f_zv * di) /
									hfi_voltage - motor->p_v2_v3_inv_avg_half) / motor->p_inv_ld_lq,
							motor, hfi_dt
					);
				}

				// Use precomputed rotation matrix
				if (motor->m_hfi.sign_last_sample > 0) {
					// +45 Degrees
					motor->m_hfi.sin_last = ONE_BY_SQRT2 * (c + s);
					motor->m_hfi.cos_last = ONE_BY_SQRT2 * (c - s);
				} else {
					// -45 Degrees
					motor->m_hfi.sin_last = ONE_BY_SQRT2 * (-c + s);
					motor->m_hfi.cos_last = ONE_BY_SQRT2 * (c + s);
				}

#ifdef HW_HAS_PHASE_SHUNTS
				if (conf_now->foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V3) {
					mod_alpha_v7 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v7 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
				} else {
					motor->m_hfi.prev_sample = motor->m_hfi.cos_last * motor->m_i_alpha_sample_next +
							motor->m_hfi.sin_last * motor->m_i_beta_sample_next;

					mod_alpha_v0 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v0 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;

					mod_alpha_v7 -= hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
					mod_beta_v7 -= hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;

					motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;
					motor->m_i_alpha_beta_has_offset = true;
				}
#else
				mod_alpha_v7 += hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
				mod_beta_v7 += hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
#endif
			} else {
				motor->m_hfi.prev_sample = motor->m_hfi.cos_last * state_m->i_alpha + motor->m_hfi.sin_last * state_m->i_beta;
				mod_alpha_v7 -= hfi_voltage * motor->m_hfi.cos_last * voltage_normalize;
				mod_beta_v7  -= hfi_voltage * motor->m_hfi.sin_last * voltage_normalize;
			}
		} else {
			if (motor->m_hfi.is_samp_n) {
				float sample_now = (utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_alpha -
						utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_beta);
				float di = (sample_now - motor->m_hfi.prev_sample);

				motor->m_hfi.buffer_current[motor->m_hfi.ind] = di;

				if (di > 0.01) {
					motor->m_hfi.buffer[motor->m_hfi.ind] = hfi_voltage / (conf_now->foc_f_zv * di);
				}

				motor->m_hfi.ind++;
				if (motor->m_hfi.ind == motor->m_hfi.samples) {
					motor->m_hfi.ind = 0;
					motor->m_hfi.ready = true;
				}

				mod_alpha_v7 += hfi_voltage * utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
				mod_beta_v7  -= hfi_voltage * utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
			} else {
				motor->m_hfi.prev_sample = utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_alpha -
						utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * state_m->i_beta;

				mod_alpha_v7 -= hfi_voltage * utils_tab_sin_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
				mod_beta_v7  += hfi_voltage * utils_tab_cos_32_1[motor->m_hfi.ind * motor->m_hfi.table_fact] * voltage_normalize;
			}
		}

		utils_saturate_vector_2d(&mod_alpha_v7, &mod_beta_v7, SQRT3_BY_2 * 0.95);
		motor->m_hfi.is_samp_n = !motor->m_hfi.is_samp_n;

		if (conf_now->foc_control_sample_mode == FOC_CONTROL_SAMPLE_MODE_V0_V7) {
			state_m->mod_alpha_raw = mod_alpha_v7;
			state_m->mod_beta_raw = mod_beta_v7;
		} else {
#ifdef HW_HAS_PHASE_SHUNTS
			if (conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V2 || conf_now->foc_sensor_mode == FOC_SENSOR_MODE_HFI_V4) {
				utils_saturate_vector_2d(&mod_alpha_v0, &mod_beta_v0, SQRT3_BY_2 * 0.95);
				state_m->mod_alpha_raw = mod_alpha_v0;
				state_m->mod_beta_raw = mod_beta_v0;
			}
#endif

			// Delay adding the HFI voltage when not sampling in both 0 vectors, as it will cancel
			// itself with the opposite pulse from the previous HFI sample. This makes more sense
			// when drawing the SVM waveform.
			foc_svm(mod_alpha_v7, mod_beta_v7, TIM1->ARR,
				(uint32_t*)&motor->m_duty1_next,
				(uint32_t*)&motor->m_duty2_next,
				(uint32_t*)&motor->m_duty3_next,
				(uint32_t*)&state_m->svm_sector);
			motor->m_duty_next_set = true;
		}
	} else {
#ifdef HW_HAS_DUAL_MOTORS
		if (motor == &m_motor_2) {
			CURRENT_FILTER_ON_M2();
		} else {
			CURRENT_FILTER_ON();
		}
#else
		CURRENT_FILTER_ON();
#endif
		motor->m_hfi.ind = 0;
		motor->m_hfi.ready = false;
		motor->m_hfi.is_samp_n = false;
		motor->m_hfi.prev_sample = 0.0;
		motor->m_hfi.double_integrator = 0.0;
	}
	// HFI部分结束

	// Set output (HW Dependent)
	uint32_t duty1, duty2, duty3, top;
	top = TIM1->ARR;

	// Calculate the duty cycles for all the phases. This also injects a zero modulation signal to
	// be able to fully utilize the bus voltage. See https://microchipdeveloper.com/mct5001:start
	// SVPWM部分, 由目标αβ轴电压计算出各相的占空比
	foc_svm(state_m->mod_alpha_raw, state_m->mod_beta_raw, top, &duty1, &duty2, &duty3, (uint32_t*)&state_m->svm_sector);

	// 更新占空比, 电流控制部分结束
	if (motor == &m_motor_1) {
		TIMER_UPDATE_DUTY_M1(duty1, duty2, duty3);
#ifdef HW_HAS_DUAL_PARALLEL
		TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
#endif
	} else {
#ifndef HW_HAS_DUAL_PARALLEL
		TIMER_UPDATE_DUTY_M2(duty1, duty2, duty3);
#endif
	}

	// do not allow to turn on PWM outputs if virtual motor is used
	if(virtual_motor_is_connected() == false) {
		if (!motor->m_output_on) {
			start_pwm_hw(motor);
		}
	}
}

static void update_valpha_vbeta(motor_all_state_t *motor, float mod_alpha, float mod_beta) {
	motor_state_t *state_m = &motor->m_motor_state;
	mc_configuration *conf_now = motor->m_conf;
	float Va, Vb, Vc;

	volatile float *ofs_volt = conf_now->foc_offsets_voltage_undriven;
	if (motor->m_state == MC_STATE_RUNNING) {
		ofs_volt = conf_now->foc_offsets_voltage;
	}

#ifdef HW_HAS_DUAL_MOTORS
#ifdef HW_HAS_3_SHUNTS
	if (&m_motor_1 != motor) {
		Va = (ADC_VOLTS(ADC_IND_SENS4) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		Vb = (ADC_VOLTS(ADC_IND_SENS5) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		Vc = (ADC_VOLTS(ADC_IND_SENS6) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	} else {
		Va = (ADC_VOLTS(ADC_IND_SENS1) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		Vb = (ADC_VOLTS(ADC_IND_SENS2) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		Vc = (ADC_VOLTS(ADC_IND_SENS3) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	}
#else
	if (&m_motor_1 != motor) {
		Va = (ADC_VOLTS(ADC_IND_SENS4) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		Vb = (ADC_VOLTS(ADC_IND_SENS6) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		Vc = (ADC_VOLTS(ADC_IND_SENS5) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	} else {
		Va = (ADC_VOLTS(ADC_IND_SENS1) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		Vb = (ADC_VOLTS(ADC_IND_SENS3) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
		Vc = (ADC_VOLTS(ADC_IND_SENS2) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	}
#endif
#else
#ifdef HW_HAS_3_SHUNTS
	// 以下是计算采样到的电压, ofs之前已解释过, 视为0即可;
	// ADC采样的到的是分压, 下面的内容是转换为实际电压;
	// 用到的分压电阻可以在硬件配置中改, LIMITI用到的是2.2k和39k的
	Va = (ADC_VOLTS(ADC_IND_SENS1) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	Vb = (ADC_VOLTS(ADC_IND_SENS2) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	Vc = (ADC_VOLTS(ADC_IND_SENS3) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
#else
	Va = (ADC_VOLTS(ADC_IND_SENS1) - ofs_volt[0]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	Vb = (ADC_VOLTS(ADC_IND_SENS3) - ofs_volt[2]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
	Vc = (ADC_VOLTS(ADC_IND_SENS2) - ofs_volt[1]) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR;
#endif
#endif

	// Deadtime compensation
	float s = state_m->phase_sin;
	float c = state_m->phase_cos;
	const float i_alpha_filter = c * state_m->id_filter - s * state_m->iq_filter;
	const float i_beta_filter = c * state_m->iq_filter + s * state_m->id_filter;
	const float ia_filter = i_alpha_filter;
	const float ib_filter = -0.5 * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
	const float ic_filter = -0.5 * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;

	// mod_alpha_sign = 2/3*sign(ia) - 1/3*sign(ib) - 1/3*sign(ic)
	// mod_beta_sign  = 1/sqrt(3)*sign(ib) - 1/sqrt(3)*sign(ic)
	const float mod_alpha_filter_sgn = (1.0 / 3.0) * (2.0 * SIGN(ia_filter) - SIGN(ib_filter) - SIGN(ic_filter));
	const float mod_beta_filter_sgn = ONE_BY_SQRT3 * (SIGN(ib_filter) - SIGN(ic_filter));

	const float mod_comp_fact = conf_now->foc_dt_us * 1e-6 * conf_now->foc_f_zv;
	const float mod_alpha_comp = mod_alpha_filter_sgn * mod_comp_fact;
	const float mod_beta_comp = mod_beta_filter_sgn * mod_comp_fact;

	mod_alpha -= mod_alpha_comp;
	mod_beta -= mod_beta_comp;

	state_m->va = Va;
	state_m->vb = Vb;
	state_m->vc = Vc;
	// 算出死区时间对电压的影响, 减掉
	state_m->mod_alpha_measured = mod_alpha;
	state_m->mod_beta_measured = mod_beta;

	// v_alpha = 2/3*Va - 1/3*Vb - 1/3*Vc
	// v_beta  = 1/sqrt(3)*Vb - 1/sqrt(3)*Vc
	// 计算上次采样得到的v_alpha和v_beta
	float v_alpha = (1.0 / 3.0) * (2.0 * Va - Vb - Vc);
	float v_beta = ONE_BY_SQRT3 * (Vb - Vc);

	// Keep the modulation updated so that the filter stays updated
	// even when the motor is undriven.
	if (motor->m_state != MC_STATE_RUNNING) {
		/* voltage_normalize = 1/(2/3*V_bus) */
		const float voltage_normalize = 1.5 / state_m->v_bus;

		mod_alpha = v_alpha * voltage_normalize;
		mod_beta = v_beta * voltage_normalize;
	}

	float abs_rpm = fabsf(RADPS2RPM_f(motor->m_speed_est_fast));
	// 弄一个速度出来干什么呢

	float filter_const = 1.0;
	if (abs_rpm < 10000.0) {
		// 用速度决定的滤波器参数
		filter_const = utils_map(abs_rpm, 0.0, 10000.0, 0.01, 1.0);
	}

	float v_mag = NORM2_f(v_alpha, v_beta);
	// The 0.1 * v_mag term below compensates for the filter attenuation as the speed increases.
	// It is chosen by trial and error, so this can be improved.
	// 进行低通滤波，速度越快滤波效果越不明显/本次数据对滤波数据作用越大
	UTILS_LP_FAST(state_m->v_mag_filter, v_mag + 0.1 * v_mag * filter_const, filter_const);
	UTILS_LP_FAST(state_m->mod_alpha_filter, mod_alpha, filter_const);
	UTILS_LP_FAST(state_m->mod_beta_filter, mod_beta, filter_const);
	UTILS_NAN_ZERO(state_m->v_mag_filter);
	UTILS_NAN_ZERO(state_m->mod_alpha_filter);
	UTILS_NAN_ZERO(state_m->mod_beta_filter);

	mod_alpha = state_m->mod_alpha_filter;
	mod_beta = state_m->mod_beta_filter;

	if (motor->m_state == MC_STATE_RUNNING) {
#ifdef HW_HAS_PHASE_FILTERS
		if (conf_now->foc_phase_filter_enable && abs_rpm < conf_now->foc_phase_filter_max_erpm) {
			float mod_mag = NORM2_f(mod_alpha, mod_beta);
			float v_mag_mod = mod_mag * (2.0 / 3.0) * state_m->v_bus;

			if (!conf_now->foc_phase_filter_disable_fault && fabsf(v_mag_mod - state_m->v_mag_filter) > (conf_now->l_max_vin * 0.05)) {
				mc_interface_set_fault_info("v_mag_mod: %.2f, v_mag_filter: %.2f", 2, v_mag_mod, state_m->v_mag_filter);
				mc_interface_fault_stop(FAULT_CODE_PHASE_FILTER, &m_motor_1 != motor, true);
			}

			// Compensate for the phase delay by using the direction of the modulation
			// together with the magnitude from the phase filters
			if (mod_mag > 0.04) {
				state_m->v_alpha = mod_alpha / mod_mag * state_m->v_mag_filter;
				state_m->v_beta = mod_beta / mod_mag * state_m->v_mag_filter;
			} else {
				state_m->v_alpha = v_alpha;
				state_m->v_beta = v_beta;
			}

			state_m->is_using_phase_filters = true;
		} else {
#endif
			state_m->v_alpha = mod_alpha * (2.0 / 3.0) * state_m->v_bus;
			state_m->v_beta = mod_beta * (2.0 / 3.0) * state_m->v_bus;
			state_m->is_using_phase_filters = false;
#ifdef HW_HAS_PHASE_FILTERS
		}
#endif
	} else {
		// 这就是更新v_alpha和v_beta的地方, 更新的是本次采样值
		state_m->v_alpha = v_alpha;
		state_m->v_beta = v_beta;
		state_m->is_using_phase_filters = false; 

#ifdef HW_USE_LINE_TO_LINE
		// rotate alpha-beta 30 degrees to compensate for line-to-line phase voltage sensing
		float x_tmp = state_m->v_alpha;
		float y_tmp = state_m->v_beta;

		state_m->v_alpha = x_tmp * COS_MINUS_30_DEG - y_tmp * SIN_MINUS_30_DEG;
		state_m->v_beta = x_tmp * SIN_MINUS_30_DEG + y_tmp * COS_MINUS_30_DEG;

		// compensate voltage amplitude
		state_m->v_alpha *= ONE_BY_SQRT3;
		state_m->v_beta *= ONE_BY_SQRT3;
#endif
	}
}

static void stop_pwm_hw(motor_all_state_t *motor) {
	motor->m_id_set = 0.0;
	motor->m_iq_set = 0.0;

	if (motor == &m_motor_1) {
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

		TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DUAL_PARALLEL
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Disable);

		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);
#endif

#ifdef HW_HAS_DRV8313
		DISABLE_BR();
#endif

		motor->m_output_on = false;
		PHASE_FILTER_OFF();
	} else {
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Disable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_ForcedAction_InActive);
		TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Disable);

		TIM_GenerateEvent(TIM8, TIM_EventSource_COM);

#ifdef HW_HAS_DRV8313_2
		DISABLE_BR_2();
#endif

		motor->m_output_on = false;
		PHASE_FILTER_OFF_M2();
	}
}

static void start_pwm_hw(motor_all_state_t *motor) {
	if (motor == &m_motor_1) {
		TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);

#ifdef HW_HAS_DUAL_PARALLEL
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);

		PHASE_FILTER_ON_M2();
#endif

		// Generate COM event in ADC interrupt to get better synchronization
		//	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

#ifdef HW_HAS_DRV8313
		ENABLE_BR();
#endif
		motor->m_output_on = true;
		PHASE_FILTER_ON();
	} else {
		TIM_SelectOCxM(TIM8, TIM_Channel_1, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_1, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_2, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_2, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_2, TIM_CCxN_Enable);

		TIM_SelectOCxM(TIM8, TIM_Channel_3, TIM_OCMode_PWM1);
		TIM_CCxCmd(TIM8, TIM_Channel_3, TIM_CCx_Enable);
		TIM_CCxNCmd(TIM8, TIM_Channel_3, TIM_CCxN_Enable);

#ifdef HW_HAS_DRV8313_2
		ENABLE_BR_2();
#endif
		motor->m_output_on = true;
		PHASE_FILTER_ON_M2();
	}
}

static void terminal_plot_hfi(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if (d == 0 || d == 1 || d == 2 || d == 3) {
			get_motor_now()->m_hfi_plot_en = d;
			if (get_motor_now()->m_hfi_plot_en == 1) {
				get_motor_now()->m_hfi_plot_sample = 0.0;
				commands_init_plot("Sample", "Value");
				commands_plot_add_graph("Phase");
				commands_plot_add_graph("Phase bin2");
				commands_plot_add_graph("Ld - Lq (uH");
				commands_plot_add_graph("L Diff Sat (uH)");
				commands_plot_add_graph("L Avg (uH)");
			} else if (get_motor_now()->m_hfi_plot_en == 2) {
				get_motor_now()->m_hfi_plot_sample = 0.0;
				commands_init_plot("Sample Index", "Value");
				commands_plot_add_graph("Current (A)");
				commands_plot_add_graph("Inductance (uH)");
			} else if (get_motor_now()->m_hfi_plot_en == 3) {
				get_motor_now()->m_hfi_plot_sample = 0.0;
				commands_init_plot("Sample Index", "Value");
				commands_plot_add_graph("Inductance");;
			}

			commands_printf(get_motor_now()->m_hfi_plot_en ?
					"HFI plot enabled" :
					"HFI plot disabled");
		} else {
			commands_printf("Invalid Argument. en has to be 0, 1, 2 or 3.\n");
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}
