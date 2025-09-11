/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2025-08-27 19:14:11
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2025-09-08 11:28:58
 * @FilePath: \VESC_Code\All_About_FOC\my_foc\MDK-ARM\foc\foc_callbacks.c
 * @Description: 
 * 
 * Copyright (c) 2025 by xiayuan, All Rights Reserved. 
 */
#include "foc_callbacks.h"
#include "datatypes.h"
#include "foc_interface.h"
#include "foc_math.h"

void dma_tc_callback(DMA_HandleTypeDef *_hdma) {

	volatile motor_all_state_t *m = &motor;
	volatile motor_state_t *motor_state = &m->m_motor_state;

	motor_state->v_bus = GET_INPUT_VOLTAGE();  //更新总线电压,之后要用

	bool is_v7 = !(TIM1->CR1 & TIM_CR1_DIR);  //用TIM1状态判断是不是“v7”
	if (is_v7) {
		return; // v7模式不需要采样电流
	}

	float ia = (GET_A_CURRENT() - m->m_conf->foc_offsets_current[0]) * FAC_CURRENT;
	float ib = (GET_B_CURRENT() - m->m_conf->foc_offsets_current[1]) * FAC_CURRENT;
	float ic = (GET_C_CURRENT() - m->m_conf->foc_offsets_current[2]) * FAC_CURRENT;
    // 我要写参数辨识的话，应该先把采样-计算-控制环路完成，保证控制器能稳定输出我想要的电流，再来验证参数辨识的代码，对吗

	m->m_currents_adc[0] = ia;
	m->m_currents_adc[1] = ib;
	m->m_currents_adc[2] = ic;

	// 等幅值Clarke变换
	motor_state->i_alpha = (2.0 / 3.0) * (ia - 0.5 * (ib + ic));
	motor_state->i_beta = (2.0 / 3.0) * (ib - ic);

	motor_state->phase = get_electrical_angle();

	// 这个phase由编码器线程获得
	utils_fast_sincos_better(motor_state->phase,
					(float*)&motor_state->phase_sin,
					(float*)&motor_state->phase_cos);
	// Park 变换
	float c = motor_state->phase_cos;
	float s = motor_state->phase_sin;
	motor_state->id = c * motor_state->i_alpha + s * motor_state->i_beta;
	motor_state->iq = c * motor_state->i_beta - s * motor_state->i_alpha;

	current_control();
}