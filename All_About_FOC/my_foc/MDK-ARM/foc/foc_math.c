/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2025-08-27 19:15:25
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2025-09-08 11:27:52
 * @FilePath: \VESC_Code\All_About_FOC\my_foc\MDK-ARM\foc\foc_math.c
 * @Description: 
 * 
 * Copyright (c) 2025 by xiayuan, All Rights Reserved. 
 */
#include "foc_math.h"
#include "utils_math.h"
#include "datatypes.h"
#include "foc_encoders.h"

extern AS504x_config_t as5047_cfg;
typedef unsigned int uint32_t;

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

static void svpwm(float alpha, float beta, uint32_t *duty_a, uint32_t *duty_b, uint32_t *duty_c, uint32_t top, uint32_t *svm_sector);

/**
 * @description: 
 * 前置变量：
 * phase_sin, phase_cos, 
 * target_id, target_iq, 
 * i_alpha, i_beta(本轮测量值)
 * conf->foc_current_ki, foc_current_kp
 * motor_state->max_duty, v_bus用于限幅
 * 
 * 输出变量：vd_int, vq_int, vd, vq, mod_q, mod_d
 * 
 * 执行频率: 1/2采样频率, 即svpwm频率
 * 
 * PI电流控制器原理:
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

 * @return {*}
 */
void current_control(void) {
    volatile motor_all_state_t *m = &motor;
	volatile motor_state_t *motor_state = &m->m_motor_state;
	volatile mc_configuration *conf = m->m_conf;

	float dt = 1.0 / (conf->foc_f_zv / 2.0);  // 执行周期

	float s = motor_state->phase_sin;
	float c = motor_state->phase_cos;

	motor_state->id = c * motor_state->i_alpha + s * motor_state->i_beta;
	motor_state->iq = -s * motor_state->i_alpha + c * motor_state->i_beta;

	//输入PI控制器的误差项
	float id_err = motor_state->id_target - motor_state->id;
	float iq_err = motor_state->iq_target - motor_state->iq;

	motor_state->vd_int += conf->foc_current_ki * id_err * dt;
	motor_state->vq_int += conf->foc_current_ki * iq_err * dt;

	// PI控制器输出电流, 不用D项因为电机是一阶系统
	motor_state->vd = conf->foc_current_kp * id_err + motor_state->vd_int;
	motor_state->vq = conf->foc_current_kp * iq_err + motor_state->vq_int;

	// 进行解耦, 解耦依赖电机参数识别
	float dec_vd = 0.0;
	float dec_vq = 0.0;
	float dec_bemf = 0.0;

	// 解耦
	switch (conf->foc_cc_decoupling) {
	case FOC_CC_DECOUPLING_DISABLED:  // 没有参数识别, 还没发解耦
		// 不进行解耦
		break;
	case FOC_CC_DECOUPLING_CROSS:  //todo: 完善id iq测量值的低通滤波, 应用到解耦中
		// // 交叉解耦
		// motor_state->vd += Ld * (motor_state->iq - motor_state->i_beta) * d_gain_scale;
		// motor_state->vq -= Lq * (motor_state->id - motor_state->i_alpha) * d_gain_scale;
		break;
	case FOC_CC_DECOUPLING_BEMF:
		// // 反电动势解耦
		// motor_state->vd += Ld * (motor_state->iq - motor_state->i_beta) * d_gain_scale;
		// motor_state->vq -= Lq * (motor_state->id - motor_state->i_alpha) * d_gain_scale;
		break;
	case FOC_CC_DECOUPLING_CROSS_BEMF:
		// // 交叉+反电动势解耦
		// motor_state->vd += Ld * (motor_state->iq - motor_state->i_beta) * d_gain_scale;
		// motor_state->vq -= Lq * (motor_state->id - motor_state->i_alpha) * d_gain_scale;
		break;
	}
	motor_state->vd -= dec_vd;
	motor_state->vq += dec_vq + dec_bemf;

	//计算出控制器输出的vd vq, 然后进行限幅和防饱和
	float max_v_mag = ONE_BY_SQRT3 * motor_state->max_duty * motor_state->v_bus;  // 最大电压幅值, 归一化到1
	float vd_pre = motor_state->vd;
	utils_truncate_number_abs((float*)&motor_state->vd, max_v_mag);
	motor_state->vd_int -= (motor_state->vd - vd_pre);  // 防止积分饱和
	// vd <= vd_presat, 相减出来的是非正数, 意味着限幅前后的差值, 
	// 如果这个差值很大, 说明积分超调, 需要减小积分增益
	float vq_pre = motor_state->vq;
	float vq_max = sqrtf(SQ(max_v_mag) - SQ(motor_state->vd));
	utils_truncate_number_abs((float*)&motor_state->vq, vq_max);
	motor_state->vq_int -= (motor_state->vq - vq_pre);

	utils_saturate_vector_2d((float*)&motor_state->vq, (float*)&motor_state->vd, max_v_mag);

	const float voltage_normalize_fac = 1.5 / motor_state->v_bus;
	motor_state->mod_d = motor_state->vd * voltage_normalize_fac;
	motor_state->mod_q = motor_state->vq * voltage_normalize_fac;

	motor_state->mod_alpha_raw = c * motor_state->mod_d - s * motor_state->mod_q;
	motor_state->mod_beta_raw = s * motor_state->mod_d + c * motor_state->mod_q;
	// 这里的raw直接用于svm计算

	uint32_t duty_a, duty_b, duty_c, top;
	top = TIM1->ARR;
	svpwm(motor_state->mod_alpha_raw, motor_state->mod_beta_raw, &duty_a, &duty_b, &duty_c, top, (uint32_t*)&motor_state->svm_sector);

	TIMER_UPDATE_DUTY(duty_a, duty_b, duty_c);

	// todo: 假设已有目标id iq, 写一个电流控制器
}

/**
 * @description: 
 * svpwm计算函数, 需要目标v_alpha和v_beta, 其中扇区和矢量编码与vesc不同,
 * 具体编号参考我的博客和ti白皮书: 
 * https://xtzhyhydwl.github.io/2025/05/07/FOC%E5%AD%A6%E4%B9%A0%E4%B9%8BSVPWM/
 * top为PWM1的ARR, 作为基准来计算占空比计数值
 * 
 * @return
 * 计算得到三个占空比(uint32_t), 范围0~top, 以及当前扇区
 */
static void svpwm(float alpha, float beta, uint32_t *duty_a, uint32_t *duty_b, uint32_t *duty_c, uint32_t top, uint32_t *svm_sector) {
	// step 1, 判断扇区
	uint32_t sector = 0;
	if (alpha >= 0) {
		if (beta >= 0) {
			if (beta * ONE_BY_SQRT3 > alpha) {
				sector = 1;
			} else {
				sector = 3;
			}
		} else {
			if (-beta * ONE_BY_SQRT3 > alpha) {
				sector = 6;
			} else {
				sector = 2;
			}
		}
	} else {
		if (beta >= 0) {
			if (beta * ONE_BY_SQRT3 > alpha) {
				sector = 1;
			} else {
				sector = 5;
			}
		} else {
			if (-beta * ONE_BY_SQRT3 > alpha) {
				sector = 6;
			} else {
				sector = 4;
			}
		}
	}

	// step2 根据扇区计算三相时间
	uint32_t ta, tb, tc;
	// uint32_t t1, t2, t3, t4, t5, t6;

	switch (sector) {
		case 1: {
			uint32_t t2 = (-alpha + ONE_BY_SQRT3 * beta) * top;
			uint32_t t6 = (alpha + ONE_BY_SQRT3 * beta) * top;

			tb = (top + t2 + t6) / 2;
			ta = tb - t2;
			tc = tb - t6;
			break;
		}
		case 2: {
			uint32_t t5 = (-TWO_BY_SQRT3 * beta) * top;
			uint32_t t4 = (alpha + ONE_BY_SQRT3 * beta) * top;

			ta = (top + t5 + t4) / 2;
			tc = ta - t4;
			tb = tc - t5;
			break;
		}
		case 3: {
			uint32_t t6 = (TWO_BY_SQRT3 * beta) * top;
			uint32_t t4 = (alpha - ONE_BY_SQRT3 * beta) * top;

			ta = (top + t6 + t4) / 2;
			tb = tc - t4;
			tc = ta - t6;
			break;
		}
		case 4: {
			uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * top;
			uint32_t t1 = (-TWO_BY_SQRT3 * beta) * top;

			// PWM timings
			tc = (top + t3 + t1) / 2;
			tb = tc - t1;
			ta = tb - t3;
			break;
		}
		case 5: {
			uint32_t t2 = (TWO_BY_SQRT3 * beta) * top;
			uint32_t t3 = (-alpha - ONE_BY_SQRT3 * beta) * top;

			tb = (top + t2 + t3) / 2;
			tc = tb - t2;
			ta = tc - t3;
			break;
		}
		case 6: {
			uint32_t t5 = (alpha - ONE_BY_SQRT3 * beta) * top;
			uint32_t t1 = (-alpha - ONE_BY_SQRT3 * beta) * top;

			tc = (top + t5 + t1) / 2;
			ta = tc - t1;
			tb = ta - t5;
			break;
    }
	}
	*duty_a = ta;
	*duty_b = tb;
	*duty_c = tc;
	*svm_sector = sector;
}

/**
 * @description: 获取电机电角度
 * 前置变量：需要完成电机参数辨识和编码器整定，
 * 得到foc_encoder_inverted, foc_encoder_offset和foc_encoder_ratio
 * @return {*} phase_temp 电角度, 单位弧度
 */
float get_electrical_angle(void) {
	volatile motor_all_state_t *m = &motor;
	volatile motor_state_t *motor_state = &m->m_motor_state;
	volatile mc_configuration *conf = m->m_conf;
	float phase_temp = 0;
	if (conf->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER) {
		switch (conf->m_sensor_port_mode) {
			case SENSOR_PORT_MODE_AS5047_SPI:
				phase_temp = enc_as504x_read_angle(&as5047_cfg);
				if (conf->foc_encoder_inverted) {
					phase_temp = 360.0 - phase_temp;
				}
				phase_temp *= conf->foc_encoder_ratio; // 变比
				phase_temp -= conf->foc_encoder_offset; // 机械零位
				utils_norm_angle((float*)&phase_temp); // 归一化到0~360度
				phase_temp = DEG2RAD_f(phase_temp);
				break;
			case SENSOR_PORT_MODE_HALL:
			case SENSOR_PORT_MODE_ABI:
			case SENSOR_PORT_MODE_AD2S1205:
			case SENSOR_PORT_MODE_SINCOS:
			case SENSOR_PORT_MODE_TS5700N8501:
			case SENSOR_PORT_MODE_TS5700N8501_MULTITURN:
			case SENSOR_PORT_MODE_MT6816_SPI_HW:
			case SENSOR_PORT_MODE_AS5x47U_SPI:
			case SENSOR_PORT_MODE_BISSC:
			case SENSOR_PORT_MODE_TLE5012_SSC_SW:
			case SENSOR_PORT_MODE_TLE5012_SSC_HW:
			case SENSOR_PORT_MODE_CUSTOM_ENCODER:
			default:
				break;
		} 
	} else if (conf->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS) {
		phase_temp = motor_state->phase;

	} else {

	}

	return phase_temp;
}
