/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se
	Copyright 2022 Jakub Tomczak

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

#include "encoder.h"
#include "encoder_datatype.h"
#include "encoder_cfg.h"

#include "utils.h"
#include "utils_math.h"
#include "commands.h"
#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "mempools.h"
#include "terminal.h"
#include "app.h"

#include <math.h>

int mul_pos_base = 0;
float mul_pos = 0;
float mul_pos_last = 0;
float pos_temp = 0;
float pos_temp_pre = 180;  //180是因为防止上电 pre = 0 但是 now > 270 导致的base多减一圈
float filter_sum = 0;

extern int mul_pos_base;
extern float mul_pos;
extern float pos_temp;
extern float pos_temp_pre;



// These rates turn into even multiples of systicks
typedef enum {
	routine_rate_1k = 0,
	routine_rate_2k,
	routine_rate_5k,
	routine_rate_10k
} routine_rate_t;

volatile routine_rate_t m_routine_rate = routine_rate_1k;
static encoder_type_t m_encoder_type_now = ENCODER_TYPE_NONE;
static float m_enc_custom_pos = 0.0;   //客制化编码器的角度储存在这里
 

static THD_WORKING_AREA(routine_thread_wa, 256);  
static THD_FUNCTION(routine_thread, arg);  //编码器专用线程和working area？

// Private functions
static void terminal_encoder(int argc, const char **argv);
static void terminal_encoder_clear_errors(int argc, const char **argv);
static void terminal_encoder_clear_multiturn(int argc, const char **argv);
static void timer_start(routine_rate_t rate);

// Function pointers
static float (*m_enc_custom_read_deg)(void) = NULL;  //读客制化编码器的函数指针，要改
static bool (*m_enc_custom_fault)(void) = NULL;
static char* (*m_enc_custom_print_info)(void) = NULL;

bool encoder_init(volatile mc_configuration *conf) {  //编码器初始化函数
	bool res = false;

	if (m_encoder_type_now != ENCODER_TYPE_NONE) {
		encoder_deinit();
	}

	nvicDisableVector(HW_ENC_EXTI_CH);
	nvicDisableVector(HW_ENC_TIM_ISR_CH);
	TIM_DeInit(HW_ENC_TIM);

	switch (conf->m_sensor_port_mode) {
	case SENSOR_PORT_MODE_ABI: {                        //使用ABI编码器
		SENSOR_PORT_5V();

		encoder_cfg_ABI.counts = conf->m_encoder_counts;

		if (!enc_abi_init(&encoder_cfg_ABI)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_ABI;
		res = true;
	} break;

	case SENSOR_PORT_MODE_AS5047_SPI: {
		SENSOR_PORT_3V3();

		if (!enc_as504x_init(&encoder_cfg_as504x)) {
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_AS504x;
		timer_start(routine_rate_10k);  //5047好像是占用一个线程的？  10k刷新率，好高

		res = true;
	} break;

	case SENSOR_PORT_MODE_MT6816_SPI_HW: {
		SENSOR_PORT_5V();

		if (!enc_mt6816_init(&encoder_cfg_mt6816)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_MT6816;
		timer_start(routine_rate_10k);

		res = true;
	} break;

	// ssc (3 wire) sw spi on hall pins
	case SENSOR_PORT_MODE_TLE5012_SSC_SW: {
		SENSOR_PORT_5V();

		// reuse global config, so must set up complete ssc config
		spi_bb_state sw_ssc = {
				HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, // nss
				HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, // sck
				HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, // mosi
				HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, // miso
				{{NULL, NULL}, NULL, NULL} // Mutex
		};
		encoder_cfg_tle5012.sw_spi = sw_ssc;

		if (!enc_tle5012_init_sw_ssc(&encoder_cfg_tle5012)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_TLE5012;
		timer_start(routine_rate_5k); // slow down sw spi as transactions long

		res = true;
	} break;

	// ssc (3 wire) hw spi w dma (sw spi using hw spi pins for now)
	case SENSOR_PORT_MODE_TLE5012_SSC_HW: {
#ifdef HW_SPI_DEV
		SENSOR_PORT_5V();

		// reuse global config, so must set up complete ssc config
		spi_bb_state sw_ssc = {
				HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, // nss
				HW_SPI_PORT_SCK, HW_SPI_PIN_SCK, // sck
				HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, // mosi
				HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, // miso (shared dat line)
				{{NULL, NULL}, NULL, NULL} // Mutex
		};
		encoder_cfg_tle5012.sw_spi = sw_ssc;	

		if (!enc_tle5012_init_sw_ssc(&encoder_cfg_tle5012)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_TLE5012;
		// timer_start(10000);
		timer_start(routine_rate_10k);

		res = true;
#else
		res = false;
#endif
	} break;

	case SENSOR_PORT_MODE_AD2S1205: {
		SENSOR_PORT_5V();

		if (!enc_ad2s1205_init(&encoder_cfg_ad2s1205)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_AD2S1205_SPI;
		timer_start(routine_rate_10k);

		res = true;
	} break;

	case SENSOR_PORT_MODE_SINCOS: {
		SENSOR_PORT_5V();

		encoder_cfg_sincos.s_gain = 1.0 / conf->m_encoder_sin_amp;
		encoder_cfg_sincos.s_offset = conf->m_encoder_sin_offset;
		encoder_cfg_sincos.c_gain = 1.0 /conf->m_encoder_cos_amp;
		encoder_cfg_sincos.c_offset =  conf->m_encoder_cos_offset;
		encoder_cfg_sincos.filter_constant = conf->m_encoder_sincos_filter_constant;
		sincosf(DEG2RAD_f(conf->m_encoder_sincos_phase_correction), &encoder_cfg_sincos.sph, &encoder_cfg_sincos.cph);

		if (!enc_sincos_init(&encoder_cfg_sincos)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_SINCOS;
		res = true;
	} break;

	case SENSOR_PORT_MODE_TS5700N8501:
	case SENSOR_PORT_MODE_TS5700N8501_MULTITURN: {
		SENSOR_PORT_5V();
		app_configuration *appconf = mempools_alloc_appconf();
		conf_general_read_app_configuration(appconf);
		if (appconf->app_to_use == APP_ADC ||
				appconf->app_to_use == APP_UART ||
				appconf->app_to_use == APP_PPM_UART ||
				appconf->app_to_use == APP_ADC_UART) {
			appconf->app_to_use = APP_NONE;
			app_set_configuration(appconf);
			conf_general_store_app_configuration(appconf);
		}
		mempools_free_appconf(appconf);

		if (!enc_ts5700n8501_init(&encoder_cfg_TS5700N8501)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_TS5700N8501;
		res = true;
	} break;

	case SENSOR_PORT_MODE_AS5x47U_SPI: {
		SENSOR_PORT_3V3();

		if (!enc_as5x47u_init(&encoder_cfg_as5x47u)) {
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_AS5x47U;
		timer_start(routine_rate_10k);

		res = true;
	} break;

	case SENSOR_PORT_MODE_BISSC: {
		SENSOR_PORT_5V();

		encoder_cfg_bissc.enc_res = conf->m_encoder_counts;

		if (!enc_bissc_init(&encoder_cfg_bissc)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		m_encoder_type_now = ENCODER_TYPE_BISSC;
		timer_start(routine_rate_10k);

		res = true;
	} break;

/**
 * 客制化编码器 #1
 * 使用AS5047的 ABI 返回和SPI返回融合
 * 实现低延迟绝对值编码器
 * 1.24.2024 先试试使用自带的ABI编码器读取，不自己写
 * TODO:定时器分配问题，定时器中断回调问题
 * 1. ABI初始化用到ChibiOS的encoder库，好像读角度要用定时器中断；AS5047似乎只是单纯的SPI通信，没有用到定时器中断 done
 * 2. ABI外部中断引脚需要去hw里面改，配置也要改 done
 * 3. 检查定时器中断里有没有写什么会产生冲突的地方，或是任何采集数据过程 done as5047的SPI和ABI可以和其它接口同时使用
 * 4. 数据融合 done 
 */
/**************************************************************************************************/

	case SENSOR_PORT_MODE_CUSTOM_ENCODER:        //客制化编码器
#if defined (USE_CUSTOM_ENCODER1)
		m_encoder_type_now = ENCODER_TYPE_CUSTOM;
		SENSOR_PORT_3V3();
		
		conf->m_encoder_counts = 4000;
		encoder_cfg_ABI.counts = conf->m_encoder_counts;

		if (!enc_abi_init(&encoder_cfg_ABI)) {   //先尝试使用内置库  
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}


		if (!enc_as504x_init(&encoder_cfg_as504x)) {
			m_encoder_type_now = ENCODER_TYPE_NONE;
			return false;
		}

		timer_start(routine_rate_10k);  //5047好像是占用一个线程的？
		encoder_set_custom_callbacks(custom_as5047_read_deg, 
									 custom_as5047_fault_check, 
									 custom_as5047_print_info);  //设置回调函数

		res = true;
#endif
		break;
/**************************************************************************************************/

	default:
		SENSOR_PORT_5V();
		m_encoder_type_now = ENCODER_TYPE_NONE;
		break;
	}

	terminal_register_command_callback(
			"encoder",
			"Prints the status of the AS5047, AS5x47U, AD2S1205, TLE5012, MT6816, or TS5700N8501 encoder.",
			0,
			terminal_encoder);

	terminal_register_command_callback(
			"encoder_clear_errors",
			"Clear error of the TS5700N8501 encoder, AD2S1205 resolver.",
			0,
			terminal_encoder_clear_errors);

	terminal_register_command_callback(
			"encoder_clear_multiturn",
			"Clear multiturn counter of the TS5700N8501 encoder.",
			0,
			terminal_encoder_clear_multiturn);

	return res;
}

void encoder_update_config(volatile mc_configuration *conf) {
	switch (conf->m_sensor_port_mode) {
	case SENSOR_PORT_MODE_SINCOS: {
		encoder_cfg_sincos.s_gain = 1.0 / conf->m_encoder_sin_amp;
		encoder_cfg_sincos.s_offset = conf->m_encoder_sin_offset;
		encoder_cfg_sincos.c_gain = 1.0 /conf->m_encoder_cos_amp;
		encoder_cfg_sincos.c_offset =  conf->m_encoder_cos_offset;
		encoder_cfg_sincos.filter_constant = conf->m_encoder_sincos_filter_constant;
		sincosf(DEG2RAD_f(conf->m_encoder_sincos_phase_correction), &encoder_cfg_sincos.sph, &encoder_cfg_sincos.cph);
	} break;

	case SENSOR_PORT_MODE_ABI: {                  //ABI编码器更新配置
		encoder_cfg_ABI.counts = conf->m_encoder_counts;
	} break;

	default:
		break;
	}
}

void encoder_deinit(void) {
	nvicDisableVector(HW_ENC_EXTI_CH);
	nvicDisableVector(HW_ENC_TIM_ISR_CH);
	TIM_DeInit(HW_ENC_TIM);

	if (m_encoder_type_now == ENCODER_TYPE_AS504x) {
		enc_as504x_deinit(&encoder_cfg_as504x);
	} else if (m_encoder_type_now == ENCODER_TYPE_MT6816) {
		enc_mt6816_deinit(&encoder_cfg_mt6816);
	} else if (m_encoder_type_now == ENCODER_TYPE_TLE5012) {
		enc_tle5012_deinit(&encoder_cfg_tle5012);
	} else if (m_encoder_type_now == ENCODER_TYPE_AD2S1205_SPI) {
		enc_ad2s1205_deinit(&encoder_cfg_ad2s1205);
	} else if (m_encoder_type_now == ENCODER_TYPE_ABI) {
		enc_abi_deinit(&encoder_cfg_ABI);
	} else if (m_encoder_type_now == ENCODER_TYPE_SINCOS) {
		enc_sincos_deinit(&encoder_cfg_sincos);
	} else if (m_encoder_type_now == ENCODER_TYPE_TS5700N8501) {
		enc_ts5700n8501_deinit(&encoder_cfg_TS5700N8501);
	} else if (m_encoder_type_now == ENCODER_TYPE_AS5x47U) {
		enc_as5x47u_deinit(&encoder_cfg_as5x47u);
	} else if (m_encoder_type_now == ENCODER_TYPE_BISSC) {
		enc_bissc_deinit(&encoder_cfg_bissc);
	} else if (m_encoder_type_now == ENCODER_TYPE_CUSTOM) {
		enc_as504x_deinit(&encoder_cfg_as504x);
		enc_abi_deinit(&encoder_cfg_ABI);
	}

	m_encoder_type_now = ENCODER_TYPE_NONE;
}

void encoder_set_custom_callbacks (   //可以自己写自己的编码器
		float (*read_deg)(void),
		bool (*has_fault)(void),
		char* (*print_info)(void)) {

	if (utils_is_func_valid(read_deg)) {
		m_enc_custom_read_deg = read_deg;
	} else {
		m_enc_custom_read_deg = NULL;
	}

	if (utils_is_func_valid(has_fault)) {
		m_enc_custom_fault = has_fault;
	} else {
		m_enc_custom_fault = NULL;
	}

	if (utils_is_func_valid(print_info)) {
		m_enc_custom_print_info = print_info;
	} else {
		m_enc_custom_print_info = NULL;
	}
}

#if defined (USE_CUSTOM_ENCODER1)
/**
 * as5047读角度，SPI绝对值和ABI融合返回位置
 * 
 */
/**************************************************************************************************/
bool custom_encoder_fault = false;
uint8_t custom_encoder_fault_count = 0;
uint8_t stop_count = 0;
uint8_t diff_fault_count = 0;
float last_ABS_ang = 0;
float last_ABI_ang = 0;
//bool get_diff = false;

float custom_as5047_read_deg (void) {
	float ABS_ang = AS504x_LAST_ANGLE(&encoder_cfg_as504x);
	float ABI_ang = enc_abi_read_deg(&encoder_cfg_ABI);
	float res = 0;

	//get_diff = true;

	if (encoder_cfg_ABI.state.index_found == true) {
//		res = last_ABS_ang + dtheta;  //当前低延迟角度 = 上次绝对值 + 上次与这次相对值角度误差
		res = ABI_ang;  //同一个AS5047 ABI输出和SPI输出的零位置似乎是一样的 之后直接用ABI的输出就行
	}
	else {
		res = ABS_ang;
	}

	last_ABS_ang = ABS_ang;
	last_ABI_ang = ABI_ang;  //更新位置
	return res;
}

bool custom_as5047_fault_check(void) {  //只有SPI会出问题，和SPI的一模一样
	return false;
}

char* custom_as5047_print_info(void) {
	//可能以后加一些
	return NULL;
}
/**************************************************************************************************/
#endif

float encoder_read_deg(void) {  //编码器读角度函数
	if (m_encoder_type_now == ENCODER_TYPE_AS504x) {
		return AS504x_LAST_ANGLE(&encoder_cfg_as504x);
	} else if (m_encoder_type_now == ENCODER_TYPE_MT6816) {
		return MT6816_LAST_ANGLE(&encoder_cfg_mt6816);
	} else if (m_encoder_type_now == ENCODER_TYPE_TLE5012) {
		return TLE5012_LAST_ANGLE(&encoder_cfg_tle5012);
	} else if (m_encoder_type_now == ENCODER_TYPE_AD2S1205_SPI) {
		return AD2S1205_LAST_ANGLE(&encoder_cfg_ad2s1205);
	} else if (m_encoder_type_now == ENCODER_TYPE_ABI) {
		return enc_abi_read_deg(&encoder_cfg_ABI);
	} else if (m_encoder_type_now == ENCODER_TYPE_SINCOS) {
		return enc_sincos_read_deg(&encoder_cfg_sincos);
	} else if (m_encoder_type_now == ENCODER_TYPE_TS5700N8501) {
		return enc_ts5700n8501_read_deg(&encoder_cfg_TS5700N8501);
	} else if (m_encoder_type_now == ENCODER_TYPE_AS5x47U) {
		return AS5x47U_LAST_ANGLE(&encoder_cfg_as5x47u);
	} else if (m_encoder_type_now == ENCODER_TYPE_BISSC) {
		return BISSC_LAST_ANGLE(&encoder_cfg_bissc);
	} else if (m_encoder_type_now == ENCODER_TYPE_CUSTOM) {
		if (m_enc_custom_read_deg) {
			return m_enc_custom_read_deg();
		} else {
			return m_enc_custom_pos;
		}
	}
	return 0.0;
}

float encoder_read_deg_multiturn(void) {
	if (m_encoder_type_now == ENCODER_TYPE_TS5700N8501) {
		float ts_mt = (float)enc_ts5700n8501_get_abm(&encoder_cfg_TS5700N8501);
		if (fabsf(ts_mt) > 5000.0) {
			ts_mt = 0;
			encoder_reset_multiturn();
		}

		ts_mt += 5000;

		return encoder_read_deg() / 10000.0 + (360 * ts_mt) / 10000.0;
	} else {
		return encoder_read_deg();
	}
}

void encoder_set_deg(float deg) {  //给ABI编码器设置角度？？
	utils_norm_angle(&deg);

	if (m_encoder_type_now == ENCODER_TYPE_ABI) {
		encoder_cfg_ABI.timer->CNT = (uint32_t)(deg / 360.0 * (float)encoder_cfg_ABI.counts);
		encoder_cfg_ABI.state.index_found = true;
	} else if (m_encoder_type_now == ENCODER_TYPE_CUSTOM) {
		m_enc_custom_pos = deg;
	}
}

encoder_type_t encoder_is_configured(void) {
	return m_encoder_type_now;
}

bool encoder_index_found(void) {
	if (m_encoder_type_now == ENCODER_TYPE_ABI) {
		return encoder_cfg_ABI.state.index_found;
	} else {
		return true;
	}
}

void encoder_reset_multiturn(void) {
	if (m_encoder_type_now == ENCODER_TYPE_TS5700N8501) {
		return enc_ts5700n8501_reset_multiturn(&encoder_cfg_TS5700N8501);
	}
	mul_pos_base = 0;  //多圈基数归零
}

void encoder_reset_errors(void) {
	if (m_encoder_type_now == ENCODER_TYPE_TS5700N8501) {
		enc_ts5700n8501_reset_errors(&encoder_cfg_TS5700N8501);
	}
	if (m_encoder_type_now == ENCODER_TYPE_AD2S1205_SPI) {
		enc_ad2s1205_reset_errors(&encoder_cfg_ad2s1205);
	}
}

float encoder_get_error_rate(void) {
	float res = -1.0;

	switch (m_encoder_type_now) {
	case ENCODER_TYPE_AS504x:
		res = encoder_cfg_as504x.state.spi_error_rate;
		break;
	case ENCODER_TYPE_MT6816:
		res = encoder_cfg_mt6816.state.encoder_no_magnet_error_rate;
		break;
	case ENCODER_TYPE_TLE5012:
		res = encoder_cfg_tle5012.state.spi_error_rate;
		break;
	case ENCODER_TYPE_AD2S1205_SPI:
		res = encoder_cfg_ad2s1205.state.resolver_loss_of_tracking_error_rate;
		if (encoder_cfg_ad2s1205.state.resolver_degradation_of_signal_error_rate > res) {
			res = encoder_cfg_ad2s1205.state.resolver_degradation_of_signal_error_rate;
		}
		if (encoder_cfg_ad2s1205.state.resolver_loss_of_signal_error_rate > res) {
			res = encoder_cfg_ad2s1205.state.resolver_loss_of_signal_error_rate;
		}
		break;
	case ENCODER_TYPE_SINCOS:
		res = encoder_cfg_sincos.state.signal_low_error_rate;
		if (encoder_cfg_sincos.state.signal_above_max_error_rate > res) {
			res = encoder_cfg_sincos.state.signal_above_max_error_rate;
		}
		break;
	case ENCODER_TYPE_AS5x47U:
		res = encoder_cfg_as5x47u.state.spi_error_rate;
		break;
	case ENCODER_TYPE_BISSC:
		res = encoder_cfg_bissc.state.spi_comm_error_rate;
		if (encoder_cfg_bissc.state.spi_data_error_rate > res) {
			res = encoder_cfg_bissc.state.spi_data_error_rate;
		}
		break;
	default:
		break;
	}

	return res;
}

// Check for encoder faults that should stop the motor with a fault code.
void encoder_check_faults(volatile mc_configuration *m_conf, bool is_second_motor) {  //编码器错误

	// Only generate fault code when the encoder is being used. Note that encoder faults
	// that occur above the sensorless ERPM won't stop the motor.
	bool is_foc_encoder = m_conf->motor_type == MOTOR_TYPE_FOC &&
			m_conf->foc_sensor_mode == FOC_SENSOR_MODE_ENCODER &&
			mcpwm_foc_is_using_encoder();

	if (is_foc_encoder) {
		switch (m_conf->m_sensor_port_mode) {
		case SENSOR_PORT_MODE_AS5047_SPI:
			if (encoder_cfg_as504x.state.spi_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, is_second_motor, false);
			}

			if (encoder_cfg_as504x.sw_spi.mosi_gpio != NULL) {   //由于磁性和连接导致的错误
				AS504x_diag diag = encoder_cfg_as504x.state.sensor_diag;
				if (!diag.is_connected) {
					mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, is_second_motor, false);
				}

				if (diag.is_Comp_high) {
					mc_interface_fault_stop(FAULT_CODE_ENCODER_NO_MAGNET, is_second_motor, false);
				} else if(diag.is_Comp_low) {
					mc_interface_fault_stop(FAULT_CODE_ENCODER_MAGNET_TOO_STRONG, is_second_motor, false);
				}
			}
			break;

		case SENSOR_PORT_MODE_MT6816_SPI_HW:
			if (encoder_cfg_mt6816.state.encoder_no_magnet_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_NO_MAGNET, is_second_motor, false);
			}
			break;
		
		case SENSOR_PORT_MODE_TLE5012_SSC_HW:
		case SENSOR_PORT_MODE_TLE5012_SSC_SW:
			if (encoder_cfg_tle5012.state.spi_error_rate > 0.10) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_FAULT, is_second_motor, false);
			}
			if (encoder_cfg_tle5012.state.last_status_error != NO_ERROR &&
			    encoder_cfg_tle5012.state.last_status_error != CRC_ERROR) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_FAULT, is_second_motor, false);
			} // allow some crc errors below 10% error rate
			break;

		case SENSOR_PORT_MODE_SINCOS:
			if (encoder_cfg_sincos.state.signal_low_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE, is_second_motor, false);
			}
			if (encoder_cfg_sincos.state.signal_above_max_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE, is_second_motor, false);
			}
			break;

		case SENSOR_PORT_MODE_AD2S1205:
			if (encoder_cfg_ad2s1205.state.resolver_loss_of_tracking_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_RESOLVER_LOT, is_second_motor, false);
			}
			if (encoder_cfg_ad2s1205.state.resolver_degradation_of_signal_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_RESOLVER_DOS, is_second_motor, false);
			}
			if (encoder_cfg_ad2s1205.state.resolver_loss_of_signal_error_rate > 0.04) {
				mc_interface_fault_stop(FAULT_CODE_RESOLVER_LOS, is_second_motor, false);
			}
			if (encoder_cfg_ad2s1205.state.resolver_void_packet_error_rate > 0.05){
				mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, is_second_motor, false);
			}
			if (encoder_cfg_ad2s1205.state.resolver_vel_packet_error_rate  > 0.05){
				mc_interface_fault_stop( FAULT_CODE_ENCODER_SPI, is_second_motor, false);
			}
			if (encoder_cfg_ad2s1205.state.spi_error_rate > 0.05){
				mc_interface_fault_stop( FAULT_CODE_ENCODER_SPI, is_second_motor, false);
			}
			break;

		case SENSOR_PORT_MODE_AS5x47U_SPI:
			if (encoder_cfg_as5x47u.state.spi_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, is_second_motor, false);
			}

			AS5x47U_diag diag = encoder_cfg_as5x47u.state.sensor_diag;
			if (!diag.is_connected) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, is_second_motor, false);
			}

			if (diag.is_Comp_high) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_NO_MAGNET, is_second_motor, false);
			} else if (diag.is_Comp_low) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_MAGNET_TOO_STRONG, is_second_motor, false);
			} else if (diag.is_broken_hall || diag.is_COF || diag.is_wdtst) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_FAULT, is_second_motor, false);
			}

			break;

		case SENSOR_PORT_MODE_BISSC:
			if (encoder_cfg_bissc.state.spi_comm_error_rate > 0.04) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, is_second_motor, false);
			}
			if (encoder_cfg_bissc.state.spi_data_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_RESOLVER_LOT, is_second_motor, false);
			}
			break;

		case SENSOR_PORT_MODE_CUSTOM_ENCODER:
			if (encoder_cfg_as504x.state.spi_error_rate > 0.05) {
				mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, is_second_motor, false);
			}

			if (encoder_cfg_as504x.sw_spi.mosi_gpio != NULL) {   //由于磁性和连接导致的错误
				AS504x_diag diag2 = encoder_cfg_as504x.state.sensor_diag;
				if (!diag2.is_connected) {
					mc_interface_fault_stop(FAULT_CODE_ENCODER_SPI, is_second_motor, false);
				}

				if (diag2.is_Comp_high) {
					mc_interface_fault_stop(FAULT_CODE_ENCODER_NO_MAGNET, is_second_motor, false);
				} else if(diag2.is_Comp_low) {
					mc_interface_fault_stop(FAULT_CODE_ENCODER_MAGNET_TOO_STRONG, is_second_motor, false);
				}
			}
			break;

		default:
			break;
		}
	}
}

void encoder_pin_isr(void) {
	enc_abi_pin_isr(&encoder_cfg_ABI);
}

void encoder_tim_isr(void) {
	// Use thread. Maybe use this one for encoders with a higher rate.
}

static void terminal_encoder(int argc, const char **argv) {
	(void)argc; (void)argv;

	const volatile mc_configuration *mcconf = mc_interface_get_configuration();

	switch (mcconf->m_sensor_port_mode) {
	case SENSOR_PORT_MODE_AS5047_SPI:
		commands_printf("SPI encoder value: %d, errors: %d, error rate: %.3f %%",
				encoder_cfg_as504x.state.spi_val, 
				encoder_cfg_as504x.state.spi_communication_error_count,
				(double)(encoder_cfg_as504x.state.spi_error_rate * 100.0));

		if (encoder_cfg_as504x.sw_spi.mosi_gpio != NULL) {
			commands_printf("\nAS5047 DIAGNOSTICS:\n"
					"Connected : %u\n"
					"AGC       : %u\n"
					"Magnitude : %u\n"
					"COF       : %u\n"
					"OCF       : %u\n"
					"COMP_low  : %u\n"
					"COMP_high : %u",
					encoder_cfg_as504x.state.sensor_diag.is_connected,
					encoder_cfg_as504x.state.sensor_diag.AGC_value,
					encoder_cfg_as504x.state.sensor_diag.magnitude,
					encoder_cfg_as504x.state.sensor_diag.is_COF,
					encoder_cfg_as504x.state.sensor_diag.is_OCF,
					encoder_cfg_as504x.state.sensor_diag.is_Comp_low,
					encoder_cfg_as504x.state.sensor_diag.is_Comp_high);
		}
		break;

	case SENSOR_PORT_MODE_MT6816_SPI_HW:
		commands_printf("Low flux error (no magnet): errors: %d, error rate: %.3f %%",
				encoder_cfg_mt6816.state.encoder_no_magnet_error_cnt,
				(double)(encoder_cfg_mt6816.state.encoder_no_magnet_error_rate * 100.0));
		break;

	case SENSOR_PORT_MODE_TLE5012_SSC_HW:
	case SENSOR_PORT_MODE_TLE5012_SSC_SW: ;
		uint8_t status = encoder_cfg_tle5012.state.last_status_error; // get before other queries
		double temperature = 0;
		uint16_t magnet_magnitude = 0;
		enc_tle5012_get_temperature(&encoder_cfg_tle5012, &temperature);
		enc_tle5012_get_magnet_magnitude(&encoder_cfg_tle5012, &magnet_magnitude);
		commands_printf("Last error: %d, ssc error rate: %.3f %%, magnet strength: %d, temp %.2f C",
				status,
				(double)(encoder_cfg_tle5012.state.spi_error_rate * 100.0),
				magnet_magnitude,
				temperature);
		// todo, get/report status word (reg 0x00), make "last error" verbose
		break;

	case SENSOR_PORT_MODE_TS5700N8501:
	case SENSOR_PORT_MODE_TS5700N8501_MULTITURN: {
		char sf[9];
		char almc[9];
		utils_byte_to_binary(enc_ts5700n8501_get_raw_status(&encoder_cfg_TS5700N8501)[0], sf);
		utils_byte_to_binary(enc_ts5700n8501_get_raw_status(&encoder_cfg_TS5700N8501)[7], almc);
		commands_printf("TS5700N8501 ABM: %d, SF: %s, ALMC: %s", enc_ts5700n8501_get_abm(&encoder_cfg_TS5700N8501), sf, almc);
	} break;

	case SENSOR_PORT_MODE_SINCOS:
		commands_printf("Sin/Cos encoder signal below minimum amplitude: errors: %d, error rate: %.3f %%",
				encoder_cfg_sincos.state.signal_below_min_error_cnt,
				(double)(encoder_cfg_sincos.state.signal_low_error_rate * 100.0));

		commands_printf("Sin/Cos encoder signal above maximum amplitude: errors: %d, error rate: %.3f %%",
				encoder_cfg_sincos.state.signal_above_max_error_cnt,
				(double)(encoder_cfg_sincos.state.signal_above_max_error_rate * 100.0));
		break;

	case SENSOR_PORT_MODE_AD2S1205:
		commands_printf("SPI encoder value: %d, errors: %d, error rate: %.3f %%",
				encoder_cfg_ad2s1205.state.spi_val,
				encoder_cfg_ad2s1205.state.spi_error_cnt,
				(double)(encoder_cfg_ad2s1205.state.spi_error_rate * 100.0));
		commands_printf("SPI encoder peak error rate: %.3f %%",
					(double)encoder_cfg_ad2s1205.state.resolver_SPI_peak_error_rate * (double)100.0);
		commands_printf("Resolver Loss Of Tracking (>5%c error): errors: %d, error rate: %.3f %%", 0xB0,
				encoder_cfg_ad2s1205.state.resolver_loss_of_tracking_error_cnt,
				(double)(encoder_cfg_ad2s1205.state.resolver_loss_of_tracking_error_rate * 100.0));
		commands_printf("Resolver Loss Of Tracking peak error rate: %.3f %%",
				(double)encoder_cfg_ad2s1205.state.resolver_LOT_peak_error_rate * (double)100.0);
		commands_printf("Resolver Degradation Of Signal (>33%c error): errors: %d, error rate: %.3f %%", 0xB0,
				encoder_cfg_ad2s1205.state.resolver_degradation_of_signal_error_cnt,
				(double)(encoder_cfg_ad2s1205.state.resolver_degradation_of_signal_error_rate * 100.0));
		commands_printf("Resolver Degradation Of Signal peak error rate: %.3f %%",
				(double)encoder_cfg_ad2s1205.state.resolver_DOS_peak_error_rate * (double)100.0);
		commands_printf("Resolver Loss Of Signal (>57%c error): errors: %d, error rate: %.3f %%", 0xB0,
				encoder_cfg_ad2s1205.state.resolver_loss_of_signal_error_cnt,
				(double)(encoder_cfg_ad2s1205.state.resolver_loss_of_signal_error_rate * 100.0));
		commands_printf("Resolver Loss Of Signal peak error rate: %.3f %%",
				(double)encoder_cfg_ad2s1205.state.resolver_LOS_peak_error_rate * (double)100.0);
		commands_printf("Resolver SPI empty packet: errors: %d, error rate: %.3f %%",
				encoder_cfg_ad2s1205.state.resolver_void_packet_cnt,
				(double)encoder_cfg_ad2s1205.state.resolver_void_packet_error_rate * (double)100.0);
		commands_printf("Resolver SPI empty packet peak error rate: %.3f %%",
				(double)encoder_cfg_ad2s1205.state.resolver_VOIDspi_peak_error_rate * (double)100.0);
		commands_printf("Resolver angle velocity readings: errors: %d, error rate: %.3f %%",
				encoder_cfg_ad2s1205.state.resolver_vel_packet_cnt,
				(double)encoder_cfg_ad2s1205.state.resolver_vel_packet_error_rate * (double)100.0);
		commands_printf("Resolver angle velocity readings peak error rate: %.3f %%",
				(double)encoder_cfg_ad2s1205.state.resolver_VELread_peak_error_rate * (double)100.0);
		commands_printf("\n");

		break;

	case SENSOR_PORT_MODE_ABI:
		commands_printf("Index found: %d", encoder_index_found());
		break;

	case SENSOR_PORT_MODE_AS5x47U_SPI:
		commands_printf("SPI AS5x47U encoder value: %d, errors: %d, error rate: %.3f %%",
				encoder_cfg_as5x47u.state.spi_val, encoder_cfg_as5x47u.state.spi_communication_error_count,
				(double)(encoder_cfg_as5x47u.state.spi_error_rate * 100.0));

		commands_printf("\nAS5x47U DIAGNOSTICS:\n"
				"Connected   : %u\n"
				"AGC         : %u\n"
				"Magnitude   : %u\n"
				"COF         : %u\n"
				"Hall_Broken : %u\n"
				"Error       : %u\n"
				"COMP_low    : %u\n"
				"COMP_high   : %u\n"
				"WatchdogTest: %u\n"
				"CRC Error   : %u\n"
				"MagHalf     : %u\n"
				"Error Flags : %04X\n"
				"Diag Flags  : %04X\n",
				encoder_cfg_as5x47u.state.sensor_diag.is_connected,
				encoder_cfg_as5x47u.state.sensor_diag.AGC_value,
				encoder_cfg_as5x47u.state.sensor_diag.magnitude,
				encoder_cfg_as5x47u.state.sensor_diag.is_COF,
				encoder_cfg_as5x47u.state.sensor_diag.is_broken_hall,
				encoder_cfg_as5x47u.state.sensor_diag.is_error,
				encoder_cfg_as5x47u.state.sensor_diag.is_Comp_low,
				encoder_cfg_as5x47u.state.sensor_diag.is_Comp_high,
				encoder_cfg_as5x47u.state.sensor_diag.is_wdtst,
				encoder_cfg_as5x47u.state.sensor_diag.is_crc_error,
				encoder_cfg_as5x47u.state.sensor_diag.is_mag_half,
				encoder_cfg_as5x47u.state.sensor_diag.serial_error_flgs,
				encoder_cfg_as5x47u.state.sensor_diag.serial_diag_flgs);
		break;

	case SENSOR_PORT_MODE_BISSC:
		commands_printf("BissC Loss SPI communication (>4%c error): errors: %d, error rate: %.3f %%", 0xB0,
				encoder_cfg_bissc.state.spi_comm_error_cnt,
				(double)(encoder_cfg_bissc.state.spi_comm_error_rate * 100.0));
		commands_printf("BissC Degradation Of Signal (>5%c error): errors: %d, error rate: %.3f %%", 0xB0,
				encoder_cfg_bissc.state.spi_data_error_cnt,
				(double)(encoder_cfg_bissc.state.spi_data_error_rate * 100.0));
		break;

	case SENSOR_PORT_MODE_CUSTOM_ENCODER:
		if (m_enc_custom_print_info) {
			commands_printf("%s", m_enc_custom_print_info);
		}
		break;

	default:
		commands_printf("No encoder debug info available.");
		break;
	}

	commands_printf(" ");
}

static void terminal_encoder_clear_errors(int argc, const char **argv) {
	(void)argc; (void)argv;
	encoder_reset_errors();
	commands_printf("Done!\n");
}

static void terminal_encoder_clear_multiturn(int argc, const char **argv) {
	(void)argc; (void)argv;
	encoder_reset_multiturn();
	commands_printf("Done!\n");
}

static THD_FUNCTION(routine_thread, arg) {    //开了一个线程，自动执行编码器读数据
	(void)arg;
	chRegSetThreadName("Enc Routine");

	for (;;) {
		switch (m_encoder_type_now) {
		case ENCODER_TYPE_AS504x:
			enc_as504x_routine(&encoder_cfg_as504x);
			break;

		case ENCODER_TYPE_MT6816:
			enc_mt6816_routine(&encoder_cfg_mt6816);
			break;

		case ENCODER_TYPE_TLE5012:
			enc_tle5012_routine(&encoder_cfg_tle5012);
			break;

		case ENCODER_TYPE_AD2S1205_SPI:
			enc_ad2s1205_routine(&encoder_cfg_ad2s1205);
			break;

		case ENCODER_TYPE_AS5x47U:
			enc_as5x47u_routine(&encoder_cfg_as5x47u);
			break;

		case ENCODER_TYPE_BISSC:
			enc_bissc_routine(&encoder_cfg_bissc);
			break;

		case ENCODER_TYPE_CUSTOM:
			enc_as504x_routine(&encoder_cfg_as504x);
			break;

		default:
			break;
		}

		encoder_multiturn_calc();
		encoder_send_back_mean_filter(mul_pos);

		switch (m_routine_rate) {
		case routine_rate_1k: chThdSleep(CH_CFG_ST_FREQUENCY / 1000); break;
		case routine_rate_2k: chThdSleep(CH_CFG_ST_FREQUENCY / 2000); break;
		case routine_rate_5k: chThdSleep(CH_CFG_ST_FREQUENCY / 5000); break;
		case routine_rate_10k: chThdSleep(CH_CFG_ST_FREQUENCY / 10000); break;
		default: chThdSleep(5);
		}
	}
}

static void timer_start(routine_rate_t rate) {
	m_routine_rate = rate;

	static bool routine_running = false;
	if (!routine_running) {
		routine_running = true;
		chThdCreateStatic(routine_thread_wa, sizeof(routine_thread_wa), NORMALPRIO + 5, routine_thread, NULL);
	}
}

/**
 * 编码器多圈计算函数，自己加的
 */
/**************************************************************************************************/
void encoder_multiturn_calc(void) {
	pos_temp = mc_interface_get_pid_pos_now();
	if (pos_temp > 359 && pos_temp_pre < 1)
		mul_pos_base -= 360;
	else if (pos_temp < 1 && pos_temp_pre > 359)
		mul_pos_base += 360;
	pos_temp_pre = pos_temp;
	mul_pos = mul_pos_base + pos_temp;
	if (mul_pos - mul_pos_last > 1.0f) {
		mul_pos = mul_pos_last;
	} else {
		mul_pos_last = mul_pos;
	}
}

float encoder_get_multiturn(void) {
	if (m_encoder_type_now == ENCODER_TYPE_NONE) {
		return 0;
	} else {
		return mul_pos;
	}
}

#define ENCODER_SEND_BACK_FILTER_WINDOW 10
//多圈数据均值滤波
void encoder_send_back_mean_filter (float now_pos) {
	static float window[10];
	static int index;
	filter_sum -= window[index];
	filter_sum += now_pos;
	window[index++] = now_pos;
	if (index >= ENCODER_SEND_BACK_FILTER_WINDOW) {
		index = 0;
	}
}
//回传经过均值滤波之后的多圈
float encoder_get_multiturn_filtered (void) {
	if (m_encoder_type_now == ENCODER_TYPE_NONE) {
		return 0;
	} else {
		return filter_sum / ENCODER_SEND_BACK_FILTER_WINDOW;
	}
}
/**************************************************************************************************/
