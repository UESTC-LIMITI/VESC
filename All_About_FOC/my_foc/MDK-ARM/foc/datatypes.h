/*
 * @Author: xiayuan 1137542776@qq.com
 * @Date: 2025-08-27 19:14:11
 * @LastEditors: xiayuan 1137542776@qq.com
 * @LastEditTime: 2025-09-07 00:08:41
 * @FilePath: \VESC_Code\All_About_FOC\my_foc\MDK-ARM\foc\datatypes.h
 * @Description: 
 * 
 * Copyright (c) 2025 by xiayuan, All Rights Reserved. 
 */
#ifndef DATATYPES_H_
#define DATATYPES_H_

#include "spi_bb.h"
#include <stdint.h>
#include "main.h"
#include <math.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>

//#include "foc_interface.h"

typedef struct AS504x_diag_{
	uint8_t is_connected;
	uint8_t AGC_value;
	uint16_t magnitude;
	uint8_t is_OCF;
	uint8_t is_COF;
	uint8_t is_Comp_low;
	uint8_t is_Comp_high;
	uint16_t serial_diag_flgs;
	uint16_t serial_magnitude;
	uint16_t serial_error_flags;
} AS504x_diag;

typedef struct AS504x_state_{
	uint16_t diag_fetch_now_count;
	uint32_t data_last_invalid_counter;
	uint32_t spi_communication_error_count;
	uint8_t spi_data_err_raised;
	AS504x_diag sensor_diag;
	uint16_t spi_val;
	float last_enc_angle;
	uint32_t spi_error_cnt;
	float spi_error_rate;
	uint32_t last_update_time;
} AS504x_state;   ///AS5047相关结构体 包含了各种读取角度

typedef struct AS504x_config{
	spi_bb_state sw_spi;
	AS504x_state state;
} AS504x_config_t;   //AS5047 初始化用的结构体

// Communication commands
typedef enum {
	COMM_FW_VERSION							= 0,
	COMM_JUMP_TO_BOOTLOADER					= 1,
	COMM_ERASE_NEW_APP						= 2,
	COMM_WRITE_NEW_APP_DATA					= 3,
	COMM_GET_VALUES							= 4,
	COMM_SET_DUTY							= 5,
	COMM_SET_CURRENT						= 6,
	COMM_SET_CURRENT_BRAKE					= 7,
	COMM_SET_RPM							= 8,
	COMM_SET_POS							= 9,
	COMM_SET_HANDBRAKE						= 10,
	COMM_SET_DETECT							= 11,
	COMM_SET_SERVO_POS						= 12,
	COMM_SET_MCCONF							= 13,
	COMM_GET_MCCONF							= 14,
	COMM_GET_MCCONF_DEFAULT					= 15,
	COMM_SET_APPCONF						= 16,
	COMM_GET_APPCONF						= 17,
	COMM_GET_APPCONF_DEFAULT				= 18,
	COMM_SAMPLE_PRINT						= 19,
	COMM_TERMINAL_CMD						= 20,
	COMM_PRINT								= 21,
	COMM_ROTOR_POSITION						= 22,
	COMM_EXPERIMENT_SAMPLE					= 23,
	COMM_DETECT_MOTOR_PARAM					= 24,
	COMM_DETECT_MOTOR_R_L					= 25,
	COMM_DETECT_MOTOR_FLUX_LINKAGE			= 26,
	COMM_DETECT_ENCODER						= 27,
	COMM_DETECT_HALL_FOC					= 28,
	COMM_REBOOT								= 29,
	COMM_ALIVE								= 30,
	COMM_GET_DECODED_PPM					= 31,
	COMM_GET_DECODED_ADC					= 32,
	COMM_GET_DECODED_CHUK					= 33,
	COMM_FORWARD_CAN						= 34,
	COMM_SET_CHUCK_DATA						= 35,
	COMM_CUSTOM_APP_DATA					= 36,
	COMM_NRF_START_PAIRING					= 37,
	COMM_GPD_SET_FSW						= 38,
	COMM_GPD_BUFFER_NOTIFY					= 39,
	COMM_GPD_BUFFER_SIZE_LEFT				= 40,
	COMM_GPD_FILL_BUFFER					= 41,
	COMM_GPD_OUTPUT_SAMPLE					= 42,
	COMM_GPD_SET_MODE						= 43,
	COMM_GPD_FILL_BUFFER_INT8				= 44,
	COMM_GPD_FILL_BUFFER_INT16				= 45,
	COMM_GPD_SET_BUFFER_INT_SCALE			= 46,
	COMM_GET_VALUES_SETUP					= 47,
	COMM_SET_MCCONF_TEMP					= 48,
	COMM_SET_MCCONF_TEMP_SETUP				= 49,
	COMM_GET_VALUES_SELECTIVE				= 50,
	COMM_GET_VALUES_SETUP_SELECTIVE			= 51,
	COMM_EXT_NRF_PRESENT					= 52,
	COMM_EXT_NRF_ESB_SET_CH_ADDR			= 53,
	COMM_EXT_NRF_ESB_SEND_DATA				= 54,
	COMM_EXT_NRF_ESB_RX_DATA				= 55,
	COMM_EXT_NRF_SET_ENABLED				= 56,
	COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP	= 57,
	COMM_DETECT_APPLY_ALL_FOC				= 58,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN			= 59,
	COMM_ERASE_NEW_APP_ALL_CAN				= 60,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN			= 61,
	COMM_PING_CAN							= 62,
	COMM_APP_DISABLE_OUTPUT					= 63,
	COMM_TERMINAL_CMD_SYNC					= 64,
	COMM_GET_IMU_DATA						= 65,
	COMM_BM_CONNECT							= 66,
	COMM_BM_ERASE_FLASH_ALL					= 67,
	COMM_BM_WRITE_FLASH						= 68,
	COMM_BM_REBOOT							= 69,
	COMM_BM_DISCONNECT						= 70,
	COMM_BM_MAP_PINS_DEFAULT				= 71,
	COMM_BM_MAP_PINS_NRF5X					= 72,
	COMM_ERASE_BOOTLOADER					= 73,
	COMM_ERASE_BOOTLOADER_ALL_CAN			= 74,
	COMM_PLOT_INIT							= 75,
	COMM_PLOT_DATA							= 76,
	COMM_PLOT_ADD_GRAPH						= 77,
	COMM_PLOT_SET_GRAPH						= 78,
	COMM_GET_DECODED_BALANCE				= 79,
	COMM_BM_MEM_READ						= 80,
	COMM_WRITE_NEW_APP_DATA_LZO				= 81,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO		= 82,
	COMM_BM_WRITE_FLASH_LZO					= 83,
	COMM_SET_CURRENT_REL					= 84,
	COMM_CAN_FWD_FRAME						= 85,
	COMM_SET_BATTERY_CUT					= 86,
	COMM_SET_BLE_NAME						= 87,
	COMM_SET_BLE_PIN						= 88,
	COMM_SET_CAN_MODE						= 89,
	COMM_GET_IMU_CALIBRATION				= 90,
	COMM_GET_MCCONF_TEMP					= 91,

	// Custom configuration for hardware
	COMM_GET_CUSTOM_CONFIG_XML				= 92,
	COMM_GET_CUSTOM_CONFIG					= 93,
	COMM_GET_CUSTOM_CONFIG_DEFAULT			= 94,
	COMM_SET_CUSTOM_CONFIG					= 95,

	// BMS commands
	COMM_BMS_GET_VALUES						= 96,
	COMM_BMS_SET_CHARGE_ALLOWED				= 97,
	COMM_BMS_SET_BALANCE_OVERRIDE			= 98,
	COMM_BMS_RESET_COUNTERS					= 99,
	COMM_BMS_FORCE_BALANCE					= 100,
	COMM_BMS_ZERO_CURRENT_OFFSET			= 101,

	// FW updates commands for different HW types
	COMM_JUMP_TO_BOOTLOADER_HW				= 102,
	COMM_ERASE_NEW_APP_HW					= 103,
	COMM_WRITE_NEW_APP_DATA_HW				= 104,
	COMM_ERASE_BOOTLOADER_HW				= 105,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW		= 106,
	COMM_ERASE_NEW_APP_ALL_CAN_HW			= 107,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW		= 108,
	COMM_ERASE_BOOTLOADER_ALL_CAN_HW		= 109,

	COMM_SET_ODOMETER						= 110,

	// Power switch commands
	COMM_PSW_GET_STATUS						= 111,
	COMM_PSW_SWITCH							= 112,

	COMM_BMS_FWD_CAN_RX						= 113,
	COMM_BMS_HW_DATA						= 114,
	COMM_GET_BATTERY_CUT					= 115,
	COMM_BM_HALT_REQ						= 116,
	COMM_GET_QML_UI_HW						= 117,
	COMM_GET_QML_UI_APP						= 118,
	COMM_CUSTOM_HW_DATA						= 119,
	COMM_QMLUI_ERASE						= 120,
	COMM_QMLUI_WRITE						= 121,

	// IO Board
	COMM_IO_BOARD_GET_ALL					= 122,
	COMM_IO_BOARD_SET_PWM					= 123,
	COMM_IO_BOARD_SET_DIGITAL				= 124,

	COMM_BM_MEM_WRITE						= 125,
	COMM_BMS_BLNC_SELFTEST					= 126,
	COMM_GET_EXT_HUM_TMP					= 127,
	COMM_GET_STATS							= 128,
	COMM_RESET_STATS						= 129,

	// Lisp
	COMM_LISP_READ_CODE						= 130,
	COMM_LISP_WRITE_CODE					= 131,
	COMM_LISP_ERASE_CODE					= 132,
	COMM_LISP_SET_RUNNING					= 133,
	COMM_LISP_GET_STATS						= 134,
	COMM_LISP_PRINT							= 135,

	COMM_BMS_SET_BATT_TYPE					= 136,
	COMM_BMS_GET_BATT_TYPE					= 137,

	COMM_LISP_REPL_CMD						= 138,
	COMM_LISP_STREAM_CODE					= 139,

	COMM_FILE_LIST							= 140,
	COMM_FILE_READ							= 141,
	COMM_FILE_WRITE							= 142,
	COMM_FILE_MKDIR							= 143,
	COMM_FILE_REMOVE						= 144,

	COMM_LOG_START							= 145,
	COMM_LOG_STOP							= 146,
	COMM_LOG_CONFIG_FIELD					= 147,
	COMM_LOG_DATA_F32						= 148,

	COMM_SET_APPCONF_NO_STORE				= 149,
	COMM_GET_GNSS							= 150,

	COMM_LOG_DATA_F64						= 151,

	COMM_LISP_RMSG							= 152,
} COMM_PACKET_ID;

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY						= 0,
	CAN_PACKET_SET_CURRENT					= 1,
	CAN_PACKET_SET_CURRENT_BRAKE			= 2,
	CAN_PACKET_SET_RPM						= 3,
	CAN_PACKET_SET_POS						= 4,
	CAN_PACKET_FILL_RX_BUFFER				= 5,
	CAN_PACKET_FILL_RX_BUFFER_LONG			= 6,
	CAN_PACKET_PROCESS_RX_BUFFER			= 7,
	CAN_PACKET_PROCESS_SHORT_BUFFER			= 8,
	CAN_PACKET_STATUS						= 9,
	CAN_PACKET_SET_CURRENT_REL				= 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL		= 11,
	CAN_PACKET_SET_CURRENT_HANDBRAKE		= 12,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL	= 13,
	CAN_PACKET_STATUS_2						= 14,
	CAN_PACKET_STATUS_3						= 15,
	CAN_PACKET_STATUS_4						= 16,
	CAN_PACKET_PING							= 17,
	CAN_PACKET_PONG							= 18,
	CAN_PACKET_DETECT_APPLY_ALL_FOC			= 19,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES		= 20,
	CAN_PACKET_CONF_CURRENT_LIMITS			= 21,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS	= 22,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN		= 23,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN	= 24,
	CAN_PACKET_CONF_FOC_ERPMS				= 25,
	CAN_PACKET_CONF_STORE_FOC_ERPMS			= 26,
	CAN_PACKET_STATUS_5						= 27,
	CAN_PACKET_POLL_TS5700N8501_STATUS		= 28,
	CAN_PACKET_CONF_BATTERY_CUT				= 29,
	CAN_PACKET_CONF_STORE_BATTERY_CUT		= 30,
	CAN_PACKET_SHUTDOWN						= 31,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4			= 32,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8			= 33,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12			= 34,
	CAN_PACKET_IO_BOARD_DIGITAL_IN			= 35,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL	= 36,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM		= 37,
	CAN_PACKET_BMS_V_TOT					= 38,
	CAN_PACKET_BMS_I						= 39,
	CAN_PACKET_BMS_AH_WH					= 40,
	CAN_PACKET_BMS_V_CELL					= 41,
	CAN_PACKET_BMS_BAL						= 42,
	CAN_PACKET_BMS_TEMPS					= 43,
	CAN_PACKET_BMS_HUM						= 44,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT		= 45,
	CAN_PACKET_PSW_STAT						= 46,
	CAN_PACKET_PSW_SWITCH					= 47,
	CAN_PACKET_BMS_HW_DATA_1				= 48,
	CAN_PACKET_BMS_HW_DATA_2				= 49,
	CAN_PACKET_BMS_HW_DATA_3				= 50,
	CAN_PACKET_BMS_HW_DATA_4				= 51,
	CAN_PACKET_BMS_HW_DATA_5				= 52,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL			= 53,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL			= 54,
	CAN_PACKET_UPDATE_PID_POS_OFFSET		= 55,
	CAN_PACKET_POLL_ROTOR_POS				= 56,
	CAN_PACKET_NOTIFY_BOOT					= 57,
	CAN_PACKET_STATUS_6						= 58,
	CAN_PACKET_GNSS_TIME					= 59,
	CAN_PACKET_GNSS_LAT						= 60,
	CAN_PACKET_GNSS_LON						= 61,
	CAN_PACKET_GNSS_ALT_SPEED_HDOP			= 62,
#if defined(SHOOT_TEST)  //SHOOT_TEST消息解码枚举体
	CAN_PACKET_SET_ACCEL_CURRENT			= 63,
	CAN_PACKET_SET_LIMIT_SPEED				= 64,
	CAN_PACKET_SET_TARGET_SPEED				= 65,
	CAN_PACKET_SET_LIMIT_POS				= 66,
	CAN_PACKET_SET_SAMPLE_POINTS			= 67,
	CAN_PACKET_SET_BRAKE_CURRENT			= 68,
	CAN_PACKET_SET_CUSTOM_MODE				= 69,
	CAN_PACKET_ALIVE						= 70,
	CAN_PACKET_SET_RESET_SPEED				= 71,
	CAN_PACKET_SET_RESET_POS_SAMPLE_POINTS	= 72,
	CAN_PACKET_SET_TARGET_DUTY				= 73,
	CAN_PACKET_SET_HOME			            = 75,
	CAN_PACKET_HOMING			            = 76,
#endif
	CAN_PACKET_SET_POS_MULTITURN			= 74,

	CAN_PACKET_GET_SUBAREA_PARA1			= 77,  //2.18.2024新增 分区PID参数读取和设置
	CAN_PACKET_GET_SUBAREA_PARA2			= 78,
	CAN_PACKET_GET_SUBAREA_PARA3			= 79,
	CAN_PACKET_SET_SUBAREA_PARA1			= 80,
	CAN_PACKET_SET_SUBAREA_PARA2			= 81,
	CAN_PACKET_SET_SUBAREA_PARA3			= 82,
	CAN_PACKET_STORE_MC_CONFIGURATION		= 83,
	CAN_PACKET_ENABLE_SUBAREA_PID	        = 84,
	CAN_PACKET_SELFLOCK	                    = 85,
	CAN_PACKET_SELFLOCK_RELEASE	            = 86,
	CAN_PACKET_RELEASE_MOTER                = 94,
	CAN_PACKET_SET_ZERO_POS                = 95,
//	CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;

typedef struct {
	int id;
	// systime_t rx_time;
	float rpm;
	float current;
	float duty;
} can_status_msg;

typedef enum {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
} mc_state;

typedef enum {
	SENSOR_MODE_SENSORLESS = 0,
	SENSOR_MODE_SENSORED,
	SENSOR_MODE_HYBRID
} mc_sensor_mode;

// FOC current controller decoupling mode.
typedef enum {
	FOC_CC_DECOUPLING_DISABLED = 0,
	FOC_CC_DECOUPLING_CROSS,
	FOC_CC_DECOUPLING_BEMF,
	FOC_CC_DECOUPLING_CROSS_BEMF
} mc_foc_cc_decoupling_mode;

typedef enum {
	FOC_SENSOR_MODE_SENSORLESS = 0,
	FOC_SENSOR_MODE_ENCODER,
	FOC_SENSOR_MODE_HALL,
	FOC_SENSOR_MODE_HFI,
	FOC_SENSOR_MODE_HFI_START,
	FOC_SENSOR_MODE_HFI_V2,
	FOC_SENSOR_MODE_HFI_V3,
	FOC_SENSOR_MODE_HFI_V4,
	FOC_SENSOR_MODE_HFI_V5
} mc_foc_sensor_mode;

typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_HIGH_CURRENT_TIMEOUT, //自定义高电流超时的错误
	FAULT_CODE_DRV,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR,
	FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
	FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
	FAULT_CODE_MCU_UNDER_VOLTAGE,
	FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
	FAULT_CODE_ENCODER_SPI,
	FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
	FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
	FAULT_CODE_FLASH_CORRUPTION,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
	FAULT_CODE_UNBALANCED_CURRENTS,
	FAULT_CODE_BRK,
	FAULT_CODE_RESOLVER_LOT,
	FAULT_CODE_RESOLVER_DOS,
	FAULT_CODE_RESOLVER_LOS,
	FAULT_CODE_FLASH_CORRUPTION_APP_CFG,
	FAULT_CODE_FLASH_CORRUPTION_MC_CFG,
	FAULT_CODE_ENCODER_NO_MAGNET,
	FAULT_CODE_ENCODER_MAGNET_TOO_STRONG,
	FAULT_CODE_PHASE_FILTER,
	FAULT_CODE_ENCODER_FAULT,
	FAULT_CODE_LV_OUTPUT_FAULT,
	FAULT_CODE_UNDER_VOLTAGE,
} mc_fault_code;

typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE,
	CONTROL_MODE_POS,
	CONTROL_MODE_HANDBRAKE,
	CONTROL_MODE_OPENLOOP,
	CONTROL_MODE_OPENLOOP_PHASE,
	CONTROL_MODE_OPENLOOP_DUTY,
	CONTROL_MODE_OPENLOOP_DUTY_PHASE,
	CONTROL_MODE_NONE,
    // CONTROL_MODE_POS_MULTITURN,  //2.15.2024新增多圈位置控制
    // CONTROL_MODE_SELFLOCK,       //3.1.2024新增舵轮自锁
	} mc_control_mode;

typedef enum {
	SENSOR_PORT_MODE_HALL = 0,
	SENSOR_PORT_MODE_ABI,
	SENSOR_PORT_MODE_AS5047_SPI,
	SENSOR_PORT_MODE_AD2S1205,
	SENSOR_PORT_MODE_SINCOS,
	SENSOR_PORT_MODE_TS5700N8501,
	SENSOR_PORT_MODE_TS5700N8501_MULTITURN,
	SENSOR_PORT_MODE_MT6816_SPI_HW,
	SENSOR_PORT_MODE_AS5x47U_SPI,
	SENSOR_PORT_MODE_BISSC,
	SENSOR_PORT_MODE_TLE5012_SSC_SW,
	SENSOR_PORT_MODE_TLE5012_SSC_HW,
	SENSOR_PORT_MODE_CUSTOM_ENCODER,
} sensor_port_mode;

typedef struct {
	// Limits
	float l_current_max;
	float l_current_min;
	float l_in_current_max;
	float l_in_current_min;
	float l_in_current_map_start;
	float l_in_current_map_filter;
	float l_abs_current_max;
	float l_min_erpm;
	float l_max_erpm;
	float l_erpm_start;
	float l_max_erpm_fbrake;
	float l_max_erpm_fbrake_cc;
	float l_min_vin;
	float l_max_vin;
	float l_battery_cut_start;
	float l_battery_cut_end;
	float l_battery_regen_cut_start;
	float l_battery_regen_cut_end;
	bool l_slow_abs_current;
	float l_temp_fet_start;
	float l_temp_fet_end;
	float l_temp_motor_start;
	float l_temp_motor_end;
	float l_temp_accel_dec;
	float l_min_duty;
	float l_max_duty;
	float l_watt_max;
	float l_watt_min;
	float l_current_max_scale;
	float l_current_min_scale;
	float l_duty_start;
	// Overridden limits (Computed during runtime)
	float lo_current_max;
	float lo_current_min;
	float lo_in_current_max;
	float lo_in_current_min;

	// Sensorless (bldc)
	float sl_min_erpm;
	float sl_min_erpm_cycle_int_limit;
	float sl_max_fullbreak_current_dir_change;
	float sl_cycle_int_limit;
	float sl_phase_advance_at_br;
	float sl_cycle_int_rpm_br;
	float sl_bemf_coupling_k;
	// Hall sensor
	int8_t hall_table[8];
	float hall_sl_erpm;

	// FOC
	float foc_current_kp;
	float foc_current_ki;
	float foc_f_zv;
	float foc_dt_us;
	float foc_encoder_offset;
	bool foc_encoder_inverted;
	float foc_encoder_ratio;
	float foc_motor_l;
	float foc_motor_ld_lq_diff;  //ld-lq diff 整定可得 
	float foc_motor_r;
	float foc_motor_flux_linkage;    //磁通量
	float foc_observer_gain;
	float foc_observer_gain_slow;
	float foc_observer_offset;
	float foc_pll_kp;
	float foc_pll_ki;
	float foc_duty_dowmramp_kp;
	float foc_duty_dowmramp_ki;
	float foc_start_curr_dec;
	float foc_start_curr_dec_rpm;
	float foc_openloop_rpm;
	float foc_openloop_rpm_low;
	float foc_d_gain_scale_start;
	float foc_d_gain_scale_max_mod;
	float foc_sl_openloop_hyst;
	float foc_sl_openloop_time;
	float foc_sl_openloop_time_lock;
	float foc_sl_openloop_time_ramp;
	float foc_sl_openloop_boost_q;
	float foc_sl_openloop_max_q;
	mc_foc_sensor_mode foc_sensor_mode;
	uint8_t foc_hall_table[8];
	float foc_hall_interp_erpm;
	float foc_sl_erpm_start;
	float foc_sl_erpm;
	// mc_foc_control_sample_mode foc_control_sample_mode;
	// mc_foc_current_sample_mode foc_current_sample_mode;
	// SAT_COMP_MODE foc_sat_comp_mode;  // 饱和补偿 观测器要用到   
	float foc_sat_comp;
	bool foc_temp_comp;
	float foc_temp_comp_base_temp;
	float foc_current_filter_const;
	mc_foc_cc_decoupling_mode foc_cc_decoupling;  // 解耦方式, 暂时先用一种吧
	// mc_foc_observer_type foc_observer_type;  // 观测器模式, 暂时不上观测器
	float foc_hfi_voltage_start;
	float foc_hfi_voltage_run;
	float foc_hfi_voltage_max;
	float foc_hfi_gain;
	float foc_hfi_hyst;
	float foc_sl_erpm_hfi;
	uint16_t foc_hfi_start_samples;
	float foc_hfi_obs_ovr_sec;
	// foc_hfi_samples foc_hfi_samples; // HFI采样方式, 暂时先不用
	bool foc_offsets_cal_on_boot;
	float foc_offsets_current[3];
	float foc_offsets_voltage[3];
	float foc_offsets_voltage_undriven[3];
	bool foc_phase_filter_enable;
	bool foc_phase_filter_disable_fault;
	float foc_phase_filter_max_erpm;
	// MTPA_MODE foc_mtpa_mode;
	// Field Weakening
	float foc_fw_current_max;
	float foc_fw_duty_start;
	float foc_fw_ramp_time;
	float foc_fw_q_current_factor;
	// FOC_SPEED_SRC foc_speed_soure;

	// 电流环pid参数
	float c_pid_kp;
	float c_pid_ki;
	float c_pid_kd;

	// Speed PID
	float s_pid_kp;
	float s_pid_ki;
	float s_pid_kd;
	float s_pid_kd_filter;
	float s_pid_min_erpm;
	bool s_pid_allow_braking;
	float s_pid_ramp_erpms_s;
	// S_PID_SPEED_SRC s_pid_speed_source;

	// Pos PID
	float p_pid_kp;
	float p_pid_ki;
	float p_pid_kd;
	float p_pid_kd_proc;
	float p_pid_kd_filter;
	float p_pid_ang_div;
	float p_pid_gain_dec_angle;
	float p_pid_offset;

	float l_pid_out_max;
	float l_c_i_term_max;
	float l_s_i_term_max;
	float l_p_i_term_max;

	// Misc
	int32_t m_fault_stop_time_ms;
	float m_duty_ramp_step;
	float m_current_backoff_gain;
	uint32_t m_encoder_counts;
	float m_encoder_sin_offset;
	float m_encoder_sin_amp;
	float m_encoder_cos_offset;
	float m_encoder_cos_amp;
	float m_encoder_sincos_filter_constant;
	float m_encoder_sincos_phase_correction;
	sensor_port_mode m_sensor_port_mode;

	// Protect from flash corruption.
	uint16_t crc;

} mc_configuration;

typedef struct {
	float va;
	float vb;
	float vc;
	float v_mag_filter;
	float mod_alpha_filter;
	float mod_beta_filter;
	float mod_alpha_measured;
	float mod_beta_measured;
	float mod_alpha_raw;
	float mod_beta_raw;
	float id_target;
	float iq_target;
	float max_duty;
	float duty_now;
	float theta;  // 机械角度
	float phase;  // 电角度
	float phase_cos;
	float phase_sin;
	float i_alpha;
	float i_beta;
	float i_abs;
	float i_abs_filter;
	float i_bus;
	float v_bus;
	float v_alpha;
	float v_beta;
	float mod_d;
	float mod_q;
	float mod_q_filter;
	float id;
	float iq;
	float id_filter;
	float iq_filter;
	float vd;
	float vq;
	float vd_int;
	float vq_int;
	uint32_t svm_sector;
	bool is_using_phase_filters;
} motor_state_t;

typedef struct {             //电机的所有参数
	mc_configuration *m_conf;
	mc_state m_state;
	mc_control_mode m_control_mode;
	motor_state_t m_motor_state;
	float m_curr_unbalance;
	float m_currents_adc[3];
	bool m_phase_override;
	float m_phase_now_override;
	float m_duty_cycle_set;
	float m_id_set;
	float m_iq_set;
	float m_i_fw_set;
	float m_current_off_delay;
	float m_openloop_speed;
	float m_openloop_phase;
	bool m_output_on;
	float m_pos_pid_set;
	float m_speed_pid_set_rpm;
	float m_speed_command_rpm;
	float m_phase_now_observer;
	float m_phase_now_observer_override;
	float m_observer_x1_override;
	float m_observer_x2_override;
	bool m_phase_observer_override;
	float m_phase_now_encoder;
	float m_phase_now_encoder_no_index;
	// observer_state m_observer_state;
	float m_pll_phase;
	float m_pll_speed;
	float m_speed_est_fast;
	float m_speed_est_fast_corrected; // Same as m_speed_est_fast, but always based on the corrected position
	float m_speed_est_faster;
	// mc_sample_t m_samples;
	int m_tachometer;
	int m_tachometer_abs;
	float m_pos_pid_now;  //当前电机单圈位置
	float m_gamma_now;
	//使用编码器
	bool m_using_encoder;
	int m_duty1_next, m_duty2_next, m_duty3_next;
	bool m_duty_next_set;
	float m_i_alpha_sample_next;
	float m_i_beta_sample_next;
	float m_i_alpha_sample_with_offset;
	float m_i_beta_sample_with_offset;
	float m_i_alpha_beta_has_offset;
	// hfi_state_t m_hfi;
	// int m_hfi_plot_en;
	// float m_hfi_plot_sample;

	// For braking
	float m_br_speed_before;
	float m_br_vq_before;
	int m_br_no_duty_samples;

	float m_duty_abs_filtered;
	float m_duty_filtered;
	bool m_was_control_duty;
	//PI控制的占空比
	float m_duty_i_term;
	bool duty_was_pi;
	float duty_pi_duty_last;
	float m_openloop_angle;
	float m_x1_prev;
	float m_x2_prev;
	float m_phase_before_speed_est;
	float m_phase_before_speed_est_corrected;
	int m_tacho_step_last;
	float m_pid_div_angle_last;
	float m_pid_div_angle_accumulator;
	float m_min_rpm_hyst_timer;
	float m_min_rpm_timer;
	//HFI参数
	bool m_cc_was_hfi;
	float m_pos_i_term;
	float m_pos_prev_error;
	float m_pos_dt_int;
	float m_pos_prev_proc;
	float m_pos_dt_int_proc;
	float m_pos_d_filter;
	float m_pos_d_filter_proc;
	float m_speed_i_term;
	float m_speed_prev_error;
	float m_speed_d_filter;
	float m_current_i_term;
	float m_current_prev_error;
	float m_current_d_filter;
	int m_ang_hall_int_prev;
	//HALL 参数
	bool m_using_hall;
	float m_ang_hall;
	float m_ang_hall_rate_limited;
	float m_hall_dt_diff_last;
	float m_hall_dt_diff_now;
	bool m_motor_released;

	// Resistance observer
	float m_res_est;
	float m_r_est_state;

	// Temperature-compensated parameters
	float m_res_temp_comp;
	float m_current_ki_temp_comp;

	// Pre-calculated values
	float p_lq;
	float p_ld;
	float p_inv_ld_lq; // (1.0/lq - 1.0/ld)
	float p_v2_v3_inv_avg_half; // (0.5/ld + 0.5/lq)
} motor_all_state_t;  
// 在FOC计算里用到的电机所有参数
// 包含interface.h里所有的电机参数

extern volatile motor_all_state_t motor;

#endif // DATATYPES_H_
