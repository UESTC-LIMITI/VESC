#ifndef FOC_MCCONFIG_H_
#define FOC_MCCONFIG_H_

#include "datatypes.h"

/* -------------------------采样部分-------------------------- */
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					39000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
// 采样电阻阻值
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0005
#endif

#ifndef ADC_CHANNEL_NUM
#define ADC_CHANNEL_NUM		    8
#endif

// ADC通道索引
#define ADC_Channel_17          ((uint8_t)0x11)  // VREFINT_Channel

#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_VIN_SENS		6
// #define ADC_IND_EXT				6
// #define ADC_IND_EXT2			7
// #define ADC_IND_TEMP_MOS		8
// #define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			7

#define GET_ADC_VALUE(channel)  (ADC_Value[channel])	
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// Current ADC to amperes factor
#define FAC_CURRENT				((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))
// CURRENT_SHUNT_RES 和采样电阻有关
// Current ADC to amperes factor for the second motor
// CURRENT_AMP_GAIN和采样芯片配置有关

#define GET_A_CURRENT()		    (float)ADC_Value[ADC_IND_CURR1]
#define GET_B_CURRENT()			(float)ADC_Value[ADC_IND_CURR2]
#define GET_C_CURRENT()			(float)ADC_Value[ADC_IND_CURR3]

/* -------------------------各项默认参数-------------------------- */
// Limits
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX			60.0	// Current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN			-60.0	// Current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			99.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-60.0	// Input current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAP_START
#define MCCONF_L_IN_CURRENT_MAP_START	1.0		// Input current to Q axis current limit map start
#endif
#ifndef MCCONF_L_IN_CURRENT_MAP_FILTER
#define MCCONF_L_IN_CURRENT_MAP_FILTER	0.005	// Input current filter for the mapped limit
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		130.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			8.0		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			57.0	// Maximum input voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_START
#define MCCONF_L_BATTERY_CUT_START		10.0	// Start limiting the positive current at this voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_END
#define MCCONF_L_BATTERY_CUT_END		8.0		// Limit the positive current completely at this voltage
#endif
#ifndef MCCONF_L_BATTERY_REGEN_CUT_START
#define MCCONF_L_BATTERY_REGEN_CUT_START    1000.0	// Start limiting the regen current at this voltage
#endif
#ifndef MCCONF_L_BATTERY_REGEN_CUT_END
#define MCCONF_L_BATTERY_REGEN_CUT_END  1100.0		// Limit the regen current completely at this voltage
#endif
#ifndef MCCONF_L_RPM_MAX
#define MCCONF_L_RPM_MAX				100000.0	// The motor speed limit (Upper)
#endif
#ifndef MCCONF_L_RPM_MIN
#define MCCONF_L_RPM_MIN				-100000.0	// The motor speed limit (Lower)
#endif
#ifndef MCCONF_L_RPM_START
#define MCCONF_L_RPM_START				0.8		// Fraction of full speed where RPM current limiting starts
#endif
#ifndef MCCONF_L_SLOW_ABS_OVERCURRENT
#define MCCONF_L_SLOW_ABS_OVERCURRENT	false	// Use the filtered (and hence slower) current for the overcurrent fault detection
#endif
#ifndef MCCONF_L_MIN_DUTY
#define MCCONF_L_MIN_DUTY				0.005	// Minimum duty cycle
#endif
#ifndef MCCONF_L_MAX_DUTY
#define MCCONF_L_MAX_DUTY				0.95	// Maximum duty cycle
#endif
#ifndef MCCONF_L_CURR_MAX_RPM_FBRAKE
#define MCCONF_L_CURR_MAX_RPM_FBRAKE	300		// Maximum electrical RPM to use full brake at
#endif
#ifndef MCCONF_L_CURR_MAX_RPM_FBRAKE_CC
#define MCCONF_L_CURR_MAX_RPM_FBRAKE_CC	1500	// Maximum electrical RPM to use full brake at with current control
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_START
#define MCCONF_L_LIM_TEMP_FET_START		85.0	// MOSFET temperature where current limiting should begin
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_END
#define MCCONF_L_LIM_TEMP_FET_END		100.0	// MOSFET temperature where everything should be shut off
#endif
#ifndef MCCONF_L_LIM_TEMP_MOTOR_START
#define MCCONF_L_LIM_TEMP_MOTOR_START	85.0	// MOTOR temperature where current limiting should begin
#endif
#ifndef MCCONF_L_LIM_TEMP_MOTOR_END
#define MCCONF_L_LIM_TEMP_MOTOR_END		100.0	// MOTOR temperature where everything should be shut off
#endif
#ifndef MCCONF_L_LIM_TEMP_ACCEL_DEC
#define MCCONF_L_LIM_TEMP_ACCEL_DEC		0.15	// Decrease temperature limits this much during acceleration
#endif
#ifndef MCCONF_L_WATT_MAX
#define MCCONF_L_WATT_MAX				1500000.0	// Maximum wattage output
#endif
#ifndef MCCONF_L_WATT_MIN
#define MCCONF_L_WATT_MIN				-1500000.0	// Minimum wattage output (braking)
#endif
#ifndef MCCONF_L_CURRENT_MAX_SCALE
#define MCCONF_L_CURRENT_MAX_SCALE		1.0	// Maximum current scale
#endif
#ifndef MCCONF_L_CURRENT_MIN_SCALE
#define MCCONF_L_CURRENT_MIN_SCALE		1.0	// Minimum current scale
#endif
#ifndef MCCONF_L_DUTY_START
#define MCCONF_L_DUTY_START				1.0 // Start limiting current at this duty cycle
#endif

// Speed PID parameters
#ifndef MCCONF_S_PID_KP
#define MCCONF_S_PID_KP					0.004	// Proportional gain
#endif
#ifndef MCCONF_S_PID_KI
#define MCCONF_S_PID_KI					0.004	// Integral gain
#endif
#ifndef MCCONF_S_PID_KD
#define MCCONF_S_PID_KD					0.0001	// Derivative gain
#endif
#ifndef MCCONF_S_PID_KD_FILTER
#define MCCONF_S_PID_KD_FILTER			0.2	// Derivative filter
#endif
#ifndef MCCONF_S_PID_MIN_RPM
#define MCCONF_S_PID_MIN_RPM			900.0	// Minimum allowed RPM
#endif
#ifndef MCCONF_S_PID_ALLOW_BRAKING
#define MCCONF_S_PID_ALLOW_BRAKING		true	// Allow braking in speed control mode
#endif
#ifndef MCCONF_S_PID_RAMP_ERPMS_S
#define MCCONF_S_PID_RAMP_ERPMS_S		25000.0	// Speed input ramping, in ERPM/s
#endif
// #ifndef MCCONF_S_PID_SPEED_SOURCE
// #define MCCONF_S_PID_SPEED_SOURCE		S_PID_SPEED_SRC_PLL
// #endif

// Position PID parameters
#ifndef MCCONF_P_PID_KP
#define MCCONF_P_PID_KP					0.025	// Proportional gain
#endif
#ifndef MCCONF_P_PID_KI
#define MCCONF_P_PID_KI					0.0		// Integral gain
#endif
#ifndef MCCONF_P_PID_KD
#define MCCONF_P_PID_KD					0.00000	// Derivative gain
#endif
#ifndef MCCONF_P_PID_KD_PROC
#define MCCONF_P_PID_KD_PROC			0.00035	// Derivative gain process
#endif
#ifndef MCCONF_P_PID_KD_FILTER
#define MCCONF_P_PID_KD_FILTER			0.2		// Derivative filter
#endif
#ifndef MCCONF_P_PID_ANG_DIV
#define MCCONF_P_PID_ANG_DIV			1.0		// Divide angle by this value
#endif
#ifndef MCCONF_P_PID_GAIN_DEC_ANGLE
#define MCCONF_P_PID_GAIN_DEC_ANGLE		0.0		// Decrease PID-gains when the error is below this value
#endif
#ifndef MCCONF_P_PID_OFFSET
#define MCCONF_P_PID_OFFSET				0.0		// Angle offset
#endif

// FOC
#ifndef MCCONF_FOC_CURRENT_KP
#define MCCONF_FOC_CURRENT_KP			0.03
#endif
#ifndef MCCONF_FOC_CURRENT_KI
#define MCCONF_FOC_CURRENT_KI			50.0
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					25000.0
#endif
#ifndef MCCONF_FOC_DT_US
#define MCCONF_FOC_DT_US				0.12 // Microseconds for dead time compensation
#endif
#ifndef MCCONF_FOC_ENCODER_INVERTED
#define MCCONF_FOC_ENCODER_INVERTED		false
#endif
#ifndef MCCONF_FOC_ENCODER_OFFSET
#define MCCONF_FOC_ENCODER_OFFSET		180.0
#endif
#ifndef MCCONF_FOC_ENCODER_RATIO
#define MCCONF_FOC_ENCODER_RATIO		7.0
#endif
#ifndef MCCONF_FOC_SENSOR_MODE
#define MCCONF_FOC_SENSOR_MODE			FOC_SENSOR_MODE_SENSORLESS
#endif
#ifndef MCCONF_FOC_PLL_KP
#define MCCONF_FOC_PLL_KP				2000.0
#endif
#ifndef MCCONF_FOC_PLL_KI
#define MCCONF_FOC_PLL_KI				30000.0
#endif
#ifndef MCCONF_FOC_MOTOR_L
#define MCCONF_FOC_MOTOR_L				0.000007
#endif
#ifndef MCCONF_FOC_MOTOR_R
#define MCCONF_FOC_MOTOR_R				0.015
#endif
#ifndef MCCONF_FOC_MOTOR_FLUX_LINKAGE
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE	0.00245
#endif
#ifndef MCCONF_FOC_MOTOR_LD_LQ_DIFF
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF		0.0
#endif
#ifndef MCCONF_FOC_OBSERVER_GAIN
#define MCCONF_FOC_OBSERVER_GAIN		9e7		// Can be something like 600 / L
#endif
#ifndef MCCONF_FOC_OBSERVER_GAIN_SLOW
#define MCCONF_FOC_OBSERVER_GAIN_SLOW	0.05	// Observer gain scale at minimum duty cycle
#endif
#ifndef MCCONF_FOC_OBSERVER_OFFSET
#define MCCONF_FOC_OBSERVER_OFFSET		-1.0	// Observer offset in timer update cycles
#endif
#ifndef MCCONF_FOC_DUTY_DOWNRAMP_KP
#define MCCONF_FOC_DUTY_DOWNRAMP_KP		50.0	// PI controller for duty control when decreasing the duty
#endif
#ifndef MCCONF_FOC_DUTY_DOWNRAMP_KI
#define MCCONF_FOC_DUTY_DOWNRAMP_KI		1000.0	// PI controller for duty control when decreasing the duty
#endif
#ifndef MCCONF_FOC_START_CURR_DEC
#define MCCONF_FOC_START_CURR_DEC		1.0	// Decrease current to this fraction at start
#endif
#ifndef MCCONF_FOC_START_CURR_DEC_RPM
#define MCCONF_FOC_START_CURR_DEC_RPM	2500.0	// At this RPM the full current is available
#endif
#ifndef MCCONF_FOC_OPENLOOP_RPM
#define MCCONF_FOC_OPENLOOP_RPM			1500.0	// Openloop RPM (sensorless low speed or when finding index pulse)
#endif
#ifndef MCCONF_FOC_OPENLOOP_RPM_LOW
#define MCCONF_FOC_OPENLOOP_RPM_LOW		0.0		// Fraction of OPENLOOP_RPM at minimum motor current
#endif
#ifndef MCCONF_FOC_D_GAIN_SCALE_START
#define MCCONF_FOC_D_GAIN_SCALE_START	0.9		// Start reducing D axis current controller gain at this modulation
#endif
#ifndef MCCONF_FOC_D_GAIN_SCALE_MAX_MOD
#define MCCONF_FOC_D_GAIN_SCALE_MAX_MOD	0.2		// D axis currnet controller gain at maximum modulation
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_HYST
#define MCCONF_FOC_SL_OPENLOOP_HYST		0.1		// Time below min RPM to activate openloop (s)
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_TIME
#define MCCONF_FOC_SL_OPENLOOP_TIME		0.05	// Time to remain in openloop after ramping (s)
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_BOOST_Q
#define MCCONF_FOC_SL_OPENLOOP_BOOST_Q	0.0		// Q-axis current boost during the open loop procedure
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_MAX_Q
#define MCCONF_FOC_SL_OPENLOOP_MAX_Q	-1.0		// Q-axis maximum current during the open loop procedure
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_T_LOCK
#define MCCONF_FOC_SL_OPENLOOP_T_LOCK	0.0		// Time to lock motor in beginning of open loop sequence
#endif
#ifndef MCCONF_FOC_SL_OPENLOOP_T_RAMP
#define MCCONF_FOC_SL_OPENLOOP_T_RAMP	0.1		// Time to ramp up motor to openloop speed
#endif
#ifndef MCCONF_FOC_SL_ERPM_START
#define MCCONF_FOC_SL_ERPM_START		2500.0	// ERPM below which only sensored commutation is used
#endif
#ifndef MCCONF_FOC_SL_ERPM
#define MCCONF_FOC_SL_ERPM				3500.0	// ERPM above which only the observer is used
#endif
// 这三个2048很重要, 因为运放的模式决定了0电流时, 运放输出的是: Vref / 2!!
// 所以一定要将偏置设定为2048, 正电流是大于2048, 负电流时小于2048, 这样才对!!!
#ifndef MCCONF_FOC_OFFSETS_CURRENT_0
#define MCCONF_FOC_OFFSETS_CURRENT_0	2048.0 // Current 0 offset
#endif
#ifndef MCCONF_FOC_OFFSETS_CURRENT_1
#define MCCONF_FOC_OFFSETS_CURRENT_1	2048.0 // Current 1 offset
#endif
#ifndef MCCONF_FOC_OFFSETS_CURRENT_2
#define MCCONF_FOC_OFFSETS_CURRENT_2	2048.0 // Current 2 offset
#endif
#ifndef MCCONF_FOC_OFFSETS_VOLTAGE_0
#define MCCONF_FOC_OFFSETS_VOLTAGE_0	0.0 // Voltage 0 offset
#endif
#ifndef MCCONF_FOC_OFFSETS_VOLTAGE_1
#define MCCONF_FOC_OFFSETS_VOLTAGE_1	0.0 // Voltage 1 offset
#endif
#ifndef MCCONF_FOC_OFFSETS_VOLTAGE_2
#define MCCONF_FOC_OFFSETS_VOLTAGE_2	0.0 // Voltage 2 offset
#endif

#endif // FOC_MCCONFIG_H_
