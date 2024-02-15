/*
	Copyright 2012-2022 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include "drv8301.h"
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"

// Variables
static volatile bool i2c_running = false;

/* 
#if defined(HW_IS_LIMITI_MK1)
static mutex_t shutdown_mutex;
static float bt_diff = 0.0;
#endif
*/

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

/* 
#if defined(HW_IS_LIMITI_MK1)
static void terminal_shutdown_now(int argc, const char **argv);
static void terminal_button_test(int argc, const char **argv);
#endif
*/

void hw_init_gpio(void) {
/* 
#if !defined(HW_IS_LIMITI_MK1)  
	chMtxObjectInit(&shutdown_mutex);
#endif
*/
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(GPIOB, 0,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, 1,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// ENABLE_GATE
	palSetPadMode(GPIOB, 5,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	ENABLE_GATE();

	// Current filter
	palSetPadMode(GPIOD, 2,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	CURRENT_FILTER_OFF();

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);
#if defined(USE_CUSTOM_ENCODER1)
	palSetPadMode(HW_ABI_ENC_GPIO1, HW_ABI_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_ABI_ENC_GPIO2, HW_ABI_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_ABI_ENC_GPIO3, HW_ABI_ENC_PIN3, PAL_MODE_INPUT_PULLUP);
#endif

	// Phase filters
#ifdef PHASE_FILTER_GPIO
	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();
#endif

// Sensor port voltage
/* 
#if defined(HW60_IS_MK6)
	SENSOR_PORT_3V3();
	palSetPadMode(SENSOR_VOLTAGE_GPIO, SENSOR_VOLTAGE_PIN,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
#endif
*/

	// Fault pin
#if defined(HW_HAS_DRV8301)	// Fault pin
	palSetPadMode(GPIOB, 7, PAL_MODE_INPUT_PULLUP);   //DRV nFault脚
#endif

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);   //SH_A
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);   //SH_B
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);   //SH_C
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);   //总线电压 3v3
//#if !defined(USE_CUSTOM_ENCODER1)
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);   //ADC_EXT1
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);   //ADC_EXT2 端口复用 
//#endif

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);   //A_CURR
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);   //B_CURR
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);   //C_CURR
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);   //总线电压 24V
//#if !defined(USE_CUSTOM_ENCODER1)
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);   //电机温控
//#endif
#if defined(HW_HAS_DRV8301)
	drv8301_init();
#endif
/* 
#if !defined(HW_IS_LIMITI_MK1)
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);   //shutdown口 没用
#endif
*/

/* 
#if defined(HW_HAS_DRV8301)
	terminal_register_command_callback(
		"shutdown",
		"Shutdown VESC now.",
		0,
		terminal_shutdown_now);

	terminal_register_command_callback(
		"test_button",
		"Try sampling the shutdown button",
		0,
		terminal_button_test);
#endif
*/
}



void hw_setup_adc_channels(void) {
	uint8_t t_samp = ADC_SampleTime_15Cycles;

	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);     //A_CURR
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, t_samp);      //SH_A  
//#if !defined(USE_CUSTOM_ENCODER1)  //端口被复用
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, t_samp);      //ADC_EXT1
//#endif
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, t_samp);     //MOTOR_TEMP
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, t_samp);//测量内部参考电压

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);     //B_CURR
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, t_samp);      //SH_B
//#if !defined(USE_CUSTOM_ENCODER1)   //端口被复用
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, t_samp);      //ADC_EXT2
//#endif
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, t_samp);     //SHUTDOWN(ADC15)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, t_samp);      //你跑去测A_CURR干什么？

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);     //C_CURR
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, t_samp);      //SH_C
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, t_samp);      //V_REF?存疑
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, t_samp);     //V_REF?存疑
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, t_samp);      //你跑去测B_CURR干什么？    //TODO:了解一下多通道ADC采样

	// Current oversampling
//	for (int i = 6;i <= 15;i++) {
//		ADC_RegularChannelConfig(ADC1, ADC_Channel_10, i, ADC_SampleTime_56Cycles);
//		ADC_RegularChannelConfig(ADC2, ADC_Channel_11, i, ADC_SampleTime_56Cycles);
//		ADC_RegularChannelConfig(ADC3, ADC_Channel_12, i, ADC_SampleTime_56Cycles);
//	}

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, t_samp);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, t_samp);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, t_samp);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, t_samp);   //重要的三相电流采样
}
/*
数据在数组中顺序：
规则通道数据：

ADC1 - Channel 10
ADC2 - Channel 11
ADC3 - Channel 12
ADC1 - Channel 0
ADC2 - Channel 1
ADC3 - Channel 2
ADC1 - Channel 5
ADC2 - Channel 6
ADC3 - Channel 3
ADC1 - Channel 14
ADC2 - Channel 15
ADC3 - Channel 13
ADC1 - Channel Vrefint
ADC2 - Channel 0
ADC3 - Channel 1
注入通道数据：

ADC1 - Channel 10 (Injected 1)
ADC2 - Channel 11 (Injected 1)
ADC3 - Channel 12 (Injected 1)
ADC1 - Channel 10 (Injected 2)
ADC2 - Channel 11 (Injected 2)
ADC3 - Channel 12 (Injected 2)
ADC1 - Channel 10 (Injected 3)
ADC2 - Channel 11 (Injected 3)
ADC3 - Channel 12 (Injected 3)
这意味着在数组中，首先是规则通道1的数据，接着是注入通道1的数据，然后是规则通道2的数据，注入通道2的数据，规则通道3的数据，注入通道3的数据，以此类推。数据的存储顺序是根据通道配置的顺序交替进行的。
*/ 



void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

/*
#if defined(HW_IS_LIMITI_MK1)
bool hw_sample_shutdown_button(void) {
	chMtxLock(&shutdown_mutex);

	bt_diff = 0.0;

	for (int i = 0;i < 3;i++) {
		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_INPUT_ANALOG);
		chThdSleep(5);
		float val1 = ADC_VOLTS(ADC_IND_SHUTDOWN);
		chThdSleepMilliseconds(1);
		float val2 = ADC_VOLTS(ADC_IND_SHUTDOWN);
		palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
		chThdSleepMilliseconds(1);

		bt_diff += (val1 - val2);
	}

	chMtxUnlock(&shutdown_mutex);

	return (bt_diff > 0.12);
}

static void terminal_shutdown_now(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	DISABLE_GATE();
	HW_SHUTDOWN_HOLD_OFF();
}

static void terminal_button_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < 40;i++) {
		commands_printf("BT: %d %.2f", HW_SAMPLE_SHUTDOWN(), (double)bt_diff);
		chThdSleepMilliseconds(100);
	}
}
#endif
*/