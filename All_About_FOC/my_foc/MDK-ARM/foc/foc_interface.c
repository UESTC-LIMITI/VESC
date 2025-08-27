#include "foc_interface.h"
#include "foc_callbacks.h"

volatile uint16_t ADC_Value[ADC_CHANNEL_NUM] = {0};  //存放DMA传输过来的采样值
volatile motor_all_state_t motor = {0};


static void foc_adc_init(void);

void foc_init(void) {
    foc_adc_init();
}

static void foc_adc_init(void) {
    HAL_DMA_RegisterCallback(&hdma_adc1, HAL_DMA_XFER_CPLT_CB_ID, dma_tc_callback);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, ADC_CHANNEL_NUM);
}

static void foc_config_init(motor_all_state_t* motor) {
    motor->m_conf->foc_dt_us = 0.08;
    
}