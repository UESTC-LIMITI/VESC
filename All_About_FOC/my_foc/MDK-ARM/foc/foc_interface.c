#include "foc_interface.h"
#include "foc_callbacks.h"
#include "utils_math.h"
#include "datatypes.h"
#include "foc_mcconfig.h"
#include "foc_encoders.h"
#include "tim.h"

volatile uint16_t ADC_Value[ADC_CHANNEL_NUM] = {0};  //存放DMA传输过来的采样值
volatile motor_all_state_t motor;
mc_configuration motor_config = {0};

extern AS504x_config_t as5047_cfg;


static void foc_adc_init(void);
static void foc_config_init(volatile motor_all_state_t* m);
static void foc_encoders_init (volatile motor_all_state_t* m);

void foc_init(void) {
    foc_adc_init();
    foc_config_init(&motor);
    foc_encoders_init(&motor);
}

static void foc_adc_init(void) {
    HAL_DMA_RegisterCallback(&hdma_adc1, HAL_DMA_XFER_CPLT_CB_ID, dma_tc_callback);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, ADC_CHANNEL_NUM);
}

static void foc_config_init(volatile motor_all_state_t* m) {
    m->m_conf = &motor_config;
    volatile motor_state_t *motor_state = &m->m_motor_state;
    volatile mc_configuration *conf = m->m_conf;
    conf->foc_dt_us = 0.08;
    conf->foc_f_zv = 25000; 

    conf->foc_current_kp = 0.05;
    conf->foc_current_ki = 0.001;
    conf->c_pid_kp = 0.02;
    conf->c_pid_ki = 0.0;
    conf->c_pid_kd = 0.0001;  //新增的电流环PID参数
    conf->s_pid_kp = 0.004;
    conf->s_pid_ki = 0.0001;
    conf->s_pid_kd = 0.0001;
    conf->s_pid_kd_filter = 0.02;
    conf->s_pid_allow_braking = true;
    conf->s_pid_ramp_erpms_s = 2000.0;
    conf->s_pid_min_erpm = 200;
    conf->p_pid_kp = 0.03;
    conf->p_pid_ki = 0.0; 
    conf->p_pid_kd = 0.0004;
    conf->p_pid_kd_proc = 0.0004;
    conf->p_pid_kd_filter = 0.02;
    conf->p_pid_ang_div = 1.0;
    conf->p_pid_gain_dec_angle = 0.0;
    conf->p_pid_offset = 0.0;
    conf->l_pid_out_max = 20.0;  //新增的定位环输出限幅参数
    conf->l_c_i_term_max = 0.5; //新增的电流环积分限幅参数
    conf->l_s_i_term_max = 1.0; //新增的速度环
    conf->l_p_i_term_max = 1.5; //新增的定位环积分限幅参数

    conf->l_current_max = 20.0;
    conf->l_current_min = -20.0;
    conf->l_max_duty = 0.95;
    conf->l_min_duty = 0.05;
    conf->lo_current_max = 0;
    conf->lo_current_min = 0;

    conf->m_sensor_port_mode = SENSOR_PORT_MODE_AS5047_SPI;
}

static void foc_encoders_init (volatile motor_all_state_t* m) {
    volatile motor_state_t *motor_state = &m->m_motor_state;
    volatile mc_configuration *conf = m->m_conf;
    switch (conf->foc_sensor_mode) {
        case SENSOR_PORT_MODE_AS5047_SPI:
            enc_as504x_init(&as5047_cfg);
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
    HAL_TIM_Base_Start_IT(&htim4);  // encoder timer
}

/**
 * @description: 编码器读取角度流程，单纯进行数据更新
 */
void interface_encoder_routine(void) {  
    volatile motor_all_state_t *m = &motor;
	volatile motor_state_t *motor_state = &m->m_motor_state;
	volatile mc_configuration *conf = m->m_conf;
    switch (conf->m_sensor_port_mode) {
        case SENSOR_PORT_MODE_AS5047_SPI:
            enc_as504x_routine(&as5047_cfg);
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
}   

void interface_set_duty (float duty) {
    volatile motor_all_state_t *m = &motor;
	volatile motor_state_t *motor_state = &m->m_motor_state;
	volatile mc_configuration *conf = m->m_conf;
    if (duty > 0) {
        utils_truncate_number(&duty, conf->l_min_duty, conf->l_max_duty);
    } else {
        utils_truncate_number(&duty, -conf->l_max_duty, -conf->l_min_duty);
    }
    m->m_control_mode = CONTROL_MODE_DUTY;
    m->m_duty_cycle_set = duty;
}

void interface_set_current(float current) {
    volatile motor_all_state_t *m = &motor;
    volatile motor_state_t *motor_state = &m->m_motor_state;
    volatile mc_configuration *conf = m->m_conf;
    utils_truncate_number(&current, conf->l_current_min, conf->l_current_max);
    m->m_control_mode = CONTROL_MODE_CURRENT;
    m->m_iq_set = current;
}

void interface_set_current_brake(float current) {
    volatile motor_all_state_t *m = &motor;
    volatile motor_state_t *motor_state = &m->m_motor_state;
    volatile mc_configuration *conf = m->m_conf;
    utils_truncate_number(&current, conf->l_current_min, conf->l_current_max);
    m->m_control_mode = CONTROL_MODE_CURRENT_BRAKE;
    m->m_iq_set = current;
}

void interface_set_speed(float speed) {
    volatile motor_all_state_t *m = &motor;
    volatile motor_state_t *motor_state = &m->m_motor_state;
    volatile mc_configuration *conf = m->m_conf;
    utils_truncate_number(&speed, conf->l_max_erpm, conf->l_max_erpm);
    m->m_control_mode = CONTROL_MODE_SPEED;
    m->m_speed_command_rpm = speed;
}

void interface_set_pos(float pos) {
    volatile motor_all_state_t *m = &motor;
    volatile motor_state_t *motor_state = &m->m_motor_state;
    volatile mc_configuration *conf = m->m_conf;
    m->m_control_mode = CONTROL_MODE_POS;
    utils_norm_angle_rad(&pos);
    m->m_pos_pid_set = pos;
}

void interface_run_pid(void) {
    volatile motor_all_state_t *m = &motor;
    volatile motor_state_t *motor_state = &m->m_motor_state;
    volatile mc_configuration *conf = m->m_conf;

    uint32_t mode = m->m_control_mode;
    float err, p_term, d_term, pos_output, speed_output, current_output;
    float dt = 0.001;  // 执行周期


    if (mode == CONTROL_MODE_POS) {
        err = m->m_pos_pid_set - m->m_pos_pid_now;
        p_term = conf->p_pid_kp * err;
        m->m_pos_i_term += conf->p_pid_ki * err * dt;
        d_term = conf->p_pid_kd * (err - m->m_pos_prev_error) / dt;
        m->m_pos_prev_error = err;
        utils_truncate_number_abs(&m->m_pos_i_term, conf->l_p_i_term_max);

        pos_output = p_term + m->m_pos_i_term + d_term;
        m->m_speed_pid_set_rpm = pos_output;  //位置环的输出作为速度环的设定值
        utils_truncate_number_abs(&pos_output, conf->l_pid_out_max);
    }
    if (mode == CONTROL_MODE_SPEED || mode == CONTROL_MODE_POS) {
        err = m->m_speed_pid_set_rpm - m->m_speed_est_faster;
        p_term = conf->s_pid_kp * err;
        m->m_speed_i_term += conf->s_pid_ki * err * dt;
        d_term = conf->s_pid_kd * (err - m->m_speed_prev_error) / dt;
        m->m_speed_prev_error = err;
        utils_truncate_number_abs(&m->m_speed_i_term, conf->l_s_i_term_max);

        speed_output = p_term + m->m_speed_i_term + d_term;
        m->m_iq_set = speed_output;  //速度环的输出作为电流环的设定值
        utils_truncate_number_abs(&speed_output, conf->l_pid_out_max);
    }
    // if (mode == CONTROL_MODE_CURRENT || mode == CONTROL_MODE_SPEED || mode == CONTROL_MODE_POS) {
    //     err = m->m_iq_set - motor_state->iq;
    //     p_term = conf->c_pid_kp * err;
    //     m->m_current_i_term += conf->c_pid_ki * err * dt;
    //     d_term = conf->c_pid_kd * (err - m->m_current_prev_error) / dt;
    //     m->m_current_prev_error = err;
    //     utils_truncate_number_abs(&m->m_current_i_term, conf->l_c_i_term_max);

    //     current_output = p_term + m->m_current_i_term + d_term;
    //     utils_truncate_number_abs(&current_output, conf->l_pid_out_max);
    // }

}