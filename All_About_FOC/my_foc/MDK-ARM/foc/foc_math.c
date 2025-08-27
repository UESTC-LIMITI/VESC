#include "foc_math.h"

void current_control(void) {
    motor_all_state_t *m = &motor;
	motor_state_t *motor_state = &m->m_motor_state;

	// todo: 假设已有目标id iq, 写一个电流控制器
}
