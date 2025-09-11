#ifndef FOC_ENCODERS_H_
#define FOC_ENCODERS_H_

#include "main.h"

void timer_init(void);
uint32_t timer_time_now(void);
float timer_seconds_elapsed_since(uint32_t time);
void timer_sleep(float seconds);

#endif // FOC_ENCODERS_H_
