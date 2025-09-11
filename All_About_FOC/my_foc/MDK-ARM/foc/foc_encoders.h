#ifndef FOC_ENCODERS_H_
#define FOC_ENCODERS_H_

#include <stdbool.h>

struct AS504x_config;
typedef struct AS504x_config AS504x_config_t;


bool enc_as504x_init(AS504x_config_t *AS504x_config);
void enc_as504x_deinit(AS504x_config_t *cfg);
void enc_as504x_routine(AS504x_config_t *cfg);
float enc_as504x_read_angle(AS504x_config_t *cfg);




#endif // FOC_ENCODERS_H_
