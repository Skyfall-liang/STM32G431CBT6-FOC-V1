#ifndef _FOC_DATAPROCESS_H
#define _FOC_DATAPROCESS_H

#include "main.h"

extern float motor_target;
extern uint8_t motor_id;
extern uint8_t motor_init_flag;

extern float zero_angle;

uint8_t fdcan_send_msg(uint8_t *msg, uint32_t len);

void data_feedback(void);

void foc_data_process(void);
void Taskloop_select(void);

#endif

