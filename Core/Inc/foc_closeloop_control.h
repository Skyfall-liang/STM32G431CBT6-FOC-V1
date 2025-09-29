#ifndef __FOC_CLOSELOOP_CONTROL_H
#define __FOC_CLOSELOOP_CONTROL_H
#include "foc_common.h"

void foc_set_angle_control(float target);
void foc_set_velocity_control(float target);
void foc_set_current_control(float target);
void foc_set_velocity_current_control(float target);
void foc_set_angle_velocity_current_control(float target);
void control_data_updata(void);

extern float angle_now;
extern float velocity_now;
extern float current_now;

extern float angle_control_output;
extern float velocity_control_output;
extern float current_control_output;




void foc_set_velocity_current_control_test(float target_velocity);




#endif



