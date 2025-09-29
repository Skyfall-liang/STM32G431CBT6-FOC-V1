#ifndef __LOWPASS_FILTER_H
#define __LOWPASS_FILTER_H
#include "foc_common.h"

extern float DEF_VEL_FILTER_Tf;
extern float DEF_CUR_FILTER_Tf;

float LPF_velocity(float x);
float LPF_current(float x);




float LPF_current_dq(float x);
float LPF_current_dd(float x);





#endif


