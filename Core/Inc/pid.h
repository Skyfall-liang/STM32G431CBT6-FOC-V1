#ifndef __PID_H
#define __PID_H
#include "main.h"

struct _PID{
	float Kp;
	float Ki;
	float Kd;
	unsigned long Timestamp_Last;
	float Last_error;
	float Last_integral;
	float Last_output;
    float output_ramp;
    float output_limit;
};


float PID_Controller(struct _PID *pid, float error);
void PID_init(void);

extern struct _PID pid_angle_control, pid_velocity_control, pid_current_control;
extern struct _PID pid_VC_velocity, pid_VC_current;
extern struct _PID pid_AVC_angle, pid_AVC_velocity, pid_AVC_current;


extern struct _PID pid_fw;
extern struct _PID pid_CC_d;
extern struct _PID pid_CC_q;

#endif



