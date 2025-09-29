#include "pid.h"

float voltage_limit = voltage_power_supply / 2;

struct _PID pid_angle_control = {3, 0, 0, 0, 0, 0, 0, 1000, 6};
struct _PID pid_velocity_control = {0.5, 3, 0, 0, 0, 0, 0, 1000, 6};
struct _PID pid_current_control = {2, 6, 0, 0, 0, 0, 0, 1000, 6};

struct _PID pid_VC_velocity = {0.5, 3, 0, 0, 0, 0, 0, 1000, 6};
struct _PID pid_VC_current = {2, 20, 0, 0, 0, 0, 0, 1000, 6};

struct _PID pid_AVC_angle = {10, 0, 0, 0, 0, 0, 0, 1000, 100};
struct _PID pid_AVC_velocity = {0.5, 3, 0, 0, 0, 0, 0, 1000, 1};
struct _PID pid_AVC_current = {2, 6, 0, 0, 0, 0, 0, 1000, 6};

void PID_init(void)
{
    pid_angle_control.Timestamp_Last    = SysTick->VAL;
    pid_velocity_control.Timestamp_Last = SysTick->VAL;
    pid_current_control.Timestamp_Last  = SysTick->VAL;
    pid_VC_velocity.Timestamp_Last      = SysTick->VAL;
    pid_VC_current.Timestamp_Last       = SysTick->VAL;
    pid_AVC_angle.Timestamp_Last        = SysTick->VAL;
    pid_AVC_velocity.Timestamp_Last     = SysTick->VAL;
    pid_AVC_current.Timestamp_Last      = SysTick->VAL;

    pid_angle_control.output_limit      = voltage_limit;
    pid_velocity_control.output_limit   = voltage_limit;
    pid_current_control.output_limit    = voltage_limit;
    pid_VC_current.output_limit         = voltage_limit;
    pid_AVC_current.output_limit        = voltage_limit;
    
}


float PID_Controller(struct _PID *pid, float error) 
{
	unsigned long now_us;
	float Ts;
	float proportional, integral, derivative, output;
    float output_rate;
	
	now_us = SysTick->VAL;
	if(now_us < pid->Timestamp_Last)
    {
        Ts = (float)(pid->Timestamp_Last - now_us)/72*1e-6f;
    }
	else
    {
        Ts = (float)(0xFFFFFF - now_us + pid->Timestamp_Last)/72*1e-6f;
    }

	if(Ts == 0.0f || Ts > 0.5f) Ts = 1e-3f;
	
	proportional = pid->Kp * error;

	integral = pid->Last_integral + pid->Ki * 0.5f * Ts * (error + pid->Last_error);

	integral = _constrain(integral, -pid->output_limit, pid->output_limit);

    derivative = pid->Kd * (error - pid->Last_error) / Ts;
	
	output = proportional + integral + derivative;

	output = _constrain(output, -pid->output_limit, pid->output_limit);
	
	// limit the acceleration by ramping the output
	output_rate = (output - pid->Last_output) / Ts;
	if(output_rate > pid->output_ramp)output = pid->Last_output + pid->output_ramp*Ts;
	else if(output_rate < -pid->output_ramp)output = pid->Last_output - pid->output_ramp*Ts;
	
	// saving for the next pass
	pid->Last_integral = integral;
	pid->Last_output = output;
	pid->Last_error = error;
    pid->Timestamp_Last = now_us;
	
	return output;
}






