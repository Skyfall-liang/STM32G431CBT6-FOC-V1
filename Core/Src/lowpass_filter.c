#include "lowpass_filter.h"


float DEF_VEL_FILTER_Tf = 0.2;     //!< default velocity filter time constant
float DEF_CUR_FILTER_Tf = 0.1;


uint32_t lpf_vel_timestamp = 0;
float y_vel_prev = 0;

float LPF_velocity(float x)
{
	unsigned long now_us;
	float Ts, alpha, y;
	
	now_us = SysTick->VAL;
    //now_us = HAL_GetTick();

	if(now_us < lpf_vel_timestamp)
    {
        Ts = (float)(lpf_vel_timestamp - now_us)/170*1e-6f;
        //Ts = (float)(lpf_vel_timestamp - now_us) * 1e-3f;
    }
	else
    {
        Ts = (float)(0xFFFFFF - now_us + lpf_vel_timestamp)/170*1e-6f; 
        //Ts = (float)(now_us - lpf_vel_timestamp) * 1e-3f;
    }

    if(Ts == 0.0f || Ts < 0) Ts = 0.0015f; 
	else if(Ts > 0.005f)
    {
        y_vel_prev = x;
        lpf_vel_timestamp = now_us;
        return x;
    }
    //printf("%f\n", Ts);
	

	alpha = DEF_VEL_FILTER_Tf / (DEF_VEL_FILTER_Tf + Ts);

	y = alpha * y_vel_prev + (1.0f - alpha)*x;

	y_vel_prev = y;

    lpf_vel_timestamp = now_us;
	
	return y;
}







/************************************************************************************* */



uint32_t lpf_cur_timestamp = 0;
float y_cur_prev = 0;
float LPF_current(float x)
{
	unsigned long now_us;
	float Ts, alpha, y;
	
	now_us = SysTick->VAL;
    //now_us = HAL_GetTick();

	if(now_us < lpf_cur_timestamp)
    {
        Ts = (float)(lpf_cur_timestamp - now_us)/170*1e-6f;
        //Ts = (float)(lpf_vel_timestamp - now_us) * 1e-3f;
    }
	else
    {
        Ts = (float)(0xFFFFFF - now_us + lpf_cur_timestamp)/170*1e-6f; 
        //Ts = (float)(now_us - lpf_vel_timestamp) * 1e-3f;
    }

    if(Ts == 0.0f || Ts < 0) Ts = 0.0015f; 
	else if(Ts > 0.003f)
    {
        y_cur_prev = x;
        lpf_cur_timestamp = now_us;
        return x;
    }
    //printf("%f\n", Ts);
	

	alpha = (DEF_CUR_FILTER_Tf) / (DEF_CUR_FILTER_Tf + Ts);

	y = alpha * y_cur_prev + (1.0f - alpha)*x;

	y_cur_prev = y;

    lpf_cur_timestamp = now_us;
	
	return y;
}





//?????????????????????????????????????????????????????????????????????????????????????????????????????????????????
float DEF_CUR_FILTER_Tf_dq = 0.1;
uint32_t lpf_cur_timestamp_dq = 0;
float y_cur_prev_dq = 0;
float LPF_current_dq(float x)
{
	unsigned long now_us;
	float Ts, alpha, y;
	
	now_us = SysTick->VAL;
    //now_us = HAL_GetTick();

	if(now_us < lpf_cur_timestamp_dq)
    {
        Ts = (float)(lpf_cur_timestamp_dq - now_us)/170*1e-6f;
        //Ts = (float)(lpf_vel_timestamp - now_us) * 1e-3f;
    }
	else
    {
        Ts = (float)(0xFFFFFF - now_us + lpf_cur_timestamp_dq)/170*1e-6f; 
        //Ts = (float)(now_us - lpf_vel_timestamp) * 1e-3f;
    }

    if(Ts == 0.0f || Ts < 0) Ts = 0.0015f; 
	else if(Ts > 0.003f)
    {
        y_cur_prev_dq = x;
        lpf_cur_timestamp_dq = now_us;
        return x;
    }
    //printf("%f\n", Ts);
	

	alpha = (DEF_CUR_FILTER_Tf_dq) / (DEF_CUR_FILTER_Tf_dq + Ts);

	y = alpha * y_cur_prev_dq + (1.0f - alpha)*x;

	y_cur_prev_dq = y;

    lpf_cur_timestamp_dq = now_us;
	
	return y;
}



float DEF_CUR_FILTER_Tf_dd = 0.1;
uint32_t lpf_cur_timestamp_dd = 0;
float y_cur_prev_dd = 0;
float LPF_current_dd(float x)
{
	unsigned long now_us;
	float Ts, alpha, y;
	
	now_us = SysTick->VAL;
    //now_us = HAL_GetTick();

	if(now_us < lpf_cur_timestamp_dd)
    {
        Ts = (float)(lpf_cur_timestamp_dd - now_us)/170*1e-6f;
        //Ts = (float)(lpf_vel_timestamp - now_us) * 1e-3f;
    }
	else
    {
        Ts = (float)(0xFFFFFF - now_us + lpf_cur_timestamp_dd)/170*1e-6f; 
        //Ts = (float)(now_us - lpf_vel_timestamp) * 1e-3f;
    }

    if(Ts == 0.0f || Ts < 0) Ts = 0.0015f; 
	else if(Ts > 0.003f)
    {
        y_cur_prev_dd = x;
        lpf_cur_timestamp_dd = now_us;
        return x;
    }
    //printf("%f\n", Ts);
	

	alpha = (DEF_CUR_FILTER_Tf_dd) / (DEF_CUR_FILTER_Tf_dd + Ts);

	y = alpha * y_cur_prev_dd + (1.0f - alpha)*x;

	y_cur_prev_dd = y;

    lpf_cur_timestamp_dd = now_us;
	
	return y;
}





