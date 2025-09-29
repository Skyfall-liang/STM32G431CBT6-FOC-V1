#ifndef __FOC_COMMON_H
#define __FOC_COMMON_H
#include "main.h"
#include "lowpass_filter.h"
#include <math.h>
#include "encoder.h"
#include "foc_Inlinecurrent.h"
#include "pid.h"
#include "foc_closeloop_control.h"
#include "foc_dataprocess.h"



//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。

extern float PI;
extern float _3PI_2;
extern float _PI_2;
extern float _PI_3;
extern float _3PI_2;
extern float _1_SQRT3;
extern float _2_SQRT3;
extern float _SQRT3;
extern float _SQRT3_2;
extern float _SQRT2;


#define voltage_power_supply    24

extern int Sensor_DIR;    //传感器方向
extern int Motor_PP;    //电机极对数
extern float zero_electric_angle;

extern float test1;

//FOC初始化
void foc_init(void);

// 开环电角度求解
float openloop_electricalAngle(float shaft_angle, int pole_pairs);

// 电角度求解
float _electricalAngle(void);

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle);

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc);

// 根据电压值，设置PWM到控制器输出
void setPhaseVoltage(float Uq, float Ud, float angle_el);

//读取磁编码器的弧度角度值
float encoder_read(void);

float getAngle(void);

float getVelocity(void);

//获取速度并滤波
float get_LPF_velocity(void);

float cal_Iq_Id(float current_a,float current_b,float angle_el);

float measureLd(void);

float measureLq(void);



float cal_Id_test(float current_a,float current_b,float angle_el);
float cal_Iq_test(float current_a,float current_b,float angle_el);

int alignSensor(void);


#endif




