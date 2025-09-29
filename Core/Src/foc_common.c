#include "foc_common.h"

float PI        =   3.14159265359f;
float _3PI_2    =   4.71238898038f;
float _PI_2     =   1.57079632679f;
float _PI_3     =   1.0471975512f;
float _1_SQRT3  =   0.57735026919f;
float _2_SQRT3  =   1.15470053838f;
float _SQRT3    =   1.73205080757;
float _SQRT3_2  =   0.86602540378;
float _SQRT2    =   1.41421356237;

int Sensor_DIR = 1;    //传感器方向
int Motor_PP = 2;    //电机极对数

float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

uint16_t raw_num = 0;
float JIAODUnum = 0;
float angle_prev = 0; 
int32_t full_rotations = 0; // full rotation tracking;


// 开环电角度求解
float openloop_electricalAngle(float shaft_angle, int pole_pairs) 
{
  return (shaft_angle * pole_pairs);
}


// 电角度求解
float _electricalAngle(void)
{
  return  _normalizeAngle((float)(Sensor_DIR *  Motor_PP * encoder_read())-zero_electric_angle);
}


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return ((a >= 0) ? a : (a + 2*PI));  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}


// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc) 
{
    // 计算占空比
    // 限制占空比从0到1
    dc_a = _constrain(Ua/voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(Ub/voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(Uc/voltage_power_supply, 0.0f, 1.0f);

    //写入PWM到PWM 0 1 2 通道
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dc_a*5000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, dc_b*5000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dc_c*5000);

}


//void setPhaseVoltage(float Uq, float Ud, float angle_el)
//{
//    angle_el = _normalizeAngle(angle_el);
//    // 帕克逆变换
//    Ualpha =  -Uq*sin(angle_el); 
//    Ubeta =   Uq*cos(angle_el); 

//    // 克拉克逆变换
//    Ua = Ualpha + voltage_power_supply/2;
//    Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
//    Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
//    setPwm(Ua,Ub,Uc);
//}



void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    float Uout = 0;
    int sector = 0;
    float T0,T1,T2 = 0;
    float Ta,Tb,Tc = 0;


    if(Ud) // only if Ud and Uq set 
    {// _sqrt is an approx of sqrt (3-4% error)
        Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
    }
    else
    {// only Uq available - no need for atan2 and sqrt
        
        if(Uq < 0)
        {
            angle_el += PI;
        }
        Uq = fabs(Uq);
        
        Uout = Uq/voltage_power_supply;
        // angle normalisation in between 0 and 2pi
        // only necessary if using _sin and _cos - approximation functions
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
        if(Uout> 0.577f)Uout = 0.577f;
        if(Uout<-0.577f)Uout = -0.577f;

    sector = floor(angle_el/_PI_3) + 1;

    T1 = _SQRT3 * sin(sector*_PI_3 - angle_el) * Uout;
    T2 = _SQRT3 * sin(angle_el - (sector-1.0)*_PI_3) * Uout;
    T0 = 1 - T1 - T2;

    // calculate the duty cycles(times)
    switch(sector)
    {
        case 1:
            Ta = T1 + T2 + T0/2;
            Tb = T2 + T0/2;
            Tc = T0/2;
            break;
        case 2:
            Ta = T1 + T0/2;
            Tb = T1 + T2 + T0/2;
            Tc = T0/2;
            break;
        case 3:
            Ta = T0/2;
            Tb = T1 + T2 + T0/2;
            Tc = T2 + T0/2;
            break;
        case 4:
            Ta = T0/2;
            Tb = T1+ T0/2;
            Tc = T1 + T2 + T0/2;
            break;
        case 5:
            Ta = T2 + T0/2;
            Tb = T0/2;
            Tc = T1 + T2 + T0/2;
            break;
        case 6:
            Ta = T1 + T2 + T0/2;
            Tb = T0/2;
            Tc = T1 + T0/2;
            break;
        default:  // possible error state
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }
    
    Ua = Ta*voltage_power_supply;
    Ub = Tb*voltage_power_supply;
    Uc = Tc*voltage_power_supply;
    
    setPwm(Ua,Ub,Uc);
}


//读取磁编码器的弧度制角度值
float encoder_read(void)
{
    raw_num = Read_Encode(); // 0 ~ 4000
    
    JIAODUnum = ((float)raw_num / 4000.0f) * 2.0f * PI;
    //JIAODUnum = ((float)raw_num / 16384.0f) * 2.0f * PI;
    
    return JIAODUnum;
}


//读取带圈数的弧度制角度值
float getAngle(void)
{
    float val = encoder_read();
    float d_angle = val - angle_prev;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(fabs(d_angle) > 0.8f*6.28318530718f ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return (float)full_rotations*6.28318530718f + angle_prev;
}


float velocity_calc_timestamp = 0;
float angle_data_prev = 0;
int alignSensor(void);
//foc初始化
void foc_init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);       /* 开启对应PWM通道 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);       /* 开启对应PWM通道 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);       /* 开启对应PWM通道 */
    
    delay_ms(500);

    CurrSense_Init();
    
    
    int aa = alignSensor();
    
//    setPhaseVoltage(2, 0,_3PI_2);
//    delay_ms(500);
//    zero_electric_angle = _electricalAngle();
//    setPhaseVoltage(0, 0,_3PI_2);
//	delay_ms(500);
    

    velocity_calc_timestamp = SysTick->VAL;
    angle_data_prev = getAngle();
    
    PID_init();
}


// Shaft velocity calculation 
float getVelocity(void)
{
	unsigned long now_us;
	float Ts, angle_c, vel;

	now_us = SysTick->VAL; //_micros();

	if(now_us<velocity_calc_timestamp)
    {
        Ts = (float)(velocity_calc_timestamp - now_us)/170*1e-6f;
    }
	else
    {
        Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/170*1e-6f;
    }

	if(Ts <= 0.0f) Ts = 0.001f;

	angle_c = getAngle();

	vel = (angle_c - angle_data_prev)/Ts;

	// save variables for future pass
	angle_data_prev = angle_c;
	velocity_calc_timestamp = now_us;
	return vel;
}


//获取速度并滤波
float get_LPF_velocity(void)
{
	float v1 = getVelocity();
	float v2 = LPF_velocity(Sensor_DIR*v1);
	return v2;
}


float tyty12 = 0;
float tyty13 = 0;
float cal_Iq_Id(float current_a,float current_b,float angle_el)
{
    float I_alpha,I_beta, ct, st;
    float I_q = 0;

    I_alpha = current_a;
    I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
    
    ct = cos(angle_el);
    st = sin(angle_el);

    tyty12 = I_beta * ct;
    tyty13 = I_alpha * st;
    
    I_q = tyty12 - tyty13;
    
  return I_q;
}



float cal_Id_test(float current_a,float current_b,float angle_el)
{
    float I_alpha, I_beta, ct, st;
    
    I_alpha = current_a;
    I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
    
    ct = cos(angle_el);
    st = sin(angle_el);
    
    return I_alpha * ct + I_beta * st;  // d轴分量
}



float cal_Iq_test(float current_a,float current_b,float angle_el)
{
    float I_alpha, I_beta, ct, st;
    
    I_alpha = current_a;
    I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
    
    ct = cos(angle_el);
    st = sin(angle_el);
    
    return I_beta * ct - I_alpha * st;  // d轴分量
}


float measureLd(void) 
{
    float Ts;
    float V_test = 3.0;  // 安全测试电压
    
    struct CurrentDetect Current_test = GetPhaseCurrent();

    
    setPhaseVoltage(0, V_test, 0);  // 施加 d 轴电压
    unsigned long now_us_1 = SysTick->VAL; //_micros();
    delay_us(500);         // 保持 500μs
    Current_test = GetPhaseCurrent();
    float Id1 = LPF_current(cal_Id_test(Current_test.I_a, Current_test.I_b, _electricalAngle()));      // 获取初始 Id
    
    delay_us(500);         // 保持 500μs
        unsigned long now_us_2 = SysTick->VAL; //_micros();
    Current_test = GetPhaseCurrent();
    float Id2 = LPF_current(cal_Id_test(Current_test.I_a, Current_test.I_b, _electricalAngle()));      // 获取结束 Id
    setPhaseVoltage(0, 0, 0);       // 关闭输出

    
    
    	if(now_us_2<now_us_1)
    {
        Ts = (float)(now_us_1 - now_us_2)/170*1e-6f;
    }
	else
    {
        Ts = (float)(0xFFFFFF - now_us_2 + now_us_1)/170*1e-6f;
    }
    
    
  
    float deltaT = Ts;            // 时间间隔 (s)
    return (V_test * deltaT) / (Id2 - Id1);
}



float measureLq(void) 
{
    float Ts;
    float V_test = 3.0;  // 安全测试电压
    
    struct CurrentDetect Current_test = GetPhaseCurrent();

    
    setPhaseVoltage(V_test, 0, PI/2);  // 施加 d 轴电压
    unsigned long now_us_1 = SysTick->VAL; //_micros();
    delay_us(500);         // 保持 500μs
    Current_test = GetPhaseCurrent();
    float Id1 = LPF_current(cal_Iq_test(Current_test.I_a, Current_test.I_b, _electricalAngle()));      // 获取初始 Id
    
    delay_us(500);         // 保持 500μs
    unsigned long now_us_2 = SysTick->VAL; //_micros();
    Current_test = GetPhaseCurrent();
    float Id2 = LPF_current(cal_Iq_test(Current_test.I_a, Current_test.I_b, _electricalAngle()));      // 获取结束 Id
    setPhaseVoltage(0, 0, 0);       // 关闭输出

    
    
    	if(now_us_2<now_us_1)
    {
        Ts = (float)(now_us_1 - now_us_2)/170*1e-6f;
    }
	else
    {
        Ts = (float)(0xFFFFFF - now_us_2 + now_us_1)/170*1e-6f;
    }
    
    
  
    float deltaT = Ts;            // 时间间隔 (s)
    return (V_test * deltaT) / (Id2 - Id1);
}


void Calibration_choose(uint8_t id, float mid)
{
    switch (id)
    {
    case 0x00:
    case 0x04:
        if((mid > 3.3f) || (mid < 2.9f))
        {
            HAL_NVIC_SystemReset(); // 触发系统复位
        }
        break;

    case 0x01:
        if((mid > 1.6f) || (mid < 1.3f))
        {
            HAL_NVIC_SystemReset(); // 触发系统复位
        }
        break;

    case 0x02:
        if((mid > 2.4f) || (mid < 2.1f))
        {
            HAL_NVIC_SystemReset(); // 触发系统复位
        }
        break;
    
    case 0x03:
        if((mid > 0.6f) || (mid < 0.2f))
        {
            HAL_NVIC_SystemReset(); // 触发系统复位
        }
        break;

    case 0x05:
        if((mid > 2.5f) || (mid < 2.2f))
        {
            HAL_NVIC_SystemReset(); // 触发系统复位
        }
        break;

    default:
        break;
    }
}

int alignSensor(void)
{
    long i;                                 // 长整型变量 i
    float angle;                            // 浮点类型变量，用于存储电角度
    float mid_angle, end_angle;             // 分别用于存储中间和结束角度
    float moved;                            // 用于记录传感器移动的角度差值

    printf("MOT: Align sensor.\r\n");       // 打印日志信息，开始对传感器进行对齐

    // 找到自然方向
    // 在一个电气周期内正向移动
    for(i=0; i<=500; i++)                                   // 循环从 0 到 500
    {
        angle = _3PI_2 + 2*PI * i / 500.0f;                  // 计算当前电角度，线性递增
        
        setPhaseVoltage(voltage_power_supply/12, 0,  angle); // 设置输出电压，并更新电角度
        delay_ms(4);                                        // 延迟 2 毫秒
        
    }
    mid_angle = getAngle();                 // 获取电机转子运行中的中间角度

    Calibration_choose(motor_id, mid_angle);

    for(i=500; i>=0; i--)                   // 循环从 500 到 0，反向移动
    {
        angle = _3PI_2 + 2*PI * i / 500.0f;  // 重新计算角度，线性递减
        setPhaseVoltage(voltage_power_supply/12, 0,  angle); // 设置反向电角度
        delay_ms(4);                        // 延迟 2 毫秒
    }

    end_angle = getAngle();                 // 获取电机转子运行中的结束角度
    setPhaseVoltage(0, 0, 0);               // 禁止所有电压输出，停止处于零位置
    delay_ms(500);                         // 延迟 200 毫秒，确保停止稳定

    printf("mid_angle=%.4f\r\n",mid_angle); // 打印中间角度
    printf("end_angle=%.4f\r\n",end_angle); // 打印结束角度

    moved = fabs(mid_angle - end_angle);    // 计算角度移动的绝对值
    if((mid_angle == end_angle)||(moved < 0.02f))  // 如果中间角度与结束角度相同，或者移动角度很小
    {
        printf("MOT: Failed to notice movement loop222.\r\n"); // 打印失败日志
        //M1_Disable;                     // 电机检测不正常，关闭驱动
         HAL_NVIC_SystemReset(); // 触发系统复位
        return 0;                       // 返回失败状态
    }
    
    
    float delta = _normalizeAngle(end_angle - mid_angle);
    if (delta > PI) delta -= 2*PI;
    if (delta < -PI) delta += 2*PI;

    Sensor_DIR = (delta < 0) ? 1 : -1;    // delta<0 → 正向→CW
    if(Sensor_DIR == 1)
    {
        printf("MOT: sensor_direction==CW\r\n"); // 打印传感器方向为顺时针
    }
    else
    {
        printf("MOT: sensor_direction==CCW\r\n"); // 打印传感器方向为逆时针
        HAL_NVIC_SystemReset(); // 触发系统复位
    }
    
//    else if(mid_angle < end_angle)      // 如果中间角度小于结束角度
//    {
//        printf("MOT: sensor_direction==CCW\r\n"); // 打印传感器方向为逆时针
//        Sensor_DIR = -1;            // 设置传感器方向为逆时针
//        HAL_NVIC_SystemReset(); // 触发系统复位
//    }
//    else                                // 如果中间角度大于结束角度
//    {
//        printf("MOT: sensor_direction==CW\r\n"); // 打印传感器方向为顺时针
//        Sensor_DIR = 1;             // 设置传感器方向为顺时针
//    }

   
    
    printf("MOT: PP check: ");           // 打印日志，检查磁极对数计算情况
    if(fabs(moved * Motor_PP - 2*PI) > 1.0f)  // 如果估算的磁极对数计算结果误差大于阈值（这里是 0.5，可以调整）
    {
        printf("fail - estimated pp:");  // 打印日志：磁极对数估算失败
        
        float uu = 2*PI / moved + 0.5f;
        printf("%f\r\n",uu);
        int est_Motor_PP = 2*PI / moved + 0.5f; // 根据移动角度估算磁极对数（浮点数转整数，四舍五入）
        printf("%d\r\n",est_Motor_PP);     // 打印最终估算的磁极对数
        
        if(est_Motor_PP != Motor_PP)
        {
            HAL_NVIC_SystemReset();
        }
    }
    else                                // 如果磁极对数估算通过
        printf("OK!\r\n");              // 打印：磁极对数检查通过


     setPhaseVoltage(voltage_power_supply/12, 0, _3PI_2);  // 再次设置输出电压到一个固定角度，计算零点偏移
     delay_ms(700);                     // 延迟 700 毫秒，确保电机定位完成
     zero_electric_angle = _normalizeAngle(_electricalAngle()); // 计算电气零偏移角度
    
     delay_us(200);                      // 延迟 20 毫秒
     printf("MOT: Zero elec. angle:");   // 打印电气零点角度
     printf("%.4f\r\n",zero_electric_angle); // 具体打印零点角度值
     setPhaseVoltage(0, 0, _3PI_2);           // 禁止全部输出电压
     delay_ms(200);                     // 延迟 200 毫秒，确保停止稳定
     return 1;                           // 返回成功状态
}



















