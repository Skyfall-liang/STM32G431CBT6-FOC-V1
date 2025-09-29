#include "foc_closeloop_control.h"

float angle_now = 0;
float angle_control_output = 0;
float velocity_now = 0;
float velocity_control_output = 0;
float current_now = 0;
float current_control_output = 0;


float I_d, I_q = 0;


void control_data_updata(void)
{
    angle_now = getAngle();

    velocity_now = get_LPF_velocity();

    struct CurrentDetect Current = GetPhaseCurrent();

	current_now = LPF_current(cal_Iq_Id(Current.I_a, Current.I_b, _electricalAngle()));
    
    
    I_d = LPF_current_dd(cal_Id_test(Current.I_a, Current.I_b, _electricalAngle()));
    I_q = LPF_current_dq(cal_Iq_test(Current.I_a, Current.I_b, _electricalAngle()));
}

void foc_set_angle_control(float target)
{
    control_data_updata();
    angle_control_output = _constrain(PID_Controller(&pid_angle_control, Sensor_DIR*(target - angle_now)), -voltage_power_supply/2, voltage_power_supply/2);
    
    setPhaseVoltage(angle_control_output, 0, _electricalAngle());
}


void foc_set_velocity_control(float target)
{
    control_data_updata();
    velocity_control_output = _constrain(PID_Controller(&pid_velocity_control, Sensor_DIR*(target - velocity_now)), -voltage_power_supply/2, voltage_power_supply/2);
    
    setPhaseVoltage(velocity_control_output, 0, _electricalAngle());
}


void foc_set_current_control(float target)
{
    control_data_updata();
	current_control_output = _constrain(PID_Controller(&pid_current_control, (target - current_now)), -voltage_power_supply/2, voltage_power_supply/2);

	setPhaseVoltage(current_control_output, 0, _electricalAngle());
}


void foc_set_velocity_current_control(float target)
{
    control_data_updata();
    velocity_control_output = PID_Controller(&pid_VC_velocity ,Sensor_DIR*(target - velocity_now));
    
    current_control_output = _constrain(PID_Controller(&pid_VC_current, (velocity_control_output - current_now)), -voltage_power_supply/2, voltage_power_supply/2);
    
    setPhaseVoltage(current_control_output, -I_d, _electricalAngle());
}


void foc_set_angle_velocity_current_control(float target)
{

    control_data_updata();
    angle_control_output = PID_Controller(&pid_AVC_angle, Sensor_DIR*(target - angle_now));

    velocity_control_output = PID_Controller(&pid_AVC_velocity, Sensor_DIR*(angle_control_output - velocity_now));
    
    current_control_output = _constrain(PID_Controller(&pid_AVC_current, (velocity_control_output - current_now)), -voltage_power_supply/2, voltage_power_supply/2);
    
    setPhaseVoltage(current_control_output, 0, _electricalAngle());
}












