#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void Set_Encoder_Zero(void);
uint16_t Read_Encode(void);

extern uint8_t encoder_angle_valid;


void CalSensorAngle(void);

float Cal_Encoder_Speed(void);

void read_angle_test(void);



















#endif
