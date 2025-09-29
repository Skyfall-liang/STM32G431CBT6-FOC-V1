#include "encoder.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == encode_Z_Pin)
  {
    // 重置编码器计数器
    Set_Encoder_Zero();
    // printf("Encoder Z interrupt: Counter reset\n");
  }
}


uint8_t encoder_angle_valid = 0;
void Set_Encoder_Zero(void)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  encoder_angle_valid = 1;
}

float sensor_angle = 0;
uint16_t encode = 0;
// 计算传感器角度
void CalSensorAngle(void)
{
  encode = Read_Encode(); // 0 ~ 4000
  //printf("encode: %d\n", encode);
  sensor_angle = ((float)encode / 4000.0f) * 2.0f * PI;
}


uint16_t Read_Encode(void)
{
  return __HAL_TIM_GET_COUNTER(&htim1);
  // __HAL_TIM_SET_COUNTER(&htim1, 0);
  // printf("encoder: %d\n", encoder);
 
}
















