#ifndef  __INLINECURRENT_H
#define  __INLINECURRENT_H

#include "delay.h"
#include "adc.h"


extern volatile uint8_t g_adc_dma_sta;
#define _ADC_CONV               0.00080586f
#define ADC_DMA_BUF_SIZE        1 * 2      /* ADC DMA采集 BUF大小, 应等于ADC通道数的整数倍 */
extern uint16_t Samp_volts[2];

#define ADC_ADCX_DMACx_IS_TC()              ( DMA1->ISR & (1 << 1) )            /* 判断 DMA1_Channel1 传输完成标志, 这是一个假函数形式,
                                                                                 * 不能当函数使用, 只能用在if等语句里面 
                                                                                 */
#define ADC_ADCX_DMACx_CLR_TC()             do{ DMA1->IFCR |= 1 << 1; }while(0) /* 清除 DMA1_Channel1 传输完成标志 */

void adc_dma_init(uint32_t mar);

extern uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE];   /* ADC DMA BUF */

void get_current_raw_data(void);


struct CurrentDetect{
	float I_a;
	float I_b;
	float U_a;
	float U_b;
};


void DriftOffsets(void);
void CurrSense_Init(void);
struct CurrentDetect GetPhaseCurrent(void);






#endif


