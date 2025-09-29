#include "foc_Inlinecurrent.h"

float _shunt_resistor = 0;
float amp_gain = 0;
float vlots_to_amps = 0;
float gain_a, gain_b, gain_c;
float offset_ia = 0.0, offset_ib = 0.0;

volatile uint8_t g_adc_dma_sta = 0;                                                  /* DMA传输状态标志, 0,未完成; 1, 已完成 */
uint16_t g_adc_dma_buf[ADC_DMA_BUF_SIZE];   /* ADC DMA BUF */
uint16_t Samp_volts[2];


void adc_dma_init(uint32_t mar)
{
    HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&ADC1->DR, mar, 2);     /* 启动DMA，并开启中断 */
    HAL_ADC_Start_DMA(&hadc1, &mar, 2);                        /* 开启ADC，通过DMA传输结果 */
}


void get_current_raw_data(void)
{
    
    while(g_adc_dma_sta == 0);
    if(g_adc_dma_sta == 1)
    {
        Samp_volts[0] = g_adc_dma_buf[0];
        Samp_volts[1] = g_adc_dma_buf[1];

        g_adc_dma_sta = 0;
    }

}


// 零飘检测
void DriftOffsets()
{
	uint16_t detect_rounds = 1000;
	for(int i = 0; i < detect_rounds; i++)
	{
        get_current_raw_data();
		offset_ia += (float)Samp_volts[0]*_ADC_CONV;
		offset_ib += (float)Samp_volts[1]*_ADC_CONV;
        //delay_us(1);
	}
	offset_ia = offset_ia / detect_rounds;
	offset_ib = offset_ib / detect_rounds;
    //printf("%f, %f\n", offset_ia, offset_ib);

}


void CurrSense_Init(void)
{

    adc_dma_init((uint32_t)&g_adc_dma_buf); /* 初始化ADC DMA采集 */
    
    DriftOffsets();

    _shunt_resistor = 0.01; 
    amp_gain = 50; 

    vlots_to_amps = 1.0f / _shunt_resistor / amp_gain;

    gain_a = vlots_to_amps * 1;
    gain_b = vlots_to_amps * 1;
    gain_c = vlots_to_amps;

}


float tran_vol_a, tran_vol_b = 0;
struct CurrentDetect GetPhaseCurrent(void)
{
	struct CurrentDetect current;

    get_current_raw_data();
    tran_vol_a = (float)Samp_volts[0]*_ADC_CONV;
    tran_vol_b = (float)Samp_volts[1]*_ADC_CONV;

	current.I_a = (tran_vol_a - offset_ia)*gain_a;
	current.I_b = (tran_vol_b - offset_ib)*gain_b;
    
	current.U_a = (tran_vol_a - offset_ia) ;
	current.U_b = (tran_vol_b - offset_ib) ;

	return current;
}

