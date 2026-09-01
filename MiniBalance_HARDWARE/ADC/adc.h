// ADC 通道宏定义
#define    ELE_ADC_L_CHANNEL					 ADC_Channel_4
#define    ELE_ADC_M_CHANNEL					 ADC_Channel_5
#define    ELE_ADC_R_CHANNEL					 ADC_Channel_15
#define    CCD_ADC_CHANNEL					 	 ADC_Channel_15

#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#define Battery_Ch 11
void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   
u16 Get_Adc1(u8 ch);
#endif 















