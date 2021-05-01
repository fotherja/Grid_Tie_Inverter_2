#ifndef _SUPPORT_H_
#define _SUPPORT_H_


#include <stdint.h>
#include <stdbool.h>

int32_t 	Integral(int32_t datum);
int32_t 	Get_Median(int16_t *Buffer, int8_t FILTER_LENGTH, int16_t OFFSET);
uint32_t 	Integrate_Mains_RMS(int16_t ADC_Line_V);
uint32_t 	Integrate_I_Error(int32_t I_Error);

#endif




