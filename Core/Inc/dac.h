/*
 * dac.h
 *
 *  Created on: Apr 23, 2022
 *      Author: dfmol
 */

#ifndef INC_DAC_H_
#define INC_DAC_H_

#include "math.h"

#define PI 3.1415926

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

uint32_t sine_buffer[100];
uint32_t timer2_clock = 72000000;
uint8_t output_active = 0;

uint8_t signal_active = 0;
uint8_t signal_type = 'd'; // 0 = d, 1 = s, 2 = p
uint16_t signal_amplitude = 0;
uint16_t signal_frequency = 0;
uint16_t signal_offset = 0;

void DAC_Calculate_Sine_Buffer()
{
	for(int i=0; i<100; i++)
	{
		sine_buffer[i] = ((signal_amplitude/2)*sin(i*2*PI/100) + signal_offset)*(4096/3300);
	}
}

void DAC_Set_Output_Frequency()
{
	uint32_t ARR_Val = timer2_clock / (signal_frequency*100);
	__HAL_TIM_SET_AUTORELOAD(&htim2, ARR_Val);
	TIM2->CR1 = 0;				// 	Disable Timer
	TIM2->EGR = TIM_EGR_UG;		//	Init registers
	TIM2->CR1 = 1;				// 	Start
}

#endif /* INC_DAC_H_ */
