/*
 * dac.c
 *
 *  Created on: 25 Apr 2022
 *      Author: dfmol
 */
#include "main.h"
#include "dac.h"
#include "math.h"
#include <stdint.h>

#define PI 3.1415926

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

OutputStateType OutputState;

void DAC_Calculate_Sine_Buffer()
{
	for(int i=0; i<100; i++)
	{
		OutputState.SineBuffer[i] = ((OutputState.Amplitude/2)*sin(i*2*PI/100) + OutputState.Offset)*(4096/3300);
	}
}

void DAC_Set_Output_Frequency()
{
	OutputState.ARR_Val = OutputState.TIM2_Clock / (OutputState.Frequency*100);
	__HAL_TIM_SET_AUTORELOAD(&htim2, OutputState.ARR_Val);
	TIM2->CR1 = 0;				// 	Disable Timer
	TIM2->EGR = TIM_EGR_UG;		//	Init registers
	TIM2->CR1 = 1;				// 	Start
}
