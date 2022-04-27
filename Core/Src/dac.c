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
extern DAC_HandleTypeDef hdac1;

OutputStateType OutputState;

void DAC_Calculate_Buffer()
{
	if(OutputState.Mode == d)
	{
		for(int i=0; i<100; i++)
		{
			OutputState.Buffer[i] = OutputState.Offset * (4096/3300) * (1000 / 920) / 2;
		}
	}
	else if (OutputState.Mode == s)
	{
		for(int i=0; i<100; i++)
		{
			OutputState.Buffer[i] = (float)((((float)OutputState.Amplitude/2)*sin(i*2*PI/100) + (float)OutputState.Offset)*(4096/3300) * (float)(1000 / 550) / 2);
		}
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

void DAC_Start()
{
	if(OutputState.Mode != p)
	{
		DAC_Calculate_Buffer();
		DAC_Set_Output_Frequency();
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, OutputState.Buffer, 100, DAC_ALIGN_12B_R);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		OutputState.On = true;
	}
}

void DAC_Stop()
{
	OutputState.On = false;
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
}

void DAC_Update_Output()
{
	DAC_Stop();
	DAC_Start();
}
