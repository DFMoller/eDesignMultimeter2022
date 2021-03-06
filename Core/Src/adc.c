/*
 * adc.c
 *
 *  Created on: Apr 26, 2022
 *      Author: dfmol
 */


#include "main.h"
#include "adc.h"
#include "lcd.h"

extern ADC_HandleTypeDef hadc1;

MeasurementStateType MeasurementState;

uint16_t raw;
uint16_t millivolts;
uint16_t adc_array[1000];
uint16_t adc_count = 0;

void ADC_Main_Function()
{
	if(adc_count > 999)
	{
		// Do calculations every 1000 readings
		adc_count = 0;
		uint32_t total = 0;
		uint16_t max = 0;
		uint16_t min = adc_array[99]; // arbitrary value
		int16_t diff = 0;
		int16_t prev_diff = 0;
		uint16_t mid_passes = 0;
		// 1000 measurements at 20kHz take 50ms
		for(int x = 0; x < 1000; x++)
		{
			total += adc_array[x];
			if(adc_array[x] > max)
			{
			  max = adc_array[x];
			}
			else if(adc_array[x] < min)
			{
			  min = adc_array[x];
			}
		}
		MeasurementState.Offset = total/1000;
		for(int x = 0; x < 1000; x++)
		{
			// Calculate frequency
			diff = adc_array[x] - MeasurementState.Offset;
			if(diff > 0 && prev_diff < 0)
			{
			  mid_passes++;
			}
			prev_diff = diff;
		}
		MeasurementState.Period = 50000/(mid_passes/2); // us
		MeasurementState.Frequency = 1000000/MeasurementState.Period;
		MeasurementState.Amplitude = max - min;
	}
	else
	{

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		raw = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		millivolts = (float)((float)raw*3300.0/4095.0);
		if(millivolts > 50)
		{
			millivolts += (float)((110.0 + 137.0*((float)millivolts - 100.0)/(2000.0 - 100.0)));
		}
		adc_array[adc_count] = millivolts;
		adc_count++;
	}
}

