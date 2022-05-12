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
	  // 1000 measurements at 5kHz take 200ms
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
	//			  measured_offset = total/1000;
//	  MeasurementState.Offset = 1000;
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
	  MeasurementState.Period = 50000/(mid_passes);
	//			  MeasurementState.Frequency = 1000000/measured_period;
//	  MeasurementState.Frequency = 5250;
	//			  MeasurementState.Amplitude = max - min;
//	  MeasurementState.Amplitude = 500;
	//			  sprintf(msg, "Max: %u\nMin: %u\nOffset: %u\nFrequency: %u\nAmplitude: %u\n\n", max, min, offset, frequency, amplitude);
	//			  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
		if(DisplayState.Mode == Measurement)
		{
//			LCD_Display_Measurement();
		}
	}
	else
	{
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  raw = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);
	  millivolts = raw*3300/4095;
	  millivolts += 100*millivolts/1000; // Calibration
	  adc_array[adc_count] = millivolts;
	  adc_count++;
	}

//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
}

