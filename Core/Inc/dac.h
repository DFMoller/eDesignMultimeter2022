/*
 * dac.h
 *
 *  Created on: Apr 23, 2022
 *      Author: dfmol
 */

#ifndef INC_DAC_H_
#define INC_DAC_H_

#include <stdbool.h>

typedef enum OutputModes{d='d', s='s', p='p'} OutputMode;

typedef struct OutputTemplate{
	uint32_t Buffer[100];
	uint32_t TIM2_Clock;
	bool On;
	enum OutputModes Mode;
	uint16_t Amplitude;
	uint16_t Frequency;
	uint16_t Offset;
	uint32_t ARR_Val;
	uint8_t DutyCycle;
	uint16_t Amplitude_Temp;
	uint16_t Offset_Temp;
	uint16_t Frequency_Temp;
	uint8_t DutyCycle_Temp;
} OutputStateType;

extern OutputStateType OutputState;

void DAC_Calculate_Buffer();
void DAC_Set_Output_Frequency();
void DAC_Start();
void DAC_Stop();
void DAC_Update_Output();

#endif /* INC_DAC_H_ */
