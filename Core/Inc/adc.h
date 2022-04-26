/*
 * adc.h
 *
 *  Created on: Apr 26, 2022
 *      Author: dfmol
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

typedef enum MeasurementModes {DV, DI, AV, AI, TC} MeasurementMode;

typedef struct MeasurementTemplate{
	enum MeasurementModes Mode;
	uint16_t Amplitude;
	uint16_t Frequency;
	uint16_t Period;
	uint16_t Offset;
} MeasurementStateType;

extern MeasurementStateType MeasurementState;

void ADC_Main_Function();

#endif /* INC_ADC_H_ */
