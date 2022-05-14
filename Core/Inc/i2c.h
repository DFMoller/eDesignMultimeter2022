/*
 * i2c.h
 *
 *  Created on: 14 May 2022
 *      Author: dfmol
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

typedef struct CurrentTemplate{
	uint8_t Measure_Flag;
	uint16_t Amplitude;
	uint16_t Frequency;
	uint16_t Period;
	uint16_t Offset; // uA
} CurrentStateType;

extern CurrentStateType CurrentState;


void Init_Current_Sensor();
void Read_Current();



#endif /* INC_I2C_H_ */
