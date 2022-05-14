/*
 * i2c.c
 *
 *  Created on: 14 May 2022
 *      Author: dfmol
 */

#include "main.h"
#include "i2c.h"
#include "uart.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

#define CurrentSensor_Address 			0x80
#define ConfigureRegister_Address 		0x00
#define ShuntVoltage_Address			0x01
#define Configuration_Byte1				0x39
#define Configuration_Byte2				0x9F

CurrentStateType CurrentState;

uint8_t i2cdata[10];
HAL_StatusTypeDef res;

void Init_Current_Sensor()
{
	i2cdata[0] = ConfigureRegister_Address;
	i2cdata[1] = Configuration_Byte1;
	i2cdata[2] = Configuration_Byte2;
	res = HAL_I2C_Master_Transmit(&hi2c1, CurrentSensor_Address, i2cdata, 3, 10);
	if(res != HAL_OK)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Configuration Transmit Error!\n", 34, 10);
		HAL_UART_Receive_IT(&huart2, UartState.rx_byte, 1);
	}
}

void Read_Current()
{

		// Set Pointer to Shunt Voltage Register
		i2cdata[0] = ShuntVoltage_Address;
		res = HAL_I2C_Master_Transmit(&hi2c1, CurrentSensor_Address, i2cdata, 1, 10);
		if(res != HAL_OK)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Transmit Pointer Change Error!\n", 35, 10);
			HAL_UART_Receive_IT(&huart2, UartState.rx_byte, 1);
		}

		// Read Current
		uint8_t bytes[2] = {0};
		res = HAL_I2C_Master_Receive(&hi2c1, CurrentSensor_Address, bytes, 2, 10);
		if(res != HAL_OK)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Receive Error!\n", 19, 10);
			HAL_UART_Receive_IT(&huart2, UartState.rx_byte, 1);
		}

		int number;
		uint8_t upper = bytes[0];
		uint8_t lower = bytes[1];
		if(bytes[0] & (1 << 7))
		{
			// Two's complement Negative
			upper = ~upper;
			lower = ~lower;
			number = -1 * (((upper << 8) | lower) + 1);
			number = 0;
		}
		else
		{
			number = (upper << 8) | lower;
		}
		CurrentState.Offset = number*100; // uA
}

