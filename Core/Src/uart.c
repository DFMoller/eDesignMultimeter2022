/*
 * uart.c
 *
 *  Created on: Apr 26, 2022
 *      Author: dfmol
 */

#include "main.h"
#include "uart.h"
#include "dac.h"
#include "adc.h"
#include "lcd.h"
#include "i2c.h"

extern UART_HandleTypeDef huart2;

UartStructType UartState;

void UART_Interpret_Rx_Message()
{
	for(int i = 0; i < 10; i++){
		UartState.rx_bytes_copy[i] = UartState.rx_bytes[i];
	}
	if(UartState.rx_bytes_length > 7)
	{
		if(UartState.rx_bytes_copy[2] == '*')
		{
			// Requests
			switch(UartState.rx_bytes_copy[4])
			{
				case 'm':
					// Request Measurement
					UART_Request_Measurement(UartState.rx_bytes_copy[6]);
					break;
				case 's':
					// Request Status
					if(UartState.rx_bytes_copy[6] == '1'){
						if(!OutputState.On) DAC_Start();
					}
					else if(UartState.rx_bytes_copy[6] == '0'){
						if(OutputState.On) DAC_Stop();
					}
					UART_Request_Status();
					LCD_changeDisplayMode(Measurement);
					break;
				default:
					// Problems
					break;
			}
		}
		else if(UartState.rx_bytes_copy[2] == '$'){
			// Set Measurement Mode
			UART_Set_Measurement_Mode(UartState.rx_bytes_copy[4], UartState.rx_bytes_copy[5]);
		}else if(UartState.rx_bytes_copy[2] == '^'){
			// Set output Parameter
			UART_Set_Output_Parameter(UartState.rx_bytes_copy, UartState.rx_bytes_length);
		}else if(UartState.rx_bytes_copy[2] == '#'){
			// Display on LCD
			UART_Display_On_LCD(UartState.rx_bytes_copy[4], UartState.rx_bytes_copy[6]);
		}
	}
}

void UART_Display_On_LCD(uint8_t rs, uint8_t byte)
{
	if(rs == '1'){
		// Set print flag; store rs and byte
		DisplayState.PrintFlag = 1;
		DisplayState.PrintByte = byte;
	}else if(rs == '0'){
		// Instruction
		LCD_Write_Instruction(byte);
		Delay_us_10(200); // 2ms
	}
}

void UART_Request_Measurement(uint8_t parameter)
{
	uint8_t msg[13] = "@,m,x,xxxx,!\n";
	uint16_t a = 0;
	uint16_t o = 0;
	uint16_t f = 0;
	if(MeasurementState.Mode == DV || MeasurementState.Mode == AV)
	{
		a = MeasurementState.Amplitude;
		o = MeasurementState.Offset;
		f = MeasurementState.Frequency;
	} else if (MeasurementState.Mode == DI || MeasurementState.Mode == AI) {
		a = CurrentState.Amplitude/1000;
		o = CurrentState.Offset/1000;
		f = CurrentState.Frequency;
	}
	switch(parameter){
		case 'a':
			// Amplitude (peak-to-peak)
			msg[4] = 'a';
			msg[6] = ((a/1000) % 10) + 48;
			msg[7] = ((a/100) % 10) + 48;
			msg[8] = ((a/10) % 10) + 48;
			msg[9] = (a % 10) + 48;
			break;
		case 'o':
			// Offset
			msg[4] = 'o';
			msg[6] = ((o/1000) % 10) + 48;
			msg[7] = ((o/100) % 10) + 48;
			msg[8] = ((o/10) % 10) + 48;
			msg[9] = (o % 10) + 48;
			break;
		case 'f':
			// Frequency
			msg[4] = 'f';
			msg[6] = ((f/1000) % 10) + 48;
			msg[7] = ((f/100) % 10) + 48;
			msg[8] = ((f/10) % 10) + 48;
			msg[9] = (f % 10) + 48;
			break;
		default:
			// Problems
			break;
	}
	HAL_UART_Transmit(&huart2, msg, 13, 10);
	HAL_UART_Receive_IT(&huart2, UartState.rx_byte, 1);
}

void UART_Request_Status()
{
	uint8_t msg[11] = "@,xx,x,x,!\n";
	switch(MeasurementState.Mode){
		case DV:
			// DV
			msg[2] = 'D';
			msg[3] = 'V';
			break;
		case AV:
			// AV
			msg[2] = 'A';
			msg[3] = 'V';
			break;
		case DI:
			// DI
			msg[2] = 'D';
			msg[3] = 'I';
			break;
		case AI:
			// AI
			msg[2] = 'A';
			msg[3] = 'I';
			break;
		default:
			// Problems
			break;
	}
	msg[5] = OutputState.Mode;
	if(OutputState.On){
		msg[7] = '1';
	} else {
		msg[7] = '0';
	}
	HAL_UART_Transmit(&huart2, msg, 11, 10);
	HAL_UART_Receive_IT(&huart2, UartState.rx_byte, 1);

}

void UART_Set_Measurement_Mode(uint8_t key1, uint8_t key2){
	if(key1 == 'D' && key2 == 'V'){
		// DC Voltage
		MeasurementState.Mode = DV;
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	} else if (key1 == 'A' && key2 == 'V'){
		// AC Voltage
		MeasurementState.Mode = AV;
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	} else if (key1 == 'D' && key2 == 'I'){
		// DC Current
		MeasurementState.Mode = DI;
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	} else if (key1 == 'A' && key2 == 'I'){
		// AC Current
		MeasurementState.Mode = AI;
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
	}
//	DisplayState.DisplayMeasurementsFlag = true;
	LCD_changeDisplayMode(Measurement);
}

void UART_Set_Output_Parameter(uint8_t *rx_array, uint8_t length)
{
	uint8_t param = rx_array[4];
	uint8_t val0 = rx_array[6];
	uint16_t received_value = 0;
	if(rx_array[7] != ','){
		val0 = rx_array[6] - 48;
		uint8_t val1 = rx_array[7] - 48;
		uint8_t val2 = rx_array[8] - 48;
		uint8_t val3 = rx_array[9] - 48;
		received_value += val0*1000;
		received_value += val1*100;
		received_value += val2*10;
		received_value += val3;
	}
	switch(param){
		case 't':
			// Type
			OutputState.Mode = val0;
			break;
		case 'a':
			// Amplitude
			OutputState.Amplitude = received_value;
			break;
		case 'o':
			// Offset
			OutputState.Offset = received_value;
			break;
		case 'f':
			// Frequency
			OutputState.Frequency = received_value;
			break;
		case 'd':
			// Duty Cycle
			OutputState.DutyCycle = received_value;
			break;
		case 'c':
			// Temperature
			break;
		default:
			// Problems
			break;
	}
	LCD_changeDisplayMode(Measurement);
	DAC_Update_Output();
}
