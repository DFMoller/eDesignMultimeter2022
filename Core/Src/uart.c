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

extern UART_HandleTypeDef huart2;

uint8_t rx_byte[1];
uint8_t rx_bytes[10] = {0};
uint8_t rx_bytes_counter = 0;

void UART_Main_Function()
{
	if(rx_byte[0] != '\n')
	{
	  rx_bytes[rx_bytes_counter] = rx_byte[0];
	  if(rx_bytes_counter == 0 && rx_byte[0] == '@'){
		  rx_bytes_counter++;
	  } else if(rx_bytes_counter > 0){
		  rx_bytes_counter++;
		  if(UART_Rx_Complete(rx_byte[0]))
		  {
			  UART_Interpret_Rx_Message(rx_bytes, rx_bytes_counter);
			  rx_bytes_counter = 0;
		  }
	  }
	  HAL_UART_Receive_IT(&huart2, rx_byte, 1);
	}
}

bool UART_Rx_Complete(uint8_t last_byte)
{
	if(last_byte == '!')
	{
		return true;
	}
	else
	{
		return false;
	}
}

void UART_Interpret_Rx_Message(uint8_t *rx_array, uint8_t length)
{

	if(length > 7)
	{
		if(rx_array[2] == '*')
		{
			// Requests
			switch(rx_array[4])
			{
				case 'm':
					// Request Measurement
					UART_Request_Measurement(rx_array[6]);
					break;
				case 's':
					// Request Status
					DAC_Switch_Output_OnOff(rx_array[6]);
					UART_Request_Status();
					break;
				default:
					// Problems
					break;
			}
		}
		else if(rx_array[2] == '$'){
			// Set Measurement Mode
			UART_Set_Measurement_Mode(rx_array[4], rx_array[5]);
		}else if(rx_array[2] == '^'){
			// Set output Parameter
			UART_Set_Output_Parameter(rx_array, length);
		}else if(rx_array[2] == '#'){
			// Display on LCD
			UART_Display_On_LCD(rx_array[4], rx_array[6]);
		}
	}
}

void UART_Display_On_LCD(uint8_t rs, uint8_t byte)
{
	if(rs == '1'){
		// Set print flag; store rs and byte
		DisplayState.PrintFlag = 1;
		DisplayState.PrintRS = 1;
		DisplayState.PrintByte = byte;
	}else if(rs == '0'){
		// Instruction
		LCD_Write_Instruction(byte);
	}
}

void UART_Request_Measurement(uint8_t parameter)
{
	uint8_t msg[13] = "@,m,x,xxxx,!\n";
	switch(parameter){
		case 't':
			// Type
			break;
		case 'a':
			// Amplitude (peak-to-peak)
			msg[4] = 'a';
			msg[6] = ((MeasurementState.Amplitude/1000) % 10) + 48;
			msg[7] = ((MeasurementState.Amplitude/100) % 10) + 48;
			msg[8] = ((MeasurementState.Amplitude/10) % 10) + 48;
			msg[9] = (MeasurementState.Amplitude % 10) + 48;
			break;
		case 'o':
			// Offset
			msg[4] = 'o';
			msg[6] = ((MeasurementState.Offset/1000) % 10) + 48;
			msg[7] = ((MeasurementState.Offset/100) % 10) + 48;
			msg[8] = ((MeasurementState.Offset/10) % 10) + 48;
			msg[9] = (MeasurementState.Offset % 10) + 48;
			break;
		case 'f':
			// Frequency
			msg[4] = 'f';
			msg[6] = ((MeasurementState.Frequency/1000) % 10) + 48;
			msg[7] = ((MeasurementState.Frequency/100) % 10) + 48;
			msg[8] = ((MeasurementState.Frequency/10) % 10) + 48;
			msg[9] = (MeasurementState.Frequency % 10) + 48;
			break;
		case 'd':
			// Duty Cycle
			break;
		case 'c':
			// Temperature
			break;
		default:
			// Problems
			break;
	}
	HAL_UART_Transmit(&huart2, msg, 13, 10);
	HAL_UART_Receive_IT(&huart2, rx_byte, 1);
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
		case TC:
			// TC
			msg[2] = 'T';
			msg[3] = 'C';
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
	HAL_UART_Receive_IT(&huart2, rx_byte, 1);

}

void UART_Set_Measurement_Mode(uint8_t key1, uint8_t key2){
	if(key1 == 'D' && key2 == 'V'){
		// DC Voltage
		MeasurementState.Mode = DV;
	} else if (key1 == 'A' && key2 == 'V'){
		// AC Voltage
		MeasurementState.Mode = AV;
	} else if (key1 == 'D' && key2 == 'I'){
		// DC Current
		MeasurementState.Mode = DI;
	} else if (key1 == 'A' && key2 == 'I'){
		// AC Current
		MeasurementState.Mode = AI;
	} else if (key1 == 'T' && key2 == 'C'){
		// Temperature
		MeasurementState.Mode = TC;
	}
}

void UART_Set_Output_Parameter(uint8_t *rx_array, uint8_t length)
{
	uint8_t param = rx_array[4];
	uint8_t val0 = rx_array[6];
	uint8_t received_value = 0;
	if(length > 9){
		uint8_t val1 = rx_array[7];
		uint8_t val2 = rx_array[8];
		uint8_t val3 = rx_array[9];
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
			break;
		case 'c':
			// Temperature
			break;
		default:
			// Problems
			break;
	}
}
