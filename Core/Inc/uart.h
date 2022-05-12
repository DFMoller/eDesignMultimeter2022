/*
 * uart.h
 *
 *  Created on: Apr 26, 2022
 *      Author: dfmol
 */

#ifndef INC_UART_H_
#define INC_UART_H_

typedef struct UartStructTemplate{
	uint8_t rx_byte[1];
	uint8_t rx_bytes[10];
	uint8_t rx_bytes_counter;
	uint8_t rx_bytes_length;
	uint8_t message_received;
} UartStructType;

extern UartStructType UartState;

void UART_Interpret_Rx_Message();
void UART_Request_Measurement(uint8_t parameter);
void UART_Request_Status();
void UART_Set_Measurement_Mode(uint8_t key1, uint8_t key2);
void UART_Set_Output_Parameter(uint8_t *rx_array, uint8_t length);
void UART_Display_On_LCD(uint8_t rs, uint8_t byte);
void DAC_Update_Output();

#endif /* INC_UART_H_ */
