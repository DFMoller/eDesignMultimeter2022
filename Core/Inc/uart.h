/*
 * uart.h
 *
 *  Created on: Apr 26, 2022
 *      Author: dfmol
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdbool.h>

extern uint8_t rx_byte[1];

void UART_Main_Function();
bool UART_Rx_Complete(uint8_t last_byte);
void UART_Interpret_Rx_Message(uint8_t *rx_array, uint8_t length);
void UART_Request_Measurement(uint8_t parameter);
void UART_Request_Status();
void UART_Set_Output_Parameter(uint8_t *rx_array, uint8_t length);

#endif /* INC_UART_H_ */
