/*
 * lcd.c
 *
 *  Created on: 25 Apr 2022
 *      Author: dfmol
 */

#include "main.h"
#include "lcd.h"

extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart2;

#define lcd_instruction_ClearDisplay    	0b00000001      // replace all characters with ASCII 'space'
#define lcd_instruction_EntryMode       	0b00000110      // shift cursor from left to right on read/write
#define lcd_instruction_DisplayOn      		0b00001111      // Display On, Cursor On, Cursor Blinking On (0b00001DCB)
#define lcd_instruction_FunctionReset   	0b00110000      // reset the LCD
#define lcd_instruction_FunctionSet4bit 	0b00101000      // 4-bit data, 2-line display, 5 x 7 font
#define lcd_instruction_DisplayShiftRight	0b00011100
#define lcd_instruction_DisplayShiftLeft	0b00011000
#define lcd_instruction_CursorShiftRight	0b00010100
#define lcd_instruction_CursorShiftLeft		0b00010000
#define lcd_RS_bit			13
#define lcd_E_bit			15
#define lcd_D4_bit			6
#define lcd_D5_bit			8
#define lcd_D6_bit			11
#define lcd_D7_bit			12

void LCD_Init()
{

	HAL_UART_Transmit(&huart2, (uint8_t*)"\nStart of Init Function:\n", 25, 10);

	HAL_Delay(20);

	LCD_Write_8bitInstruction(lcd_instruction_FunctionReset);
	HAL_Delay(10);

	LCD_Write_8bitInstruction(lcd_instruction_FunctionReset);
	HAL_Delay(1);

	LCD_Write_8bitInstruction(lcd_instruction_FunctionReset);
	HAL_Delay(10);

	LCD_Write_8bitInstruction(lcd_instruction_FunctionSet4bit);
	HAL_Delay(1);

	LCD_Write_Instruction(lcd_instruction_FunctionSet4bit);
	HAL_Delay(1);

	LCD_Write_Instruction(lcd_instruction_DisplayOn);
	HAL_Delay(1);

	LCD_Write_Instruction(lcd_instruction_ClearDisplay);
	HAL_Delay(3);

	LCD_Write_Instruction(lcd_instruction_EntryMode);
	HAL_Delay(1);

	HAL_UART_Transmit(&huart2, (uint8_t*)"\nEnd of Init Function:\n", 23, 10);

}

void LCD_Write_String(uint8_t string[])
{
	int i = 0;
	while (string[i] != 0)
	{
		LCD_Write_Character(string[i]);
		i++;
		HAL_Delay(1);
	}
}

void LCD_Write_Character(uint8_t character)
{
	lcd_RS_GPIO_Port->ODR |= (1<<lcd_RS_bit);			// select data register (RS High)
	LCD_Write_Nibbles(character);
}

void LCD_Write_Instruction(uint8_t instruction)
{
	lcd_RS_GPIO_Port->ODR &= ~(1<<lcd_RS_bit);			// select the Instruction Register (RS low)
	LCD_Write_Nibbles(instruction);
}

void LCD_Write_Nibbles(uint8_t byte)
{
	lcd_E_GPIO_Port->ODR |= (1<<lcd_E_bit);			// set E high
	LCD_ZeroPins();

	// Write first (most significant) nibble
	if(byte & 1<<7)	lcd_D7_GPIO_Port->ODR |= (1<<lcd_D7_bit);
	if(byte & 1<<6)	lcd_D6_GPIO_Port->ODR |= (1<<lcd_D6_bit);
	if(byte & 1<<5)	lcd_D5_GPIO_Port->ODR |= (1<<lcd_D5_bit);
	if(byte & 1<<4)	lcd_D4_GPIO_Port->ODR |= (1<<lcd_D4_bit);


	// Pulse Enable
	HAL_Delay(1);
	lcd_E_GPIO_Port->ODR &= ~(1<<lcd_E_bit);			// Set to 0
	lcd_E_GPIO_Port->ODR |= (1<<lcd_E_bit);				// Set to 1

	LCD_ZeroPins();
	if(byte & 1<<3)	lcd_D7_GPIO_Port->ODR |= (1<<lcd_D7_bit);
	if(byte & 1<<2)	lcd_D6_GPIO_Port->ODR |= (1<<lcd_D6_bit);
	if(byte & 1<<1)	lcd_D5_GPIO_Port->ODR |= (1<<lcd_D5_bit);
	if(byte & 1<<0)	lcd_D4_GPIO_Port->ODR |= (1<<lcd_D4_bit);

	// Drop Enable
	HAL_Delay(1);
	lcd_E_GPIO_Port->ODR &= ~(1<<lcd_E_bit);		// Set to 0
}

void LCD_Write_8bitInstruction(uint8_t byte)
{
	lcd_RS_GPIO_Port->ODR &= ~(1<<lcd_RS_bit);			// Set RS to 0
	lcd_E_GPIO_Port->ODR |= (1<<lcd_E_bit);				// Set E to 1
	LCD_ZeroPins();

	 // Set to zero first
	if(byte & 1<<7)	lcd_D7_GPIO_Port->ODR |= (1<<lcd_D7_bit);
	if(byte & 1<<6)	lcd_D6_GPIO_Port->ODR |= (1<<lcd_D6_bit);
	if(byte & 1<<5)	lcd_D5_GPIO_Port->ODR |= (1<<lcd_D5_bit);
	if(byte & 1<<4)	lcd_D4_GPIO_Port->ODR |= (1<<lcd_D4_bit);

	HAL_Delay(1);
	lcd_E_GPIO_Port->ODR &= ~(1<<lcd_E_bit);		// Set E to 0
}

void LCD_ZeroPins()
{
	lcd_D7_GPIO_Port->ODR &= ~(1<<lcd_D7_bit);
	lcd_D6_GPIO_Port->ODR &= ~(1<<lcd_D6_bit);
	lcd_D5_GPIO_Port->ODR &= ~(1<<lcd_D5_bit);
	lcd_D4_GPIO_Port->ODR &= ~(1<<lcd_D4_bit);
}

void LCD_Clear_Display()
{
	LCD_Write_Instruction(lcd_instruction_ClearDisplay);
}


