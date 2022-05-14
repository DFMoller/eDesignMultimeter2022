/*
 * lcd.c
 *
 *  Created on: 25 Apr 2022
 *      Author: dfmol
 */

#include "main.h"
#include "lcd.h"
#include "adc.h"
#include "dac.h"
#include "i2c.h"

extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart2;

#define lcd_instruction_ClearDisplay    	0b00000001      // replace all characters with ASCII 'space'
#define lcd_instruction_ReturnHome			0b00000010
#define lcd_instruction_CursorHome			0b10000000
#define lcd_instruction_EntryMode       	0b00000110      // shift cursor from left to right on read/write
#define lcd_instruction_DisplayOn      		0b00001111      // Display On, Cursor On, Cursor Blinking On (0b00001DCB)
#define lcd_instruction_FunctionReset   	0b00110000      // reset the LCD
#define lcd_instruction_FunctionSet4bit 	0b00101000      // 4-bit data, 2-line display, 5 x 7 font
#define lcd_instruction_DisplayShiftRight	0b00011100
#define lcd_instruction_DisplayShiftLeft	0b00011000
#define lcd_instruction_CursorShiftRight	0b00010100
#define lcd_instruction_CursorShiftLeft		0b00010000
#define lcd_instruction_CursorNewLine		0b11000000
#define lcd_RS_bit			13
#define lcd_E_bit			15
#define lcd_D4_bit			6
#define lcd_D5_bit			8
#define lcd_D6_bit			11
#define lcd_D7_bit			12

DisplayStateType DisplayState;

void LCD_Init()
{
	HAL_Delay(20);

	LCD_Write_8bitInstruction(lcd_instruction_FunctionReset);
	HAL_Delay(5);

	LCD_Write_8bitInstruction(lcd_instruction_FunctionReset);
	Delay_us_10(11);

	LCD_Write_8bitInstruction(lcd_instruction_FunctionReset);
	Delay_us_10(5);

	LCD_Write_8bitInstruction(lcd_instruction_FunctionSet4bit);
	Delay_us_10(5);

	LCD_Write_Instruction(lcd_instruction_FunctionSet4bit);
	Delay_us_10(5);

	LCD_Write_Instruction(lcd_instruction_DisplayOn);
	Delay_us_10(5);

	LCD_Write_Instruction(lcd_instruction_ClearDisplay);
	HAL_Delay(2);

	LCD_Write_Instruction(lcd_instruction_EntryMode);
	Delay_us_10(5);
}

void LCD_Write_String(uint8_t string[])
{
//	HAL_Delay(1);
	int i = 0;
	while (string[i] != 0)
	{
		LCD_Write_Character(string[i]);
		i++;
//		HAL_Delay(1);
	}
}

void LCD_Write_Character_Shift(uint8_t character)
{
	lcd_RS_GPIO_Port->ODR |= (1<<lcd_RS_bit);			// select data register (RS High)
	LCD_Write_Nibbles(character);
	if(DisplayState.CurrentLine == Topline){
		DisplayState.ToplineCharacters ++;
	} else if (DisplayState.CurrentLine == Bottomline){
		DisplayState.BottomlineCharacters ++;
	}
	if((DisplayState.CurrentLine == Topline && DisplayState.ToplineCharacters > 15)
			|| (DisplayState.CurrentLine == Bottomline && DisplayState.BottomlineCharacters > 15))
	{
		// Scroll Screen
		LCD_Shift_Left();
	}
}

void LCD_Write_Character(uint8_t character)
{
	lcd_RS_GPIO_Port->ODR |= (1<<lcd_RS_bit);			// select data register (RS High)
	LCD_Write_Nibbles(character);
	if(DisplayState.CurrentLine == Topline){
		DisplayState.ToplineCharacters ++;
	} else if (DisplayState.CurrentLine == Bottomline){
		DisplayState.BottomlineCharacters ++;
	}
}

void LCD_Write_Instruction(uint8_t instruction)
{
	lcd_RS_GPIO_Port->ODR &= ~(1<<lcd_RS_bit);			// select the Instruction Register (RS low)
	LCD_Write_Nibbles(instruction);
}

void LCD_Write_Nibbles(uint8_t byte)
{
	Delay_us_10(1); // tsu1 > 40ns
	lcd_E_GPIO_Port->ODR |= (1<<lcd_E_bit);			// set E high
	LCD_ZeroPins();

	// Write first (most significant) nibble
	if(byte & 1<<7)	lcd_D7_GPIO_Port->ODR |= (1<<lcd_D7_bit);
	if(byte & 1<<6)	lcd_D6_GPIO_Port->ODR |= (1<<lcd_D6_bit);
	if(byte & 1<<5)	lcd_D5_GPIO_Port->ODR |= (1<<lcd_D5_bit);
	if(byte & 1<<4)	lcd_D4_GPIO_Port->ODR |= (1<<lcd_D4_bit);


	// Pulse Enable
//	HAL_Delay(1);
	Delay_us_10(10);
	lcd_E_GPIO_Port->ODR &= ~(1<<lcd_E_bit);			// Set to 0
	lcd_E_GPIO_Port->ODR |= (1<<lcd_E_bit);				// Set to 1

	LCD_ZeroPins();
	if(byte & 1<<3)	lcd_D7_GPIO_Port->ODR |= (1<<lcd_D7_bit);
	if(byte & 1<<2)	lcd_D6_GPIO_Port->ODR |= (1<<lcd_D6_bit);
	if(byte & 1<<1)	lcd_D5_GPIO_Port->ODR |= (1<<lcd_D5_bit);
	if(byte & 1<<0)	lcd_D4_GPIO_Port->ODR |= (1<<lcd_D4_bit);

	// Drop Enable
//	HAL_Delay(1);
	Delay_us_10(10);
	lcd_E_GPIO_Port->ODR &= ~(1<<lcd_E_bit);		// Set to 0
}

void LCD_Write_8bitInstruction(uint8_t byte)
{
	lcd_RS_GPIO_Port->ODR &= ~(1<<lcd_RS_bit);			// Set RS to 0
	Delay_us_10(1); // tsu1 > 40ns
	lcd_E_GPIO_Port->ODR |= (1<<lcd_E_bit);				// Set E to 1
	LCD_ZeroPins();

	// Set to zero first
	if(byte & 1<<7)	lcd_D7_GPIO_Port->ODR |= (1<<lcd_D7_bit);
	if(byte & 1<<6)	lcd_D6_GPIO_Port->ODR |= (1<<lcd_D6_bit);
	if(byte & 1<<5)	lcd_D5_GPIO_Port->ODR |= (1<<lcd_D5_bit);
	if(byte & 1<<4)	lcd_D4_GPIO_Port->ODR |= (1<<lcd_D4_bit);

//	HAL_Delay(1);
	Delay_us_10(1);
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
	LCD_Write_Instruction(lcd_instruction_ReturnHome);
	Delay_us_10(200); // 2ms
	LCD_Write_Instruction(lcd_instruction_ClearDisplay);
	Delay_us_10(200); // 2ms
	DisplayState.ToplineCharacters = 0;
	DisplayState.BottomlineCharacters = 0;
	DisplayState.CurrentLine = Topline;
	DisplayState.DisplayPosition = 0;
}

void LCD_NewLine()
{
	LCD_Write_Instruction(lcd_instruction_CursorNewLine);
	Delay_us_10(5);
	DisplayState.CurrentLine = Bottomline;
}

void LCD_AutoScroll()
{
	uint8_t returnflag = DisplayState.DisplayPosition + 12;
	uint8_t longestline;
	if(DisplayState.ToplineCharacters >= DisplayState.BottomlineCharacters) longestline = DisplayState.ToplineCharacters;
	else longestline = DisplayState.BottomlineCharacters;
	if(longestline > 16)
	{
		if(returnflag > longestline)
		{
			LCD_Shift_Home();
		}
		else
		{
			LCD_Shift_Left();
		}
	}
}

void LCD_Shift_Left()
{
	LCD_Write_Instruction(lcd_instruction_DisplayShiftLeft);
	Delay_us_10(5);
	DisplayState.DisplayPosition ++;
}

void LCD_Shift_Right()
{
	LCD_Write_Instruction(lcd_instruction_DisplayShiftRight);
	Delay_us_10(5);
	DisplayState.DisplayPosition -= 1;
}

void LCD_Shift_Home()
{
	uint8_t num_shifted = DisplayState.DisplayPosition;
	for(int i = 0; i < num_shifted; i++)
	{
		LCD_Shift_Right();
	}
}

void LCD_Cursor_Home()
{
	LCD_Write_Instruction(lcd_instruction_CursorHome);
	Delay_us_10(5);
	DisplayState.ToplineCharacters = 0;
	DisplayState.BottomlineCharacters = 0;
	DisplayState.CurrentLine = Topline;
}

void LCD_changeDisplayMode(DisplayMode newDisplayMode)
{
	LCD_Clear_Display();
	if (newDisplayMode == Menu)
	{
		// Change to Menu Display State
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		LCD_Display_Menu();
		DisplayState.LastMode = DisplayState.Mode;
		DisplayState.Mode = Menu;
	}
	else if (newDisplayMode == Measurement)
	{
		// Change to Measurement Display State
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
		LCD_Display_Measurement();
		DisplayState.LastMode = DisplayState.Mode;
		DisplayState.Mode = Measurement;
	}
	else if (newDisplayMode == Output)
	{
		// Change to Output Display State
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		DisplayState.LastMode = DisplayState.Mode;
		DisplayState.Mode = Output;
	}
}

void LCD_Display_Menu()
{
	uint8_t lcd_string[] = "Menu Top Level";
	LCD_Write_String(lcd_string);
}

void LCD_Display_Measurement()
{
//	LCD_Clear_Display();
	LCD_Cursor_Home();
	switch(MeasurementState.Mode)
	{
		case DV:
		{
			uint8_t topline[] = "x.xxxV";
			topline[0] = ((MeasurementState.Offset/1000) % 10) + 48;
			topline[2] = ((MeasurementState.Offset/100) % 10) + 48;
			topline[3] = ((MeasurementState.Offset/10) % 10) + 48;
			topline[4] = ((MeasurementState.Offset) % 10) + 48;
			LCD_Write_String(topline);
			break;
		}
		case DI:
		{
			uint8_t topline[] = "xxx.xmA";
			topline[0] = ((CurrentState.Offset/100000) % 10) + 48;
			topline[1] = ((CurrentState.Offset/10000) % 10) + 48;
			topline[2] = ((CurrentState.Offset/1000) % 10) + 48;
			topline[4] = ((CurrentState.Offset/100) % 10) + 48;
			LCD_Write_String(topline);
			break;
		}
		case AV:
		{
			uint8_t topline[] = "O:x.xxxV,A:x.xxxV,F:xxxxHz";
			topline[2] = ((MeasurementState.Offset/1000) % 10) + 48;
			topline[4] = ((MeasurementState.Offset/100) % 10) + 48;
			topline[5] = ((MeasurementState.Offset/10) % 10) + 48;
			topline[6] = ((MeasurementState.Offset) % 10) + 48;
			topline[11] = ((MeasurementState.Amplitude/1000) % 10) + 48;
			topline[13] = ((MeasurementState.Amplitude/100) % 10) + 48;
			topline[14] = ((MeasurementState.Amplitude/10) % 10) + 48;
			topline[15] = ((MeasurementState.Amplitude) % 10) + 48;
			topline[20] = ((MeasurementState.Frequency/1000) % 10) + 48;
			topline[21] = ((MeasurementState.Frequency/100) % 10) + 48;
			topline[22] = ((MeasurementState.Frequency/10) % 10) + 48;
			topline[23] = ((MeasurementState.Frequency) % 10) + 48;
			LCD_Write_String(topline);
			break;
		}
		case AI:
		{
			uint8_t topline[] = "AC Current";
			LCD_Write_String(topline);
			break;
		}
		default:
			// Problems
			break;
	}
	LCD_NewLine();
	if(OutputState.On){
		if(OutputState.Mode == d){
			uint8_t bottomline[] = "x.xxxV";
			bottomline[0] = ((OutputState.Offset/1000) % 10) + 48;
			bottomline[2] = ((OutputState.Offset/100) % 10) + 48;
			bottomline[3] = ((OutputState.Offset/10) % 10) + 48;
			bottomline[4] = ((OutputState.Offset) % 10) + 48;
			LCD_Write_String(bottomline);
		} else if (OutputState.Mode == s){
			uint8_t bottomline[] = "O:x.xxxV,A:x.xxxV,F:xxxxHz";
			bottomline[2] = ((OutputState.Offset/1000) % 10) + 48;
			bottomline[4] = ((OutputState.Offset/100) % 10) + 48;
			bottomline[5] = ((OutputState.Offset/10) % 10) + 48;
			bottomline[6] = ((OutputState.Offset) % 10) + 48;
			bottomline[11] = ((OutputState.Amplitude/1000) % 10) + 48;
			bottomline[13] = ((OutputState.Amplitude/100) % 10) + 48;
			bottomline[14] = ((OutputState.Amplitude/10) % 10) + 48;
			bottomline[15] = ((OutputState.Amplitude) % 10) + 48;
			bottomline[20] = ((OutputState.Frequency/1000) % 10) + 48;
			bottomline[21] = ((OutputState.Frequency/100) % 10) + 48;
			bottomline[22] = ((OutputState.Frequency/10) % 10) + 48;
			bottomline[23] = ((OutputState.Frequency) % 10) + 48;
			LCD_Write_String(bottomline);
		} else if (OutputState.Mode == p){
			uint8_t bottomline[] = "O:x.xxxV,A:x.xxxV,F:xxxxHz,D:xxx%";
			bottomline[2] = ((OutputState.Offset/1000) % 10) + 48;
			bottomline[4] = ((OutputState.Offset/100) % 10) + 48;
			bottomline[5] = ((OutputState.Offset/10) % 10) + 48;
			bottomline[6] = ((OutputState.Offset) % 10) + 48;
			bottomline[11] = ((OutputState.Amplitude/1000) % 10) + 48;
			bottomline[13] = ((OutputState.Amplitude/100) % 10) + 48;
			bottomline[14] = ((OutputState.Amplitude/10) % 10) + 48;
			bottomline[15] = ((OutputState.Amplitude) % 10) + 48;
			bottomline[20] = ((OutputState.Frequency/1000) % 10) + 48;
			bottomline[21] = ((OutputState.Frequency/100) % 10) + 48;
			bottomline[22] = ((OutputState.Frequency/10) % 10) + 48;
			bottomline[23] = ((OutputState.Frequency) % 10) + 48;
			bottomline[29] = ((OutputState.DutyCycle/100) % 10) + 48;
			bottomline[30] = ((OutputState.DutyCycle/10) % 10) + 48;
			bottomline[31] = ((OutputState.DutyCycle) % 10) + 48;
			LCD_Write_String(bottomline);
		}
	} else {
		uint8_t bottomline[] = "OUTPUT OFF";
		LCD_Write_String(bottomline);
	}
}


