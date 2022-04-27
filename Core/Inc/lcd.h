/*
 * lcd.h
 *
 *  Created on: 22 Apr 2022
 *      Author: dfmol
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include <stdbool.h>

typedef enum DisplayStates {Menu, Measurement, Output} DisplayMode;
typedef enum DisplayLines {Topline, Bottomline} DisplayLine;

typedef struct DisplayTemplate{
	enum DisplayStates Mode;
	enum DisplayStates LastMode;
	enum DisplayLines CurrentLine;
	uint8_t ToplineCharacters;
	uint8_t BottomlineCharacters;
	uint8_t PrintFlag;
	uint8_t PrintByte;
	uint8_t RefreshFlag;
	uint8_t AutoScrollCounter;
	uint8_t DisplayPosition;
} DisplayStateType;

extern DisplayStateType DisplayState;

void LCD_Init();
void LCD_Write_Character(uint8_t character);
void LCD_Write_Character_Shift(uint8_t character);
void LCD_Write_Instruction(uint8_t instruction);
void LCD_Write_Nibbles(uint8_t byte);
void LCD_Write_8bitInstruction(uint8_t byte);
void LCD_ZeroPins();
void LCD_Write_String(uint8_t string[]);
void LCD_Clear_Display();
void LCD_changeDisplayMode(DisplayMode newDisplayMode);
void LCD_NewLine();
void LCD_Display_Measurement();
void LCD_Display_Menu();
void LCD_DisplayShift(bool autoscroll);
void LCD_Shift_Left();
void LCD_Shift_Right();
void LCD_Shift_Home();
void LCD_AutoScroll();
void LCD_Cursor_Home();


#endif /* INC_LCD_H_ */
