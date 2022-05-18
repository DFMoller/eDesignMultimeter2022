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
typedef enum DisplayBranches {Top, Measure, Signal, DC_V, DC_I, AC_V,  AC_I, Type, Parameter, OnOff, OUT_ON, OUT_OFF, DC_OUT, SINUSOIDAL_OUT, PULSE_OUT, Amp, Amp_Val, Offset, Offset_Val, Freq, Freq_Val, Duty, Duty_Val} DisplayBranchType;
typedef enum BranchActions {Up, Down, Left, Right, Enter} BranchActionType;

typedef struct DisplayTemplate{
	enum DisplayStates Mode;
	enum DisplayStates LastMode;
	enum DisplayLines CurrentLine;
	enum DisplayBranches CurrentBranch;
	uint8_t ToplineCharacters;
	uint8_t BottomlineCharacters;
	uint8_t PrintFlag;
	uint8_t PrintByte;
	uint8_t RefreshFlag;
	uint8_t RefreshCounter;
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
void LCD_Branch_Action(BranchActionType Action);


#endif /* INC_LCD_H_ */
