/*
 * lcd.h
 *
 *  Created on: 22 Apr 2022
 *      Author: dfmol
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

typedef enum DisplayStates {Menu, Measurement, Output} DisplayMode;

typedef struct DisplayTemplate{
	enum DisplayStates Mode;
	uint8_t NumCharacters;
} DisplayStateType;

extern DisplayStateType DisplayState;

void LCD_Init();
void LCD_Write_Character(uint8_t character);
void LCD_Write_Instruction(uint8_t instruction);
void LCD_Write_Nibbles(uint8_t byte);
void LCD_Write_8bitInstruction(uint8_t byte);
void LCD_ZeroPins();
void LCD_Write_String(uint8_t string[]);
void LCD_Clear_Display();
void LCD_changeDisplayMode(DisplayMode newDisplayMode);


#endif /* INC_LCD_H_ */
