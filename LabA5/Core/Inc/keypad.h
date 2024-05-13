/*
 *******************************************************************************
 * @file           : keypad.h
 * @brief          : keypad configuration and debounced detection of keypresses
 * project         : EE 329 S'24 Assignment 2
 * author(s)       : Bradley Buzzini
 * version         : 0.2
 * date            : 240424
 * compiler        : STM32CubeIDE v.1.15.0 Build: 20695_20240315_1429 (UTC)
 * target          : NUCLEO-L496ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * See keypad.c for more details
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#include "stm32l496xx.h"
#include <stdint.h>
// Header file for GPIO_PIN_# not included because they break compiler


// Change following define parameters for LCD ports you use
#define NUM_ROWS			(4)
#define NUM_COLS			(4)
#define KEYPAD_PORT			(GPIOC)
#define KEYPAD_PORT_CLOCK	(RCC_AHB2ENR_GPIOCEN)

#define ROW_PINS	{4, 5, 6, 8}		// In order of top to bottom rows

#define ROW_IO		((uint16_t)0x0170)	// (GPIO_PIN_4 | GPIO_PIN_5 | ...)
#define ROW_0		((uint16_t)0x0010)	// (GPIO_PIN_4)	//first pin of rows

#define COL_PINS	{0, 1, 2, 3}		// In order of left to right columns
#define COL_IO		((uint16_t)0x000F)	// (GPIO_PIN_0 | GPIO_PIN_1 | ...)
#define COL_0		((uint16_t)0x0001)	// GPIO_PIN_0	//first pin of columns
//	End change


// Function defines
void 	Keypad_Config 				( void );
int 	Keypad_IsAnyKeyPressed		( void );
uint8_t	Keypad_WhichKeyIsPressed 	( void );
uint8_t Keypad_CheckKeyPressed 		( uint8_t iKey );


#endif /* INC_KEYPAD_H_ */

