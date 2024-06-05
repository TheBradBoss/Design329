/*
*******************************************************************************
* @file           : lcd.h
* @brief          : lcd configuration and commands
* project         : EE 329 S'24 Assignment 4
* author(s)       : Bradley Buzzini
* version         : 0.3
* date            : 240424
* compiler        : STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
* target          : NUCLEO-L496ZG
* clocks          : 4 MHz MSI to AHB2
* @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
*******************************************************************************
* See lcd.c for details
*/
#ifndef INC_LCD_H_
#define INC_LCD_H_
/* Neccessary includes for lcd.c */
#include "stm32l496xx.h"
//#include "stm32l4xx_hal_gpio.h"	//would be useful but breaks compiler
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* Change following define parameters for LCD ports you use */
#define LCD_PORT		GPIOD
#define LCD_PORT_CLOCK	RCC_AHB2ENR_GPIODEN
#define LCD_DATA_BITS	((uint16_t)0x000F)	// (GPIO_PIN_0 | GPIO_PIN_1 ...)
#define LCD_NUM			6					// How many output pins for lcd.
#define LCD_PINS		{0, 1, 2, 3, 4, 5}	// GPIO Port pins 0 though 5
#define LCD_EN			((uint16_t)0x0010)	// Equivalent to GPIO_PIN_4
#define LCD_RS			((uint16_t)0x0020)	// Equivalent to GPIO_PIN_5
//Only ever writing to LCD so RW pin (6) stays low (is grounded)
/* Commands for "LCD_command" function*/
#define CLEAR_HOME		0x01
#define RETURN_HOME		0x02	// Cursor to original position
#define CURSOR_LEFT		0x04	// Cursor moves left, no shift
#define CURSOR_RIGHT	0x06	// Cursor moves right, no shift
#define CURSOR_OFF		0x0C	// No blink, no cursor
#define CURSOR_ON		0x0F	// Display, cursor, cursor position on
#define CURSOR_SHIFT_L	0x10	// Shift cursor left
#define LINE_TWO		0xC0	// Move cursor to line two
//Initialize lcd functions
void LCD_init			( void );
void LCD_pulse_ENA		( void ) ;
void LCD_4b_command		( uint8_t command ) ;
void LCD_command		( uint8_t command ) ;
void LCD_write_char		( uint8_t letter );
void LCD_write_string	( uint8_t sentence[] );
void LCD_write_volt		( uint16_t millivolt );
#endif /* INC_LCD_H_ */

