/*
/*******************************************************************************
 * @file           : lcd.c
 * @brief          : lcd configuration and commands
 * project         : EE 329 S'24 Assignment 4
 * author(s)       : Bradley Buzzini
 * version         : 0.3
 * date            : 240501
 * compiler        : STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target          : NUCLEO-L496ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * LCD PLAN:
 * send initialization commands
 * add function to enable commands
 * add different functions for different command types
 * add #defines for different commands
 *******************************************************************************
 * LCD WIRING 4-BIT CONFIGURATION (pinout NUCLEO-L496ZG = L4A6ZG)
 *      peripheral – Nucleo I/O
 * lcd 4  RS - PA4 = CN7 - 10 - OUT, PD ON
 * lcd 6  E  - PA5 = CN7 - 9  - OUT, PD ON
 * lcd 11 DB4- PA0 = CN10- 29 - OUT, PD ON
 * lcd 12 DB5- PA1 = CN10- 11 - OUT, PD ON
 * lcd 13 DB6- PA2 = CN10- 13 - OUT, PD ON
 * lcd 14 DB7- PA3 = CN9 - 1  - OUT, PD ON
 *
 * Note: DB pins must be lowest 4 of port
 *******************************************************************************
 * REVISION HISTORY
 * 0.1 240422 BB  Printed first character
 * 0.2 240424 BB  Includes requirements for A3
 * 0.3 240501 BB  Fixed commands and adjusted LCD_write_time for A4
 *******************************************************************************
 * REFERENCES
 * Code adapted from A1 in manual
 * OpenAI. (2024-Apr-22). ChatGPT, query synopsis "Write code in c that
 * 	converts minutes and seconds in an MM:SS format to seconds"
 * 	to  https://chat.openai.com/chat
 * OpenAI. (2024-Apr-22). ChatGPT, query synopsis "Write a function in c that
 * 	outputs the character constants of each character in a given string"
 * 	to  https://chat.openai.com/chat
 *******************************************************************************
 */


#include "lcd.h"
#include "delay.h"

#include "stm32l496xx.h"	// To write to GPIO registers


//COMMANDS ONLY WORK IF GPIO LCD DB PINS ARE 0,1,2,3


void LCD_init( void )  {
	// RCC & GPIO config for LCD port and pins
	RCC->AHB2ENR |= (LCD_PORT_CLOCK);

	//Set pins to outputs
	uint32_t lcd_pins[] = LCD_PINS;
	for (uint32_t i = 0; i < LCD_NUM ; i++) {
		uint32_t pin = lcd_pins[i];
		LCD_PORT->MODER &= ~(0x3 << (pin * 2));
		LCD_PORT->MODER |= (0x1 << (pin * 2));	//Output 01
		LCD_PORT->OTYPER &= ~(0x1 << pin);
		LCD_PORT->OTYPER |= (0x0 << pin);		//0
		LCD_PORT->OSPEEDR &= ~(0x3 << (pin * 2));
		LCD_PORT->OSPEEDR |= (0x3 << (pin * 2));//Highest speed
		LCD_PORT->PUPDR &= ~(0x3 << (pin * 2));
		LCD_PORT->PUPDR |= (0x2 << (pin * 2));	//PULL DOWN (no false highs)
		LCD_PORT->BRR |= (0x1 << (pin * 1));	//initialize off
	}

	delay_us( 40000 );                     	// power-up wait 40 ms
	for ( int idx = 0; idx < 3; idx++ ) {  	// wake up 1,2,3: DATA = 0011 XXXX
		LCD_4b_command( 0x30 );             // HI 4b of 8b cmd, low nibble = X
		delay_us( 200 );
	}
	LCD_4b_command( 0x20 ); 		// fcn set #4: 4b cmd set 4b mode
	delay_us( 40 );         		// remainder of LCD init removed
	LCD_command( 0x28 );			//Selects 2-line mode instead of 1-line
	LCD_command( CURSOR_SHIFT_L );			//Shift cursor to the left
	LCD_command( CURSOR_ON );		//Display, cursor, cursor position on
	LCD_command( CURSOR_RIGHT );	//Cursor moves right, no shift
}

void LCD_pulse_ENA( void )  {
	// ENAble line sends command on falling edge
	// set to restore default then clear to trigger
	LCD_PORT->ODR   |= ( LCD_EN );         	// ENABLE = HI
	delay_us( 100 );                         // TDDR > 320 ns
	LCD_PORT->ODR   &= ~( LCD_EN );        // ENABLE = LOW
	delay_us( 100 );                         // low values flakey, see A3:p.1
}

void LCD_4b_command( uint8_t command )  {
	// LCD command using high nibble only - used for 'wake-up' 0x30 commands
	LCD_PORT->ODR   &= ~( LCD_DATA_BITS ); 	// clear DATA bits
	LCD_PORT->ODR   |= ( command >> 4 );   	// DATA = command
	delay_us( 100 );
	LCD_pulse_ENA( );						//send
}

void LCD_command( uint8_t command )  {
	// send command to LCD in 4-bit instruction mode
	// HIGH nibble then LOW nibble, timing sensitive

	LCD_PORT->ODR   &= ~( LCD_DATA_BITS );               // isolate cmd bits
	LCD_PORT->ODR   |= ( (command>>4) & LCD_DATA_BITS ); // HIGH shifted low
	delay_us( 100 );

	LCD_pulse_ENA( );                                    // latch HIGH NIBBLE

	LCD_PORT->ODR   &= ~( LCD_DATA_BITS );               // isolate cmd bits
	LCD_PORT->ODR   |= ( command & LCD_DATA_BITS );      // LOW nibble
	delay_us( 100 );
	LCD_pulse_ENA( );                                    // latch LOW NIBBLE
	if ( (command == CLEAR_HOME) || (command == RETURN_HOME) ) {
		delay_us( 1000 );	// These commands need more time
	}
}

void LCD_write_char( uint8_t letter )  { //0x41 is A
	// calls LCD_command() w/char data
	LCD_PORT->ODR   |= (LCD_RS);       // RS = HI for data to address
	delay_us( 100 );
	LCD_command( letter );             // character to print
	LCD_PORT->ODR   &= ~(LCD_RS);      // RS = LO
	delay_us( 100 );
}

void LCD_write_string( uint8_t sentence[] ) {
	// extracts each character from a string and passes it to LCD_write_char()
	uint8_t len = strlen(sentence);
	for ( uint8_t i = 0 ; i < len ; i++ ) {
		LCD_write_char( sentence[i] );	// Write each character to LCD
	}
}


void LCD_write_time( uint32_t milli_s ) {
	// Takes seconds and outputs time on LCD (N.SSS). Prints right to left
	// SPECIFIC TO LAB A4 TIME LOCATION ON LCD

	LCD_command(CURSOR_LEFT);					// Cursor moves left
	LCD_command(0xCD);							// Cursor to last S in N.SSS
	LCD_write_char( 's' );
	LCD_write_char( ' ' );
	for ( uint8_t i = 0; i < 4 ; i++ ) {
		LCD_write_char( (milli_s % 10) + '0' );	// Print each digit
		if ( i == 2 ) LCD_write_char( '.' );	// Add dot between s/ms
		milli_s /= 10;							// Next left digit
	}
	LCD_command(CURSOR_RIGHT);
}

//Include if you want to hack the LCD (prints constant garbage)
//void LCD_write_time( uint32_t totalSeconds ) {
//	//Takes total seconds and outputs time on LCD
//	uint32_t minutes = totalSeconds / 60;
//	uint32_t seconds = totalSeconds % 60;
//	uint32_t time = (minutes * 100) + seconds;	//MMSS
//	LCD_command(0xCB);	//First M in MM:SS
//	uint32_t digits[4];
//	uint32_t i = 0;
//	while (time > 0) {
//		digits[i++] = time % 10;
//		time /= 10;
//	}
//
//	for (uint32_t j = i - 1; j >= 0; j--) {
//		uint8_t digit_char = '0' + digits[j];
//		LCD_write_char(digit_char);
//		if (j == 2) LCD_write_char(':');
//	}
//	}
