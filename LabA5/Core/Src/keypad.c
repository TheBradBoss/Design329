/* USER CODE BEGIN Header */
/*******************************************************************************
 * @file           : keypad.c
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
 * KEYPAD PLAN :
 * set columns as outputs, rows as inputs w pulldowns
 * loop:
 * drive all columns HI one by one, read all rows
 * if any row N is HI
 * 		pressed key loc’n = N, M
 * key value = 3N+M+1 for location
 * convert key value to a character with switch case
 *******************************************************************************
 * KEYPAD WIRING 4 ROWS 4 COLS (pinout NUCLEO-L496ZG = L4A6ZG)
 *      peripheral – Nucleo I/O
 * keypad 1  COL 2 - PC1 = CN8 - 7 - OUT
 * keypad 2  ROW 1 - PC4 = CN9 -21 - IN, PD ON
 * keypad 3  COL 1 - PC0 = CN7 -20 - OUT
 * keypad 4  ROW 4 - PC8 = CN8 -2  - IN, PD ON
 * keypad 5  COL 3 - PC2 = CN10- 8 - OUT
 * keypad 6  ROW 3 - PC6 = CN9 -17 - IN, PD ON
 * keypad 7  ROW 2 - PC5 = CN9 -19 - IN, PD ON
 * keypad 8	 COL 4 - PC3 = CN9 -5  - OUT
 *******************************************************************************
 * REVISION HISTORY
 * 0.1 240418 BB  Includes requirements for A2
 * 0.2 240424 BB  Made more general for A3
 * 0.3 240511 BB  IsAnyKeyPressed returns single press
 *******************************************************************************
 * REFERENCES
 * Code adapted from A2 in manual
 *******************************************************************************
/* USER CODE END Header */

#include "keypad.h"

#include <stdint.h>
#include <stdio.h>
#include "stm32l496xx.h"


// Initialize functions
uint8_t Keypad_CheckKeyPressed 		( uint8_t iKey );
void 	Keypad_Config 				( void );
int 	Keypad_IsAnyKeyPressed 		( void );
uint8_t Keypad_WhichKeyIsPressed	( void );


// Declare local variable
uint8_t debounce_state = 0;	// If key is currently pressed ==1


void Keypad_Config ( void ) {
	// pins must be an array with pin numbers.
	// count(number of pins being used) must be entered manually
	RCC->AHB2ENR |= (KEYPAD_PORT_CLOCK);

	// Set columns to outputs
	uint32_t col_pins[] = COL_PINS;
	for (uint32_t i = 0; i < NUM_COLS; i++) {
		uint32_t pin = col_pins[i];
		KEYPAD_PORT->MODER &= ~(0x3 << (pin * 2));
		KEYPAD_PORT->MODER |= (0x1 << (pin * 2));	// 01
		KEYPAD_PORT->OTYPER &= ~(0x1 << pin);
		KEYPAD_PORT->OTYPER |= (0x0 << pin);		// 0
		KEYPAD_PORT->OSPEEDR &= ~(0x3 << (pin * 2));
		KEYPAD_PORT->OSPEEDR |= (0x3 << (pin * 2));	// Fastest speed
		KEYPAD_PORT->PUPDR &= ~(0x3 << (pin * 2));
		KEYPAD_PORT->PUPDR |= (0x0 << (pin * 2));	// No PU or PD
		KEYPAD_PORT->ODR &= ~(COL_IO);				// Start low
	}

	// Set rows to inputs
	uint32_t row_pins[] = ROW_PINS;
	for (uint32_t i = 0; i < NUM_ROWS; i++) {
		uint32_t pin = row_pins[i];
		KEYPAD_PORT->MODER &= ~(0x3 << (pin * 2));
		KEYPAD_PORT->MODER |= (0x0 << (pin * 2));
		KEYPAD_PORT->OTYPER &= ~(0x1 << pin);
		KEYPAD_PORT->OTYPER |= (0x0 << pin);
		KEYPAD_PORT->OSPEEDR &= ~(0x3 << (pin * 2));
		KEYPAD_PORT->OSPEEDR |= (0x3 << (pin * 2));
		KEYPAD_PORT->PUPDR &= ~(0x3 << (pin * 2));
		KEYPAD_PORT->PUPDR |= (0x2 << (pin * 2));
	}
}


int Keypad_IsAnyKeyPressed(void) {
	// drive all COLUMNS HI; see if any ROWS are HI
	// return true if a key is pressed, false if not
	// Debounce only records a single press when button held
	uint16_t settle = 1000;	// Small delay for debounce to settle
	uint16_t debounce_count = 0;
	if ( debounce_state == 0 ) {
		KEYPAD_PORT->BSRR |= COL_IO;	// set all columns HI
		for ( uint16_t idx=0; idx<settle; idx++ )   	// let it settle
			;
		while ( (KEYPAD_PORT->IDR & ROW_IO) != 0 ) {	// Button pressed
			debounce_count++;
			if ( debounce_count > settle ) {			// Button high for awhile
				debounce_state = 1;
				return 1;
			}
		}
		return 0;
	}
	if ( debounce_state == 1 ) {	// Button was pressed
		if ( (KEYPAD_PORT->IDR & ROW_IO) == 0 ) {		// Button released
			debounce_state = 0;
			return 0;
		}
		else return  0;
	}
}


uint8_t Keypad_WhichKeyIsPressed(void) {
	// detect and encode a pressed key at {row,col}
	// assumes a previous call to Keypad_IsAnyKeyPressed() returned TRUE
	// verifies the Keypad_IsAnyKeyPressed() result (no debounce here),
	// determines which key is pressed and returns the encoded key ID

	int8_t iRow=0, iCol=0, iKey=0;  // keypad row & col index, key ID result
	int8_t bGotKey = 0;             // bool for keypress, 0 = no press
	uint16_t settle = 1000;			// Small delay for latching to settle

	uint32_t col_pins[] = COL_PINS;
	uint32_t row_pins[] = ROW_PINS;

	KEYPAD_PORT->ODR &= ~(COL_IO);	// Disable columns from IsAnyKeyPressed

	for ( iCol = 0; iCol < NUM_COLS; iCol++ ) {
		// turn column on one by one
		uint32_t pinC = col_pins[iCol];
		KEYPAD_PORT->ODR = ((KEYPAD_PORT->ODR & ~(COL_IO)) | (0x1 << pinC ));
		for ( uint16_t idx=0; idx<settle; idx++ )   	// let it settle
			;
		for( iRow = 0; iRow < NUM_ROWS; iRow++ ){		// check each row
			uint32_t pinR = row_pins[iRow];
			if ( KEYPAD_PORT->IDR & (0x1 << pinR) ) {	// row returned high
				bGotKey = 1;							// key found!
				break;
			}
		}
		if ( bGotKey )
			break;
	}
	if ( bGotKey ) {
		iKey = ( iRow * NUM_COLS ) + iCol + 1;  		// Gives location 1-16
		uint8_t KEY_out = Keypad_CheckKeyPressed(iKey);	// get char constant
		return( KEY_out );                         		// encoded keypress
	}
	return( -1 );                     					// failed

}

uint8_t Keypad_CheckKeyPressed (uint8_t iKey){
	// Converts keypad location to its character
	uint8_t var;

	switch (iKey) {
	case 0x01: //Column 1, Row 1
		var = ('1');
		break;
	case 0x02: //Column 2, Row 1
		var = ('2');
		break;

	case 0x03: //Column 3, Row 1
		var = ('3');
		break;

	case 0x04: //Column 4, Row 1
		var = ('A');
		break;

	case 0x05: //Column 1, Row 2
		var = ('4');
		break;

	case 0x06: //Column 2, Row 2
		var = ('5');
		break;

	case 0x07: //Column 3, Row 2
		var = ('6');
		break;

	case 0x08: //Column 4, Row 2
		var = ('B');
		break;

	case 0x09: //Column 1, Row 3
		var = ('7');
		break;

	case 0x0A: //Column 2, Row 3
		var = ('8');
		break;

	case 0x0B: //Column 3, Row 3
		var = ('9');
		break;

	case 0x0C: //Column 4, Row 3
		var = ('C');
		break;

	case 0x0D: //Column 3, Row 1
		var = ('*');
		break;

	case 0x0E: //Column 3, Row 2
		var = ('0');
		break;

	case 0x0F: //Column 3, Row 3
		var = ('#');
		break;

	case 0x10: //Column 3, Row 4
		var = ('D');
		break;

	default:
		var = ('.'); // period for no press (error)
		break;
	}

	return var;
}
