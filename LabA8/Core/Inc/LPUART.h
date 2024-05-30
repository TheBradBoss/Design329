/*
 * LPUART.h
 *
 *  Created on: May 8, 2024
 *      Author: bradl
 */

#ifndef INC_LPUART_H_
#define INC_LPUART_H_

#include <stm32l496xx.h>

void SystemClock_Config(void);
void LPUART_init( void );
void LPUART_print( const char* message );
void LPUART_ESC_print( const char* message);
void LPUART1_IRQHandler( void  );
void GAME_character(void);
void GAME_background(void);
void LPUART_move_cursor ( uint8_t x_location, uint8_t y_location );

#define CLEAR_SCREEN	"[2J"
#define RETURN_CURSOR	"[H"

#endif /* INC_LPUART_H_ */
