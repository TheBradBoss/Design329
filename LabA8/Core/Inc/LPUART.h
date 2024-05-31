/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file        : LPUART.h
 * @brief       : Configure LPUART and printing and escape code functions
 * project		: EE 329 S'24 Lab A8
 * authors		: Bradley Buzzini
 * version		: 0.1
 * date			: 240528
 * computer		: STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target		: NUCLEO-L496ZG
 * clock		: 4 MHz MSI to LPUART
 ******************************************************************************
 * See LPUART.c for more in depth description.
 *******************************************************************************
/* USER CODE END Header */

#ifndef INC_LPUART_H_
#define INC_LPUART_H_

// Includes
#include <stm32l496xx.h>
#include <stdint.h>
#include <stdio.h>

// Declare LPUART Functions
void SystemClock_Config(void);
void LPUART_init( void );
void LPUART_print( const char* message );
void LPUART_ESC_print( const char* message);
void LPUART1_IRQHandler( void  );			// Not needed for A8

// Commands to use with LPUART_ESC_print
#define CLEAR_SCREEN	"[2J"
#define RETURN_CURSOR	"[H"

#endif /* INC_LPUART_H_ */
