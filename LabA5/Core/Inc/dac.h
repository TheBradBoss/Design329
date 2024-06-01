/*
 *******************************************************************************
 * @file           : dac.h
 * @brief          : DAC configuration and millivolt input to output
 * project         : EE 329 S'24 Assignment 2
 * author(s)       : Bradley Buzzini, Daniel Hoefer
 * version         : 0.2
 * date            : 240517
 * compiler        : STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target          : NUCLEO-L496ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * See dac.c for more details
 */

#ifndef INC_DAC_H_
#define INC_DAC_H_

#include <stdint.h>
#include "stm32l496xx.h"

/*Defines for DAC pins*/
#define DAC_PORT		GPIOA
#define DAC_PORT_CLOCK	RCC_AHB2ENR_GPIOAEN
#define DAC_NUM			3					// Number of pins used
#define DAC_PINS		{4, 5, 7}			// GPIO Port pins
#define DAC_GAIN2		(uint16_t)(0x1000)	// Control nibble for G=2
#define DAC_GAIN1		(uint16_t)(0x3000)	// Control nibble for G=1


// Function declarations
void 		DAC_init (void);
uint16_t 	DAC_volt_conv( uint16_t voltage );
void		DAC_write ( uint16_t voltage_16bit );

#endif /* INC_DAC_H_ */
