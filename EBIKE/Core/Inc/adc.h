/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file        : adc.h
 * @brief       : ADC Configuration with Calibration
 * project		: EE 329 S'24 Lab A8
 * authors		: Bradley Buzzini, Jack Ryan
 * version		: 0.3
 * date			: 240530
 * computer		: STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target		: NUCLEO-L496ZG
 * clock		: 4 MHz MSI to AHB2; 24 MHz HSI to ADC_Clock
 ******************************************************************************
 * See adc.c for more in depth description.
 ******************************************************************************
/* USER CODE END Header */

#ifndef INC_ADC_H_
#define INC_ADC_H_

// Includes
#include <stm32l496xx.h>
#include "stdint.h"

// Declare ADC functions and ISRs.
void 		ADC_init( void );
void		ADC_sample( uint16_t *avg_5, uint16_t *avg_7 );
uint16_t	ADC_raw_to_volt( uint16_t raw_sample );
char* 		ADC_CountToString( uint16_t number );
char* 		ADC_VoltToString( uint16_t number );
void 		ADC1_2_IRQHandler( void );

#endif /* INC_ADC_H_ */
