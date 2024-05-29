/*
 * adc.h
 *
 *  Created on: May 28, 2024
 *      Author: bradl
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stm32l496xx.h>

void ADC_init( void );
void ADC_sample( uint16_t *ave, uint16_t *min, uint16_t *max);
uint16_t ADC_raw_to_volt( uint16_t raw_sample );
void ADC1_2_IRQHandler( void );

#endif /* INC_ADC_H_ */
