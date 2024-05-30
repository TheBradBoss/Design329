/*
 * adc.c
 *
 *  Created on: May 28, 2024
 *      Author: bradl
 */
#include "adc.h"
#include "delay.h"

uint8_t	ADC_conversion_flag = 0;		// Flag set in ISR to get conversion
uint16_t ADC_value1 = 0;


void ADC_init( void ) {
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;         // turn on clock for ADC
	// power up & calibrate ADC

	RCC->PLLCFGR |= 0x2;	// PLLSRC clock source = HSI16 (16 MHz)
	ADC123_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos); // clock = HCLK/1 = 16*8

	ADC1->CR &= ~(ADC_CR_DEEPPWD);             // disable deep-power-down
	ADC1->CR |= (ADC_CR_ADVREGEN);             // enable V regulator - see RM 18.4.6
	delay_us(20);                              // wait 20us for ADC to power up
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5);    // PA0=ADC1_IN5, single-ended
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); // disable ADC, single-end calib
	ADC1->CR |= ADC_CR_ADCAL;                  // start calibration
	while (ADC1->CR & ADC_CR_ADCAL) {;}        // wait for calib to finish
	// enable ADC
	ADC1->ISR |= (ADC_ISR_ADRDY);              // set to clr ADC Ready flag
	ADC1->CR |= ADC_CR_ADEN;                   // enable ADC
	while(!(ADC1->ISR & ADC_ISR_ADRDY)) {;}    // wait for ADC Ready flag
	ADC1->ISR |= (ADC_ISR_ADRDY);              // set to clr ADC Ready flag
	// configure ADC sampling & sequencing
	ADC1->SQR1  |= (5 << ADC_SQR1_SQ1_Pos);    // sequence = 1 conv., ch 5
	ADC1->SMPR1 |= (1 << ADC_SMPR1_SMP5_Pos);  // ch 5 sample time = 6.5 clocks
	//ADC1->SMPR1 |= (4 << ADC_SMPR1_SMP5_Pos);  // ch 5 sample time = 47.5 clocks
	//ADC1->SMPR1 |= (7 << ADC_SMPR1_SMP5_Pos);  // ch 5 sample time = 640.5 clocks
	ADC1->CFGR  &= ~( ADC_CFGR_CONT  |         // single conversion mode
	                  ADC_CFGR_EXTEN |         // h/w trig disabled for s/w trig
	                  ADC_CFGR_RES   );        // 12-bit resolution
	// configure & enable ADC interrupt
	ADC1->IER |= ADC_IER_EOCIE;                // enable end-of-conv interrupt
	ADC1->ISR |= ADC_ISR_EOC;                  // set to clear EOC flag
	NVIC->ISER[0] = (1<<(ADC1_2_IRQn & 0x1F)); // enable ADC interrupt service
	__enable_irq();                            // enable global interrupts
	// configure GPIO pin PA0
	RCC->AHB2ENR  |= (RCC_AHB2ENR_GPIOAEN);    // connect clock to GPIOA
	GPIOA->MODER  |= (GPIO_MODER_MODE0);       // analog mode for PA0 (set MODER last)

	ADC1->CR |= ADC_CR_ADSTART;                // start 1st conversion
	// Does this need to be reset after every conversion

}

void ADC_sample( uint16_t *ave, uint16_t *min, uint16_t *max ) {
	/* When called, this function will take 20 samples from the ADC and
	 * return the maximum, minimum, and average of the samples. */


	uint16_t sample[20];
	uint16_t sample_min = 4096;
	uint16_t sample_max = 0;
	uint32_t sample_sum = 0;	// 32-bit to prevent overflow


	// Acquire samples first
	for (int i = 0 ; i < 20 ; i++ ) {
		while( !ADC_conversion_flag ) // *global* await next ISR trip
			;
		sample[i] = ADC_value1;
		ADC_conversion_flag = 0; // clr for next sample
		ADC1->CR |= ADC_CR_ADSTART;// start conversion
	}

	// Perform characterization operations.
	for (int i = 0 ; i < 20 ; i++ ) {
		if (sample[i] < sample_min) sample_min = sample[i];
		if (sample[i] > sample_max) sample_max = sample[i];
		sample_sum += sample[i];
	}

	*ave = (sample_sum / 20);
	*min = sample_min;
	*max = sample_max;
}

uint16_t ADC_raw_to_volt( uint16_t raw_sample ) {
	/* Will include calibration later */
	//return (raw_sample * 3300) / 4095;


//	uint32_t conversion = ((raw_sample * 8092) - 13805) / 10000000;
//	return conversion;

	return ((raw_sample * 8092) - 13805) / 10000;
}

void ADC1_2_IRQHandler( void ) {
	if ( ADC1->ISR & ADC_ISR_EOC ) { // conversion done?
		ADC_value1 = ADC1->DR; // get data, auto-clr EOC
		ADC_conversion_flag = 1; // tell main(): got one
	}
}
