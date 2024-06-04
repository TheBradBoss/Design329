/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file        : adc.c
 * @brief       : ADC Configuration with Calibration
 * project		: EE 329 S'24 Lab A8
 * authors		: Bradley Buzzini, Jack Ryan
 * version		: 0.3
 * date			: 240530
 * computer		: STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target		: NUCLEO-L496ZG
 * clock		: 4 MHz MSI to AHB2; 24 MHz HSI to ADC_Clock
 *
 * Citations:
 * OpenAI. (2024-Apr-08). ChatGPT, query synopsis "Write code in c that
 * 	 converts a 4 digit decimal to a string" to  https://chat.openai.com/chat
 * Sample Code from A8 Manual
 ******************************************************************************
 * REVISION HISTORY
 * 0.1 240528 BB  Able to receive and aggregate ADC samples
 * 0.2 240529 BB  Implemented LPUART.c
 * 0.3 240530 BB  Added extra credit
 *******************************************************************************
 * ADC PLAN
 * Configure ADC for use with pin A0
 * Wait for interrupt saying a conversion has been completed
 * Collect twenty samples and aggregate them
 * Return strings to main for use with LPUART.c
 *******************************************************************************
/* USER CODE END Header */
// Includes
#include "adc.h"
#include "delay.h"

#include "stdio.h"

uint8_t	ADC_conversion_flag = 0;		// Flag set in ISR to get conversion
uint16_t ADC_value1 = 0;				// Acquire ADC raw output
uint16_t ADC_value5 = 0;
uint16_t ADC_value7 = 0;
uint8_t conversion_num = 1;


void ADC_init( void ) {
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;         // turn on clock for ADC
	// power up & calibrate ADC

	// configure GPIO pin PA0
	RCC->AHB2ENR  |= (RCC_AHB2ENR_GPIOAEN);    // connect clock to GPIOA
	GPIOA->MODER  |= (GPIO_MODER_MODE0 | GPIO_MODER_MODE2);       // analog mode PA0 (MODER last)



	//RCC->PLLCFGR |= 0x2;	// PLLSRC clock source = HSI16 (16 MHz)
	ADC123_COMMON->CCR |= (1 << ADC_CCR_CKMODE_Pos); // clock = HCLK/1 = 16*8

	ADC1->CR &= ~(ADC_CR_DEEPPWD);             // disable deep-power-down
	ADC1->CR |= (ADC_CR_ADVREGEN);             // enable V regulator
	delay_us(20);                              // wait 20us for ADC to power up
	ADC1->DIFSEL &= ~(ADC_DIFSEL_DIFSEL_5 | 	// PA0=ADC1_IN5, single-ended
			ADC_DIFSEL_DIFSEL_7);    			// PA2=ADC1_IN7, single-ended
	ADC1->CR &= ~(ADC_CR_ADEN | ADC_CR_ADCALDIF); // disable ADC single-end cal
	ADC1->SQR1  |= (ADC_SQR1_L_0);
	ADC1->SMPR1 |= ((1 << ADC_SMPR1_SMP5_Pos) | (1 << ADC_SMPR1_SMP7_Pos));// ch 5 sample time = 6.5 clocks

	ADC1->CR |= ADC_CR_ADCAL;                  // start calibration
	while (ADC1->CR & ADC_CR_ADCAL) {;}        // wait for calib to finish
	// enable ADC
	ADC1->ISR |= (ADC_ISR_ADRDY);              // set to clr ADC Ready flag
	ADC1->CR |= ADC_CR_ADEN;                   // enable ADC
	while(!(ADC1->ISR & ADC_ISR_ADRDY)) {;}    // wait for ADC Ready flag
	ADC1->ISR |= (ADC_ISR_ADRDY);              // set to clr ADC Ready flag
	// configure ADC sampling & sequencing
	ADC1->SQR1  |= (5 << ADC_SQR1_SQ1_Pos) |	// sequence = 1 conv., ch 5
			(7 << ADC_SQR1_SQ2_Pos);    		// sequence = 2 conv., ch 7
//	ADC1->SQR1  |= (ADC_SQR1_L_0);
//	ADC1->SMPR1 |= ((1 << ADC_SMPR1_SMP5_Pos) | (1 << ADC_SMPR1_SMP7_Pos));// ch 5 sample time = 6.5 clocks
	//ADC1->SMPR1 |= (4 << ADC_SMPR1_SMP5_Pos);  // sample time = 47.5 clocks
	//ADC1->SMPR1 |= (7 << ADC_SMPR1_SMP5_Pos);  // sample time = 640.5 clocks
	ADC1->CFGR  &= ~( ADC_CFGR_CONT  |         // single conversion mode
			ADC_CFGR_EXTEN |         // h/w trig disabled for s/w trig
			ADC_CFGR_RES   );        // 12-bit resolution
	// configure & enable ADC interrupt
	ADC1->IER |= ADC_IER_EOCIE;                // enable end-of-conv interrupt
	ADC1->ISR |= ADC_ISR_EOC;                  // set to clear EOC flag
	NVIC->ISER[0] = (1<<(ADC1_2_IRQn & 0x1F)); // enable ADC interrupt service
	__enable_irq();                            // enable global interrupts

	// Re-set after every conversion
	ADC1->CR |= ADC_CR_ADSTART;                // start 1st conversion


}

void ADC_sample( uint16_t *avg_5, uint16_t *avg_7 ) {
	/* When called, this function will take 20 samples from the ADC and
	 * return the raw maximum, minimum, and average of the samples. */
	uint16_t sample[20];
	uint32_t sample_sum_5 = 0;	// ch.5
	uint32_t sample_sum_7 = 0;	// ch.7

	// Acquire 10 samples from each channel
	for (int i = 0 ; i < 19 ; i += 2 ) {
		while( !ADC_conversion_flag ) // *global* await next ISR trip
			;
		sample[i] = ADC_value5;
		sample[i+1] = ADC_value7;
		ADC_conversion_flag = 0; // clr for next sample
		//printf("Sample 5 is %d\n", sample[i]);
		//printf("Sample 7 is %d\n", sample[i+1]);
		ADC1->CR |= ADC_CR_ADSTART;// start conversions

	}

	// Perform characterization operations.
	for (int i = 0 ; i < 19 ; i += 2 ) {
		sample_sum_5 += sample[i];
		sample_sum_7 += sample[i+1];
	}

	// Return results
	*avg_5 = (sample_sum_5 / 10);
	*avg_7 = (sample_sum_7 / 10);

}

uint16_t ADC_raw_to_volt( uint16_t raw_sample ) {
	/* Calibrated recorded voltage conversion from
	 * Measured Volt vs. raw digital output of ADC */
	if (raw_sample < 2) {
		return 0;
	}
	else {
		return ((raw_sample * 8092) - 13805) / 10000;
	}
}

char* ADC_CountToString(uint16_t number ) {
	// Converts raw ADC output to string
	static char str[5];
	uint16_t digits[4];

	digits[0] = (number / 1000) % 10;  // Thousands
	digits[1] = (number / 100) % 10;   // Hundreds
	digits[2] = (number / 10) % 10;    // Tens
	digits[3] = number % 10;           // Units

	for (int i = 0 ; i < 4 ; i++) {
		str[i] = digits[i] + '0';	// Convert to ASCII
	}
	str[4] = '\0';

	return str;
}

char* ADC_VoltToString(uint16_t number ) {
	// Convert millivolts to volts in a string
	static char str[6];
	uint16_t digits[4];

	digits[0] = (number / 1000) % 10;  // Thousands
	digits[1] = (number / 100) % 10;   // Hundreds
	digits[2] = (number / 10) % 10;    // Tens
	digits[3] = number % 10;           // Units

	str[0] = digits[0] + '0';
	str[1] = '.';
	for (int i = 1 ; i < 4 ; i++) {
		str[i+1] = digits[i] + '0';	// Convert to ASCII
	}
	str[5] = '\0';

	return str;
}

void ADC1_2_IRQHandler( void ) {
	/* Record ADC value to global variable when ready.
	 * Tell other functions they are ready to take a new sample */
	if ( ADC1->ISR & ADC_ISR_EOC ) { // conversion done?
		if (conversion_num == 1) {
			ADC_value5 = ADC1->DR;
			conversion_num++;
		}
		else if (conversion_num == 2) {
			ADC_value7 = ADC1->DR;
			conversion_num = 1;
			ADC_conversion_flag = 1; // tell ADC_sample() sequence complete
		}
		//ADC_value1 = ADC1->DR; // get data, auto-clr EOC
		//ADC_conversion_flag = 1; // tell main(): got one
	}
}
