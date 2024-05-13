/*
 * dac.c
 *
 *  Created on: May 11, 2024
 *      Author: bradl
 */

#include "dac.h"

//#include <stdint.h>



#define DAC_PORT		GPIOA
#define DAC_PORT_CLOCK	RCC_AHB2ENR_GPIOAEN
#define DAC_NUM			3
#define DAC_PINS		{4, 5, 7}	// GPIO Port pins


// Function declarations
void 		DAC_init (void);
uint16_t 	DAC_volt_conv( uint16_t voltage );
void		DAC_write ( uint16_t voltage_12bit );


void DAC_init (void) {
	// enable clock for GPIOA & SPI1
	RCC->AHB2ENR |= (DAC_PORT_CLOCK);                // GPIOA: DAC NSS/SCK/SDO
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);                 // SPI1 port

	/* USER ADD GPIO configuration of MODER/PUPDR/OTYPER/OSPEEDR registers HERE */


	uint32_t dac_pins[] = DAC_PINS;
	for (uint32_t i = 0; i < DAC_NUM ; i++) {
		uint32_t pin = dac_pins[i];
		DAC_PORT->MODER &= ~(0x3 << (pin * 2));
		DAC_PORT->MODER |= (0x2 << (pin * 2));
		DAC_PORT->OTYPER &= ~(0x1 << pin);
		DAC_PORT->OTYPER |= (0x0 << pin);
		DAC_PORT->OSPEEDR &= ~(0x3 << (pin * 2));
		DAC_PORT->OSPEEDR |= (0x3 << (pin * 2));//Highest speed
		DAC_PORT->PUPDR &= ~(0x3 << (pin * 2));
		DAC_PORT->BRR |= (0x1 << (pin * 1));	//initialize off
		GPIOA->AFR[0] &= ~((0x000F << (pin * 4))); // clear for 4 AF
		GPIOA->AFR[0] |=  ((0x0005 << (pin * 4))); // set A4 AF to SPI1 (fcn 5)

	}


	// SPI config as specified @ STM32L4 RM0351 rev.9 p.1459
	// called by or with DAC_init()
	// build control registers CR1 & CR2 for SPI control of peripheral DAC
	// assumes no active SPI xmits & no recv data in process (BSY=0)
	// CR1 (reset value = 0x0000)
	SPI1->CR1 &= ~( SPI_CR1_SPE );             	// disable SPI for config
	SPI1->CR1 &= ~( SPI_CR1_RXONLY );          	// recv-only OFF
	SPI1->CR1 &= ~( SPI_CR1_LSBFIRST );        	// data bit order MSb:LSb
	SPI1->CR1 &= ~( SPI_CR1_CPOL | SPI_CR1_CPHA ); // SCLK polarity:phase = 0:0 // clock should be low in idle state
	SPI1->CR1 |=	 SPI_CR1_MSTR;              	// MCU is SPI controller
	// CR2 (reset value = 0x0700 : 8b data)
	SPI1->CR2 &= ~( SPI_CR2_TXEIE | SPI_CR2_RXNEIE ); // disable FIFO intrpts
	SPI1->CR2 &= ~( SPI_CR2_FRF);              	// Moto frame format
	SPI1->CR2 |=	 SPI_CR2_NSSP;              	// auto-generate NSS pulse
	SPI1->CR2 |=	 SPI_CR2_DS;                	// 16-bit data
	SPI1->CR2 |=	 SPI_CR2_SSOE;              	// enable SS(slave select CS) output
	// CR1
	SPI1->CR1 |=	 SPI_CR1_SPE;               	// re-enable SPI for ops




//	// Pin A4: configure AFR for SPI1_NSS function
//	GPIOA->AFR[0] &= ~((0x000F << GPIO_AFRL_AFSEL4_Pos)); // clear for 4 AF
//	GPIOA->AFR[0] |=  ((0x0005 << GPIO_AFRL_AFSEL4_Pos)); // set A4 AF to SPI1 (fcn 5)
//	// Pin A5: configure AFR for SPI1_SCK function
//	GPIOA->AFR[0] &= ~((0x000F << GPIO_AFRL_AFSEL5_Pos)); // clear for 5 AF
//	GPIOA->AFR[0] |=  ((0x0005 << GPIO_AFRL_AFSEL5_Pos)); // set A5 AF to SPI1 (fcn 5)
//	// Pin A7: configure AFR for SPI1_MOSI function
//	GPIOA->AFR[0] &= ~((0x000F << GPIO_AFRL_AFSEL7_Pos)); // clear for bit 7 AF
//	GPIOA->AFR[0] |=  ((0x0005 << GPIO_AFRL_AFSEL7_Pos)); // set A7 AF to SPI1 (fcn 5)

}

uint16_t DAC_volt_conv( uint16_t voltage ) {
	// Takes user entered voltage in mV
}

void DAC_write ( uint16_t voltage_12bit ) {
	uint16_t red = voltage_12bit;
	while(!(SPI1->SR & SPI_SR_TXE))
		;	// Hold when TXE bit is zero
	SPI1->DR = 0x36A7;

}
void LPUART_ESC_print(const char* keycode){
	uint16_t iStrIdx = 0;
	while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
				;
	LPUART1->TDR = 0x1B;
	while ( keycode[iStrIdx] != 0 ) {
		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
			;
		LPUART1->TDR = keycode[iStrIdx];       // send this character
		iStrIdx++;                             // advance index to next char
	}
}
