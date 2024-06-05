/* USER CODE BEGIN Header */
/*******************************************************************************
 * @file           : dac.c
 * @brief          : DAC configuration and millivolt input to output
 * project         : EE 329 S'24 Assignment 6
 * author(s)       : Bradley Buzzini, Daniel Hoefer
 * version         : 0.2
 * date            : 240517
 * compiler        : STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target          : NUCLEO-L496ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * DAC PLAN :
 * Set STM32 as controller
 * Configure pins to SPI alternate functions.
 * Convert millivolts to 12-bit DAC data.
 * Calibrate DAC performance
 * Write to data register when TXE is ready.
 *******************************************************************************
 * DAC WIRING (pinout NUCLEO-L496ZG = L4A6ZG)
 *      peripheral – Nucleo I/O
 * DAC 1 - VDD	 - 3V3 = CN8 - 7
 * DAC 2 - ~CS   - PA4 = CN7 - 9 - OUT, AF5
 * DAC 3 - SCK   - PA5 = CN7 - 10- OUT, AF5
 * DAC 4 - SDI   - PA7 = CN7 - 14- OUT, AF5
 * DAC 5 - ~LDAC - GND = CN9 - 23
 * DAC 6 - ~SHDN - 3V3 = CN8 - 7
 * DAC 7 - VSS   - GND = CN9 - 23
 * DAC 8 - VOUT
 *******************************************************************************
 * REVISION HISTORY
 * 0.1 240513 BB  Includes up to deliverable 5
 * 0.2 240516 DH  Calibrated and added code to test DAC performance.
 *******************************************************************************
 * REFERENCES
 * Code adapted from A5 in lab manual
 *******************************************************************************
/* USER CODE END Header */

#include "dac.h"

void DAC_init (void) {
	// enable clock for GPIOA & SPI1
	RCC->AHB2ENR |= (DAC_PORT_CLOCK);    	// GPIOA: DAC NSS/SCK/SDO
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);  	// SPI1 port

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
		//Pin A4: configure AFR for SPI1_NSS function
		//Pin A5: configure AFR for SPI1_SCK function
		//Pin A7: configure AFR for SPI1_MOSI function
	}
	// SPI config as specified @ STM32L4 RM0351 rev.9 p.1459
	// build control registers CR1 & CR2 for SPI control of peripheral DAC
	// assumes no active SPI xmits & no recv data in process (BSY=0)
	// CR1 (reset value = 0x0000)
	SPI1->CR1 &= ~( SPI_CR1_SPE );             	// disable SPI for config
	SPI1->CR1 &= ~( SPI_CR1_RXONLY );          	// recv-only OFF
	SPI1->CR1 &= ~( SPI_CR1_LSBFIRST );        	// data bit order MSb:LSb
	SPI1->CR1 &= ~( SPI_CR1_CPOL | SPI_CR1_CPHA ); // SCLK polarity:phase = 0:0
	SPI1->CR1 |=	 SPI_CR1_MSTR;              	// MCU is SPI controller(master)
	// CR2 (reset value = 0x0700 : 8b data)
	SPI1->CR2 &= ~( SPI_CR2_TXEIE | SPI_CR2_RXNEIE ); // disable FIFO intrpts??
	SPI1->CR2 &= ~( SPI_CR2_FRF);              	// Motorola frame format
	SPI1->CR2 |=	 SPI_CR2_NSSP;              	// auto-generate NSS pulse
	SPI1->CR2 |=	 SPI_CR2_DS;                	// 16-bit data
	SPI1->CR2 |=	 SPI_CR2_SSOE;              	// enable SS(slave select CS) output
	SPI1->CR1 |=	 SPI_CR1_SPE;               	// re-enable SPI for ops
}

//uint16_t DAC_volt_conv( uint16_t millivolt ) {
//	// Takes user entered voltage in mV, outputs 16-bit control and data code
//	// Outputs ideal Dn for desired Vout.
//	// For deliverable 7 pre-calibration
//	uint16_t data = 0;
//
//	if (millivolt > 3300) {
//		return (DAC_GAIN2 + 3300);	// Max Output Given
//	}
//	else if (millivolt > 2047) {
//		return (DAC_GAIN2 + (millivolt)*1);	// Ideal word
//	}
//	else {
//		return (DAC_GAIN1 + (millivolt)*2);	// Ideal word
//	}
//}

uint16_t DAC_volt_conv( uint16_t millivolt ) {
	// Takes voltage in mV, outputs 16-bit control and data code
	// to be sent on SDI to DAC.
	// Includes calibration

	if (millivolt > 2047) {
		// Scale equation to avoid float operations
		return (DAC_GAIN2 + (uint16_t)(((millivolt*100000) + 447000) / 100426));
	}
	else {
		// Scale equation to avoid float operations
		return (DAC_GAIN1 + (uint16_t)(((millivolt*100000) - 153733) / 50114));	// G = 1
	}
}

void DAC_write ( uint16_t message ) {
	// Sends message as it is to DAC on SDI
	while(!(SPI1->SR & SPI_SR_TXE))
		;	// Hold when TXE bit is zero
	SPI1->DR = message;
}
