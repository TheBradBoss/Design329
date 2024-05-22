/*
 * eeprom.c
 *
 *  Created on: May 20, 2024
 *      Author: bradl
 */
// Uses AFR[1] pins
#include "eeprom.h"
#include "delay.h"

#include "stm32l496xx.h"

#define EEPROM_PORT			GPIOB
#define EEPROM_PORT_CLOCK	RCC_AHB2ENR_GPIOBEN
#define EEPROM_NUM			2
#define EEPROM_PINS			{8, 9}
#define EEPROM_ADDRESS		0x52
#define EEPROM_MEMORY_ADDR	0x1111

void 	EEPROM_init		( void );
uint8_t EEPROM_read 	( uint16_t address );
void 	EEPROM_write 	( uint16_t address );

void EEPROM_init( void ) {
	/* USER configure GPIO pins for I2C alternate functions SCL and SDA */
	// Configure PB8 and PB9 for I2C alternate functions SCL and SDA
	SysTick_Init();
	RCC->AHB2ENR |= (EEPROM_PORT_CLOCK);    	// GPIOA: EEPROM NSS/SCK/SDO
	uint32_t dac_pins[] = EEPROM_PINS;
	for (uint32_t i = 0; i < EEPROM_NUM ; i++) {
		uint32_t pin = dac_pins[i];
		EEPROM_PORT->MODER &= ~(0x3 << (pin * 2));
		EEPROM_PORT->MODER |= (0x2 << (pin * 2));
		EEPROM_PORT->OTYPER &= ~(0x1 << pin);
		EEPROM_PORT->OTYPER |= (0x0 << pin);
		EEPROM_PORT->OSPEEDR &= ~(0x3 << (pin * 2));
		EEPROM_PORT->OSPEEDR |= (0x3 << (pin * 2));//Highest speed
		EEPROM_PORT->PUPDR &= ~(0x3 << (pin * 2));
		EEPROM_PORT->BRR |= (0x1 << (pin * 1));	//initialize off
		EEPROM_PORT->AFR[1] &= ~((0x000F << ((pin-8) * 4))); // clear for AF
		EEPROM_PORT->AFR[1] |=  ((0x0004 << ((pin-8) * 4))); // set AF to (fcn 4)
		//Pin B8: configure AFR for I2C1_SCL function
		//Pin B9: configure AFR for I2C1_SDA function
	}

	// Configure I2C
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;  // enable I2C bus clock
	I2C1->CR1   &= ~( I2C_CR1_PE );        // put I2C into reset (release SDA, SCL)
	I2C1->CR1   &= ~( I2C_CR1_ANFOFF );    // filters: enable analog
	I2C1->CR1   &= ~( I2C_CR1_DNF );       // filters: disable digital
	I2C1->TIMINGR = 0x00303D5B;            // 16 MHz SYSCLK timing from CubeMX				// Need to change junk number
	I2C1->CR2   |=  ( I2C_CR2_AUTOEND );   // auto send STOP after transmission
	I2C1->CR2   &= ~( I2C_CR2_ADD10 );     // 7-bit address mode
	I2C1->CR1   |=  ( I2C_CR1_PE );        // enable I2C


//	// build EEPROM transaction
//	I2C1->CR2   &= ~( I2C_CR2_RD_WRN );    // set WRITE mode
//	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
//	I2C1->CR2   |=  ( 3 << I2C_CR2_NBYTES_Pos); // write 3 bytes (2 addr, 1 data)
//	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
//	I2C1->CR2   |=  ( EEPROM_ADDRESS << (I2C_CR2_SADD_Pos+1) ); // device addr SHL 1
//	I2C1->CR2   |=    I2C_CR2_START;       // start I2C WRITE op



}

uint8_t EEPROM_read (uint16_t address ) {
	delay_us(5000);
	uint8_t read_data = 0;
	/* Write the address to be read from */
	I2C1->CR2   &= ~( I2C_CR2_RD_WRN );    // set WRITE mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
	I2C1->CR2   |=  ( 2 << I2C_CR2_NBYTES_Pos); // write 2 bytes (2 addr)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
	I2C1->CR2   |=  ( EEPROM_ADDRESS << (I2C_CR2_SADD_Pos+1) ); // device addr SHL 1
	I2C1->CR2   |=    I2C_CR2_START;       // start I2C WRITE op
	while(!(I2C1->ISR & I2C_ISR_TXIS)) ;   // wait for start condition to transmit
	I2C1->TXDR = (EEPROM_MEMORY_ADDR >> 8); // xmit MSByte of address
	while(!(I2C1->ISR & I2C_ISR_TXIS)) ;   // wait for start condition to transmit
	I2C1->TXDR = (EEPROM_MEMORY_ADDR & 0xFF); // xmit LSByte of address

	delay_us(5000);
	/* Read from the address */
	I2C1->CR2   |= ( I2C_CR2_RD_WRN );    // set READ mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
	I2C1->CR2   |=  ( 1 << I2C_CR2_NBYTES_Pos); // get 1 bytes (1 data)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
	I2C1->CR2   |=  ( EEPROM_ADDRESS << (I2C_CR2_SADD_Pos+1) ); // device addr SHL 1
	I2C1->CR2   |=    I2C_CR2_START;       // start I2C READ op
	while(!(I2C1->ISR & I2C_ISR_TXIS)) ;   // wait for start condition to transmit
	read_data = I2C1->RXDR;
	return read_data;


}


void EEPROM_write (uint16_t address ) {

	// build EEPROM transaction
	I2C1->CR2   &= ~( I2C_CR2_RD_WRN );    // set WRITE mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
	I2C1->CR2   |=  ( 3 << I2C_CR2_NBYTES_Pos); // write 3 bytes (2 addr, 1 data)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
	I2C1->CR2   |=  ( EEPROM_ADDRESS << (I2C_CR2_SADD_Pos+1) ); // device addr SHL 1
	I2C1->CR2   |=    I2C_CR2_START;       // start I2C WRITE op

	/* USER wait for I2C_ISR_TXIS to clear before writing each Byte, e.g. ... */
	while(!(I2C1->ISR & I2C_ISR_TXIS)) ;   // wait for start condition to transmit
	I2C1->TXDR = (EEPROM_MEMORY_ADDR >> 8); // xmit MSByte of address
	while(!(I2C1->ISR & I2C_ISR_TXIS)) ;   // wait for start condition to transmit
	I2C1->TXDR = (EEPROM_MEMORY_ADDR & 0xFF); // xmit LSByte of address
	while(!(I2C1->ISR & I2C_ISR_TXIS)) ;   // wait for start condition to transmit
	I2C1->TXDR = (0xBB); // xmit data			/////////////////////////////////////////////////////////////////////////////
	/* address high, address low, data  -  wait at least 5 ms before READ
	   the READ op has new NBYTES (WRITE 2 then READ 1) & new RD_WRN for 3rd Byte */
}
