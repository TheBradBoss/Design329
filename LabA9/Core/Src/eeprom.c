/* USER CODE BEGIN Header */
/*******************************************************************************
 * @file           : eeprom.c
 * @brief          : eeprom configuration and WRITE/READ operations.
 * project         : EE 329 S'24 Lab A9
 * author(s)       : Bradley Buzzini, Jaden Nguyen
 * version         : 0.2
 * date            : 240523
 * compiler        : STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target          : NUCLEO-L496ZG
 * clocks          : 2 MHz PCLK to I2C1, 4 MHz MSI to AHB2
 * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * EEPROM Plan:
 * Configure I2C1, set I2C1 to 7-bit control and give peripheral address
 * For write, send 3 bytes: MSbyte address, LSbyte address, data
 * For read, send 2 bytes: MSbyte address, LSbyte address, then get 1 byte data
 *******************************************************************************
 * EEPROM WIRING (pinout NUCLEO-L496ZG = L4A6ZG)
 *    peripheral - Nucleo I/O
 * eeprom 1  A0  - GND = CN8 - 11
 * eeprom 2  A1  - 3V3 = CN8 - 7
 * eeprom 3  A3  - GND = CN8 - 11
 * eeprom 4  VSS - GND = CN8 - 11
 * eeprom 5  VCC - 3V3 = CN8 - 7
 * eeprom 6  WP  - GND = CN8 - 11
 * eeprom 7  SCL - PB8 = CN7 - 2  - AF4, OD
 * eeprom 8  SDA - PB9 = CN7 - 4  - AF4, OD
 *******************************************************************************
 * REVISION HISTORY
 * 0.1 240520 BB  Able to transmit a full WRITE op.
 * 0.2 240523 BB  Contains up to deliverable 6.
 *******************************************************************************
 * TODO
 *
 *******************************************************************************
 * NOTES
 * EEPROM_init() must be modified for pins that use AFR[0]
 *******************************************************************************
 * REFERENCES
 * Code adapted from A9 in manual
 *******************************************************************************
/* USER CODE END Header */
#include "eeprom.h"
#include "delay.h"

void EEPROM_init( void ) {
	SysTick_Init();		// Initialize delay function

	/* Configure PB8 and PB9 for I2C alternate functions SCL and SDA */
	RCC->AHB2ENR |= (EEPROM_PORT_CLOCK);
	uint32_t eeprom_pins[] = EEPROM_PINS;
	for (uint32_t i = 0; i < EEPROM_NUM ; i++) {
		uint32_t pin = eeprom_pins[i];
		EEPROM_PORT->MODER &= ~(0x3 << (pin * 2));
		EEPROM_PORT->MODER |= (0x2 << (pin * 2));	// Alternate Function
		EEPROM_PORT->OTYPER &= ~(0x1 << pin);
		EEPROM_PORT->OTYPER |= (0x1 << pin);		// Open Drain
		EEPROM_PORT->OSPEEDR &= ~(0x3 << (pin * 2));
		EEPROM_PORT->OSPEEDR |= (0x3 << (pin * 2));	// Highest speed
		EEPROM_PORT->PUPDR &= ~(0x3 << (pin * 2));	// NO PU/PD
		EEPROM_PORT->BRR |= (0x1 << (pin * 1));		//initialize off
		EEPROM_PORT->AFR[1] &= ~((0x000F << ((pin-8) * 4))); 	// clear AF
		EEPROM_PORT->AFR[1] |=  ((EEPROM_AF << ((pin-8) * 4))); // AF to 4
		//Pin B8: configure AFR for I2C1_SCL function
		//Pin B9: configure AFR for I2C1_SDA function
	}

	/* Configure I2C */
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;  // enable I2C bus clock
	I2C1->CR1   &= ~( I2C_CR1_PE );        // put I2C into reset (release SDA, SCL)
	I2C1->CR1   &= ~( I2C_CR1_ANFOFF );    // filters: enable analog
	I2C1->CR1   &= ~( I2C_CR1_DNF );       // filters: disable digital
	I2C1->TIMINGR = 0x00000509;            // 100 kHz SYSCLK timing from CubeMX
	I2C1->CR2   |=  ( I2C_CR2_AUTOEND );   // auto send STOP after transmission
	I2C1->CR2   &= ~( I2C_CR2_ADD10 );     // 7-bit address mode
	I2C1->CR1   |=  ( I2C_CR1_PE );        // enable I2C
}

uint8_t EEPROM_read (uint16_t address ) {
	// Will return data from EEPROM address given
	uint8_t data_read = 0;

	/* READ has new NBYTES (WRITE 2 then READ 1) & new RD_WRN for 3rd Byte */
	/* build EEPROM read transaction (Write Address) */
	I2C1->CR2   &= ~( I2C_CR2_RD_WRN );    	// set WRITE mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    	// clear Byte count
	I2C1->CR2   |=  ( 2 << I2C_CR2_NBYTES_Pos); // write 2 bytes (2 addr)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      	// clear device address
	// device addr SHL 1
	I2C1->CR2   |=  ( EEPROM_SLAVE_ADDR << (I2C_CR2_SADD_Pos+1) );
	I2C1->CR2   |=    I2C_CR2_START;       	// start I2C READ op write

	while(!(I2C1->ISR & I2C_ISR_TXE)) ;   	// wait for start condition
	I2C1->TXDR = ( address >> 8 ); 			// xmit MSByte of address
	while(!(I2C1->ISR & I2C_ISR_TXE)) ;   	// wait for start condition
	I2C1->TXDR = ( address & 0xFF ); 		// xmit LSByte of address

	delay_us(5000);	// Wait for address to settle in EEPROM

	/* build EEPROM read transaction (Read Data) */
	I2C1->CR2   |= ( I2C_CR2_RD_WRN ); 		// set READ mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    	// clear Byte count
	I2C1->CR2   |=  ( 1 << I2C_CR2_NBYTES_Pos); // get 1 bytes (1 data)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      	// clear device address
	// device addr SHL 1
	I2C1->CR2   |=  ( EEPROM_SLAVE_ADDR << (I2C_CR2_SADD_Pos+1) );
	I2C1->CR2   |=    I2C_CR2_START;       	// start I2C READ op

	while(!(I2C1->ISR & I2C_ISR_RXNE)) ;	// wait for start condition to transmit
	data_read = (I2C1->RXDR);				// get data

	return data_read;
}

void EEPROM_write (uint16_t address, uint8_t data ) {
	/* Write address high, address low, data - wait at least 5 ms before READ*/
	/* The WRITE op has 3 NBYTES and low RD_WRN for write*/
	// build EEPROM write transaction
	I2C1->CR2   &= ~( I2C_CR2_RD_WRN );    // set WRITE mode
	I2C1->CR2   &= ~( I2C_CR2_NBYTES );    // clear Byte count
	I2C1->CR2   |=  ( 3 << I2C_CR2_NBYTES_Pos); // write 3 bytes (2 addr, 1 data)
	I2C1->CR2   &= ~( I2C_CR2_SADD );      // clear device address
	// device addr SHL 1
	I2C1->CR2   |=  ( EEPROM_SLAVE_ADDR << (I2C_CR2_SADD_Pos+1) );
	I2C1->CR2   |=    I2C_CR2_START;       // start I2C WRITE op

	/* Wait for I2C_ISR_TXE to clear before writing each Byte*/
	while(!(I2C1->ISR & I2C_ISR_TXE)) ;		// wait for start condition
	I2C1->TXDR = ( address >> 8); 			// xmit MSByte of address
	while(!(I2C1->ISR & I2C_ISR_TXE)) ; 	// wait for start condition
	I2C1->TXDR = ( address & 0xFF); 		// xmit LSByte of address
	while(!(I2C1->ISR & I2C_ISR_TXE)) ; 	// wait for start condition
	I2C1->TXDR = ( data ); 					// xmit data

	delay_us(5000);		// Wait 5ms for data to write to EEPROM
}
