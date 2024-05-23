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
 * NOTES
 * See eeprom.c for more details
 * EEPROM_init() must be modified for pins that use AFR[0]
 *******************************************************************************
/* USER CODE END Header */
#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include <stdint.h>
#include "stm32l496xx.h"

/* Define EEPROM parameters; Input Pins/Parameters Used */
#define EEPROM_PORT			GPIOB
#define EEPROM_PORT_CLOCK	RCC_AHB2ENR_GPIOBEN
#define EEPROM_NUM			2					// Number of pins used
#define EEPROM_PINS			{8, 9}				// {SCL, SDA}
#define EEPROM_AF			0x04				// If SCL/SDA have same AF
#define EEPROM_SLAVE_ADDR	0x52				// Match LSnibble to A0,A1,A2
#define EEPROM_MEMORY_ADDR	0x1256				// Enter 15-bit address
#define EEPROM_RNG			0xB9				// Enter random data

/* Declare EEPROM functions */
void 	EEPROM_init		( void );
uint8_t EEPROM_read 	( uint16_t address );
void 	EEPROM_write 	( uint16_t address, uint8_t data );

#endif /* INC_EEPROM_H_ */
