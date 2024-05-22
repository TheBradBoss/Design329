/*
 * eeprom.h
 *
 *  Created on: May 20, 2024
 *      Author: bradl
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_PORT			GPIOB
#define EEPROM_PORT_CLOCK	RCC_AHB2ENR_GPIOBEN
#define EEPROM_NUM			2
#define EEPROM_PINS			{8, 9}
#define EEPROM_ADDRESS		0x52
#define EEPROM_MEMORY_ADDR	0x1234
#define EEPROM_RNG			0xBB


#endif /* INC_EEPROM_H_ */
