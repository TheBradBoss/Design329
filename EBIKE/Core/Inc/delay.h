/*
*******************************************************************************
 * @file           : delay.h
 * @brief          : add microsecond delays
 * project         : EE 329 S'24 Assignment 3
 * author(s)       : John Penvenne (jlp) - jpenvenn@calpoly.edu
 * version         : 0.1
 * date            : 240424
 * compiler        : STM32CubeIDE v.1.15.0 Build: 20695_20240315_1429 (UTC)
 * target          : NUCLEO-L496ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * See delay.c for details
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include <stdint.h>

void SysTick_Init(void);
void delay_us(const uint32_t time_us);

#endif /* INC_DELAY_H_ */
