/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file        : main.c
 * @brief       : Switch relay coild & use ADC to measure coil current
 * project		: EE 329 S'24 Lab A8
 * authors		: Bradley Buzzini, Jack Ryan
 * version		: 0.3
 * date			: 240530
 * computer		: STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target		: NUCLEO-L496ZG
 * clock		: 4 MHz MSI to AHB2
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
 * TODO
 *
 *******************************************************************************
 * PIN DESCRIPTIONS
 *      peripherals – Nucleo I/O
 * BJT NPN1 Emitter - PA0 = CN10 - 29 - analog mode, ADC
 * BJT NPN1    Base - PA1 = CN10 - 11 - OUT
 * BJT NPN2    Base - PA2 = CN10 - 13 - OUT
 * Emitter Resistor - GND = CN9  - 23
 *******************************************************************************
 * BREADBOARD
 * Repeat the following for each relay coil:
 *   Connect NPN as a pulldown in series with the relay coil (collector to (-))
 *   Add a flyback diode in parallel with the coil for protection
 *   Choose a base and emitter resistance with the 'rule of ten' (560 and 16)
 *   Connect PA1 or PA2 in series with the base resistance
 *   Connect ground to the bottom of the emitter resistance
 *   Connect +5V to the positive terminal of the relay coil
 *   Use the ADC to measure emitter resistor voltage
 *******************************************************************************
 * DISPLAY EXAMPLE
 * 		####
 * 		|----|----|----|----|----|----|
 * 		0   0.5  1.0  1.5  2.0  2.5  3.0
 *
 * 		ADC counts volts
 * 		MIN  0381  0.307 V
 * 		MAX  0383  0.308 V
 * 		AVG  0382  0.307 V
 * 		coil current = 0.085 A
 *
 *******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "delay.h"
#include "LPUART.h"

#include <stdio.h>
#include <stdint.h>

void SystemClock_Config(void);
uint8_t button1(void);
void DisplayResults ( void );

/* Declare global variables */
uint8_t debounce_state = 0;	// If key is currently pressed ==1

// holds ADC digital output
uint16_t adc_avg = 0;
uint16_t adc_min = 0;
uint16_t adc_max = 0;

// holds ADC digital output converted to millivolts
uint16_t volt_avg = 0;
uint16_t volt_min = 0;
uint16_t volt_max = 0;
uint16_t volt_bar = 0;	// EC: bar length

uint16_t coil_current = 0;	// Current held in millamps

void main(void)
{
	// System Initialization
	HAL_Init();
	SystemClock_Config();
	SysTick_Init();
	LPUART_init();
	ADC_init();

	// User Button (PC13) Configuration to command relay:
	// input mode, with pull down
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);

	// Configure relay coil control pins PA1 & PA2:
	// Output, push/pull, no PD/PU, high speed
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER   &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE1);
	GPIOA->MODER   |=  (GPIO_MODER_MODE2_0 | GPIO_MODER_MODE1_0);
	GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT1);
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD1);
	GPIOA->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED2_Pos) |
			(3 << GPIO_OSPEEDR_OSPEED1_Pos));
	GPIOA->BRR = (GPIO_PIN_2 | GPIO_PIN_1);

	uint8_t display_count = 0;

	LPUART_ESC_print(CLEAR_SCREEN);
	LPUART_ESC_print(RETURN_CURSOR);

	while (1)
	{
		delay_us(1000);

		// Collect and aggregate ADC samples
		ADC_sample( &adc_avg, &adc_min, &adc_max);
		volt_avg = ADC_raw_to_volt(adc_avg);
		volt_min = ADC_raw_to_volt(adc_min);
		volt_max = ADC_raw_to_volt(adc_max);
		volt_bar = ((volt_avg / 100)) + 1;	// EC: Extra credit

		if (display_count > 200) {	// update serial display occasionally
			DisplayResults();
			display_count = 0;
		}
		display_count++;

		if (button1()) {	// Switch coil one way
			GPIOA->ODR |= GPIO_PIN_1;
			GPIOA->ODR &= ~(GPIO_PIN_2);
			coil_current = ((volt_avg*1000)/16294) -
					((3300000-(700000+(volt_avg*1000)))/553050);	// Ie - Ib
		}
		else {				// Switch coil other way
			GPIOA->ODR |= GPIO_PIN_2;
			GPIOA->ODR &= ~(GPIO_PIN_1);
			coil_current = (volt_avg*1000)/16294;	// NPN OFF: Ib = 0

		}

		if (display_count > 200) {	// update serial display occasionally
			display_count = 0;
		}
		display_count++;
	}
}

void DisplayResults ( void ) {
	LPUART_ESC_print(CLEAR_SCREEN);
	LPUART_ESC_print(RETURN_CURSOR);

	// Extra credit
	LPUART_ESC_print("[6;10H");
	for (int i = 0; i < volt_bar; i++) {
		LPUART_print("#");
	}
	//

	LPUART_ESC_print("[7;10H");	// Move cursor to about center
	LPUART_print("|----|----|----|----|----|----|");
	LPUART_ESC_print("[8;10H");
	LPUART_print("0   0.5  1.0  1.5  2.0  2.5  3.0");
	LPUART_ESC_print("[10;10H");
	LPUART_print("ADC counts volts");
	LPUART_ESC_print("[11;10H");
	LPUART_print("MIN  ");
	LPUART_print(ADC_CountToString(adc_min));
	LPUART_print("  ");
	LPUART_print(ADC_VoltToString(volt_min));
	LPUART_print(" V");
	LPUART_ESC_print("[12;10H");
	LPUART_print("MAX  ");
	LPUART_print(ADC_CountToString(adc_max));
	LPUART_print("  ");
	LPUART_print(ADC_VoltToString(volt_max));
	LPUART_print(" V");
	LPUART_ESC_print("[13;10H");
	LPUART_print("AVG  ");
	LPUART_print(ADC_CountToString(adc_avg));
	LPUART_print("  ");
	LPUART_print(ADC_VoltToString(volt_avg));
	LPUART_print(" V");
	LPUART_ESC_print(RETURN_CURSOR);
	LPUART_ESC_print("[14;10H");
	LPUART_print("coil current = ");
	LPUART_print(ADC_VoltToString(coil_current));
	LPUART_print(" A");
}

uint8_t button1(void) {
	// Debounce function for buttons.
	// Change BUTTON defines in main.h
	uint16_t settle = 1000;			// Small delay for debounce to settle
	uint16_t debounce_count = 0;	// Compare to settle time

	if ( debounce_state == 0 ) {
		while ( (BUTTON1_PORT->IDR & BUTTON1_IO) != 0 ) {	// Button pressed
			debounce_count++;
			if ( debounce_count > settle ) {			// Button high for awhile
				debounce_state = 1;
				return 1;
			}
		}
		return 0;
	}
	if ( debounce_state == 1 ) {	// Button was pressed
		if ( (BUTTON1_PORT->IDR & BUTTON1_IO) == 0 ) {		// Button released
			debounce_state = 0;
			return 0;
		}
		else return  0;
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

int _write(int file, char *ptr, int len)
{
	(void)file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
