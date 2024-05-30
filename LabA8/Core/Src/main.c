/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
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
 * CITATIONS:

 * chatgpt: I need a function in c that takes in an array with 20 16-bit numbers and returns the average, maximum, and minimum of the numbers
 * chat: I need a c program that turns 15 into the string "15"
 ******************************************************************************
 * NOTES:
 * lcd uses D pins 0-5
 * keypad uses C pins 0-8
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "delay.h"
#include <stdio.h>
#include <stdint.h>

#define BUTTON1_PORT	GPIOC
#define BUTTON1_I	GPIO_PIN_13


void SystemClock_Config(void);
uint8_t button1(void);
char* countToString(uint16_t number );
char* voltToString(uint16_t number );

// Declare global variable
uint8_t debounce_state = 0;	// If key is currently pressed ==1


void main(void)
{
	HAL_Init();
	SystemClock_Config();
	SysTick_Init();

	LPUART_init();
	ADC_init();

	// User Button Configuration to command relay:
	// configure GPIO pin PC13 for:
	// input mode, with pull down
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);


	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER   &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE1);		//Clears mode
	GPIOA->MODER   |=  (GPIO_MODER_MODE2_0 | GPIO_MODER_MODE1_0);	//Sets mode to 01
	GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT1);		//Sets type to 0 (push/pull)
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD1);		//Sets to 00(No PU or PD)
	GPIOA->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED2_Pos) |			//Very High Speed
			(3 << GPIO_OSPEEDR_OSPEED1_Pos));
	GPIOA->BRR = (GPIO_PIN_2 | GPIO_PIN_1); // preset PC0, PC1 to 0


	//uint16_t conv_results[20];	// Holds 20 samples 12-bit ADC conversions

	// holds unconverted values
	uint16_t raw_ave = 0;
	uint16_t raw_min = 0;
	uint16_t raw_max = 0;

	// In millivolts
	uint16_t volt_ave = 0;
	uint16_t volt_min = 0;
	uint16_t volt_max = 0;

	uint8_t display_count = 0;
	uint16_t coil_current = 0;

	LPUART_ESC_print("[2J");
	LPUART_ESC_print("[H");


	while (1)
	{


		if (GPIOC->IDR & GPIO_PIN_13) {
			GPIOA->ODR |= GPIO_PIN_1;
			GPIOA->ODR &= ~(GPIO_PIN_2);
		}
		else {
			GPIOA->ODR |= GPIO_PIN_2;
			GPIOA->ODR &= ~(GPIO_PIN_1);
		}

		display_count++;

		delay_us(1000);
		ADC_sample( &raw_ave, &raw_min, &raw_max);
		printf("Raw Average: %d\n", raw_ave);
		printf("Raw Minimum: %d\n", raw_min);
		printf("Raw Maximum: %d\n", raw_max);
		volt_ave = ADC_raw_to_volt(raw_ave);
		volt_min = ADC_raw_to_volt(raw_min);
		volt_max = ADC_raw_to_volt(raw_max);
		printf("Volt Average: %d\n", volt_ave);
		printf("Volt Minimum: %d\n", volt_min);
		printf("Volt Maximum: %d\n", volt_max);

		if (display_count > 200) {	// update display every few seconds

			coil_current = (volt_ave * 1000) / 16000;

			LPUART_ESC_print("[2J");
			LPUART_ESC_print("[H");
			LPUART_ESC_print("[10;10H");	// Cursor at about center
			LPUART_print("ADC counts volts");
			LPUART_ESC_print("[11;10H");
			LPUART_print("MIN  ");
			LPUART_print(countToString(raw_min));
			LPUART_print("  ");
			LPUART_print(voltToString(volt_min));
			LPUART_print(" V");
			LPUART_ESC_print("[12;10H");
			LPUART_print("MAX  ");
			LPUART_print(countToString(raw_max));
			LPUART_print("  ");
			LPUART_print(voltToString(volt_max));
			LPUART_print(" V");
			LPUART_ESC_print("[13;10H");
			LPUART_print("AVG  ");
			LPUART_print(countToString(raw_ave));
			LPUART_print("  ");
			LPUART_print(voltToString(volt_ave));
			LPUART_print(" V");
			LPUART_ESC_print("[H");
			LPUART_ESC_print("[14;10H");
			LPUART_print("coil current = ");
			LPUART_print(voltToString(coil_current));
			LPUART_print(" A");
			display_count = 0;
		}
	}



}


//void parse4DigitsToString(uint16_t number, char str[]) {
//    // Extract each digit
//	uint16_t digits[4];
//
//    digits[0] = (number / 1000) % 10;  // Thousands
//    digits[1] = (number / 100) % 10;   // Hundreds
//    digits[2] = (number / 10) % 10;    // Tens
//    digits[3] = number % 10;           // Units
//
//    for (int i = 0 ; i < 4 ; i++) {
//    	str[i] = digits[i] + '0';
//    }
//    str[4] = '\0';
//}

char* countToString(uint16_t number ) {
    // Extract each digit
	static char str[5];
	uint16_t digits[4];

    digits[0] = (number / 1000) % 10;  // Thousands
    digits[1] = (number / 100) % 10;   // Hundreds
    digits[2] = (number / 10) % 10;    // Tens
    digits[3] = number % 10;           // Units

    for (int i = 0 ; i < 4 ; i++) {
    	str[i] = digits[i] + '0';
    }
    str[4] = '\0';
    return str;
}

char* voltToString(uint16_t number ) {
    // Extract each digit
	static char str[6];
	uint16_t digits[4];

    digits[0] = (number / 1000) % 10;  // Thousands
    digits[1] = (number / 100) % 10;   // Hundreds
    digits[2] = (number / 10) % 10;    // Tens
    digits[3] = number % 10;           // Units

    str[0] = digits[0] + '0';
    str[1] = '.';
    for (int i = 1 ; i < 4 ; i++) {
    	str[i+1] = digits[i] + '0';
    }
    str[5] = '\0';
    return str;
}

//void parse16BitToString(uint16_t number, uint16_t digits[4]) {
//    // Extract each digit
//    digits[0] = (number / 1000) % 10;  // Thousands
//    digits[1] = (number / 100) % 10;   // Hundreds
//    digits[2] = (number / 10) % 10;    // Tens
//    digits[3] = number % 10;           // Units
//
//}

uint8_t button1(void) {
	// Debounce function for buttons.
	// Add appropriate defines in main.h
	uint16_t settle = 1000;	// Small delay for debounce to settle
	uint16_t debounce_count = 0;
	if ( debounce_state == 0 ) {
		while ( (BUTTON1_PORT->IDR & BUTTON1_I) != 0 ) {	// Button pressed
			debounce_count++;
			if ( debounce_count > settle ) {			// Button high for awhile
				debounce_state = 1;
				return 1;
			}
		}
		return 0;
	}
	if ( debounce_state == 1 ) {	// Button was pressed
		if ( (BUTTON1_PORT->IDR & BUTTON1_I) == 0 ) {		// Button released
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
