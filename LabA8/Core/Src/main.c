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


void SystemClock_Config(void);
void parse16Bit(uint16_t number, uint16_t digits[4]);



void main(void)
{
	HAL_Init();
	SystemClock_Config();
	SysTick_Init();


	ADC_init();

	// User Button Configuration to command relay:
	// configure GPIO pin PC13 for:
	// input mode, with pull down
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);


	//uint16_t conv_results[20];	// Holds 20 samples 12-bit ADC conversions

	// holds unconverted values
	uint16_t raw_ave = 0;
	uint16_t raw_min = 0;
	uint16_t raw_max = 0;

	// In millivolts
	uint16_t volt_ave = 0;
	uint16_t volt_min = 0;
	uint16_t volt_max = 0;


	while (1)
	{
		delay_us(3000);
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
	}

}


void parse16Bit(uint16_t number, uint16_t digits[4]) {
    // Extract each digit
    digits[0] = (number / 1000) % 10;  // Thousands
    digits[1] = (number / 100) % 10;   // Hundreds
    digits[2] = (number / 10) % 10;    // Tens
    digits[3] = number % 10;           // Units
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
