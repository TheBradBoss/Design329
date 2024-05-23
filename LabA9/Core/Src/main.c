/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file        : main.c
 * @brief       : Write to EEPROM, read EEPROM, check if read matches write.
 * project		: EE 329 S'24 Lab A9
 * authors		: Bradley Buzzini, Jaden Nguyen
 * version		: 0.2
 * date			: 240523
 * computer		: STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target		: NUCLEO-L496ZG
 * clock		: 4 MHz MSI to AHB2
 *
 * Citations:
 * Sample Code from A9 Manual
 ******************************************************************************
 * REVISION HISTORY
 * 0.1 240520 BB  Able to transmit a full WRITE op.
 * 0.2 240523 BB  Contains up to deliverable 6.
 *******************************************************************************
 * TODO
 * Add keypad and allow user to read/write data
 * Add LCD to show user input and output
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "delay.h"
#include "eeprom.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{
	/* Initialize Board and Peripherals */
	HAL_Init();
	SystemClock_Config();
	EEPROM_init();

	// LED 2 Configuration:
	// configure GPIO pin PB7 for:
	// output mode, no pull up or down, high speed,
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
	GPIOB->MODER &= ~(GPIO_MODER_MODE7);
	GPIOB->MODER |= (GPIO_MODER_MODE7_0);
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT7);
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD7);
	GPIOB->OSPEEDR |=  (3 << GPIO_OSPEEDR_OSPEED7_Pos);

	uint8_t data_read = 0;	// Will hold data from EEPROM

	/* Write random data to EEPROM and then grab it from same location */
	EEPROM_write(EEPROM_MEMORY_ADDR, EEPROM_RNG);
	data_read = EEPROM_read(EEPROM_MEMORY_ADDR);

	while (1)
	{
		if (data_read == EEPROM_RNG) {	// Data in EEPROM matches RNG
			GPIOB->ODR |= (GPIO_PIN_7);	// Turn on on-board LED
		}
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
