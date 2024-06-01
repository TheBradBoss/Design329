/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file        : main.c
 * @brief       : Main program body
 * project		: EE 329 S'24 Lab A1
 * authors		: Bradley Buzzini, Daniel Hoefer
 * version		: 0.2
 * date			: 240517
 * computer		: STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target		: NUCLEO-L496ZG
 *
 * Citations:
 * OpenAI. (2024-May-14). ChatGPT, query synopsis "Write c code that takes
 * 	in Millivolts in individual digits and stores it as Millivolts"
 * 	to  https://chat.openai.com/chat
 * Sample Code from A5 Manual
 ******************************************************************************
 * REVISION HISTORY
 * 0.1 240513 BB  Includes up to deliverable 5
 * 0.2 240516 DH  Calibrated and added code to test DAC performance.
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
#include "keypad.h"
#include "lcd.h"
#include "delay.h"
#include "dac.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

uint8_t pad = 0;			// Record keypad press
uint8_t digit_count = 3;	// 3 Digit Code Count
uint16_t millivolt = 0;
uint16_t perform = 1;		// Which performance voltage

void main(void)
{

	/* Initialize Components */
	HAL_Init();
	SystemClock_Config();
	Keypad_Config();
	LCD_init();
	DAC_init();

	while (1)
	{
		/* Start Deliverable 5*/
		if (Keypad_IsAnyKeyPressed()) {
			pad = Keypad_WhichKeyIsPressed();	// Get key
			if ( pad == 42 ) {					// '*' key pressed
				// Reset digit count and user input
				millivolt = 0;
				digit_count = 3;
			}
			else if ( (pad > 57) || (pad < 46) ) {
				; // ignore key press if not a number
			}
			else {
				// Convert keypad presses to millivolts
				millivolt += ((pad - '0') * (pow(10, digit_count)));
				digit_count --;
			}
		}
		if (digit_count == 0) {							// 3 digit code entered
			DAC_write( DAC_volt_conv( millivolt ) );	// Output input to DAC
			digit_count = 3;							// Reset digit count
			millivolt = 0;								// Reset user input

		}
		/* End Deliverable 5 */

		//	  /* Start Deliverable 7*/
		//	  // Make keypad press cycle through test codes
		//	  if (Keypad_IsAnyKeyPressed()) {
		//		  switch (perform) {
		//		  case 1:
		//			  DAC_write(0x3000);
		//			  break;
		//		  case 2:
		//			  DAC_write(0x1BB7);
		//			  break;
		//		  case 3:
		//			  DAC_write(0x34D8);
		//			  break;
		//		  case 4:
		//			  DAC_write(0x34D9);
		//			  break;
		//		  case 5:
		//			  DAC_write(0x39B1);
		//			  break;
		//		  case 6:
		//			  DAC_write(0x39B2);
		//			  break;
		//		  case 7:
		//			  DAC_write(0x1AEF);
		//			  break;
		//		  case 8:
		//			  DAC_write(0x1C7F);
		//			  break;
		//		  case 9:
		//			  perform = 0;
		//			  break;
		//		  }
		//		  perform++;
		//	  }
		//	  /* End Deliverable 7*/

	}
	/* USER CODE END 3 */
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

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */

/* USER CODE BEGIN 4 */
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
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
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
