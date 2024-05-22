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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "delay.h"
#include "eeprom.h"
#include "keypad.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

uint8_t data = 0;

int main(void)
{




  HAL_Init();
  SystemClock_Config();
  EEPROM_init();
  Keypad_Config();


  	// LED 2 Configuration:
  	// configure GPIO pin PB7 for:
  	// output mode, no pull up or down, high speed,
  	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
  	GPIOB->MODER &= ~(GPIO_MODER_MODE7);
  	GPIOB->MODER |= (GPIO_MODER_MODE7_0);
  	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT7);
  	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD7);
  	GPIOB->OSPEEDR |=  (3 << GPIO_OSPEEDR_OSPEED7_Pos);

//	// Example for LED pin config
//  GPIOC->MODER   &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);		//Clears mode
//  GPIOC->MODER   |=  (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0);	//Sets mode to 01
//  GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);		//Sets type to 0 (push/pull)
//  GPIOC->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);		//Sets to 00(No PU or PD)
//  GPIOC->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos) |			//Very High Speed
//					  (3 << GPIO_OSPEEDR_OSPEED1_Pos));
//  GPIOC->BRR = (GPIO_PIN_0 | GPIO_PIN_1); // preset PC0, PC1 to 0




  uint8_t pad = 0;

  while (1)
  {
	  if (Keypad_IsAnyKeyPressed()) {
		  pad = Keypad_WhichKeyIsPressed();
		  if (pad == '1') {
			  GPIOB->ODR &= ~(GPIO_PIN_7);
		  }
		  else if (pad == '2') {
			  GPIOB->ODR |= (GPIO_PIN_7);
		  }
		  else if (pad == '*') {
			  printf("we really good \n");
				EEPROM_write(0xFFFF);
		  }
		  else if (pad == '#') {
			  printf("Got here\n");
			  data = EEPROM_read(0xFFFF);
			  printf("Data read %d\n", data);
		  }
		  printf("Key is %c\n", pad);
	  }
	  if (data == EEPROM_RNG) {
		  GPIOB->ODR |= (GPIO_PIN_7);
	  }
  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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


/* USER CODE BEGIN 4 */

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
