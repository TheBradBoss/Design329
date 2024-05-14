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
#include "keypad.h"
#include "lcd.h"
#include "delay.h"
#include "dac.h"


#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


uint8_t pad = 0;
uint8_t digit_count = 3;
uint16_t millivolt = 0;
//uint16_t dac_command = 0;

void main(void)
{

    /* Configure the system clock */

	HAL_Init();
	SystemClock_Config();
  Keypad_Config();
  LCD_init();
  DAC_init();

  // DONT NEED LCD FOR A5, HERE FOR LOOKS
  delay_us(2000);
	LCD_command( CURSOR_SHIFT_L );		// Shift cursor to the left
	LCD_command( CURSOR_OFF );			// Display, cursor, position on
	LCD_command( CURSOR_RIGHT );		// Cursor moves right, no shift
	LCD_command( CLEAR_HOME );
	LCD_write_string("EE 329 A5 DAC");
	LCD_command(LINE_TWO);
	LCD_write_string("*=reset V=0.00V");
  //GPIOD->ODR = 0x3F;
  while (1)
  {



	  if (Keypad_IsAnyKeyPressed()) {
		  pad = Keypad_WhichKeyIsPressed();
		  printf("Key pressed is %d\n", pad);
		  if ( pad == 42 ) {
			  millivolt = 0;
			  digit_count = 3;
		  }
		  else if ( (pad > 57) || (pad < 46) ) {
			  ; // ignore key press
		  }
		  else {
			  printf("Key pressed is %d\n", (pad-'0'));
			  millivolt += ((pad - '0') * (pow(10, digit_count)));
			  digit_count --;
			  printf("Millivolt is %d\n", millivolt);

		  }
	  }
	  if (digit_count == 0) {
		  DAC_write( DAC_volt_conv( millivolt ) );
		  printf("Command sent is %d\n", DAC_volt_conv( millivolt ));
		  digit_count = 3;
		  millivolt = 0;

	  }
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


















/* Code that takes three digit codes consecutively and outputs them up to Vdd.
 * Does not include reset key or display of lcd
 *  */
//#include "main.h"
//#include "keypad.h"
//#include "lcd.h"
//#include "delay.h"
//#include "dac.h"
//
//
//#include <stdio.h>
//#include <stdint.h>
//#include <math.h>
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//
//
//uint8_t pad = 0;
//uint8_t digit_count = 3;
//uint16_t millivolt = 0;
////uint16_t dac_command = 0;
//
//void main(void)
//{
//
//    /* Configure the system clock */
//
//	HAL_Init();
//	SystemClock_Config();
//  Keypad_Config();
//  LCD_init();
//  DAC_init();
//
//  delay_us(2000);
//	LCD_command( CURSOR_SHIFT_L );		// Shift cursor to the left
//	LCD_command( CURSOR_OFF );			// Display, cursor, position on
//	LCD_command( CURSOR_RIGHT );		// Cursor moves right, no shift
//	LCD_command( CLEAR_HOME );
//	LCD_write_string("EE 329 A5 DAC");
//	LCD_command(LINE_TWO);
//	LCD_write_string("PUSH SW TO TRIG");
//  //GPIOD->ODR = 0x3F;
//  while (1)
//  {
//
//
//
//	  if (Keypad_IsAnyKeyPressed()) {
//		  pad = Keypad_WhichKeyIsPressed();
//		  if ( (pad > 57) || (pad < 46) ) {
//			  ; // ignore key press
//		  }
//		  else {
//			  printf("Key pressed is %d\n", (pad-'0'));
//			  millivolt += ((pad - '0') * (pow(10, digit_count)));
//			  digit_count --;
//			  printf("Millivolt is %d\n", millivolt);
//
//		  }
//	  }
//	  if (digit_count == 0) {
//		  DAC_write( DAC_volt_conv( millivolt ) );
//		  printf("Command sent is %d\n", DAC_volt_conv( millivolt ));
//		  digit_count = 3;
//		  millivolt = 0;
//
//	  }
//  }
//
//
//  /* USER CODE END 3 */
//}
