/* USER CODE BEGIN Header */
/*******************************************************************************
 * EE 329 A4 Interrupts & Timers (Part D)
 *******************************************************************************
 * @file           : main.c
 * @brief          : Use LCD to display reaction triggered by timer interrupt
 * project         : EE 329 S'24 Assignment 4
 * authors         : Bradley Buzzini - Ethan Zane
 * version         : 0.3
 * date            : 240501
 * compiler        : STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target          : NUCLEO-L496ZG
 * clocks          : 4 MHz MSI to AHB2, APB1, and RNG
 * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * Plan and Execution
 * Initialize everything
 * When button presses, get random time and make interrupt start at time.
 * At that interrupt, turn on the LED and wait for a button press.
 * After button, grab time and display it on the LCD.
 * If a button isn't pressed in time, a different interrupt occurs.
 * Button press should reset everything.
 *******************************************************************************
 * REVISION HISTORY
 * 0.1 240429 BB  Included interrupts for part A,B, and C.
 * 0.2 240430 BB  Acquired random number upon button press.
 * 0.3 240430 BB  Met full criteria for part D operation.
 *******************************************************************************
 * TODO
 * Add a debounce mechanism to PBSW
 * Replace "state" var with enum type.
 * Add procedure for RNG failure in RNG function
 *******************************************************************************
 * REFERENCES
 * N/A
 *******************************************************************************
 * 45678-1-2345678-2-2345678-3-2345678-4-2345678-5-2345678-6-2345678-7-234567 */
/* USER CODE END Header */

#include "main.h"

/* Includes of outside modules*/
#include "delay.h"
#include "lcd.h"

/* Includes of standard modules*/
#include <stdio.h>
#include <stdint.h>

/* Function Declarations */
void SystemClock_Config(void);
void setup_TIM2( int iDutyCycle );
uint32_t get_rand_TIM2(void);

uint8_t state = 0;	// Track state of operations

void main(void)
{
	// Initialize HAL, Clock, ISR, and LCD
	HAL_Init();
	SystemClock_Config();
	LCD_init();
	setup_TIM2(CCR1_VALUE);

	// User Button Configuration:
	// configure GPIO pin PC13 for:
	// input mode, with pull down
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);

	// LED 2 Configuration:
	// configure GPIO pin PB7 for:
	// output mode, no pull up or down, high speed,
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
	GPIOB->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);
	GPIOB->MODER |= (GPIO_MODER_MODE7_0 | GPIO_MODER_MODE8_0);
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);
	GPIOB->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) | (3 << GPIO_OSPEEDR_OSPEED8_Pos));

	// Enable RNG
	RCC->CCIPR    |=  (0xC000000);			// MUX selects MSI to RNGclk
	RCC->AHB2ENR  |=  (RCC_AHB2ENR_RNGEN);	// Enable r/w to RNG regs
	RNG->CR       |=  (RNG_CR_RNGEN);		// RNG in polling mode

	uint32_t user_lag = 0;	// Initialize var to capture reaction time.


	while (1)
	{
		while ( state == 0 ) { 	// Reset everything
			TIM2->CR1 &= ~TIM_CR1_CEN;			// Turn off TIM2
			TIM2->CNT &= 0x00000000;			// Reset counter
			GPIOB->ODR &= ~GPIO_PIN_7;			// LED off
			LCD_command( CURSOR_SHIFT_L );		// Shift cursor to the left
			LCD_command( CURSOR_OFF );			// Display, cursor, position on
			LCD_command( CURSOR_RIGHT );		// Cursor moves right, no shift
			LCD_command( CLEAR_HOME );
			LCD_write_string("EE 329 A4 REACT");
			LCD_command(LINE_TWO);
			LCD_write_string("PUSH SW TO TRIG");
			state = 1;
		}

		while ( state == 1 ) {	// Wait to start game
			if ( (GPIOC->IDR & GPIO_PIN_13) == GPIO_PIN_13 ) { 	// Button press
				uint32_t rng_CCR1 = 0;					// Holds 32-bit RN
				uint32_t wait_in_s = 0;					// RN in milliseconds
				rng_CCR1 = TIM2_FIVE + get_rand_TIM2();	// add 5 seconds
				LCD_command(LINE_TWO);
				LCD_write_string("Get Ready!      ");

				TIM2->CCR1 = rng_CCR1;				// Set CCR1 to RNG MHz
				TIM2->ARR = rng_CCR1 + TIM2_TEN;	// Set ARR to RNG + 10s
				TIM2->CR1 |= TIM_CR1_CEN;			// Turn on TIM2
				state = 2;	// Wait for LED
			}
		}

		while ( state == 2 ) { // Wait for interrupt
			// Wait for LED on ISR
		}

		while ( state == 3 ) {	// LED has turned on
			if ( (GPIOC->IDR & GPIO_PIN_13) == GPIO_PIN_13 ) {	// Button press
				TIM2->CR1 &= ~TIM_CR1_CEN;						// Turn off TIM2
				user_lag = ((TIM2->CNT) - (TIM2->CCR1)) / 4000;	// Lag in ms
				delay_us(100000);								// for debounce
				TIM2->CNT &= 0x00000000;						// Reset counter
				GPIOB->ODR &= ~GPIO_PIN_7;						// LED off

				LCD_command( CLEAR_HOME );
				LCD_write_string("EE 329 A4 REACT");
				LCD_command(LINE_TWO);
				LCD_write_string("Time = ");
				LCD_write_time( user_lag );

				state = 5;
			}
		}

		while ( state == 4 ) { 	// 10s passed
			TIM2->CR1 &= ~TIM_CR1_CEN;	// Turn off TIM2
			TIM2->CNT &= 0x00000000;	// Reset counter
			GPIOB->ODR &= ~GPIO_PIN_7;	// LED off

			LCD_command( CLEAR_HOME );
			LCD_write_string("EE 329 A4 REACT");
			LCD_command(LINE_TWO);
			LCD_write_string("No Reaction!    ");

			state = 5;
		}

		while ( state == 5 ) {	// Wait for user to reset after game
			if ( (GPIOC->IDR & GPIO_PIN_13) == GPIO_PIN_13 ) { // Button press
				delay_us(100000);	// for debounce
				state = 0;
			}
		}

	}		// END WHILE
}	// END MAIN


uint32_t get_rand_TIM2 (void) {
	// Code to get random seconds (0-10s)
	// Returns CCR1 Value for 0-10s.
	uint32_t rng_val = 0;

	if ( ((RNG->SR) & 0x60) == 0 ) {		// Error bits == 0
		printf("No error \n");
		if ( ((RNG->SR) & 0x1) == 0x1 ) {	// RNG is ready
			rng_val = (RNG->DR) / RNG_CONVERTER;
		}
	}
	return rng_val;
}

///////////////////////////////////////////////////////////////////////////////
// ISR CODE

void setup_TIM2( int iDutyCycle ) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;           // enable clock for TIM2
	TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_UIE);  // enable event gen, rcv CCR1
	TIM2->ARR = TIME_UP;                             // ARR = T = counts @4MHz
	TIM2->CCR1 = iDutyCycle;                        // ticks for duty cycle
	TIM2->SR &= ~(TIM_SR_CC1IF | TIM_SR_UIF);       // clr IRQ flag in status reg
	NVIC->ISER[0] |= (1 << (TIM2_IRQn & 0x1F));     // set NVIC interrupt: 0x1F
	__enable_irq();                                 // global IRQ enable
	//TIM2->CR1 |= TIM_CR1_CEN;                       // start TIM2 CR1
}

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_CC1IF) {		// triggered by CCR1 event ...
		TIM2->SR &= ~(TIM_SR_CC1IF);    // manage the flag
		GPIOB->ODR |= GPIO_PIN_7;
		state = 3;	// LED on, wait for response
	}
	if (TIM2->SR & TIM_SR_UIF) {        // triggered by ARR event ...
		TIM2->SR &= ~(TIM_SR_UIF);      // manage the flag
		GPIOB->ODR &= ~GPIO_PIN_7;
		state = 4; // LED off, 10s passed
	}
}
///////////////////////////////////////////////////////////////////////////////


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








































///* USER CODE BEGIN Header */
///*******************************************************************************
// * EE 329 A4 Interrupts & Timers (Part B)
// *******************************************************************************
// * @file           : main.c
// * @brief          : Use single interrupt to output square wave
// * project         : EE 329 S'24 Assignment 4
// * authors         : Bradley Buzzini - Ethan Zane
// * version         : 0.1
// * date            : 240430
// * compiler        : STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
// * target          : NUCLEO-L496ZG
// * clocks          : 4 MHz MSI to AHB2, APB1, and RNG
// * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
// *******************************************************************************
// * Plan and Execution
// * Initialize everything
// * CCR1 will toggle the LED
// * Increment CCR1 to occur at 10 kHz
// *******************************************************************************
// * REVISION HISTORY
// * 0.1 240429 BB  Included interrupts for part A,B, and C.
// *******************************************************************************
// * TODO
// *******************************************************************************
// * REFERENCES
// * N/A
// *******************************************************************************
// * 45678-1-2345678-2-2345678-3-2345678-4-2345678-5-2345678-6-2345678-7-234567 */
///* USER CODE END Header */
//
//#include "main.h"
//
//#include <stdio.h>
//#include <stdint.h>
//
///* Function declarations */
//void SystemClock_Config(void);
//void setup_MCO_CLK(void);
//void setup_TIM2( int iDutyCycle );
//
//uint32_t my_val = CCR1_START;	// Part B
//
//int main(void)
//{
//  HAL_Init();
//  SystemClock_Config();
//
//	// configure GPIO pins PB7 (LED2) for:
//	// output mode, no pull up or down, high speed,
//	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
//	GPIOB->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);
//	GPIOB->MODER |= (GPIO_MODER_MODE7_0 | GPIO_MODER_MODE8_0);
//	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);
//	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);
//	GPIOB->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) | (3 << GPIO_OSPEEDR_OSPEED8_Pos));
//
//	// Part A
////	setup_TIM2(CCR1_VALUE);	// Setup and give duty cycle
//
//	// Part B
//	setup_MCO_CLK();		// Output 4 MHz clock
//	setup_TIM2(CCR1_START);	// Setup with large CCR1
//
//
//  while (1)
//  {
//	  // do nothing
//  }
//}
//
//void setup_MCO_CLK(void) {
//   // Enable MCO, select MSI (4 MHz source)
//   RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));
//   // Configure MCO output on PA8
//   RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
//   GPIOA->MODER   &= ~(GPIO_MODER_MODE8);    	// clear MODER bits
//   GPIOA->MODER   |=  (GPIO_MODER_MODE8_1);	// set alternate function mode
//   GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT8);     	// Push-pull output
//   GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD8);    	// no resistor
//   GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8);   // high speed
//   GPIOA->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);    	// select MCO function
//}
//
//void setup_TIM2( int iDutyCycle ) {
//   RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;           // enable clock for TIM2
//   TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_UIE);  // enable event gen, rcv CCR1
//   TIM2->ARR = PERIOD;                             // ARR = T = counts @4MHz
//   TIM2->CCR1 = iDutyCycle;                        // ticks for duty cycle
//   TIM2->SR &= ~(TIM_SR_CC1IF | TIM_SR_UIF);       // clr IRQ flag in status reg
//   NVIC->ISER[0] |= (1 << (TIM2_IRQn & 0x1F));     // set NVIC interrupt: 0x1F
//   __enable_irq();                                 // global IRQ enable
//   TIM2->CR1 |= TIM_CR1_CEN;                       // start TIM2 CR1
//}
//
//// Part A
////void TIM2_IRQHandler(void) {
////   if (TIM2->SR & TIM_SR_CC1IF) {      // triggered by CCR1 event ...
////      TIM2->SR &= ~(TIM_SR_CC1IF);     // manage the flag
////      GPIOB->ODR |= GPIO_PIN_7;                          =
////   }
////   if (TIM2->SR & TIM_SR_UIF) {        // triggered by ARR event ...
////      TIM2->SR &= ~(TIM_SR_UIF);       // manage the flag
////      GPIOB->ODR &= ~GPIO_PIN_7;                             =
////   }
////}
//
//// Part B
//void TIM2_IRQHandler(void) {
//   	GPIOB->ODR |= (GPIO_PIN_8);        // Toggle LED at 10 kHz
//    if (TIM2->SR & TIM_SR_CC1IF) {      // triggered by CCR1 event ...
//    	TIM2->SR &= ~(TIM_SR_CC1IF);     // manage the flag
//    	GPIOB->ODR ^= (GPIO_PIN_7);        // Toggle LED at 10 kHz
//    	my_val += CCR1_VALUE;
//    	TIM2->CCR1 = my_val;	// Moves CCR1 ahead of timer to interrupt
//    	//TIM2->CCR1 = ( (TIM2->CNT) + CCR1_VALUE );	// Works
//        //TIM2->CCR1 += CCR1_VALUE;						// Doesn't work
//    }
//    if (TIM2->SR & TIM_SR_UIF) {        // triggered by ARR event ...
//        TIM2->SR &= ~(TIM_SR_UIF);      // manage the flag
//        //Do nothing                     // <-- manage GPIO pin here
//    }
// 	GPIOB->ODR &= ~(GPIO_PIN_8);        // Toggle LED at 10 kHz
//
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//
//int _write(int file, char *ptr, int len)
//{
//	(void)file;
//	int DataIdx;
//
//	for (DataIdx = 0; DataIdx < len; DataIdx++)
//	{
//		ITM_SendChar(*ptr++);
//	}
//	return len;
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
