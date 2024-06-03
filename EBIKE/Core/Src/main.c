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

/* Interrupt */
#define TIME_UP		0xFFFFFFFF				// ARR initialize val
#define CCR1_VALUE 	((uint32_t)50000000) 	// CCR1 initialize val ~ 12s
#define PI			3.14159


#define TIM2_TEN	40000000	// TIM2 count for 10s
#define TIM2_FIVE	20000000	// TIM2 count for 5s

// Interrupt pin being treated like unstable interrupt: needs debounce
#define BUTTON1_PORT	GPIOA
#define BUTTON1_IO		GPIO_PIN_1



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void setup_EXTI1( void );
void setup_TIM2( uint32_t iDutyCycle );
uint32_t HALL_speed( void );
uint8_t button1(void);

uint8_t debounce1_state = 0;	// If key is currently pressed ==1


uint8_t state = 0;	// Track state of operations
uint8_t interrupt_num = 0;
uint8_t interrupt_flag = 0;

// Speed Calculation Variables:
/* Intended display on top line of LCD:
 * "Speed: 20.0 MPH" */



uint32_t test_sense = 0;
uint8_t speed_flag = 0;


void main(void)
{

	HAL_Init();
	SystemClock_Config();
	setup_TIM2(CCR1_VALUE);
	setup_EXTI1();




	// User Button (PC13) Configuration to command relay:
	// input mode, with pull down
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);



	// Configure Interrupt Pin A1:
	// Input. no PU/PD, push/pull,
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER   &= ~(GPIO_MODER_MODE1);
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD1);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD1_1;



	// Configure LED
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
	GPIOB->MODER   &= ~(GPIO_MODER_MODE1);
	GPIOB->MODER   |=  (GPIO_MODER_MODE1_0);
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT1);
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD1);
	GPIOB->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED1_Pos));
	GPIOB->BRR = (GPIO_PIN_1);

	uint32_t speed_result = 0;

	while (1)
	{
		//delay_us(100000);	// .25s

		// Interrupt if
		//if (button1()) {
//		if (GPIOA->IDR & GPIO_PIN_1) {
//			//test_sense++;
//			printf("Rot count = %d\n", test_sense);
//			GPIOB->ODR |= GPIO_PIN_1;
//		}
//		else {
//			GPIOB->ODR &= ~(GPIO_PIN_1);
//		}



		// PBSW if
		if (GPIOC->IDR & GPIO_PIN_13) {
			//GPIOB->ODR |= GPIO_PIN_1;
			printf("Rot count = %d\n", test_sense);
			speed_result = HALL_speed();
			printf("speed result %d\n", speed_result);
			//GPIOB->ODR &= ~(GPIO_PIN_1);	// off
		}
		else {
			//GPIOB->ODR &= ~(GPIO_PIN_1);
		}
	}

}

uint32_t HALL_speed( void ) {
	/* waits for EXTI pin to be low before looking for rising edge by
	 * enabling EXTI interrupt.  Waits for ISR to get edge.
	 * Waits till low again, then gets second time to compare.
	 * Returns speed calculated */
	// NOTE: this function can take up to 100ms to complete

	uint32_t circumference = 128916;	// *10^-8 miles
	uint32_t previous_count = 0;
	uint32_t current_count = 0;
	uint32_t time_per_sense = 0;
	uint32_t time_per_rot = 0;
	uint32_t speed = 0;

	uint32_t frequency = 0;

	uint16_t settle = 100;			// Small delay for debounce to settle
	uint16_t debounce_count = 0;	// Compare to settle time
	uint8_t unsettled = 1;			// HALL still high

	while( unsettled == 1 ) {	// Wait till HALL low
		if ( (GPIOA->IDR & GPIO_PIN_1) ) {	// HALL still high?
			debounce_count = 0;
		}
		else {
			debounce_count++;
		}
		if (debounce_count > settle){	// HALL been low for awhile
			unsettled = 0;	// break the loop
		}
	}

	debounce_count = 0;
	unsettled = 1;
	EXTI->IMR1 |= EXTI_IMR1_IM1;	// Enable EXTI1 interrupts
	while( speed_flag == 0 ) {
		; // Wait for first rise detection from ISR
	}
	previous_count = TIM2->CNT;
	speed_flag = 0;

	while( unsettled == 1 ) {	// Wait till HALL low again
		if ( (GPIOA->IDR & GPIO_PIN_1) ) {	// HALL still high?
			debounce_count = 0;
		}
		else {
			debounce_count++;
		}
		if (debounce_count > settle){	// HALL been low for awhile
			unsettled = 0;	// break the loop
		}
	}
	EXTI->IMR1 |= EXTI_IMR1_IM1;	// Enable EXTI1 interrupts

	while( speed_flag == 0 ) {
		; // Wait for second rise detection from ISR
	}
	current_count = TIM2->CNT;
	speed_flag = 0;

	time_per_sense = (current_count - previous_count) / 4; // micro s
	frequency = 1000000 / time_per_sense;
	printf("frequency is %d\n", frequency);
	time_per_rot = time_per_sense * 26;	// 26 senses per rotation
	speed = ( (circumference * 3600) / time_per_rot ) / 10; // == MPH *10^1

	return speed;
}

//uint8_t HALL_ready(void) {
//	// Looking for steady low on HALL sensor
//	uint16_t settle = 1000;			// Small delay for low to settle
//	uint16_t debounce_count = 0;	// Compare to settle time
//	while ( (GPIOA->IDR & GPIO_PIN_1) == 0 ) {	// HALL is low
//		debounce_count++;
//		if (debounce_count > settle){	// HALL been low for awhile
//			return 1;	// break the loop
//		}
//	}
//	return 0;
//
//}

///////////////////////////////////////////////////////////////////////////////
// ISR EXTI1 (PA1) CODE
void setup_EXTI1( void ) {
	EXTI->IMR1 |= EXTI_IMR1_IM1;	// Enable EXTI1 interrupts
	EXTI->RTSR1 |= EXTI_RTSR1_RT1;	// Trigger on RISE
	SYSCFG->EXTICR[0] &= ~(0x0);	// PA[1] -> Line 1
	NVIC->ISER[0] |= (1 << (EXTI1_IRQn & 0x1F));	// set NVIC interrupt: 0x1F
	__enable_irq();                                 // global IRQ enable


}

void EXTI1_IRQHandler (void) {
	if (EXTI->PR1 & EXTI_PR1_PIF1) {	// Line 1 interrupt?
		speed_flag = 1;
		test_sense++;
		printf("stuck here?\n");
		EXTI->PR1 |= EXTI_PR1_PIF1; // cleared by setting
	}
	EXTI->IMR1 &= ~(EXTI_IMR1_IM1);	// Disable EXTI1 interrupts
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// ISR TIM2 CODE

void setup_TIM2( uint32_t iDutyCycle ) {
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;           // enable clock for TIM2
	TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_UIE);  // enable event gen, rcv CCR1
	TIM2->ARR = TIME_UP;                             // ARR = T = counts @4MHz
	TIM2->CCR1 = iDutyCycle;                        // ticks for duty cycle
	TIM2->SR &= ~(TIM_SR_CC1IF | TIM_SR_UIF);       // clr IRQ flag in status reg
	NVIC->ISER[0] |= (1 << (TIM2_IRQn & 0x1F));     // set NVIC interrupt: 0x1F
	__enable_irq();                                 // global IRQ enable
	TIM2->CR1 |= TIM_CR1_CEN;                       // start TIM2 CR1
}

void TIM2_IRQHandler(void) {	// Not going to use specific timing interrupts
	if (TIM2->SR & TIM_SR_CC1IF) {		// triggered by CCR1 event ...
		TIM2->SR &= ~(TIM_SR_CC1IF);    // manage the flag
		// do nothing
	}
	if (TIM2->SR & TIM_SR_UIF) {        // triggered by ARR event ...
		TIM2->SR &= ~(TIM_SR_UIF);      // manage the flag
		// do nothing

	}
}
///////////////////////////////////////////////////////////////////////////////




uint8_t button1(void) {
	// Debounce function for buttons.
	// Change BUTTON defines in main.h
	uint16_t settle = 1000;			// Small delay for debounce to settle
	uint16_t debounce_count = 0;	// Compare to settle time

	if ( debounce1_state == 0 ) {
		while ( (BUTTON1_PORT->IDR & BUTTON1_IO) != 0 ) {	// Button pressed
			debounce_count++;
			if ( debounce_count > settle ) {			// Button high for awhile
				debounce1_state = 1;
				return 1;
			}
		}
		return 0;
	}
	if ( debounce1_state == 1 ) {	// Button was pressed
		if ( (BUTTON1_PORT->IDR & BUTTON1_IO) == 0 ) {		// Button released
			debounce1_state = 0;
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
