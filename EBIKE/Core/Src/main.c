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
 *
 * 3-WAY SWITCHING: SWITCH_1<->SWITCH_2
 * 00 - Medium Mode - 750W, full throttle range
 * 10 - Slow Mode   - 750W, half throttle range
 * 01 - Fast Mode   - 1000W, full throttle range
 *
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "delay.h"
#include "adc.h"
#include "dac.h"
#include "lcd.h"


#include "stdio.h"

/* Interrupt */
#define TIME_UP		0xFFFFFFFF				// ARR initialize val
#define CCR1_VALUE 	((uint32_t)50000000) 	// CCR1 initialize val ~ 12s
#define PI			3.14159


#define TIM2_TEN	40000000	// TIM2 count for 10s
#define TIM2_FIVE	20000000	// TIM2 count for 5s


// Defines for SWITCH inputs
#define SWITCH_1_CLOCK 	RCC_AHB2ENR_GPIOBEN
#define SWITCH_1_PORT	GPIOB
#define SWITCH_1_IO		GPIO_PIN_2
#define SWITCH_2_PORT	GPIOB
#define SWITCH_2_IO		GPIO_PIN_3

// Defines for BREAK input
#define BREAK_CLOCK	RCC_AHB2ENR_GPIOAEN
#define BREAK_PORT	GPIOA
#define BREAK_IO	GPIO_PIN_3



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SWITCH_init( void );
void BREAK_init(void);
void setup_EXTI1( void );
void setup_TIM2( uint32_t iDutyCycle );
uint32_t HALL_speed( void );

uint8_t SWITCH_1(void); // get input and debounce
uint8_t SWITCH_2(void); // get input and debounce
uint8_t BREAK(void);




// SWITCH variables
uint8_t SWITCH_1_state = 0;	// If key is currently pressed ==1
uint8_t SWITCH_2_state = 0;	// If key is currently pressed ==1

// BREAK variables
uint8_t BREAK_state = 0;	// If key is currently pressed ==1


uint8_t state = 0;	// Track state of operations
uint8_t interrupt_num = 0;
uint8_t interrupt_flag = 0;

// Speed Calculation Variables:
/* Intended display on top line of LCD:
 * "Speed: 20.0 MPH" */



uint32_t test_sense = 0;
uint32_t grab_count = 0;
uint8_t speed_flag = 0;

// holds ADC digital output
uint16_t adc_avg_throttle = 0;		// Ch. 5 = PA0
uint16_t adc_avg_current = 0;		// Ch. 7 = PA2

// holds ADC digital output converted to millivolts
uint16_t volt_avg_throttle = 0;	// Ch. 5 = PA0
uint16_t volt_avg_current = 0;	// Ch. 7 = PA2

void main(void)
{

	HAL_Init();
	SystemClock_Config();
	//SWITCH_init();
	//BREAK_init();
	setup_TIM2(CCR1_VALUE);
	setup_EXTI1();
	LCD_init();
	ADC_init();
	//DAC_init();




	// User Button (PC13) Configuration to command relay:
	// input mode, with pull down
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);

	uint8_t current_mode = 0;
	uint8_t previous_mode = 0;
	uint32_t throttle_DAC = 0;
	uint32_t speed_result = 0;
	uint32_t amp = 0;
	uint32_t watt = 0;

	uint32_t display_count = 0;


	LCD_command( CURSOR_SHIFT_L );		// Shift cursor to the left
	LCD_command( CURSOR_OFF );			// Display, cursor, position on
	LCD_command( CURSOR_RIGHT );		// Cursor moves right, no shift
	LCD_command( CLEAR_HOME );
	LCD_write_string("Power:");
	LCD_command(LINE_TWO);
	LCD_write_string("Speed:  ");
	//LCD_write_power(watt);
	//LCD_write_volt(speed_result);

	while (1)
	{
		delay_us(1000000);
		printf("help me \n");

		// PBSW pressed
		if (GPIOC->IDR & GPIO_PIN_13) {
			ADC_sample( &adc_avg_throttle, &adc_avg_current);
			volt_avg_throttle = ADC_raw_to_volt(adc_avg_throttle);
			volt_avg_current = ADC_raw_to_volt(adc_avg_current);

			// 50 mV drop per Amp, offset by 2.508 V (result in mA)
			amp = ((2508 - volt_avg_current) * 1000) / 50;
			watt = (52 * amp) / 1000;


			//GPIOB->ODR |= GPIO_PIN_0;
			printf("Rot count = %d\n", test_sense);
			speed_result = HALL_speed();
			printf("speed result %d\n", speed_result);
			//GPIOB->ODR &= ~(GPIO_PIN_0);	// off
			printf("Wattage is %d\n", watt);
			printf("AVG Throttle V: %d\n", volt_avg_throttle);
			printf("AVG Current V: %d\n", volt_avg_current);

		}










		//delay_us(100000);	// 100 ms

		// Get sensor data
//		ADC_sample( &adc_avg_throttle, &adc_avg_current);
//		volt_avg_throttle = ADC_raw_to_volt(adc_avg_throttle);
//		volt_avg_current = ADC_raw_to_volt(adc_avg_current);
//
//		// 50 mV drop per Amp, offset by 2.508 V (result in mA)
//		amp = ((2508 - volt_avg_current) * 1000) / 50;
//		watt = (52 * amp) / 1000;
//
//		speed_result = HALL_speed();


//		if ( BREAK() ) {
//			DAC_write(DAC_volt_conv(0));
//		}
//		else {
//			throttle_DAC = DAC_volt_conv(volt_avg_throttle);
//			if ( SWITCH_1() ) {
//				current_mode = 0;
//				DAC_write( (throttle_DAC *3300) / 4094 );	// Throttle V / 2
//				//GPIOB->ODR |= GPIO_PIN_0;	// 750 Watt mode
//			}
//			else if ( SWITCH_2() ) {
//				current_mode = 2;
//				DAC_write( (throttle_DAC *3300 * 2) / 4094 );	// Throttle V
//				//GPIOB->ODR &= ~(GPIO_PIN_0);	// 1000 Watt mode
//			}
//			else {
//				current_mode = 1;
//				DAC_write( (throttle_DAC *3300 * 2) / 4094 );	// Throttle V
//				//GPIOB->ODR |= GPIO_PIN_0;	// 750 Watt mode
//			}
//			if ( !(current_mode == previous_mode) ) {
//				previous_mode = current_mode;
//				if ( (current_mode == 0) | (current_mode == 1) ) {
//					GPIOB->ODR |= GPIO_PIN_0;	// relay switched to 750W
//					delay_us(500000);	// Wait for realy to switch
//					GPIOB->ODR &= ~(GPIO_PIN_0);	// don't waste power
//				}
//				else {
//					GPIOB->ODR |= GPIO_PIN_1;	// relay switched to 1000W
//					delay_us(500000);
//					GPIOB->ODR &= ~(GPIO_PIN_1);
//				}
//			}
//		}
//
//		display_count++;
//		if (display_count > 3) {	// Every 0.3s update LCD
//
//			LCD_command( CURSOR_SHIFT_L );		// Shift cursor to the left
//			LCD_command( CURSOR_OFF );			// Display, cursor, position on
//			LCD_command( CURSOR_RIGHT );		// Cursor moves right, no shift
//			LCD_command( CLEAR_HOME );
//			LCD_write_string("Power:");
//			LCD_command(LINE_TWO);
//			LCD_write_string("Speed:  ");
//			LCD_write_power(watt);
//			LCD_write_volt(speed_result);
//
//			display_count = 0;
//		}



	} // end while
} // end main



///////////////////////////////////////////////////////////////////////////////
// HALL SPEED FUNCTIONS
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
	uint16_t stop_time = 10000;		// No action for long time
	uint16_t stop_count = 0;
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
		stop_count++;
		if (stop_count > stop_time) {
			return 0;	// Exit if bike not moving (no interrupt in time)
		}
		; // Wait for first rise detection from ISR
	}
	previous_count = grab_count;
	stop_count = 0;
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
		stop_count++;
		if (stop_count > stop_time) {
			return 0;	// Exit if bike not moving (no interrupt in time)
		}
		; // Wait for second rise detection from ISR
	}
	current_count = grab_count;
	stop_count = 0;
	speed_flag = 0;

	time_per_sense = (current_count - previous_count) / 4; // micro s
	frequency = 1000000 / time_per_sense;
	printf("frequency is %d\n", frequency);
	time_per_rot = time_per_sense * 26;	// 26 senses per rotation
	speed = ( (circumference * 3600) / time_per_rot ) / 10; // == MPH *10^1

	return speed;
}

// ISR EXTI1 (PA1) CODE
void setup_EXTI1( void ) {
	// Configure Interrupt Pin A1:
	// Input. PD, push/pull,
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER   &= ~(GPIO_MODER_MODE1);
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD1);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD1_1;

	EXTI->IMR1 |= EXTI_IMR1_IM1;	// Enable EXTI1 interrupts
	EXTI->RTSR1 |= EXTI_RTSR1_RT1;	// Trigger on RISE
	SYSCFG->EXTICR[0] &= ~(0xF);	// PA1 -> Line 1
	NVIC->ISER[0] |= (1 << (EXTI1_IRQn & 0x1F));	// set NVIC interrupt: 0x1F
	__enable_irq();                                 // global IRQ enable
}

void EXTI1_IRQHandler (void) {
	if (EXTI->PR1 & EXTI_PR1_PIF1) {	// Line 1 interrupt?
		grab_count = TIM2->CNT;
		speed_flag = 1;
		test_sense++;
		//printf("stuck here?\n");
		EXTI->PR1 |= EXTI_PR1_PIF1; // cleared by setting
	}
	EXTI->IMR1 &= ~(EXTI_IMR1_IM1);	// Disable EXTI1 interrupts
}

// ISR TIM2 CODE //
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

void SWITCH_init( void ) {
	/* Will configure input pins to switch and the output
	 * pin to the BLUE 1000/750W relay circuit.
	 * Pins used are:
	 * PB0 - OUTPUT - High speed, PD, PP - BLUE 1000/750W BJT/Relay control
	 * PB1 - INPUT - PD - Low Mode Switch Input
	 * PB2 - INPUT - PD - High Mode Switch Input*/
	//RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
	RCC->AHB2ENR   |=  (SWITCH_1_CLOCK);
	// Configure PB0,PB1 as output
	SWITCH_1_PORT->MODER   &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
	SWITCH_1_PORT->MODER   |=  (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0);
	SWITCH_1_PORT->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);
	SWITCH_1_PORT->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
	SWITCH_1_PORT->PUPDR   |=  (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1);//////////////////////////////////
	SWITCH_1_PORT->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos) |
			(3 << GPIO_OSPEEDR_OSPEED1_Pos));
	SWITCH_1_PORT->BRR = (GPIO_PIN_0 | GPIO_PIN_1);

	// Configure PB1/PB2 as input
	SWITCH_1_PORT->MODER  &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	SWITCH_1_PORT->PUPDR  &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
	SWITCH_1_PORT->PUPDR  |= (GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD3_1 );
}

uint8_t SWITCH_1(void) {
	// Debounce function for buttons.
	// Change BUTTON defines in main.h
	uint16_t settle = 1000;			// Small delay for debounce to settle
	uint16_t debounce_count = 0;	// Compare to settle time

	if ( SWITCH_1_state == 0 ) {
		while ( (SWITCH_1_PORT->IDR & SWITCH_1_IO) != 0 ) {	// Button pressed
			debounce_count++;
			if ( debounce_count > settle ) {			// Button high for awhile
				SWITCH_1_state = 1;
				return 1;
			}
		}
		return 0;
	}
	if ( SWITCH_1_state == 1 ) {	// Button was pressed
		while ( (SWITCH_1_PORT->IDR & SWITCH_1_IO) == 0 ) {		// Button released
			debounce_count++;
			if ( debounce_count > settle ) {			// Button low for awhile
				SWITCH_1_state = 0;
				return 0;
			}
		}
		return  1;
	}
}

uint8_t SWITCH_2(void) {
	// Debounce function for buttons.
	// Change BUTTON defines in main.h
	uint16_t settle = 1000;			// Small delay for debounce to settle
	uint16_t debounce_count = 0;	// Compare to settle time

	if ( SWITCH_2_state == 0 ) {
		while ( (SWITCH_2_PORT->IDR & SWITCH_2_IO) != 0 ) {	// Button pressed
			debounce_count++;
			if ( debounce_count > settle ) {			// Button high for awhile
				SWITCH_2_state = 1;
				return 1;
			}
		}
		return 0;
	}
	if ( SWITCH_2_state == 1 ) {	// Button was pressed
		while ( (SWITCH_2_PORT->IDR & SWITCH_2_IO) == 0 ) {		// Button released
			debounce_count++;
			if ( debounce_count > settle ) {			// Button low for awhile
				SWITCH_2_state = 0;
				return 0;
			}
		}
		return  1;
	}
	return 0;
}

void BREAK_init( void ) {
	/* Will configure input pins to switch and the output
	 * pin to the BLUE 1000/750W relay circuit.
	 * Pins used are:
	 * PA3 - INPUT - PD - Low Mode Switch Input */

	// Configure PA3 as input
	//RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	RCC->AHB2ENR   |=  (BREAK_CLOCK);
	BREAK_PORT->MODER  &= ~(GPIO_MODER_MODE3);
	BREAK_PORT->PUPDR  &= ~(GPIO_PUPDR_PUPD3);
	BREAK_PORT->PUPDR  |= (GPIO_PUPDR_PUPD3_1);	////////////////////////////////
}

uint8_t BREAK(void) {
	// Debounce function for buttons.
	// Change BUTTON defines in main.h
	uint16_t settle = 100;			// Small delay for debounce to settle
	uint16_t debounce_count = 0;	// Compare to settle time

	if ( BREAK_state == 0 ) {
		while ( (BREAK_PORT->IDR & BREAK_IO) != 0 ) {	// Button pressed
			debounce_count++;
			if ( debounce_count > settle ) {			// Button high for awhile
				BREAK_state = 1;
				return 1;
			}
		}
		return 0;
	}
	if ( BREAK_state == 1 ) {	// Button was pressed
		while ( (BREAK_PORT->IDR & BREAK_IO) == 0 ) {	// Button released
			debounce_count++;
			if ( debounce_count > settle ) {			// Button low for awhile
				BREAK_state = 0;
				return 0;
			}
		}
		return 1;
	}
	return 0;
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
