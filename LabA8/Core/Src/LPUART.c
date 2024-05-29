/*
 * LPUART.c
 *
 *  Created on: May 8, 2024
 *      Author: bradl
 */



#include "LPUART.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void LPUART_init( void );
void LPUART_print( const char* message );
void LPUART_ESC_print( const char* message);
void LPUART1_IRQHandler( void  );
void GAME_character(void);
void GAME_background(void);


/* Global variables */
uint8_t flag_left = 0;
uint8_t flag_right = 0;
uint8_t flag_up = 0;
uint8_t flag_down = 0;
uint8_t rec = 0;


void LPUART_init( void ) {
	PWR->CR2 |= (PWR_CR2_IOSV);              // power avail on PG[15:2] (LPUART1)
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge
	RCC->CCIPR   |= (0x1UL << 10U);			// 4 MHz SYSCLOCK

	/* USER: configure GPIOG registers MODER/PUPDR/OTYPER/OSPEEDR then
	 select AF mode and specify which function with AFR[0] and AFR[1] */
	// Follow order AFR, OTYPER, PUPDR, OSPEEDR, MODDER
	// Configure PG7 for TX and PG8 for RX
	GPIOG->AFR[0]  &= ~(GPIO_AFRL_AFSEL7);    	// Clear AF for PG7
	GPIOG->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);    	// Clear AF for PG7
	GPIOG->AFR[0]  |= (0x0008UL << (28U));			// Set PG7 AF8
	GPIOG->AFR[1]  |= (0x0008UL << (0U));			// Set PG8 AF8
	GPIOG->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);		// Clear
	GPIOG->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);	// Clear
	GPIOG->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) | (3 << GPIO_OSPEEDR_OSPEED8_Pos));	// High Speed
	GPIOG->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);		// Clear
	GPIOG->MODER |= (GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1);	// AF mode


	// Configure LPUART
	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
	LPUART1->CR3 |= (0x1UL << 12U);	// disable overrun error			// IF SOMETHING STOPS WORKING GET RID OF ME
	/* Parity/Stop bits: Can only be done before UE is set*/
	//LPUART1->CR1 |= USART_CR1_PCE;					// parity enable
	//LPUART1->CR1 &= ~USART_CR1_PS;					// even parity
	//LPUART1->CR1 |= USART_CR1_PS;					// odd parity
	//LPUART1->CR2 |= USART_CR2_STOP_1;				// two stop bits
	/**/

	LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
	LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
	LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag
	//LPUART1->BRR = 0x115C;					// Set baud rate for 2 MHz
	LPUART1->BRR = 0x22B9;					// Set baud rate for 4 MHz
	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
	__enable_irq();                          // enable global interrupts
}

void LPUART_print( const char* message) {
	uint16_t iStrIdx = 0;
	while ( message[iStrIdx] != 0 ) {
		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
			;
		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
		iStrIdx++;                             // advance index to next char
	}
}

void LPUART_move_cursor ( uint8_t x_location, uint8_t y_location ) {
	char str[20];
	sprintf(str, "[%d;%dH", y_location, x_location);
	LPUART_ESC_print(str);

}

void LPUART_ESC_print( const char* message) {
	uint16_t iStrIdx = 0;
	while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
		;
	LPUART1->TDR = 0x1B;       // initiate ESC
	while ( message[iStrIdx] != 0 ) {
		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
			;
		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
		iStrIdx++;                             // advance index to next char
	}
}

void LPUART1_IRQHandler( void  ) {
	uint8_t charRecv;
	if (LPUART1->ISR & USART_ISR_RXNE) {
		charRecv = LPUART1->RDR;	// Reading RDR resets receive process.
		switch ( charRecv ) {
//		case 'R':
//			LPUART_ESC_print("[31m");
//			break;
//		case 'G':
//			LPUART_ESC_print("[32m");
//			break;
//		case 'B':
//			LPUART_ESC_print("[34m");
//			break;
//		case 'W':
//			LPUART_ESC_print("[37m");
//			break;
		case 'a':
			flag_left = 1;
			break;
		case 'd':
			flag_right = 1;
			break;
		case 'w':
			flag_up = 1;
			break;
		case 's':
			flag_down = 1;
			break;
		default:
			while( !(LPUART1->ISR & USART_ISR_TXE) )
				;    // wait for empty TX buffer
			LPUART1->TDR = charRecv;  // echo char to terminal
		}  // end switch
	}
}

void GAME_character (void) {
	/*Prints the character at the desired location on interface grid*/
	/*Leaves cursor where it started*/
//	LPUART_ESC_print("");
	LPUART_ESC_print("[2A");
	LPUART_print("\\_//");
	LPUART_ESC_print("[1B");
	LPUART_ESC_print("[6D");
	LPUART_print("__/''.");
	LPUART_ESC_print("[1B");
	LPUART_ESC_print("[7D");
	LPUART_print("/__ |");
	LPUART_ESC_print("[1B");
	LPUART_ESC_print("[5D");
	LPUART_print("|| ||");
	LPUART_ESC_print("[1A");
	LPUART_ESC_print("[3D");
/*
prs \\_//
   __/''.
  /__ |
  || ||
*/
	; // nothing
}
//
//void SystemClock_Config(void)
//{
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//	/** Configure the main internal regulator output voltage
//	 */
//	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the RCC Oscillators according to the specified parameters
//	 * in the RCC_OscInitTypeDef structure.
//	 */
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//	RCC_OscInitStruct.MSICalibrationValue = 0;
//	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the CPU, AHB and APB buses clocks
//	 */
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}
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
//void Error_Handler(void)
//{
//	/* USER CODE BEGIN Error_Handler_Debug */
//	/* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
//	while (1)
//	{
//	}
//	/* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
// * @brief  Reports the name of the source file and the source line number
// *         where the assert_param error has occurred.
// * @param  file: pointer to the source file name
// * @param  line: assert_param error line source number
// * @retval None
// */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//	/* USER CODE BEGIN 6 */
//	/* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//	/* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */









//
//#include "main.h"
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void LPUART_init( void );
//void LPUART_print( const char* message );
//void LPUART_ESC_print( const char* message);
//void LPUART1_IRQHandler( void  );
//void GAME_character(void);
//void GAME_background(void);
//
//
///* Global variables */
//uint8_t flag_left = 0;
//uint8_t flag_right = 0;
//uint8_t flag_up = 0;
//uint8_t flag_down = 0;
//int main(void)
//{
//
//
//
//
//	HAL_Init();
//	SystemClock_Config();
//	LPUART_init();
//
//	// Configure user button
//	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
//	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
//	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);
//
//
//	while (1)
//	{
//		if ( (GPIOC->IDR & GPIO_PIN_13) == GPIO_PIN_13 ) {
////			LPUART_ESC_print("[2J");	// clear screen
////			LPUART_ESC_print("[H");		// Cursor upperleft
////			LPUART_ESC_print("[3B");	// Down 3 lines
////			LPUART_ESC_print("[5C");	// right 5 spaces
////			LPUART_print("All good students read the");
////			LPUART_ESC_print("[1B");	// down one line
////			LPUART_ESC_print("[21D");	// left 21 spaces
////			LPUART_ESC_print("[5m");	// turn on blinking mode
////			LPUART_print("Reference Manual");
////			LPUART_ESC_print("[H");		// cursor upperleft
////			LPUART_ESC_print("[0m");	// character attributes off
////			LPUART_print("Input:");
//			// background is 36 X 100
//			LPUART_ESC_print("[2J");	// clear screen
//			LPUART_ESC_print("[H");		// Cursor upperleft
//			LPUART_ESC_print("[18B");
//			LPUART_ESC_print("[50C");	// cursor in middle
//			GAME_background();
//			GAME_character();
//			//LPUART_ESC_print("[s");
//			//LPUART_ESC_print("[s");//save
//
//		}
//		if ( flag_left == 1 ) {
//			LPUART_ESC_print("[u");
//			GAME_background();
//			LPUART_ESC_print("[2D");
//			GAME_character();
//			flag_left = 0;
//		}
//		else if ( flag_right == 1 ) {
//			GAME_background();
//			LPUART_ESC_print("[u");
//			LPUART_ESC_print("[2C");
//			GAME_character();
//			LPUART_ESC_print("[s");
//			flag_right = 0;
//		}
//		else if ( flag_up == 1 ) {
//			GAME_background();
//			LPUART_ESC_print("[u");
//			LPUART_ESC_print("[2A");
//			GAME_character();
//			LPUART_ESC_print("[s");
//			flag_up = 0;
//		}
//		else if ( flag_down == 1 ) {
//			GAME_background();
//			LPUART_ESC_print("[u");
//			LPUART_ESC_print("[2B");
//			GAME_character();
//			LPUART_ESC_print("[s");
//			flag_down = 0;
//		}
//
//	}
//
//}
//
//
//
//void LPUART_init( void ) {
//	PWR->CR2 |= (PWR_CR2_IOSV);              // power avail on PG[15:2] (LPUART1)
//	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
//	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge
//	RCC->CCIPR   |= (0x1UL << 10U);			// 4 MHz SYSCLOCK
//
//	/* USER: configure GPIOG registers MODER/PUPDR/OTYPER/OSPEEDR then
//	 select AF mode and specify which function with AFR[0] and AFR[1] */
//	// Follow order AFR, OTYPER, PUPDR, OSPEEDR, MODDER
//	// Configure PG7 for TX and PG8 for RX
//	GPIOG->AFR[0]  &= ~(GPIO_AFRL_AFSEL7);    	// Clear AF for PG7
//	GPIOG->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);    	// Clear AF for PG7
//	GPIOG->AFR[0]  |= (0x0008UL << (28U));			// Set PG7 AF8
//	GPIOG->AFR[1]  |= (0x0008UL << (0U));			// Set PG8 AF8
//	GPIOG->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);		// Clear
//	GPIOG->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);	// Clear
//	GPIOG->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) | (3 << GPIO_OSPEEDR_OSPEED8_Pos));	// High Speed
//	GPIOG->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);		// Clear
//	GPIOG->MODER |= (GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1);	// AF mode
//
//
//	// Configure LPUART
//	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
//	LPUART1->CR3 |= (0x1UL << 12U);	// disable overrun error			// IF SOMETHING STOPS WORKING GET RID OF ME
//	/* Parity/Stop bits: Can only be done before UE is set*/
//	//LPUART1->CR1 |= USART_CR1_PCE;					// parity enable
//	//LPUART1->CR1 &= ~USART_CR1_PS;					// even parity
//	//LPUART1->CR1 |= USART_CR1_PS;					// odd parity
//	//LPUART1->CR2 |= USART_CR2_STOP_1;				// two stop bits
//	/**/
//
//	LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
//	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
//	LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
//	LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag
//	//LPUART1->BRR = 0x115C;					// Set baud rate for 2 MHz
//	LPUART1->BRR = 0x22B9;					// Set baud rate for 4 MHz
//	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
//	__enable_irq();                          // enable global interrupts
//}
//
//void LPUART_print( const char* message) {
//	uint16_t iStrIdx = 0;
//	while ( message[iStrIdx] != 0 ) {
//		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//			;
//		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
//		iStrIdx++;                             // advance index to next char
//	}
//}
//
//void LPUART_ESC_print( const char* message) {
//	uint16_t iStrIdx = 0;
//	while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//		;
//	LPUART1->TDR = 0x1B;       // initiate ESC
//	while ( message[iStrIdx] != 0 ) {
//		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//			;
//		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
//		iStrIdx++;                             // advance index to next char
//	}
//}
//
//void LPUART1_IRQHandler( void  ) {
//	uint8_t charRecv;
//	if (LPUART1->ISR & USART_ISR_RXNE) {
//		charRecv = LPUART1->RDR;	// Reading RDR resets receive process.
//		switch ( charRecv ) {
//		case 'R':
//			LPUART_ESC_print("[31m");
//			break;
//		case 'G':
//			LPUART_ESC_print("[32m");
//			break;
//		case 'B':
//			LPUART_ESC_print("[34m");
//			break;
//		case 'W':
//			LPUART_ESC_print("[37m");
//			break;
//		case 'a':
//			LPUART_ESC_print("[s");
//			flag_left = 1;
//			break;
//		case 'd':
//			flag_right = 1;
//			break;
//		case 'w':
//			flag_up = 1;
//			break;
//		case 's':
//			flag_down = 1;
//			break;
//		default:
//			while( !(LPUART1->ISR & USART_ISR_TXE) )
//				;    // wait for empty TX buffer
//			LPUART1->TDR = charRecv;  // echo char to terminal
//		}  // end switch
//	}
//}
//
//
//void GAME_background (void) {
//	/* User: enter game background in form as follows. */
//	/* Returns cursor to original position after */
//	LPUART_ESC_print("[s");		// save cursor loc
//	LPUART_ESC_print("[2J");	// clear screen
//	LPUART_ESC_print("[H");		// Cursor upperleft
//LPUART_print("%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@%@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("#@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@%%#@@@@%@@@@@@@@#@@@@@*@@@@%+@%@@@@@@@%@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@%@@@@@@@@@@@@@@@@@%%@@@@@@@@@@@@@@%%@@@@%@@@@@#@%@@@%@@@@@@@@%@@@@@@@@@@@@@@@@%@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("%@@@@@@@@@@%@@@@@@@@@#@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@%@@%@@#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@%%@@@#%@=@@@@@@@@@@@@@@%@@%@@@@%%*@%@@@@@@@%%@@@@@@@@@@@@@@%@@@@@@#%@@@@#@@@@@@@@@@@@@@@#@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@#@@@@@@@@%@@@@@@@@@%%@:#@@@@@@%@%@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@%@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("%@@@@@@@@@@@@@@@%@@@@@@@@@@@@@%@@%@@@@@@%%%%%@@@@@@%@#@@@@@@@@@@@*#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@@@@@@@@@@%%@@@@@@@@%#@@%@@@@@@@@%@@@@#@@@@@%%@@@%@@@@@@@@@@@@@@@@@@@@%");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@%@#@@@@@@@@@@@@@%%%@%@%@@@@@@%@@@*@@@%@@@%@@@@@@@@@@@%%@@@@@@@#@%@@@@@@@%@@@%@@@@@@%@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@%@@@@@@@%@@@@@@@@@@@@%%@@@@%#%@@@@@@%%@@@@%@%@@@@@@@@@@@@@@@%@@@@@@@@@@%%@@@@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@%@%@@@@@@%%@%@@@@@@@%@@@%%@@%@@@@@@@@%@@%@@@@%@@@@@@@@@@@%@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%@@@@@@@@@@@%@%@@@@@@@@%%%@@@@%@@%@@@@%@%@@@@@@@@@%@@@@@@@@%@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@@@@%%%@@@@@@@@@#%@@@@@@@@@@%%@@@@@@@@@@%@@@@@@@@@@@@@@@@%@@@@@@@@@@@@%@@@%@@@@@@@@@");\
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@%##@%%%%@%@@@@@@@@@%#%@@@@%%@@@@@@%@@%@@@@@@@*@%@@@%@@@@@@@@@@@@%@@%@%@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@%@@@@@#%@@@@@@@@@@@@@@@+%@@%@@@@@@@@@@@@%@@%@#@@@@@@%@@@@@@@@@@@@@@@@@%@%%@@@@@@@@@@@@#@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@*%@@@@@@@%@@@@%@@@@@@@%@@@@@@@@@%@@@%@@%%%##%%#@@@@@@@@@@@@@@@%@@@@@@@@@@@%@@@@@@@@@@@@@%@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@%%@@@%@@@@@@@@@@@@@@%@@@@%%@@@@@@@@@#@%%@@@%#. +%#%%@%#@@@@@@%@@@@@#@@@@@@@#@@@@@@@@@@@@@@%@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@%@%@@@@@@@@@@@@%@@#%@@@@@@@@@@@@@@@@@@@%#**#%%@%@@@@@@@@%@@@@@@@#@@@@@%@@@@@@@@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@%@@@@@@@@@%%@@@@@@@@@@@%@@@%@@@@@@@@@@@@%%%@@@%#%@%@*@@@@@@@@@@@@@@@@@@@@@**@@@@@@@%@@@@@@%");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@%@@@@@@@@@@@@%*@@@@@@@@@@@@@#@@@@%%%@@@#%%@@@@@@@@@%#@%@@@@@@@@@@@@@@@@%@@@@@@@@@@@#@@@%%@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@%@@@@@@@@@@@@@@@%@@%@@@@@@%%@@@@@%@@@@@@@%@@%%@@@@@@@@@@@@@@@@@@@%%%@@@@@@@@@@@%@@%@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@@@@@@@@@%#%@@%%@@@@@@@@@@@@@@@@@@@%@%%@@@@%@@@@@@@@@@@%%%%@@@@@@@#%@@@@@@%@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%#@@@%@@@@@@@@@@@@@@%#@@@@@@%@@@@@@@@%@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@%@@@@@@@%@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@%@%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@%@%@@%@@@@@@@@@@@@@@# %@@@@%@@@@@@@@@@%@@@@@%@@%%%@%@@@@@@@@@@##@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@%@@#%@@@@@@@@@@@%#@@@@@@@@@@@@@@@@@@#@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@+@@@@@@@@@@%@@%@@%@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@%@@@@#%@@@@@@@@@@@@@@@@@@@@@%@@@%%@@%%@@@@@@%%@@%@@@@@@%@@@@@@@@@%@@@@@@@@@@@@#@@%@@%%@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@%%@@@@@@@@@@@@@*%@@@@@@@@@@@@@@@@@@@@@@@%#%@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@*@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@%%@@@@@@@@@@@@@@@@%@@@@@@@@@@@@-@@@@@@@@@@@@@@#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%@@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@%@@@@@%@@@@@@%%%%@@@@@@@@@@@%@#@%@@%@@@@@@%@%-@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@=@@@@@%@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@%#@@@@@@@@@@@@@@@@@@%%@@@@@%@@@@@@@@@@@@@@@@@@%%%%%@%%%@%%@@%@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@%@%%@@@#@@@@@@@*%@@@@@@@%@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%%%%##%%##%%%%%@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@%@%%@@@@@@%@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@%%##%%%%%%%%%#%@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@#@@%@@%@@@@@@@@%@@@@@@%@@@@@@@@@@@@@@@@@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@#@%@@@@@@@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%@@@@@@%@@@@@@@@@@@@@@@@@@@@@@@%@@@@@%@@@@@@@@@@@@@@@@@@@@@@*@@@@@@@");
//LPUART_ESC_print("[1B");
//LPUART_ESC_print("[100D");
//LPUART_ESC_print("[u");		// upload cursor loc
//}
//
//void GAME_character (void) {
//	/*Prints the character at the desired location on interface grid*/
//	/*Leaves cursor where it started*/
////	LPUART_ESC_print("");
//	LPUART_ESC_print("[2A");
//	LPUART_print("\\_//");
//	LPUART_ESC_print("[1B");
//	LPUART_ESC_print("[6D");
//	LPUART_print("__/''.");
//	LPUART_ESC_print("[1B");
//	LPUART_ESC_print("[7D");
//	LPUART_print("/__ |");
//	LPUART_ESC_print("[1B");
//	LPUART_ESC_print("[5D");
//	LPUART_print("|| ||");
//	LPUART_ESC_print("[1A");
//	LPUART_ESC_print("[3D");
///*
//prs \\_//
//   __/''.
//  /__ |
//  || ||
//*/
//	; // nothing
//}
//
//void SystemClock_Config(void)
//{
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//	/** Configure the main internal regulator output voltage
//	 */
//	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the RCC Oscillators according to the specified parameters
//	 * in the RCC_OscInitTypeDef structure.
//	 */
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//	RCC_OscInitStruct.MSICalibrationValue = 0;
//	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the CPU, AHB and APB buses clocks
//	 */
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}
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
//void Error_Handler(void)
//{
//	/* USER CODE BEGIN Error_Handler_Debug */
//	/* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
//	while (1)
//	{
//	}
//	/* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
// * @brief  Reports the name of the source file and the source line number
// *         where the assert_param error has occurred.
// * @param  file: pointer to the source file name
// * @param  line: assert_param error line source number
// * @retval None
// */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//	/* USER CODE BEGIN 6 */
//	/* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//	/* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */










// Code with color interrupts
//#include "main.h"
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void LPUART_init( void );
//void LPUART_print( const char* message );
//void LPUART_ESC_print( const char* message);
//void LPUART1_IRQHandler( void  );
//
//int main(void)
//{
//
//
//
//
//	HAL_Init();
//	SystemClock_Config();
//	LPUART_init();
//
//	// Configure user button
//	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
//	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
//	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);
//
//
//	while (1)
//	{
//		if ( (GPIOC->IDR & GPIO_PIN_13) == GPIO_PIN_13 ) {
//			LPUART_ESC_print("[2J");	// clear screen
//			//delay_us( 50 );
//			LPUART_ESC_print("[H");		// Cursor upperleft
//			//delay_us( 50 );
//			LPUART_ESC_print("[3B");	// Down 3 lines
//			//delay_us( 50 );
//			LPUART_ESC_print("[5C");	// right 5 spaces
//			//delay_us( 50 );
//			LPUART_print("All good students read the");
//			//delay_us( 50 );
//			LPUART_ESC_print("[1B");	// down one line
//			//delay_us( 50 );
//			LPUART_ESC_print("[21D");	// left 21 spaces
//			//delay_us( 50 );
//			LPUART_ESC_print("[5m");	// turn on blinking mode
//			//delay_us( 50 );
//			LPUART_print("Reference Manual");
//			//delay_us( 50 );
//			LPUART_ESC_print("[H");		// cursor upperleft
//			//delay_us( 50 );
//			LPUART_ESC_print("[0m");	// character attributes off
//			//delay_us( 50 );
//			LPUART_print("Input:");
//
//			LPUART_print("");
//		}
//	}
//
//}
//
//
//
//void LPUART_init( void ) {
//	PWR->CR2 |= (PWR_CR2_IOSV);              // power avail on PG[15:2] (LPUART1)
//	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
//	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge
//	RCC->CCIPR   |= (0x1UL << 10U);			// 4 MHz SYSCLOCK
//
//	/* USER: configure GPIOG registers MODER/PUPDR/OTYPER/OSPEEDR then
//	 select AF mode and specify which function with AFR[0] and AFR[1] */
//	// Follow order AFR, OTYPER, PUPDR, OSPEEDR, MODDER
//	// Configure PG7 for TX and PG8 for RX
//	GPIOG->AFR[0]  &= ~(GPIO_AFRL_AFSEL7);    	// Clear AF for PG7
//	GPIOG->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);    	// Clear AF for PG7
//	GPIOG->AFR[0]  |= (0x0008UL << (28U));			// Set PG7 AF8
//	GPIOG->AFR[1]  |= (0x0008UL << (0U));			// Set PG8 AF8
//	GPIOG->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);		// Clear
//	GPIOG->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);	// Clear
//	GPIOG->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) | (3 << GPIO_OSPEEDR_OSPEED8_Pos));	// High Speed
//	GPIOG->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);		// Clear
//	GPIOG->MODER |= (GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1);	// AF mode
//
//
//	// Configure LPUART
//	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
//	/*Can only be done before UE is set*/
//	//LPUART1->CR1 |= USART_CR1_PCE;					// parity enable
//	//LPUART1->CR1 &= ~USART_CR1_PS;					// even parity
//	//LPUART1->CR1 |= USART_CR1_PS;					// odd parity
//	//LPUART1->CR2 |= USART_CR2_STOP_1;				// two stop bits
//	/**/
//	LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
//	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
//	LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
//	LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag
//	//LPUART1->BRR = 0x115C;					// Set baud rate for 2 MHz
//	LPUART1->BRR = 0x22B9;					// Set baud rate for 4 MHz
//	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
//	__enable_irq();                          // enable global interrupts
//}
//
//void LPUART_print( const char* message) {
//	uint16_t iStrIdx = 0;
//	while ( message[iStrIdx] != 0 ) {
//		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//			;
//		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
//		iStrIdx++;                             // advance index to next char
//	}
//}
//
//void LPUART_ESC_print( const char* message) {
//	uint16_t iStrIdx = 0;
//	while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//		;
//	LPUART1->TDR = 0x1B;       // initiate ESC
//	while ( message[iStrIdx] != 0 ) {
//		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//			;
//		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
//		iStrIdx++;                             // advance index to next char
//	}
//}
//
//void LPUART1_IRQHandler( void  ) {
//	uint8_t charRecv;
//	if (LPUART1->ISR & USART_ISR_RXNE) {
//		charRecv = LPUART1->RDR;	// Reading RDR resets receive process.
//		switch ( charRecv ) {
//		case 'R':
//			LPUART_ESC_print("[31m");
//			break;
//		case 'G':
//			LPUART_ESC_print("[32m");
//			break;
//		case 'B':
//			LPUART_ESC_print("[34m");
//			break;
//		case 'W':
//			LPUART_ESC_print("[37m");
//			break;
//		default:
//			while( !(LPUART1->ISR & USART_ISR_TXE) )
//				;    // wait for empty TX buffer
//			LPUART1->TDR = charRecv;  // echo char to terminal
//		}  // end switch
//	}
//}
//
//
//void Debounce_button (void) {
//
//}
//
//
//void SystemClock_Config(void)
//{
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//	/** Configure the main internal regulator output voltage
//	 */
//	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the RCC Oscillators according to the specified parameters
//	 * in the RCC_OscInitTypeDef structure.
//	 */
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//	RCC_OscInitStruct.MSICalibrationValue = 0;
//	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the CPU, AHB and APB buses clocks
//	 */
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}
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
//void Error_Handler(void)
//{
//	/* USER CODE BEGIN Error_Handler_Debug */
//	/* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
//	while (1)
//	{
//	}
//	/* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
// * @brief  Reports the name of the source file and the source line number
// *         where the assert_param error has occurred.
// * @param  file: pointer to the source file name
// * @param  line: assert_param error line source number
// * @retval None
// */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//	/* USER CODE BEGIN 6 */
//	/* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//	/* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */









// Added function for ESC commands
//
//#include "main.h"
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void LPUART_init( void );
//void LPUART_print( const char* message );
//void LPUART_ESC_print( const char* message);
//void LPUART1_IRQHandler( void  );
//
//int main(void)
//{
//
//
//
//
//	HAL_Init();
//	SystemClock_Config();
//	LPUART_init();
//
//	// Configure user button
//	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
//	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
//	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);
//
//
//	while (1)
//	{
//		if ( (GPIOC->IDR & GPIO_PIN_13) == GPIO_PIN_13 ) {
//			LPUART_ESC_print("[2J");	// clear screen
//			//delay_us( 50 );
//			LPUART_ESC_print("[H");		// Cursor upperleft
//			//delay_us( 50 );
//			LPUART_ESC_print("[3B");	// Down 3 lines
//			//delay_us( 50 );
//			LPUART_ESC_print("[5C");	// right 5 spaces
//			//delay_us( 50 );
//			LPUART_print("All good students read the");
//			//delay_us( 50 );
//			LPUART_ESC_print("[1B");	// down one line
//			//delay_us( 50 );
//			LPUART_ESC_print("[21D");	// left 21 spaces
//			//delay_us( 50 );
//			LPUART_ESC_print("[5m");	// turn on blinking mode
//			//delay_us( 50 );
//			LPUART_print("Reference Manual");
//			//delay_us( 50 );
//			LPUART_ESC_print("[H");		// cursor upperleft
//			//delay_us( 50 );
//			LPUART_ESC_print("[0m");	// character attributes off
//			//delay_us( 50 );
//			LPUART_print("Input:");
//		}
//	}
//
//}
//
//
//
//void LPUART_init( void ) {
//	PWR->CR2 |= (PWR_CR2_IOSV);              // power avail on PG[15:2] (LPUART1)
//	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
//	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge
//	RCC->CCIPR   |= (0x1UL << 10U);			// 4 MHz SYSCLOCK
//
//	/* USER: configure GPIOG registers MODER/PUPDR/OTYPER/OSPEEDR then
//	 select AF mode and specify which function with AFR[0] and AFR[1] */
//	// Follow order AFR, OTYPER, PUPDR, OSPEEDR, MODDER
//	// Configure PG7 for TX and PG8 for RX
//	GPIOG->AFR[0]  &= ~(GPIO_AFRL_AFSEL7);    	// Clear AF for PG7
//	GPIOG->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);    	// Clear AF for PG7
//	GPIOG->AFR[0]  |= (0x0008UL << (28U));			// Set PG7 AF8
//	GPIOG->AFR[1]  |= (0x0008UL << (0U));			// Set PG8 AF8
//	GPIOG->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);		// Clear
//	GPIOG->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);	// Clear
//	GPIOG->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) | (3 << GPIO_OSPEEDR_OSPEED8_Pos));	// High Speed
//	GPIOG->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);		// Clear
//	GPIOG->MODER |= (GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1);	// AF mode
//
//
//	// Configure LPUART
//	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
//	/*Can only be done before UE is set*/
//	//LPUART1->CR1 |= USART_CR1_PCE;					// parity enable
//	//LPUART1->CR1 &= ~USART_CR1_PS;					// even parity
//	//LPUART1->CR1 |= USART_CR1_PS;					// odd parity
//	//LPUART1->CR2 |= USART_CR2_STOP_1;				// two stop bits
//	/**/
//	LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
//	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
//	LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
//	LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag
//	//LPUART1->BRR = 0x115C;					// Set baud rate for 2 MHz
//	LPUART1->BRR = 0x22B9;					// Set baud rate for 4 MHz
//	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
//	__enable_irq();                          // enable global interrupts
//}
//
//void LPUART_print( const char* message) {
//	uint16_t iStrIdx = 0;
//	while ( message[iStrIdx] != 0 ) {
//		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//			;
//		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
//		iStrIdx++;                             // advance index to next char
//	}
//}
//
//void LPUART_ESC_print( const char* message) {
//	uint16_t iStrIdx = 0;
//	while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//		;
//	LPUART1->TDR = 0x1B;       // initiate ESC
//	while ( message[iStrIdx] != 0 ) {
//		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//			;
//		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
//		iStrIdx++;                             // advance index to next char
//	}
//}
//
//void LPUART1_IRQHandler( void  ) {
//	uint8_t charRecv;
//	if (LPUART1->ISR & USART_ISR_RXNE) {
//		charRecv = LPUART1->RDR;	// Reading RDR resets receive process.
//		switch ( charRecv ) {
//		case 'R':
//			/* USER: process R to ESCape code back to terminal */
//			break;
//			/* USER : handle other ESCape code cases */
//		default:
//			while( !(LPUART1->ISR & USART_ISR_TXE) )
//				;    // wait for empty TX buffer
//			LPUART1->TDR = charRecv;  // echo char to terminal
//		}  // end switch
//	}
//}
//
//
//
//
//
//void SystemClock_Config(void)
//{
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//	/** Configure the main internal regulator output voltage
//	 */
//	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the RCC Oscillators according to the specified parameters
//	 * in the RCC_OscInitTypeDef structure.
//	 */
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//	RCC_OscInitStruct.MSICalibrationValue = 0;
//	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the CPU, AHB and APB buses clocks
//	 */
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}
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
//void Error_Handler(void)
//{
//	/* USER CODE BEGIN Error_Handler_Debug */
//	/* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
//	while (1)
//	{
//	}
//	/* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
// * @brief  Reports the name of the source file and the source line number
// *         where the assert_param error has occurred.
// * @param  file: pointer to the source file name
// * @param  line: assert_param error line source number
// * @retval None
// */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//	/* USER CODE BEGIN 6 */
//	/* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//	/* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */








// Code to print a character
//
//#include "main.h"
//
//
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//void LPUART_init( void );
//void LPUART_print( const char* message );
//void LPUART1_IRQHandler( void  );
//
//
//
//
//int main(void)
//{
//
//
//
//
//	HAL_Init();
//	SystemClock_Config();
//	LPUART_init();
//
//	// Configure user button
//	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
//	GPIOC->MODER   &= ~(GPIO_MODER_MODE13);
//	GPIOC->PUPDR   |= (GPIO_PUPDR_PUPD13_1);
//
//
//	while (1)
//	{
//		if ( (GPIOC->IDR & GPIO_PIN_13) == GPIO_PIN_13 ) {
//			LPUART_print("Clairo");
//		}
//	}
//
//}
//
//
//
//void LPUART_init( void ) {
//	PWR->CR2 |= (PWR_CR2_IOSV);              // power avail on PG[15:2] (LPUART1)
//	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
//	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge
//	RCC->CCIPR   |= (0x1UL << 10U);			// 4 MHz SYSCLOCK
//
//	/* USER: configure GPIOG registers MODER/PUPDR/OTYPER/OSPEEDR then
//	 select AF mode and specify which function with AFR[0] and AFR[1] */
//	// Follow order AFR, OTYPER, PUPDR, OSPEEDR, MODDER
//	// Configure PG7 for TX and PG8 for RX
//	GPIOG->AFR[0]  &= ~(GPIO_AFRL_AFSEL7);    	// Clear AF for PG7
//	GPIOG->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);    	// Clear AF for PG7
//	GPIOG->AFR[0]  |= (0x0008UL << (28U));			// Set PG7 AF8
//	GPIOG->AFR[1]  |= (0x0008UL << (0U));			// Set PG8 AF8
//	GPIOG->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);		// Clear
//	GPIOG->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);	// Clear
//	GPIOG->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) | (3 << GPIO_OSPEEDR_OSPEED8_Pos));	// High Speed
//	GPIOG->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);		// Clear
//	GPIOG->MODER |= (GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1);	// AF mode
//
//
//	// Configure LPUART
//	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
//	LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
//	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
//	LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
//	LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag
//	//LPUART1->BRR = 0x115C;					// Set baud rate for 2 MHz
//	LPUART1->BRR = 0x22B9;					// Set baud rate for 4 MHz
//	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
//	__enable_irq();                          // enable global interrupts
//}
//
//void LPUART_print( const char* message) {
//	uint16_t iStrIdx = 0;
//	while ( message[iStrIdx] != 0 ) {
//		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
//			;
//		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
//		iStrIdx++;                             // advance index to next char
//	}
//}
//
//void LPUART1_IRQHandler( void  ) {
//	uint8_t charRecv;
//	if (LPUART1->ISR & USART_ISR_RXNE) {
//		charRecv = LPUART1->RDR;	// Reading RDR resets receive process.
//		switch ( charRecv ) {
//		case 'R':
//			/* USER: process R to ESCape code back to terminal */
//			break;
//			/* USER : handle other ESCape code cases */
//		default:
//			while( !(LPUART1->ISR & USART_ISR_TXE) )
//				;    // wait for empty TX buffer
//			LPUART1->TDR = charRecv;  // echo char to terminal
//		}  // end switch
//	}
//}
//
//
//
//
//
//void SystemClock_Config(void)
//{
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//	/** Configure the main internal regulator output voltage
//	 */
//	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the RCC Oscillators according to the specified parameters
//	 * in the RCC_OscInitTypeDef structure.
//	 */
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//	RCC_OscInitStruct.MSICalibrationValue = 0;
//	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	/** Initializes the CPU, AHB and APB buses clocks
//	 */
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}
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
//void Error_Handler(void)
//{
//	/* USER CODE BEGIN Error_Handler_Debug */
//	/* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
//	while (1)
//	{
//	}
//	/* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
// * @brief  Reports the name of the source file and the source line number
// *         where the assert_param error has occurred.
// * @param  file: pointer to the source file name
// * @param  line: assert_param error line source number
// * @retval None
// */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//	/* USER CODE BEGIN 6 */
//	/* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//	/* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
