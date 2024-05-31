/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file        : LPUART.c
 * @brief       : Configure LPUART and printing and escape code functions
 * project		: EE 329 S'24 Lab A8
 * authors		: Bradley Buzzini
 * version		: 0.1
 * date			: 240528
 * computer		: STM32CubeIDE v.1.15.1 Build: 21094_20240412_1041 (UTC)
 * target		: NUCLEO-L496ZG
 * clock		: 4 MHz MSI to LPUART
 *
 * Citations:
 * Sample Code from A7 Manual
 ******************************************************************************
 * REVISION HISTORY
 * 0.1 240528 BB  Got rid of game functions
 *******************************************************************************
 * LPUART PLAN
 * Configure LPUART for communication
 * Add function to print strings
 * Add function to accept escape commands
 * Handle user transmission via interrupt
 *******************************************************************************
/* USER CODE END Header */
#include "LPUART.h"

void LPUART_init( void ) {
	PWR->CR2 |= (PWR_CR2_IOSV);              // power on PG[15:2] (LPUART1)
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge
	RCC->CCIPR   |= (0x1UL << 10U);			// 4 MHz SYSCLOCK

	/* Configure GPIOG registers MODER/PUPDR/OTYPER/OSPEEDR then
	 select AF mode and specify which function with AFR[0] and AFR[1] */
	// Follow order AFR, OTYPER, PUPDR, OSPEEDR, MODDER
	// Configure PG7 for TX and PG8 for RX
	GPIOG->AFR[0]  &= ~(GPIO_AFRL_AFSEL7);    		// Clear AF for PG7
	GPIOG->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);    		// Clear AF for PG7
	GPIOG->AFR[0]  |= (0x0008UL << (28U));			// Set PG7 AF8
	GPIOG->AFR[1]  |= (0x0008UL << (0U));			// Set PG8 AF8
	GPIOG->OTYPER  &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);		// Clear
	GPIOG->PUPDR   &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);	// Clear
	GPIOG->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED7_Pos) |
			(3 << GPIO_OSPEEDR_OSPEED8_Pos));		// High Speed
	GPIOG->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);		// Clear
	GPIOG->MODER |= (GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1);	// AF mode


	// Configure LPUART
	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
	LPUART1->CR3 |= (0x1UL << 12U);	// disable overrun error
	/* Parity/Stop bits: Can only be done before UE is set*/
	//LPUART1->CR1 |= USART_CR1_PCE;				// parity enable
	//LPUART1->CR1 &= ~USART_CR1_PS;				// even parity
	//LPUART1->CR1 |= USART_CR1_PS;					// odd parity
	//LPUART1->CR2 |= USART_CR2_STOP_1;				// two stop bits
	LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
	LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
	LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag
	//LPUART1->BRR = 0x115C;					// Set baud rate for 2 MHz
	LPUART1->BRR = 0x22B9;						// Set baud rate for 4 MHz
	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
	__enable_irq();                          // enable global interrupts
}

void LPUART_print( const char* message) {
	// Prints given string to Serial Interface
	uint16_t iStrIdx = 0;
	while ( message[iStrIdx] != 0 ) {
		while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
			;
		LPUART1->TDR = message[iStrIdx];       // send this character (resets transmit process)
		iStrIdx++;                             // advance index to next char
	}
}

void LPUART_ESC_print( const char* message) {
	// sends the ESC command for user, followed by code given in string
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

void LPUART1_IRQHandler( void  ) {	// Not needed for A8
	uint8_t charRecv;
	if (LPUART1->ISR & USART_ISR_RXNE) {
		charRecv = LPUART1->RDR;	// Reading RDR resets receive process.
		switch ( charRecv ) {
		case 'R':	// Cursor red
			LPUART_ESC_print("[31m");
			break;
		case 'G':	// Cursor green
			LPUART_ESC_print("[32m");
			break;
		case 'B':	// Cursor Blue
			LPUART_ESC_print("[34m");
			break;
		case 'W':	// Cursor White
			LPUART_ESC_print("[37m");
			break;
		default:
			while( !(LPUART1->ISR & USART_ISR_TXE) )
				;    // wait for empty TX buffer
			LPUART1->TDR = charRecv;  // echo char to terminal
		}  // end switch
	}
}
