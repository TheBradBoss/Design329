/* USER CODE BEGIN Header */
/*******************************************************************************
 * EE 329 A4 Timers & Interrupts
 *******************************************************************************
 * @file           : main.h
 * @brief          : Use LCD to display reaction triggered by timer interrupt
 * project         : EE 329 S'24 Assignment 4
 * authors         : Bradley Buzzini - Ethan Zane
 * version         : 0.3
 * date            : 240501
 * compiler        : STM32CubeIDE v.1.15.0 Build: 20695_20240315_1429 (UTC)
 * target          : NUCLEO-L496ZG
 * clocks          : 4 MHz MSI to AHB2
 * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
 *******************************************************************************
 * See main.c for more details
 *******************************************************************************
 * Configure defines for different timing in game.
 *******************************************************************************
 *
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Defines -------------------------------------------------------------------*/
/* Interrupt */
#define TIME_UP		0xFFFFFFFF				// ARR initialize val
#define CCR1_VALUE 	((uint32_t)50000000) 	// CCR1 initialize val

/* RNG Defines */
#define RNG_CONVERTER	((uint32_t)107) // RNG 0-10s (in 4 Mhz)

#define TIM2_TEN	40000000	// TIM2 count for 10s
#define TIM2_FIVE	20000000	// TIM2 count for 5s

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define USB_OverCurrent_Pin GPIO_PIN_5
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define STLK_RX_Pin GPIO_PIN_7
#define STLK_RX_GPIO_Port GPIOG
#define STLK_TX_Pin GPIO_PIN_8
#define STLK_TX_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SMPS_V1_Pin GPIO_PIN_10
#define SMPS_V1_GPIO_Port GPIOG
#define SMPS_EN_Pin GPIO_PIN_11
#define SMPS_EN_GPIO_Port GPIOG
#define SMPS_PG_Pin GPIO_PIN_12
#define SMPS_PG_GPIO_Port GPIOG
#define SMPS_SW_Pin GPIO_PIN_13
#define SMPS_SW_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */






























///* USER CODE BEGIN Header */
///*******************************************************************************
// * EE 329 A4 Timers & Interrupts (Part B)
// *******************************************************************************
// * @file           : main.h
// * @brief          : Use single interrupt to output square wave
// * project         : EE 329 S'24 Assignment 4
// * authors         : Bradley Buzzini - Ethan Zane
// * version         : 0.1
// * date            : 240430
// * compiler        : STM32CubeIDE v.1.15.0 Build: 20695_20240315_1429 (UTC)
// * target          : NUCLEO-L496ZG
// * clocks          : 4 MHz MSI to AHB2
// * @attention      : (c) 2024 STMicroelectronics.  All rights reserved.
// *******************************************************************************
// * See main.c for more details
// *******************************************************************************
// *
///* USER CODE END Header */
//
///* Define to prevent recursive inclusion -------------------------------------*/
//#ifndef __MAIN_H
//#define __MAIN_H
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
///* Includes ------------------------------------------------------------------*/
//#include "stm32l4xx_hal.h"
//
///* Defines -------------------------------------------------------------------*/
//// For part A
////#define PERIOD 0x31F // 800: 5kHz period given 4Mhz clock
////#define CCR1_VALUE 0x257 // 600: 25% duty cycle given 5kHz period
//
//
////For part B
//#define PERIOD 0xFFFFFFFF 	// Max ARR
//#define CCR1_VALUE 400 		// 5kHz, 50% Square Wave
////#define CCR1_VALUE 110 	// Minimum value before square wave 'breaks'
//#define CCR1_START 8000000
//
///* Exported functions prototypes ---------------------------------------------*/
//void Error_Handler(void);
//
///* Private defines -----------------------------------------------------------*/
//#define B1_Pin GPIO_PIN_13
//#define B1_GPIO_Port GPIOC
//#define MCO_Pin GPIO_PIN_0
//#define MCO_GPIO_Port GPIOH
//#define LD3_Pin GPIO_PIN_14
//#define LD3_GPIO_Port GPIOB
//#define USB_OverCurrent_Pin GPIO_PIN_5
//#define USB_OverCurrent_GPIO_Port GPIOG
//#define USB_PowerSwitchOn_Pin GPIO_PIN_6
//#define USB_PowerSwitchOn_GPIO_Port GPIOG
//#define STLK_RX_Pin GPIO_PIN_7
//#define STLK_RX_GPIO_Port GPIOG
//#define STLK_TX_Pin GPIO_PIN_8
//#define STLK_TX_GPIO_Port GPIOG
//#define USB_SOF_Pin GPIO_PIN_8
//#define USB_SOF_GPIO_Port GPIOA
//#define USB_VBUS_Pin GPIO_PIN_9
//#define USB_VBUS_GPIO_Port GPIOA
//#define USB_ID_Pin GPIO_PIN_10
//#define USB_ID_GPIO_Port GPIOA
//#define USB_DM_Pin GPIO_PIN_11
//#define USB_DM_GPIO_Port GPIOA
//#define USB_DP_Pin GPIO_PIN_12
//#define USB_DP_GPIO_Port GPIOA
//#define TMS_Pin GPIO_PIN_13
//#define TMS_GPIO_Port GPIOA
//#define TCK_Pin GPIO_PIN_14
//#define TCK_GPIO_Port GPIOA
//#define SMPS_V1_Pin GPIO_PIN_10
//#define SMPS_V1_GPIO_Port GPIOG
//#define SMPS_EN_Pin GPIO_PIN_11
//#define SMPS_EN_GPIO_Port GPIOG
//#define SMPS_PG_Pin GPIO_PIN_12
//#define SMPS_PG_GPIO_Port GPIOG
//#define SMPS_SW_Pin GPIO_PIN_13
//#define SMPS_SW_GPIO_Port GPIOG
//#define SWO_Pin GPIO_PIN_3
//#define SWO_GPIO_Port GPIOB
//#define LD2_Pin GPIO_PIN_7
//#define LD2_GPIO_Port GPIOB
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif /* __MAIN_H */
