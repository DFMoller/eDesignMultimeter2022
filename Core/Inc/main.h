/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Delay_us_10(uint8_t tens);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define btn_mid_Pin GPIO_PIN_6
#define btn_mid_GPIO_Port GPIOA
#define btn_mid_EXTI_IRQn EXTI9_5_IRQn
#define btn_right_Pin GPIO_PIN_7
#define btn_right_GPIO_Port GPIOA
#define btn_right_EXTI_IRQn EXTI9_5_IRQn
#define btn_down_Pin GPIO_PIN_10
#define btn_down_GPIO_Port GPIOB
#define btn_down_EXTI_IRQn EXTI15_10_IRQn
#define lcd_blown_Pin GPIO_PIN_11
#define lcd_blown_GPIO_Port GPIOB
#define lcd_RS_Pin GPIO_PIN_13
#define lcd_RS_GPIO_Port GPIOB
#define lcd_E_Pin GPIO_PIN_15
#define lcd_E_GPIO_Port GPIOB
#define lcd_D4_Pin GPIO_PIN_6
#define lcd_D4_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_7
#define LD3_GPIO_Port GPIOC
#define lcd_D5_Pin GPIO_PIN_8
#define lcd_D5_GPIO_Port GPIOC
#define LD5_Pin GPIO_PIN_9
#define LD5_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_10
#define LD4_GPIO_Port GPIOA
#define lcd_D6_Pin GPIO_PIN_11
#define lcd_D6_GPIO_Port GPIOA
#define lcd_D7_Pin GPIO_PIN_12
#define lcd_D7_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define btn_left_Pin GPIO_PIN_8
#define btn_left_GPIO_Port GPIOB
#define btn_left_EXTI_IRQn EXTI9_5_IRQn
#define btn_up_Pin GPIO_PIN_9
#define btn_up_GPIO_Port GPIOB
#define btn_up_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
