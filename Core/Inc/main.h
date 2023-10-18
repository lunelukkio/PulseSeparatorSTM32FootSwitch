/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Rotary_encorder_A_Pin GPIO_PIN_2
#define Rotary_encorder_A_GPIO_Port GPIOC
#define Rotary_encorder_A_EXTI_IRQn EXTI2_IRQn
#define Rotary_encorder_B_Pin GPIO_PIN_3
#define Rotary_encorder_B_GPIO_Port GPIOC
#define Rotary_encorder_B_EXTI_IRQn EXTI3_IRQn
#define Camera_trigger_Pin GPIO_PIN_1
#define Camera_trigger_GPIO_Port GPIOA
#define Camera_trigger_EXTI_IRQn EXTI1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LED_Yellow_Pin GPIO_PIN_6
#define LED_Yellow_GPIO_Port GPIOA
#define LED_White_Pin GPIO_PIN_7
#define LED_White_GPIO_Port GPIOA
#define Cursor_Pin GPIO_PIN_4
#define Cursor_GPIO_Port GPIOC
#define Cursor_EXTI_IRQn EXTI4_IRQn
#define stim_camera_trigger_Pin GPIO_PIN_5
#define stim_camera_trigger_GPIO_Port GPIOC
#define stim_camera_trigger_EXTI_IRQn EXTI9_5_IRQn
#define EPSP_Pin GPIO_PIN_1
#define EPSP_GPIO_Port GPIOB
#define bAP_Pin GPIO_PIN_2
#define bAP_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define stim_software_trigger_Pin GPIO_PIN_7
#define stim_software_trigger_GPIO_Port GPIOB
#define stim_software_trigger_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
