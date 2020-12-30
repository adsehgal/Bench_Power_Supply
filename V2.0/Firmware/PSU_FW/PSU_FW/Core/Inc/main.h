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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define REG_EN_Pin GPIO_PIN_1
#define REG_EN_GPIO_Port GPIOC
#define I_SENSE_Pin GPIO_PIN_3
#define I_SENSE_GPIO_Port GPIOC
#define VO_SENSE_Pin GPIO_PIN_0
#define VO_SENSE_GPIO_Port GPIOA
#define VI_SENSE_Pin GPIO_PIN_1
#define VI_SENSE_GPIO_Port GPIOA
#define OE_LED_Pin GPIO_PIN_15
#define OE_LED_GPIO_Port GPIOA
#define CC_LED_Pin GPIO_PIN_10
#define CC_LED_GPIO_Port GPIOC
#define nSW_INT_Pin GPIO_PIN_11
#define nSW_INT_GPIO_Port GPIOC
#define nSW_INT_EXTI_IRQn EXTI15_10_IRQn
#define nSW_VI_Pin GPIO_PIN_12
#define nSW_VI_GPIO_Port GPIOC
#define nSW_DW_Pin GPIO_PIN_2
#define nSW_DW_GPIO_Port GPIOD
#define nSW_UP_Pin GPIO_PIN_4
#define nSW_UP_GPIO_Port GPIOB
#define nSW_OE_Pin GPIO_PIN_5
#define nSW_OE_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_8
#define LCD_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
