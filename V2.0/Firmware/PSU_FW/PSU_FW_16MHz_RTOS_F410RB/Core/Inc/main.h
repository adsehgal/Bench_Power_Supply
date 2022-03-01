/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_crc.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

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
#define HSE_IN_Pin LL_GPIO_PIN_0
#define HSE_IN_GPIO_Port GPIOH
#define HSE_OUT_Pin LL_GPIO_PIN_1
#define HSE_OUT_GPIO_Port GPIOH
#define REG_EN_Pin LL_GPIO_PIN_1
#define REG_EN_GPIO_Port GPIOC
#define SENSE_IOUT_Pin LL_GPIO_PIN_3
#define SENSE_IOUT_GPIO_Port GPIOC
#define SENSE_VOUT_Pin LL_GPIO_PIN_0
#define SENSE_VOUT_GPIO_Port GPIOA
#define SENSE_VIN_Pin LL_GPIO_PIN_1
#define SENSE_VIN_GPIO_Port GPIOA
#define USART2_TX_Pin LL_GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin LL_GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define SPI1_SCK_Pin LL_GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin LL_GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin LL_GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define SPI_FLASH_NCS_Pin LL_GPIO_PIN_4
#define SPI_FLASH_NCS_GPIO_Port GPIOC
#define SPI_FLASH_NHOLD_Pin LL_GPIO_PIN_5
#define SPI_FLASH_NHOLD_GPIO_Port GPIOC
#define SPI_FLASH_NWP_Pin LL_GPIO_PIN_0
#define SPI_FLASH_NWP_GPIO_Port GPIOB
#define LED_VI_Pin LL_GPIO_PIN_12
#define LED_VI_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWDCK_Pin LL_GPIO_PIN_14
#define SWDCK_GPIO_Port GPIOA
#define LED_OE_Pin LL_GPIO_PIN_15
#define LED_OE_GPIO_Port GPIOA
#define LED_CC_Pin LL_GPIO_PIN_10
#define LED_CC_GPIO_Port GPIOC
#define NBTN_INT_Pin LL_GPIO_PIN_11
#define NBTN_INT_GPIO_Port GPIOC
#define NBTN_VI_Pin LL_GPIO_PIN_12
#define NBTN_VI_GPIO_Port GPIOC
#define NBTN_DWN_Pin LL_GPIO_PIN_11
#define NBTN_DWN_GPIO_Port GPIOB
#define SWDO_Pin LL_GPIO_PIN_3
#define SWDO_GPIO_Port GPIOB
#define NBTN_UP_Pin LL_GPIO_PIN_4
#define NBTN_UP_GPIO_Port GPIOB
#define NTBN_OE_Pin LL_GPIO_PIN_5
#define NTBN_OE_GPIO_Port GPIOB
#define I2C1_SCL_Pin LL_GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin LL_GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define NOLED_RST_Pin LL_GPIO_PIN_8
#define NOLED_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
