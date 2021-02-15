/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
  struct Stats
  {
    uint8_t vSet;  //voltage set in a byte for MCP4018
    uint32_t iSet; //current set in mA
    uint8_t iLim;  //bit[0] ? limit reached : limit NOT reached
    uint8_t OE;    //bit[0] ? enabled : disabled
    uint8_t VI;    //bit[0] ? voltage : current
  };
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
  enum textPositions
  {
    INFO_X = 2,
    ON_OFF_X = 105,
    VIN_Y = 0,
    VSET_Y = 12,
    VOUT_Y = 24,
    ISET_Y = 36,
    IOUT_Y = 48,
  };
  enum enables
  {
    OE_ENABLED = SET,
    OE_DISABLED = RESET,
    VI_V_SEL = SET,
    VI_I_SEL = RESET,
    I_LIM_SET = SET,
    I_LIM_NSET = RESET,
  };
  enum defaults
  {
    V_DEFAULT = 0x3F,
    I_DEFAULT = 1000,
    I_LIM_DEFAULT = I_LIM_NSET,
    OE_DEFAULT = OE_DISABLED,
	VI_DEFAULT = VI_V_SEL,
  };
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define SET 1
#define RESET 0
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
  void initPSU(void);

  void showStartup(void);

  void displayVoltageCurrent(double Vin, double V, double I);

  void enableOutput(void);

  void disableOutput(void);

  void buttonsHandler(uint8_t buttons);

  uint32_t vSetCalc(void);

  void fatalErrorScreen(void);

  void printSettings(void);
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
#define VI_LED_Pin GPIO_PIN_12
#define VI_LED_GPIO_Port GPIOA
#define OE_LED_Pin GPIO_PIN_15
#define OE_LED_GPIO_Port GPIOA
#define CC_LED_Pin GPIO_PIN_10
#define CC_LED_GPIO_Port GPIOC
#define nSW_INT_Pin GPIO_PIN_11
#define nSW_INT_GPIO_Port GPIOC
#define nSW_INT_EXTI_IRQn EXTI15_10_IRQn
#define nSW_VI_Pin GPIO_PIN_12
#define nSW_VI_GPIO_Port GPIOC
#define nSW_DW_Pin GPIO_PIN_11
#define nSW_DW_GPIO_Port GPIOB
#define nSW_UP_Pin GPIO_PIN_4
#define nSW_UP_GPIO_Port GPIOB
#define nSW_OE_Pin GPIO_PIN_5
#define nSW_OE_GPIO_Port GPIOB
#define OLED_RST_Pin GPIO_PIN_8
#define OLED_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
