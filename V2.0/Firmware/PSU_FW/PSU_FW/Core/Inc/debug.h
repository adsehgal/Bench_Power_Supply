/*
 * debug.h
 *
 *  Created on: Dec 30, 2020
 *      Author: adityasehgal
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdarg.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "ssd1306_conf.h"
#include "mcp4018.h"

#define OLED_FOUND 0x01
#define POT_FOUND 0x02

//#define CC_LED_PORT GPIOC
//#define CC_LED_PIN GPIO_PIN_10
#define CC_LED_PORT GPIOA
#define CC_LED_PIN GPIO_PIN_5
#define OE_LED_PORT GPIOA
#define OE_LED_PIN GPIO_PIN_15
#define FLASH_FREQ 100

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

void printMsg(char *format,...);


uint8_t i2cScan(void);

void errorLEDs(uint8_t error);


#endif /* INC_DEBUG_H_ */
