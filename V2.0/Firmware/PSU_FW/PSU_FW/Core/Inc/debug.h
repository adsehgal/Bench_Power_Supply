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
#include "ssd1306.h"
#include "mcp4018.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

void printMsg(char *format, ...);

uint8_t i2cScan(void);

void oeLedOn(void);

void oeLedOff(void);

void oeLedToggle(void);

void ccLedOn(void);

void ccLedOff(void);

void ccLedToggle(void);

#endif /* INC_DEBUG_H_ */
