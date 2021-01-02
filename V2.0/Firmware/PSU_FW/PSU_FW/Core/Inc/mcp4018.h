/*
 * mcp4018.h
 *
 *  Created on: Dec 30, 2020
 *      Author: adityasehgal
 */

#ifndef INC_MCP4018_H_
#define INC_MCP4018_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

#define MCP4018_I2C_ADDR (0x2F << 1)

extern I2C_HandleTypeDef hi2c1;

uint8_t MCP4018_ReadVal();

void MCP4018_WriteVal(uint8_t val);

#endif /* INC_MCP4018_H_ */
