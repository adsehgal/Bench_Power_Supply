/*
 * msc4018.h
 *
 *  Created on: 18 Mar 2021
 *      Author: adityasehgal
 */

#ifndef INC_MCP4018_H_
#define INC_MCP4018_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

#define MCP4018_I2C_ADDR (0x2F << 1)

extern I2C_HandleTypeDef hi2c1;

uint8_t mcp4018ReadVal(void);

void mcp4018WriteVal(uint8_t val);

#endif /* INC_MCP4018_H_ */
