/*
 * mcp4018.c
 *
 *  Created on: 18 Mar 2021
 *      Author: adityasehgal
 */

#include "mcp4018.h"
#include "main.h"

uint8_t mcp4018ReadVal(void) {
	uint8_t retVal = 0;
	HAL_I2C_Master_Receive(&hi2c1, MCP4018_I2C_ADDR, &retVal, 1, HAL_MAX_DELAY);
	return retVal;
}

void mcp4018WriteVal(uint8_t val) {
	HAL_I2C_Master_Transmit(&hi2c1, MCP4018_I2C_ADDR, &val, 1, HAL_MAX_DELAY);
}
