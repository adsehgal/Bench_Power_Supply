/*
 * mcp4018.c
 *
 *  Created on: Jan 2, 2021
 *      Author: adityasehgal
 */

#include "mcp4018.h"
#include "main.h"


uint8_t MCP4018_ReadVal(){
	uint8_t retVal = 0;
	HAL_I2C_Master_Receive(&hi2c1, MCP4018_I2C_ADDR, &retVal, 1, HAL_MAX_DELAY);
	return retVal;
}

void MCP4018_WriteVal(uint8_t val){
	HAL_I2C_Master_Transmit(&hi2c1, MCP4018_I2C_ADDR, &val, 1, HAL_MAX_DELAY);
}
