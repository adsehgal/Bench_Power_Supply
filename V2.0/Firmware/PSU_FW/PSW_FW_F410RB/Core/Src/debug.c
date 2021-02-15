/*
 * debug.c
 *
 *  Created on: Feb 14, 2021
 *      Author: adityasehgal
 */


#include "debug.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "leds.h"
#include "ssd1306.h"

void printMsg(char *format, ...)
{
	char str[80];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
}

uint8_t i2cScan(void)
{
	char info[] = "Scanning I2C bus...\r\n";
	printMsg(info);

	uint8_t devices = 0;

	HAL_StatusTypeDef res;

	for (uint16_t i = 0; i < 128; i++)
	{
		res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
		if (res == HAL_OK && (i << 1) == SSD1306_I2C_ADDR)
		{
			devices |= OLED_FOUND;
			printMsg("OLED found at 0x%02X\n", i);
		}
		else if (res == HAL_OK && (i << 1) == MCP4018_I2C_ADDR)
		{
			devices |= POT_FOUND;
			printMsg("MCP4018 found at 0x%02X\n", i);
		}
		else if (res == HAL_OK)
		{
			printMsg("Unknown device found at 0x%02X\n", i);
		}
		else if (res == HAL_OK)
		{
			printMsg("Unknown device found at 0x%02X\n", i);
			HAL_Delay(10);
		}
	}
	printMsg("returning from scan %02X\n", devices);
	return devices;
}
