/*
 * uart.c
 *
 *  Created on: Mar 8, 2021
 *      Author: adityasehgal
 */

#include <stm32f4xx_hal.h>
#include "uart.h"

void txString(char *format, ...)
{
	char str[80];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
	va_end(args);
}
