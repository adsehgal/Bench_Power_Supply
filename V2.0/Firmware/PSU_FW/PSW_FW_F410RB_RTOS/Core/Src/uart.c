/*
 * uart.c
 *
 *  Created on: Mar 8, 2021
 *      Author: adityasehgal
 */

#include <stm32f4xx_hal.h>
#include "cmsis_os2.h"
#include "uart.h"

void txString(char *format, ...) {
	char str[80];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	int len = vsprintf(str, format, args);
	va_end(args);
	while ((HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX)
			|| (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX_RX)) {
		osDelay(10);

	}
//	osDelay(10);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) str, len);
//	HAL_UART_Transmit_IT(&huart2, (uint8_t *)str, len);
//	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

}
