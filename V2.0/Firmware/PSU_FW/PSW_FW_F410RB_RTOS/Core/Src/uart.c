/*
 * uart.c
 *
 *  Created on: Mar 8, 2021
 *      Author: adityasehgal
 */

#include <stm32f4xx_hal.h>
#include "cmsis_os2.h"
#include "uart.h"
#include "stats.h"
#include "leds.h"

extern uint8_t interruptFlags;

void uartTxString(char *format, ...) {
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
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) str, len);
//	HAL_UART_Transmit_IT(&huart2, (uint8_t *)str, len);
	HAL_UART_Transmit(&huart2, (uint8_t*) str, len, HAL_MAX_DELAY);
}

void uartRxIntHandler(uartRxData *uartRx) {
	if (uartRx->uartRxChar == '\n' || uartRx->uartRxChar == '\r') {
		interruptFlags |= INT_FLAG_UART_RX;
	} else {
		strncat(uartRx->uartRxBuff, &uartRx->uartRxChar, 1); //build user input char by char
	}
}

void uartRxStringParser(uartRxData *uartRx, Stats *psuStats) {
	if (interruptFlags & INT_FLAG_UART_RX) {
		interruptFlags &= !INT_FLAG_UART_RX;
		uartRxStringDecoder(uartRx->uartRxBuff, psuStats);
		memset(uartRx->uartRxBuff, '\0', sizeof(uartRx->uartRxBuff));
	}
}

uartRxMsg uartRxStringDecoder(char *str, Stats *psuStats) {
	char *token;
	uartRxMsg ret = MSG_NO_CMD; //Clear command variable
	token = strtok(str, " ");     //get first word
	if (token != NULL)              //check for empty string
	{
		if (!strcmp("V_SET", token)) {
			ret |= MSG_V_SET;
			token = strtok(NULL, " ");
			if (token[0] >= '0' && token[0] <= '9') {
				int vVal = atoi(token);
				double potCalc = 4960.0 / (((double) vVal / 800.0) - 1.0);
				potCalc = (potCalc / 5000.0) * 128.0;
				uint32_t vSet = (uint32_t) potCalc;
				psuStats->vSet = vSet;
				ledToggle(CC_LED);
			} else {
				ret |= MSG_ERR_CMD;
			}
		} else if (!strcmp("I_SET", token)) {
			ret |= MSG_I_SET;
			token = strtok(NULL, " ");
			if (token[0] >= '0' && token[0] <= '9') {
				int iVal = atoi(token);
				psuStats->iSet = (uint32_t)iVal;
			}
		} else if (!strcmp("OE_OFF", token)) {
			ret |= MSG_OE_NEN;
		} else if (!strcmp("OE_ON", token)) {
			ret |= MSG_OE_EN;
		} else if (!strcmp("VI_V_SEL", token)) {
			ret |= MSG_VI_V_SEL;
		} else if (!strcmp("VI_I_SEL", token)) {
			ret |= MSG_VI_I_SEL;
		} else {
			ret = MSG_ERR_CMD;
		}
		token = strtok(NULL, " ");
		if (token != NULL) { //check for more words in string
			ret = MSG_ERR_CMD;
		}
		return ret;
	}

	return MSG_NO_CMD;

}
