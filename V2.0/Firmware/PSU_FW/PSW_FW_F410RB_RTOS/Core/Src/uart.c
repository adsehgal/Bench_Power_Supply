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

//void uartTxString(char *format, ...) {
//	char str[UART_TX_BUFF_SIZE];
//
//	va_list args;
//	va_start(args, format);
//	int len = vsprintf(str, format, args);
//	va_end(args);
//
//	osDelay(1);
//	while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY) {
//		osDelay(1);	//make sure nothing else is being transmitted
//	}
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) str, len);
//	while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY) {
//		osDelay(1);	//makes sure string is sent before clearing it
//	}
//}
void uartTxString(char *str) {

	osDelay(1);
//	ledToggle(CC_LED);
	while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY) {
		ledToggle(CC_LED);
		osDelay(100);	//make sure nothing else is being transmitted
	}
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) str, strlen(str));
	while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY) {
		osDelay(1);	//makes sure string is sent before clearing it
	}
}

void uartRxIntHandler(uartRxData *uartRx) {
	if (uartRx->uartRxChar == '\n' || uartRx->uartRxChar == '\r') {
		interruptFlags |= INT_FLAG_UART_RX;
	} else {
		strncat(uartRx->uartRxBuff, &uartRx->uartRxChar, 1); //build user input char by char
	}
}

void uartRxStringParser(uartRxData *uartRx, Stats *psuStats) {
	uint32_t valueToSet = 0;
	if (interruptFlags & INT_FLAG_UART_RX) {
		interruptFlags &= !INT_FLAG_UART_RX;
		uartRxMsg msgType = uartRxStringDecoder(uartRx->uartRxBuff, psuStats,
				&valueToSet);
		uartRxConfigSet(psuStats, msgType, valueToSet);
		memset(uartRx->uartRxBuff, '\0', sizeof(uartRx->uartRxBuff));
	}
}

uartRxMsg uartRxStringDecoder(char *str, Stats *psuStats, uint32_t *valueToSet) {
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
				*valueToSet = vSet;
			} else {
				ret |= MSG_ERR_CMD;
			}
		} else if (!strcmp("I_SET", token)) {
			ret |= MSG_I_SET;
			token = strtok(NULL, " ");
			if (token[0] >= '0' && token[0] <= '9') {
				uint32_t iVal = (uint32_t) atoi(token);
				*valueToSet = iVal;
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
			ledToggle(CC_LED);
		}
		return ret;
	}

	return MSG_NO_CMD;

}

void uartRxConfigSet(Stats *psuStats, uartRxMsg msgType, uint32_t valueToSet) {

	if (msgType == MSG_ERR_CMD) {
		uartTxString("Invalid command, enter \"HELP\" for commands\n");
		return;
	} else if (msgType == MSG_NO_CMD) {
		uartTxString("Invalid command, enter \"HELP\" for commands\n");
		return;
	} else if (msgType == MSG_V_SET) {
		char buff[UART_TX_BUFF_SIZE];
		sprintf(buff, "Voltage Set to: %umV\n", (unsigned int)valueToSet);
//		uartTxString("Voltage Set to: mV\n");//, valueToSet);
		uartTxString(buff);
		psuStats->vSet = valueToSet;
	} else if (msgType == MSG_I_SET) {
//		uartTxString("Current Limit Set to: %dmA", valueToSet);
		psuStats->iSet = valueToSet;
	} else if (msgType == MSG_OE_EN) {
		uartTxString("Output Enabled!\n");
		psuStats->OE = OE_ENABLED;
	} else if (msgType == MSG_OE_NEN) {
		uartTxString("Output Disabled!\n");
		psuStats->OE = OE_DISABLED;
	} else if (msgType == MSG_VI_V_SEL) {
		uartTxString("Voltage Change Selected\n");
		psuStats->VI = VI_V_SEL;
	} else if (msgType == MSG_VI_I_SEL) {
		uartTxString("Current Change Selected\n");
		psuStats->VI = VI_I_SEL;
	} else {
		uartTxString("Invalid command, enter \"HELP\" for commands\n");
		return;
	}
}
