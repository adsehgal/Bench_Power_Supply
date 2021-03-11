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

void uartTxString(char *str) {
	osDelay(1);
	while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY) {
		osDelay(1);	//make sure nothing else is being transmitted
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
				uint32_t vSet = (uint32_t) voltageToPot(vVal);
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
		} else if (!strcmp("HELP", token)) {
			ret |= MSG_HELP;
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
		double voltage = potToVoltage((uint8_t)valueToSet);
		sprintf(buff, "Voltage Set to: %4.2fmV\n", (double) voltage);
		uartTxString(buff);
		psuStats->vSet = valueToSet;
	} else if (msgType == MSG_I_SET) {
		char buff[UART_TX_BUFF_SIZE];
		sprintf(buff, "Current Set to: %4.2umA\n", (unsigned int) valueToSet);
		uartTxString(buff);
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
	} else if (msgType == MSG_HELP) {
		uartTxString("PSU Commands:\n");
		osDelay(5);
		uartTxString("	HELP: Produces Valid Commands List\n");
		osDelay(5);
		uartTxString("	V_SET #: Sets output voltage to user input in mV\n");
		osDelay(5);
		uartTxString("	I_SET #: Sets current limit to user input in mA\n");
		osDelay(5);
		uartTxString("	ΟΕ_ΟΝ: Enables the voltage regulator\n");
		osDelay(5);
		uartTxString("	ΟΕ_ΟFF: Disables the voltage regulator\n");
		osDelay(5);
		uartTxString(
				"	VI_V_SEL: Allows user to change output voltage on board\n");
		osDelay(5);
		uartTxString(
				"	VI_I_SEL: Allows user to change current limit on board\n");
	} else {
		uartTxString("Invalid command, enter \"HELP\" for commands\n");
		osDelay(5);
		return;
	}
	osDelay(5);
}
