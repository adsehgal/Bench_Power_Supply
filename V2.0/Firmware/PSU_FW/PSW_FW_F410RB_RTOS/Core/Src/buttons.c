/*
 * buttons.c
 *
 *  Created on: Mar 16, 2021
 *      Author: adityasehgal
 */
#include "buttons.h"
#include "stdio.h"
#include "uart.h"

uint8_t btnRead(uint8_t btn) {
	if (btn == BTN_NUM_UP)
		return !HAL_GPIO_ReadPin(nSW_UP_GPIO_Port, nSW_UP_Pin);

	if (btn == BTN_NUM_DWN)
		return !HAL_GPIO_ReadPin(nSW_DW_GPIO_Port, nSW_DW_Pin);

	if (btn == BTN_NUM_VI)
		return !HAL_GPIO_ReadPin(nSW_VI_GPIO_Port, nSW_VI_Pin);

	if (btn == BTN_NUM_OE)
		return !HAL_GPIO_ReadPin(nSW_OE_GPIO_Port, nSW_OE_Pin);
	return BTN_NUM_NONE;
}

uint8_t btnsRead(void) {
	uint8_t ret = BTN_NUM_NONE;
	if (btnRead(BTN_NUM_UP))
		return BTN_NUM_UP;
	if (btnRead(BTN_NUM_DWN))
		return BTN_NUM_DWN;
	if (btnRead(BTN_NUM_VI))
		return BTN_NUM_VI;
	if (btnRead(BTN_NUM_OE))
		return BTN_NUM_OE;
//	char buff[80];

//	sprintf(buff, "btn = %X\n", ret);
//	uartTxString(buff);

	return ret;
}
