/*
 * buttons.c
 *
 *  Created on: Feb 14, 2021
 *      Author: adityasehgal
 */

#include "buttons.h"
#include "debug.h"

//#define UNCONNECTED	//TODO: comment when buttons are connected

uint8_t readBtnUp(void)
{
#ifdef UNCONNECTED
	return HAL_GPIO_ReadPin(nSW_UP_GPIO_Port, nSW_UP_Pin);
#endif
	return !HAL_GPIO_ReadPin(nSW_UP_GPIO_Port, nSW_UP_Pin);
}

uint8_t readBtnDw(void)
{
#ifdef UNCONNECTED
	return HAL_GPIO_ReadPin(nSW_DW_GPIO_Port, nSW_DW_Pin);
#endif
	return !HAL_GPIO_ReadPin(nSW_DW_GPIO_Port, nSW_DW_Pin);
}

uint8_t readBtnVi(void)
{
#ifdef UNCONNECTED
	return HAL_GPIO_ReadPin(nSW_VI_GPIO_Port, nSW_VI_Pin);
#endif
	return !HAL_GPIO_ReadPin(nSW_VI_GPIO_Port, nSW_VI_Pin);
}

uint8_t readBtnOe(void)
{
#ifdef UNCONNECTED
	return HAL_GPIO_ReadPin(nSW_OE_GPIO_Port, nSW_OE_Pin);
#endif
	return !HAL_GPIO_ReadPin(nSW_OE_GPIO_Port, nSW_OE_Pin);
}

uint8_t whichBtn(void)
{
#ifdef UNCONNECTED
#warning This build is for a dev board without all buttons and returns inverted button reads
#endif
	uint8_t ret = 0b0000;
	while (readBtnUp())
		ret |= UP_BTN;
	while (readBtnDw())
		ret |= DW_BTN;
	while (readBtnVi())
		ret |= VI_BTN;
	if (readBtnOe())
		ret |= OE_BTN;

	return ret;
}

void printBtns(uint8_t btns)
{
	if (btns & UP_BTN)
		printMsg("UP Pressed\n");
	if (btns & DW_BTN)
		printMsg("DW Pressed\n");
	if (btns & VI_BTN)
		printMsg("VI Pressed\n");
	if (btns & OE_BTN)
		printMsg("OE Pressed\n");
}
