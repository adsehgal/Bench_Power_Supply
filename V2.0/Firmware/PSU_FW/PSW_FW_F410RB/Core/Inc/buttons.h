/*
 * buttons.h
 *
 *  Created on: Feb 14, 2021
 *      Author: adityasehgal
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include <sys/_stdint.h>
#include "main.h"

enum Btns
{
	BTN_NUM_UP = 0b0001, BTN_NUM_DWN = 0b0010, BTN_NUM_VI = 0b0100, BTN_NUM_OE = 0b1000,
};

uint8_t readBtnUp(void);

uint8_t readBtnDw(void);

uint8_t readBtnVi(void);

uint8_t readBtnOe(void);

uint8_t whichBtn(void);

void printBtns(uint8_t btns);

#endif /* INC_BUTTONS_H_ */
