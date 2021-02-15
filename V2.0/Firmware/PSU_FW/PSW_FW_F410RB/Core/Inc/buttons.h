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
	UP_BTN = 0b0001, DW_BTN = 0b0010, VI_BTN = 0b0100, OE_BTN = 0b1000,
};

uint8_t readBtnUp(void);

uint8_t readBtnDw(void);

uint8_t readBtnVi(void);

uint8_t readBtnOe(void);

uint8_t whichBtn(void);

void printBtns(uint8_t btns);

#endif /* INC_BUTTONS_H_ */
