/*
 * buttons.h
 *
 *  Created on: Mar 16, 2021
 *      Author: adityasehgal
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include <sys/_stdint.h>
#include "main.h"

enum Btns {
	BTN_NUM_NONE = 0xFF,
	BTN_NUM_UP = 0x01,
	BTN_NUM_DWN = 0x02,
	BTN_NUM_VI = 0x04,
	BTN_NUM_OE = 0x08,
};

uint8_t btnsRead(void);

uint8_t btnRead(uint8_t btn);

#endif /* INC_BUTTONS_H_ */
