/*
 * leds.h
 *
 *  Created on: Mar 7, 2021
 *      Author: adityasehgal
 */

#ifndef INC_LEDS_H_
#define INC_LEDS_H_

#include <sys/_stdint.h>
#include "stats.h"

#define FLASH_FREQ 100

typedef enum led_type{
	VI_LED = 0x01,
	CC_LED = 0x02,
	OE_LED = 0x04
}leds;


void ledsError(uint8_t error);

void ledOn(leds led);

void ledOff(leds led);

void ledToggle(leds led);

void ledSet(Stats *psuStats);

void ledsInit(void);

#endif /* INC_LEDS_H_ */
