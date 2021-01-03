/*
 * leds.c
 *
 *  Created on: 31-Dec-2020
 *      Author: adityasehgal
 */

#include "leds.h"

void errorLEDs(uint8_t error) {
	while (1) {
		if (error == OLED_FOUND) {
			HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
			HAL_Delay(FLASH_FREQ);
			HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
			HAL_Delay(FLASH_FREQ >> 1);
			HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
		} else if (error == POT_FOUND) {
			HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
			HAL_Delay(FLASH_FREQ);
			HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
			HAL_Delay(FLASH_FREQ >> 1);
			HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
		} else if (error == 0) {
			HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
			HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
			HAL_Delay(FLASH_FREQ >> 1);
			HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
			HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
			HAL_Delay(FLASH_FREQ >> 1);
		} else {
			break;
		}
	}
}

void viLedOn(void) {
	HAL_GPIO_WritePin(VI_LED_GPIO_Port, VI_LED_Pin, GPIO_PIN_SET);
}

void viLedOff(void) {
	HAL_GPIO_WritePin(VI_LED_GPIO_Port, VI_LED_Pin, GPIO_PIN_RESET);
}

void viLedToggle(void) {
	HAL_GPIO_TogglePin(VI_LED_GPIO_Port, VI_LED_Pin);
}

void oeLedOn(void) {
	HAL_GPIO_WritePin(OE_LED_GPIO_Port, OE_LED_Pin, GPIO_PIN_SET);
}

void oeLedOff(void) {
	HAL_GPIO_WritePin(OE_LED_GPIO_Port, OE_LED_Pin, GPIO_PIN_RESET);
}

void oeLedToggle(void) {
	HAL_GPIO_TogglePin(OE_LED_GPIO_Port, OE_LED_Pin);
}

void ccLedOn(void) {
	HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_SET);
}

void ccLedOff(void) {
	HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);
}

void ccLedToggle(void) {
	HAL_GPIO_TogglePin(CC_LED_GPIO_Port, CC_LED_Pin);
}

void setLeds(struct Stats psu) {
	if (psu.OE == OE_ENABLED) {
		oeLedOn();
	} else {
		oeLedOff();
	}

	if (psu.VI == VI_V_SEL) {
		viLedOn();
	} else {
		viLedOff();
	}

	if (psu.iLim == I_LIM_SET) {
		ccLedOn();
	} else {
		ccLedOff();
	}
}

