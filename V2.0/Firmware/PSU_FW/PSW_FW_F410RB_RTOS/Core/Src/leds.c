/*
 * leds.c
 *
 *  Created on: Mar 7, 2021
 *      Author: adityasehgal
 */

#include <cmsis_os2.h>
#include <leds.h>
#include <main.h>
#include <stm32f4xx_hal_gpio.h>

void ledsError(uint8_t error) {
	while (1) {
		if (error == OLED_FOUND) {
			HAL_GPIO_TogglePin(CC_LED_GPIO_Port, CC_LED_Pin);
			osDelay(FLASH_FREQ);
			HAL_GPIO_TogglePin(OE_LED_GPIO_Port, OE_LED_Pin);
			osDelay(FLASH_FREQ >> 1);
			HAL_GPIO_TogglePin(OE_LED_GPIO_Port, OE_LED_Pin);
			osDelay(FLASH_FREQ);
		} else if (error == POT_FOUND) {
			HAL_GPIO_TogglePin(OE_LED_GPIO_Port, OE_LED_Pin);
			osDelay(FLASH_FREQ);
			HAL_GPIO_TogglePin(CC_LED_GPIO_Port, CC_LED_Pin);
			osDelay(FLASH_FREQ >> 1);
			HAL_GPIO_TogglePin(CC_LED_GPIO_Port, CC_LED_Pin);
			osDelay(FLASH_FREQ >> 1);
		} else if (error == 0) {
			HAL_GPIO_TogglePin(OE_LED_GPIO_Port, OE_LED_Pin);
			HAL_GPIO_TogglePin(CC_LED_GPIO_Port, CC_LED_Pin);
			osDelay(FLASH_FREQ >> 1);
			HAL_GPIO_TogglePin(OE_LED_GPIO_Port, OE_LED_Pin);
			HAL_GPIO_TogglePin(CC_LED_GPIO_Port, CC_LED_Pin);
			osDelay(FLASH_FREQ >> 1);
		} else {
			break;
		}
	}
}

void ledOn(leds led) {
	if (led & VI_LED)
		HAL_GPIO_WritePin(VI_LED_GPIO_Port, VI_LED_Pin, GPIO_PIN_SET);
	if (led & CC_LED)
		HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_SET);
	if (led & OE_LED)
		HAL_GPIO_WritePin(OE_LED_GPIO_Port, OE_LED_Pin, GPIO_PIN_SET);
}

void ledOff(leds led) {
	if (led & VI_LED)
		HAL_GPIO_WritePin(VI_LED_GPIO_Port, VI_LED_Pin, GPIO_PIN_RESET);
	if (led & CC_LED)
		HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);
	if (led & OE_LED)
		HAL_GPIO_WritePin(OE_LED_GPIO_Port, OE_LED_Pin, GPIO_PIN_RESET);
}

void ledToggle(leds led) {
	if (led & VI_LED)
		HAL_GPIO_TogglePin(VI_LED_GPIO_Port, VI_LED_Pin);
	if (led & CC_LED)
		HAL_GPIO_TogglePin(CC_LED_GPIO_Port, CC_LED_Pin);
	if (led & OE_LED)
		HAL_GPIO_TogglePin(OE_LED_GPIO_Port, OE_LED_Pin);
}

void ledSet(struct Stats psuStats) {
	if (psuStats.OE == OE_ENABLED) {
		ledOn(OE_LED);
	} else {
		ledOff(OE_LED);
	}

	if (psuStats.VI == VI_V_SEL) {
		ledOn(VI_LED);
	} else {
		ledOff(VI_LED);
	}

	if (psuStats.iLim == I_LIM_SET) {
		ledOn(CC_LED);
	} else {
		ledOff(CC_LED);
	}
}

void ledsInit(void) {
	ledOff(VI_LED);
	ledOff(CC_LED);
	ledOff(OE_LED);
	const uint8_t delay = 50;
	for (int i = 0; i < 10; i++) {
		ledOn(CC_LED);
		osDelay(delay);
		ledOff(CC_LED);
		ledOn(OE_LED);
		osDelay(delay);
		ledOff(OE_LED);
		ledOn(VI_LED);
		osDelay(delay);
		ledOff(VI_LED);
		ledOn(OE_LED);
		osDelay(delay);
		ledOff(OE_LED);
	}

}
