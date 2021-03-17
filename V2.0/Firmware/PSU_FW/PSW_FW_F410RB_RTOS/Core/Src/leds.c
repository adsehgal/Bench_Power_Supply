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
	if (led & LED_NUM_VI)
		HAL_GPIO_WritePin(VI_LED_GPIO_Port, VI_LED_Pin, GPIO_PIN_SET);
	if (led & LED_NUM_CC)
		HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_SET);
	if (led & LED_NUM_OE)
		HAL_GPIO_WritePin(OE_LED_GPIO_Port, OE_LED_Pin, GPIO_PIN_SET);
}

void ledOff(leds led) {
	if (led & LED_NUM_VI)
		HAL_GPIO_WritePin(VI_LED_GPIO_Port, VI_LED_Pin, GPIO_PIN_RESET);
	if (led & LED_NUM_CC)
		HAL_GPIO_WritePin(CC_LED_GPIO_Port, CC_LED_Pin, GPIO_PIN_RESET);
	if (led & LED_NUM_OE)
		HAL_GPIO_WritePin(OE_LED_GPIO_Port, OE_LED_Pin, GPIO_PIN_RESET);
}

void ledToggle(leds led) {
	if (led & LED_NUM_VI)
		HAL_GPIO_TogglePin(VI_LED_GPIO_Port, VI_LED_Pin);
	if (led & LED_NUM_CC)
		HAL_GPIO_TogglePin(CC_LED_GPIO_Port, CC_LED_Pin);
	if (led & LED_NUM_OE)
		HAL_GPIO_TogglePin(OE_LED_GPIO_Port, OE_LED_Pin);
}

void ledSet(Stats *psuStats) {
	if (psuStats->OE == OE_ENABLED) {
		ledOn(LED_NUM_OE);
	} else {
		ledOff(LED_NUM_OE);
	}

	if (psuStats->VI == VI_V_SEL) {
		ledOn(LED_NUM_VI);
	} else {
		ledOff(LED_NUM_VI);
	}

	if (psuStats->iLim == I_LIM_SET) {
		ledOn(LED_NUM_CC);
	} else {
		ledOff(LED_NUM_CC);
	}
}

void ledsInit(void) {
	ledOff(LED_NUM_VI);
	ledOff(LED_NUM_CC);
	ledOff(LED_NUM_OE);
	const uint8_t delay = 50;
	for (int i = 0; i < 10; i++) {
		ledOn(LED_NUM_CC);
		osDelay(delay);
		ledOff(LED_NUM_CC);
		ledOn(LED_NUM_OE);
		osDelay(delay);
		ledOff(LED_NUM_OE);
		ledOn(LED_NUM_VI);
		osDelay(delay);
		ledOff(LED_NUM_VI);
		ledOn(LED_NUM_OE);
		osDelay(delay);
		ledOff(LED_NUM_OE);
	}

}
