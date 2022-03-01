/*
 * led.c
 *
 *  Created on: 28 Feb 2022
 *      Author: adityasehgal
 */

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"

#include "cmsis_os.h"

#include "led.h"

#define LED_VI_PIN LL_GPIO_PIN_12
#define LED_VI_PORT GPIOA

#define LED_OE_PIN LL_GPIO_PIN_15
#define LED_OE_PORT GPIOA

#define LED_CC_PIN LL_GPIO_PIN_10
#define LED_CC_PORT GPIOC

typedef struct led_s {
	uint32_t pin;
	GPIO_TypeDef *port;
	uint8_t state;
} led_t;

// The ordering in this array must follow that of ledIdx_t in led.h
static led_t leds[LED_COUNT] = {
// LED data
		{ .pin = LED_VI_PIN, .port = LED_VI_PORT, .state = 0 }, //
		{ .pin = LED_OE_PIN, .port = LED_OE_PORT, .state = 0 }, //
		{ .pin = LED_CC_PIN, .port = LED_CC_PORT, .state = 0 }, //
		};

void led_init(void) {
	LL_GPIO_InitTypeDef gpio = { 0 };
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	gpio.Mode = LL_GPIO_MODE_OUTPUT;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_NO;
	gpio.Speed = LL_GPIO_SPEED_FREQ_LOW;

	for (ledIdx_t i = 0; i < LED_COUNT; i++) {
		gpio.Pin = leds[i].pin;
		LL_GPIO_Init(leds[i].port, &gpio);

		if (leds[i].state) {
			LL_GPIO_SetOutputPin(leds[i].port, leds[i].pin);
		} else {
			LL_GPIO_ResetOutputPin(leds[i].port, leds[i].pin);
		}
	}
}

void led_on(ledIdx_t idx) {
	if (idx >= LED_COUNT) {
		return;
	} else {
		LL_GPIO_SetOutputPin(leds[idx].port, leds[idx].pin);
	}
}

void led_off(ledIdx_t idx) {
	if (idx >= LED_COUNT) {
		return;
	} else {
		LL_GPIO_ResetOutputPin(leds[idx].port, leds[idx].pin);
	}
}

void led_toggle(ledIdx_t idx) {
	if (idx >= LED_COUNT) {
		return;
	} else {
		LL_GPIO_TogglePin(leds[idx].port, leds[idx].pin);
	}
}
