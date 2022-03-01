/*
 * led.h
 *
 *  Created on: 28 Feb 2022
 *      Author: adityasehgal
 */

#ifndef LED_H_
#define LED_H_

typedef enum ledIdx_e {
	LED_VI = 0,	//
	LED_OE,		//
	LED_CC,		//
	LED_COUNT,	//
} ledIdx_t;

/**
 * @fn void led_init(void)
 * @brief	Initialize all the onboard LEDs
 */
void led_init(void);

/**
 * @fn void led_on(ledIdx_t)
 * @brief		Turn ON the indexed LED
 * @param idx	Index of the LED to turn ON
 */
void led_on(ledIdx_t idx);

/**
 * @fn void led_off(ledIdx_t)
 * @brief		Turn OFF the indexed LED
 * @param idx	Index of the LED to turn OFF
 */
void led_off(ledIdx_t idx);

/**
 * @fn void led_toggle(ledIdx_t)
 * @brief		Toggle the indexed LED
 * @param idx	Index of the LED to toggle
 */
void led_toggle(ledIdx_t idx);

#endif /* LED_H_ */
