/*
 * leds.c
 *
 *  Created on: 31-Dec-2020
 *      Author: adityasehgal
 */

#include "leds.h"

void errorLEDs(uint8_t error)
{
    while (1)
    {
        if (error == OLED_FOUND)
        {
            HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
            HAL_Delay(FLASH_FREQ);
            HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
            HAL_Delay(FLASH_FREQ >> 1);
            HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
        }
        else if (error == POT_FOUND)
        {
            HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
            HAL_Delay(FLASH_FREQ);
            HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
            HAL_Delay(FLASH_FREQ >> 1);
            HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
        }
        else if (error == 0)
        {
            HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
            HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
            HAL_Delay(FLASH_FREQ >> 1);
            HAL_GPIO_TogglePin(OE_LED_PORT, OE_LED_PIN);
            HAL_GPIO_TogglePin(CC_LED_PORT, CC_LED_PIN);
            HAL_Delay(FLASH_FREQ >> 1);
        }
        else
        {
            break;
        }
    }
}

void oeLedOn(void)
{
}

void oeLedOff(void)
{
}

void oeLedToggle(void)
{
}

void ccLedOn(void)
{
}

void ccLedOff(void)
{
}

void ccLedToggle(void)
{
}
