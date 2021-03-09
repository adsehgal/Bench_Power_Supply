/*
 * oled.h
 *
 *  Created on: Mar 7, 2021
 *      Author: adityasehgal
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "ssd1306.h"
#include "stats.h"

#define	INFO_X 2
#define ON_OFF_X 105
#define	VIN_Y 0
#define	VSET_Y 12
#define	VOUT_Y 24
#define	ISET_Y 36
#define	IOUT_Y 48

#define INFO_TEXT_SIZE Font_7x10
#define ERROR_TEXT_SIZE Font_11x18

/**
 * @brief shows the startup screen with logo
 * @param void
 * @retval void
 */
void displayStartUpScreen(void);

/**
 * @brief initializes SSD1306 display
 * @param void
 * @retval void
 */
void displayInit(void);

/**
 * @brief displays input voltage
 * @param Vin input voltage in mV
 * @retval void
 */
void displayVin(double Vin);

/**
 * @brief displays set voltage
 * @param void
 * @retval void
 */
void displaySetVoltage(Stats *psuStats);

/**
 * @brief displays input voltage
 * @param Vout output voltage in mV
 * @retval void
 */
void displayVout(double Vout);

/**
 * @brief displays set current
 * @param void
 * @retval void
 */
void displaySetCurrent(Stats *psuStats);

/**
 * @brief displays input voltage
 * @param Iout output current in mA
 * @retval void
 */
void displayIout(double Iout);

/**
 * @brief displays whether reg output is enabled
 * @param void
 * @retval void
 */
void displayOnOffStatus(Stats *psuStats);

/**
 * @brief displays all V and I stats
 * @param Vin input voltage in mV
 * @param Vout output voltage in mV
 * @param Iout output current in mA
 * @retval void
 */
void displayVoltageCurrent(Stats *psuStats, double Vin, double Vout,
		double Iout);

/**
 * @brief displays error when a fatal condition is met
 * @param void
 * @retval void
 */
void displayFatalError(void);

uint32_t displayVSetCalc(uint8_t val);

#endif /* INC_DISPLAY_H_ */
