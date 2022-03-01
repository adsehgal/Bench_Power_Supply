/*
 * pwr.h
 *
 *  Created on: Feb 28, 2022
 *      Author: adityasehgal
 */

#ifndef PWR_H_
#define PWR_H_

#include "analog.h"

typedef anaIdx_t pwrIdx_t;

/**
 * @fn uint16_t pwr_getVal(pwrIdx_t)
 * @brief 		Get the scaled value of the analog reading
 * @param idx	Index of the channel being queried
 * @return		16 bit value in the respective units
 */
uint16_t pwr_getVal(pwrIdx_t idx);

/**
 * @fn const char pwr_getName*(pwrIdx_t)
 * @brief 		Get the human readable name of the analog reading
 * @param idx	Index of the channel being queried
 * @return		Pointer to string containing the name of the channel
 */
const char* pwr_getName(pwrIdx_t idx);

/**
 * @fn const char pwr_getUnits(pwrIdx_t)
 * @brief 		Get the human readable units for the analog reading
 * @param idx	Index of the channel being queried
 * @return		Pointer to string containing the units of the channel
 */
const char* pwr_getUnits(pwrIdx_t idx);

/**
 * @fn void pwr_init(void)
 * @brief Initialize the power thread and periodically publish data to UART
 */
void pwr_init(void);

#endif /* PWR_H_ */
