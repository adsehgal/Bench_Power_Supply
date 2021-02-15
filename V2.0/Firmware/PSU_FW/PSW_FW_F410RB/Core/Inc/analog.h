/*
 * analog.h
 *
 *  Created on: Feb 14, 2021
 *      Author: adityasehgal
 */

#ifndef INC_ANALOG_H_
#define INC_ANALOG_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;

double readVin(void);

double readVout(void);

double readIOut(void);


#endif /* INC_ANALOG_H_ */
