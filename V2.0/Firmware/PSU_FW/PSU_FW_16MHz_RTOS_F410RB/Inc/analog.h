/*
 * analog.h
 *
 *  Created on: 28 Feb 2022
 *      Author: adityasehgal
 */

#ifndef ANALOG_H_
#define ANALOG_H_

typedef enum anaIdx_e {
	ANA_IOUT = 0,	//
	ANA_VOUT,		//
	ANA_VIN,		//
	ANA_COUNT,		//
} anaIdx_t;

void analog_init(void);

#endif /* ANALOG_H_ */
