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
	ANA_TEMP,		//
	ANA_VREF,		//
	ANA_COUNT,		//
} anaIdx_t;

/**
 * @fn uint16_t analog_getRaw(anaIdx_t)
 * @brief 		Get the raw ADC reading of the channel
 * @param idx	Index of the channel being queried
 * @return		12 bit raw ADC value
 */
uint16_t analog_getRaw(anaIdx_t idx);

/**
 * @fn uint16_t analog_getVolt(anaIdx_t)
 * @brief 		Get the raw voltage of the ADC channel on the 3V3 scale
 * @param idx	Index of the channel being queried
 * @return		16 bit voltage in mV
 */
uint16_t analog_getVolt(anaIdx_t idx);

/**
 * @fn const char analog_getName*(anaIdx_t)
 * @brief 		Get the human readable name of the ADC channel
 * @param idx	Index of the channel being queried
 * @return		Pointer to string containing the name of the channel
 */
const char* analog_getName(anaIdx_t idx);

/**
 * @fn const char analog_getUnits*(anaIdx_t)
 * @brief 		Get the human readable units for the ADC channel
 * @param idx	Index of the channel being queried
 * @return		Pointer to string containing the units of the channel
 */
const char* analog_getUnits(anaIdx_t idx);

/**
 * @fn void analog_init(void)
 * @brief	Initialize the ADC, DMA and relative interrupts
 */
void analog_init(void);

#endif /* ANALOG_H_ */
