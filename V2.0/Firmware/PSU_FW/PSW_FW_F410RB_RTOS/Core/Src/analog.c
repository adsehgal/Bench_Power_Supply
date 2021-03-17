/*
 * analog.c
 *
 *  Created on: Mar 16, 2021
 *      Author: adityasehgal
 */

#include "analog.h"


double analogReadVin(uint16_t Vin) {
//	uint32_t retVal = 0;

	double tempV = ((double) Vin / MAX_ADC_VAL) * SYS_VOLTAGE;
	tempV = (double) ((tempV * (RESISTOR_TOP_VIN + RESISTOR_BOT_VIN)) / (RESISTOR_BOT_VIN))
			* 1000.00;
	return tempV;

}

double analogReadVout(uint16_t Vout) {
//	uint32_t retVal = 0;

	double tempV = ((double) Vout / MAX_ADC_VAL) * SYS_VOLTAGE;
	tempV = (double) ((tempV * (RESISTOR_TOP_VOUT + RESISTOR_BOT_VOUT)) / (RESISTOR_BOT_VOUT))
			* 1000.00;
	return tempV;
}

double analogReadIOut(uint16_t Iout) {
//	uint32_t retVal = 0;

	if (Iout <= 30)
		Iout = 0;
	double tempI = ((double) Iout / MAX_ADC_VAL) * SYS_VOLTAGE;
	tempI = (tempI * 1000) / (SENSE_GAIN * (R_SENSE / 1000.00));
	if ((tempI <= IDLE_I_HIGH) && (tempI >= IDLE_I_LOW))
		return 0;
	return tempI - 2.500;
}
