/*
 * analog.c
 *
 *  Created on: Mar 16, 2021
 *      Author: adityasehgal
 */

#include "analog.h"


double analogReadVin(uint16_t Vin) {
//	uint32_t retVal = 0;

	double tempV = ((double) Vin / ANALOG_MAX_ADC_VAL) * ANALOG_SYSTEM_VOLTAGE;
	tempV = (double) ((tempV * (ANALOG_VIN_R_TOP + ANALOG_VIN_R_BOT)) / (ANALOG_VIN_R_BOT))
			* 1000.00;
	return tempV;

}

double analogReadVout(uint16_t Vout) {
//	uint32_t retVal = 0;

	double tempV = ((double) Vout / ANALOG_MAX_ADC_VAL) * ANALOG_SYSTEM_VOLTAGE;
	tempV = (double) ((tempV * (ANALOG_VOUT_R_TOP + ANALOG_VOUT_R_BOT)) / (ANALOG_VOUT_R_BOT))
			* 1000.00;
	return tempV;
}

double analogReadIOut(uint16_t Iout) {
//	uint32_t retVal = 0;

	if (Iout <= 30)
		Iout = 0;
	double tempI = ((double) Iout / ANALOG_MAX_ADC_VAL) * ANALOG_SYSTEM_VOLTAGE;
	tempI = (tempI * 1000) / (ANALOG_I_SENSE_GAIN * (ANALOG_SENSE_R / 1000.00));
	if ((tempI <= ANALOG_IDLE_I_HIGH) && (tempI >= ANALOG_IDLE_I_LOW))
		return 0;
	return tempI - 2.500;
}
