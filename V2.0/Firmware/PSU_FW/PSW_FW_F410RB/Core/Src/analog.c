/*
 * analog.c
 *
 *  Created on: Feb 14, 2021
 *      Author: adityasehgal
 */


#include <analog.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_def.h>
#include <sys/_stdint.h>

const double maxADCVal = 4095.0;	//12bits
const double sysVolt = 3.3;	//system voltage
const uint16_t SENSE_GAIN = 500;	//gain of diff amp for I
const uint8_t R_SENSE = 10;		//mOhms
const uint8_t IDLE_I_HIGH = 100;    //mA
const uint8_t IDLE_I_LOW = 100;    //mA
const uint16_t RESISTOR_TOP_VIN = 29210;    //Ohm R19
const uint16_t RESISTOR_BOT_VIN = 10000;    //Ohm R20
const uint16_t RESISTOR_TOP_VOUT = 14010;    //Ohm R22
const uint16_t RESISTOR_BOT_VOUT = 4720;    //Ohm R25


double readVin(void) {
	uint32_t retVal = 0;

	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		//error handle
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	retVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	double tempV = ((double) retVal / maxADCVal) * sysVolt;
	tempV = (double) ((tempV * (RESISTOR_TOP_VIN + RESISTOR_BOT_VIN)) / (RESISTOR_BOT_VIN))
			* 1000.00;
	return tempV;

}

double readVout(void) {
	uint32_t retVal = 0;

	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		//error handle
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	retVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	double tempV = ((double) retVal / maxADCVal) * sysVolt;
	tempV = (double) ((tempV * (RESISTOR_TOP_VOUT + RESISTOR_BOT_VOUT)) / (RESISTOR_BOT_VOUT))
			* 1000.00;
	return tempV;
}

double readIOut(void) {
	uint32_t retVal = 0;

	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		//error handle
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	retVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	if (retVal <= 30)
		retVal = 0;
	double tempI = ((double) retVal / maxADCVal) * sysVolt;
	tempI = (tempI * 1000) / (SENSE_GAIN * (R_SENSE / 1000.00));
	if ((tempI <= IDLE_I_HIGH) && (tempI >= IDLE_I_LOW))
		return 0;
	return tempI - 2.500;
}

