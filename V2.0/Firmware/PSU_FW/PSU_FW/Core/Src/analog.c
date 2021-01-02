/*
 * analog.c
 *
 *  Created on: Jan 2, 2021
 *      Author: adityasehgal
 */

#include "analog.h"

uint32_t readVin(){
	uint32_t retVal = 0;
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		//error handle
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	retVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return retVal;
}

uint32_t readVout(){
	uint32_t retVal = 0;
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		//error handle
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	retVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return retVal;
}

uint32_t readIOut(){
	uint32_t retVal = 0;
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		//error handle
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	retVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	return retVal;
}


