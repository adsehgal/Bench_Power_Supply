/*
 * analog.c
 *
 *  Created on: 28 Feb 2022
 *      Author: adityasehgal
 */

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_dma.h"

#include "analog.h"

#define ADC_BASE_	ADC1
#define DMA_BASE_	DMA2
#define DMA_STREAM	LL_DMA_STREAM_0
#define DMA_CH		LL_DMA_CHANNEL_0

#define IOUT_PIN	LL_GPIO_PIN_3
#define IOUT_PORT	GPIOC
#define IOUT_RANK	LL_ADC_REG_RANK_1
#define IOUT_CH		LL_ADC_CHANNEL_13

#define VOUT_PIN	LL_GPIO_PIN_0
#define VOUT_PORT	GPIOA
#define VOUT_RANK	LL_ADC_REG_RANK_2
#define VOUT_CH		LL_ADC_CHANNEL_0

#define VIN_PIN		LL_GPIO_PIN_1
#define VIN_PORT	GPIOA
#define VIN_RANK	LL_ADC_REG_RANK_3
#define VIN_CH		LL_ADC_CHANNEL_1

#define TEMP_RANK	LL_ADC_REG_RANK_4
#define TEMP_CH		LL_ADC_CHANNEL_TEMPSENSOR

#define VREF_RANK	LL_ADC_REG_RANK_5
#define VREF_CH		LL_ADC_CHANNEL_VREFINT

typedef struct adc_s {
	uint32_t pin;			// Physical pin the ADC reading is being taken at
	GPIO_TypeDef *port;		// Port the ADC reading is being taken at
	const char name[15];	// Human readable name of the ADC reading
	uint32_t rank;			// ADC rank of the channel
	uint32_t channel;		// ADC channel being read
	uint32_t raw;			// raw adc reading
	uint32_t volt;	// adc reading converted to voltage (mV) in the 3v3 scale
	const char units[3];	// Human readable unit for the reading
} adc_t;

// Initialization must follow the order of anaIdx_t
static adc_t adc[ANA_COUNT] = {
// Analog pins and their data

		// IOUT
		{ .pin = IOUT_PIN,				//
				.port = IOUT_PORT,		//
				.name = "I OUT",		//
				.rank = IOUT_RANK,		//
				.channel = IOUT_CH,		//
				.units = "mA" },		//

		// VOUT
		{ .pin = VOUT_PIN,				//
				.port = VOUT_PORT,		//
				.name = "V OUT",		//
				.rank = VOUT_RANK,		//
				.channel = VOUT_CH,		//
				.units = "mV" },		//

		// VOUT
		{ .pin = VIN_PIN,				//
				.port = VIN_PORT,		//
				.name = "V IN",			//
				.rank = VIN_RANK,		//
				.channel = VIN_CH,		//
				.units = "mV" },		//

		// TEMP SENSOR
		{ .pin = 0,						//
				.port = NULL,			//
				.name = "TEMPERATURE",	//
				.rank = TEMP_RANK,		//
				.channel = TEMP_CH,		//
				.units = "C" },		//

		// VREF
		{ .pin = 0,						//
				.port = NULL,			//
				.name = "VREF",			//
				.rank = VREF_RANK,		//
				.channel = VREF_CH,		//
				.units = "mV" },		//

		};

static uint16_t rawVals[ANA_COUNT] = { 0 };

void DMA2_Stream0_IRQHandler(void) {
	if (LL_DMA_IsActiveFlag_TC0(DMA_BASE_)) {
		LL_DMA_DisableIT_TC(DMA_BASE_, DMA_STREAM);

		uint32_t vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(rawVals[ANA_VREF],
				LL_ADC_RESOLUTION_12B);

		for (anaIdx_t i = 0; i < ANA_COUNT; i++) {

			adc[i].raw = rawVals[i];
			adc[i].volt = __LL_ADC_CALC_DATA_TO_VOLTAGE(vref, rawVals[i],
					LL_ADC_RESOLUTION_12B);
			if (i == ANA_TEMP) {
				adc[i].volt = __LL_ADC_CALC_TEMPERATURE(vref, adc[i].raw,
						LL_ADC_RESOLUTION_12B);
			}
		}

		LL_DMA_ClearFlag_TC0(DMA_BASE_);
		LL_DMA_EnableIT_TC(DMA_BASE_, DMA_STREAM);
	}
}

uint16_t analog_getRaw(anaIdx_t idx) {
	if (idx >= ANA_COUNT) {
		return 0;
	}

	return adc[idx].raw;
}

uint16_t analog_getVolt(anaIdx_t idx) {
	if (idx >= ANA_COUNT) {
		return 0;
	}

	return adc[idx].volt;

}

const char* analog_getName(anaIdx_t idx) {
	if (idx >= ANA_COUNT) {
		return "";
	}

	return adc[idx].name;
}

const char* analog_getUnits(anaIdx_t idx) {
	if (idx >= ANA_COUNT) {
		return "";
	}

	return adc[idx].units;
}

void analog_init(void) {

	LL_GPIO_InitTypeDef gpio = { 0 };
	LL_DMA_InitTypeDef dma = { 0 };
	LL_ADC_InitTypeDef adcInit = { 0 };
	LL_ADC_REG_InitTypeDef adcReg = { 0 };
	LL_ADC_CommonInitTypeDef adcComm = { 0 };

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	NVIC_SetPriority(DMA2_Stream0_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	// Init the analog pins
	gpio.Mode = LL_GPIO_MODE_ANALOG;
	gpio.Pull = LL_GPIO_PULL_NO;
	for (anaIdx_t i = 0; i < ANA_COUNT; i++) {
		gpio.Pin = adc[i].pin;
		LL_GPIO_Init(adc[i].port, &gpio);
	}

	// Init the DMA
	LL_DMA_DisableStream(DMA_BASE_, DMA_STREAM);
	dma.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	dma.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
	dma.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_FULL;
	dma.MemBurst = LL_DMA_MBURST_SINGLE;
	dma.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
	dma.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
	dma.Mode = LL_DMA_MODE_CIRCULAR;
	dma.NbData = ANA_COUNT;
	dma.PeriphBurst = LL_DMA_PBURST_SINGLE;
	dma.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
	dma.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	dma.Priority = LL_DMA_PRIORITY_HIGH;

	dma.Channel = DMA_CH;
	dma.PeriphOrM2MSrcAddress = LL_ADC_DMA_GetRegAddr(ADC_BASE_,
	LL_ADC_DMA_REG_REGULAR_DATA);
	dma.MemoryOrM2MDstAddress = (uint32_t) &rawVals;
	LL_DMA_Init(DMA_BASE_, DMA_STREAM, &dma);

	// Init the common ADC params
	adcComm.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC_BASE_), &adcComm);

	// Init the ADC
	adcInit.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	adcInit.Resolution = LL_ADC_RESOLUTION_12B;
	adcInit.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
	LL_ADC_Init(ADC_BASE_, &adcInit);

	// Init the ADC regular params
	adcReg.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
	adcReg.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
	adcReg.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	adcReg.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS;
	adcReg.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	LL_ADC_REG_Init(ADC_BASE_, &adcReg);
	LL_ADC_REG_SetFlagEndOfConversion(ADC_BASE_,
	LL_ADC_REG_FLAG_EOC_SEQUENCE_CONV);

	// Setup the individual channels
	for (anaIdx_t i = 0; i < ANA_COUNT; i++) {
		LL_ADC_REG_SetSequencerRanks(ADC_BASE_, adc[i].rank, adc[i].channel);
		LL_ADC_SetChannelSamplingTime(ADC_BASE_, adc[i].channel,
		LL_ADC_SAMPLINGTIME_480CYCLES);
	}

	// Set the internal channels
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC_BASE_),
	LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_PATH_INTERNAL_VREFINT);

	LL_ADC_Enable(ADC_BASE_);
	LL_DMA_EnableIT_TC(DMA_BASE_, DMA_STREAM);
	LL_DMA_EnableStream(DMA_BASE_, DMA_STREAM);
	LL_ADC_REG_StartConversionSWStart(ADC_BASE_);

}
