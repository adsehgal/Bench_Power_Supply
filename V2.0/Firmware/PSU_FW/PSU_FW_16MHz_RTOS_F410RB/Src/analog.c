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

#define ANA_IOUT_PIN	LL_GPIO_PIN_3
#define ANA_IOUT_PORT	GPIOC
#define ANA_IOUT_CH		LL_ADC_CHANNEL_13

#define ANA_VOUT_PIN	LL_GPIO_PIN_0
#define ANA_VOUT_PORT	GPIOA
#define ANA_VOUT_CH		LL_ADC_CHANNEL_0

#define ANA_VIN_PIN		LL_GPIO_PIN_1
#define ANA_VIN_PORT	GPIOA
#define ANA_VIN_CH		LL_ADC_CHANNEL_1

#define ADC_BASE_	ADC1
#define DMA_BASE	DMA2
#define DMA_STREAM	LL_DMA_STREAM_0
#define DMA_CH		LL_DMA_CHANNEL_0

typedef struct analog_s {
	uint32_t pin;
	GPIO_TypeDef *port;
	char name[15];
	uint32_t rank;
	uint32_t channel;
	uint32_t raw;	// raw adc reading
	uint32_t volt;	// adc reading converted to voltage (mV) in the 3v3 scale
	uint32_t value;	// actual value WRT to the unit of the measurement
	char units[3];
} analog_t;

// Initialization must follow the order of anaIdx_t
static analog_t analog[ANA_COUNT] = {
// Analog pins and their data

		// IOUT
		{ .pin = ANA_IOUT_PIN, 				//
				.port = ANA_IOUT_PORT, 		//
				.name = "I OUT", 			//
				.rank = LL_ADC_REG_RANK_1, 	//
				.channel = ANA_IOUT_CH, 	//
				.units = "mA" }, 			//

		// VOUT
		{ .pin = ANA_VOUT_PIN, 				//
				.port = ANA_VOUT_PORT, 		//
				.name = "V OUT", 			//
				.rank = LL_ADC_REG_RANK_2, 	//
				.channel = ANA_VOUT_CH, 	//
				.units = "mV" }, 			//

		// VOUT
		{ .pin = ANA_VIN_PIN, 				//
				.port = ANA_VIN_PORT, 		//
				.name = "V IN", 			//
				.rank = LL_ADC_REG_RANK_3, 	//
				.channel = ANA_VIN_CH, 	//
				.units = "mV" }, 			//

		};

void DMA2_Stream0_IRQHandler(void) {
}

void analog_init(void) {

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	LL_GPIO_InitTypeDef gpio = { 0 };
	gpio.Mode = LL_GPIO_MODE_ANALOG;
	gpio.Pull = LL_GPIO_PULL_NO;
//	LL_GPIO_Init(GPIOx, GPIO_InitStruct);

	NVIC_SetPriority(DMA2_Stream0_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}
