/*
 * config.c
 *
 *  Created on: Feb 27, 2022
 *      Author: adityasehgal
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32_assert.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "config.h"

#define TIM_BASE TIM6

///////////////////////////////////////////////////////////////////////////////
//////////////////// STATIC CONFIG FUNCTION DEFINITIONS ///////////////////////
///////////////////////////////////////////////////////////////////////////////
/**
 * @fn void TIM6_DAC_IRQHandler(void)
 * @brief	Handles the HAL tick increment on an interrupt basis
 */
void TIM6_DAC_IRQHandler(void) {
	if (LL_TIM_IsActiveFlag_UPDATE(TIM_BASE)) {
		if (LL_TIM_IsEnabledIT_UPDATE(TIM_BASE)) {
			LL_TIM_ClearFlag_UPDATE(TIM_BASE);
			HAL_IncTick();
		}
	}
}

/**
 * @fn void initTick(uint32_t)
 * @brief		Initialize TIM6 as the system base clock at 1MHz
 * @param prio	priority to set the HAL tick to
 */
static void initTick(uint32_t prio) {
	uint32_t timClk = 0;
	uint32_t prescale = 0;

	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, prio, 0);

	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

	LL_RCC_ClocksTypeDef rcc = { 0 };
	LL_RCC_GetSystemClocksFreq(&rcc);
	timClk = 2 * rcc.PCLK1_Frequency;

	prescale = (uint32_t) ((timClk / 1000000U) - 1U);

	/*
	 * Initialize TIM6 peripheral as follow:
	 + Period = [(TIM6CLK/1000) - 1]. to have a (1/1000) s time base.
	 + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
	 + ClockDivision = 0
	 + Counter direction = Up
	 */
	LL_TIM_InitTypeDef tim = { 0 };
	tim.Prescaler = prescale;
	tim.ClockDivision = 0;
	tim.CounterMode = TIM_COUNTERMODE_UP;
	tim.Autoreload = (1000000U / 1000U) - 1U;
	LL_TIM_Init(TIM_BASE, &tim);
	LL_TIM_EnableIT_UPDATE(TIM_BASE);
	LL_TIM_EnableCounter(TIM_BASE);

}

/**
 * @fn void configClock(void)
 * @brief	Configure the system flash, power and RCC settings for 100MHz
 */
static void configClock(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3)
		;

	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_HSI_Enable();
	while (LL_RCC_HSI_IsReady() != 1)
		;

	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_8, 100,
	LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_Enable();
	while (LL_RCC_PLL_IsReady() != 1)
		;

	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
		;

	LL_SetSystemCoreClock(100000000);

	initTick(TICK_INT_PRIORITY);

	LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

///////////////////////////////////////////////////////////////////////////////
//////////////////// PUBLIC CONFIG FUNCTION DEFINITIONS ///////////////////////
///////////////////////////////////////////////////////////////////////////////
void config_sysInit(void) {
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	initTick(TICK_INT_PRIORITY);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

	configClock();

}

///////////////////////////////////////////////////////////////////////////////
/////////////////////// FREERTOS RELATED ALLOCATIONS //////////////////////////
///////////////////////////////////////////////////////////////////////////////
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;

}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
		StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
		StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize) {
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
	*ppxTimerTaskStackBuffer = &xTimerStack[0];
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

///////////////////////////////////////////////////////////////////////////////
///////////////// MISC SYSTEM RELATED FUNCTION DEFINITIONS ////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifdef  USE_FULL_ASSERT
/**
  * @fn void assert_failed(uint8_t*, uint32_t)
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
