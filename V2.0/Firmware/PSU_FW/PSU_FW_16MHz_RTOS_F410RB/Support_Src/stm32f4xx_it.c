#include "main.h"
#include "stm32f4xx_it.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	while (1) {
	}
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	while (1) {
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	while (1) {
	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	while (1) {
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	while (1) {
	}
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 */


/**
 * @brief This function handles DMA2 stream2 global interrupt.
 */
void DMA2_Stream2_IRQHandler(void) {
}

/**
 * @brief This function handles DMA2 stream3 global interrupt.
 */
void DMA2_Stream3_IRQHandler(void) {
}
