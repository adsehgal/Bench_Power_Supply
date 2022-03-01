/*
 * uart.c
 *
 *  Created on: 28 Feb 2022
 *      Author: adityasehgal
 */

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"

#include "os.h"

#include "uart.h"

#define UART_BASE USART2

#define UART_RX_PIN		LL_GPIO_PIN_3
#define UART_TX_PIN		LL_GPIO_PIN_2

#define UART_RX_TX_PORT	GPIOA

mut_t printMut;

void USART2_IRQHandler(void) {
	if (LL_USART_IsActiveFlag_RXNE(UART_BASE)) {
		LL_USART_ClearFlag_RXNE(UART_BASE);
		LL_USART_TransmitData8(UART_BASE, LL_USART_ReceiveData8(UART_BASE));
	}

	if (LL_USART_IsActiveFlag_ORE(UART_BASE)) {
		LL_USART_ClearFlag_ORE(UART_BASE);
		// force read all data if FIFO has overrun
		while (LL_USART_IsActiveFlag_RXNE(UART_BASE)) {
			LL_USART_ReceiveData8(UART_BASE);
		}
	}
}

int _write(int file, char *ptr, int len) {
	UNUSED(file);
	mutex_lock(&printMut);
	for (int i = 0; i < len; i++) {
		LL_USART_TransmitData8(UART_BASE, ptr[i]);
		while (!LL_USART_IsActiveFlag_TXE(UART_BASE))
			;
	}
	mutex_unlock(&printMut);
	return len;

}

void uart_init(void) {

	mutex_new(&printMut);

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	LL_GPIO_InitTypeDef gpio = { 0 };

	LL_USART_InitTypeDef uart = { 0 };

	gpio.Pin = UART_RX_PIN | UART_TX_PIN;
	gpio.Mode = LL_GPIO_MODE_ALTERNATE;
	gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	gpio.Pull = LL_GPIO_PULL_NO;
	gpio.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(UART_RX_TX_PORT, &gpio);

	NVIC_SetPriority(USART2_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(USART2_IRQn);

	uart.BaudRate = 115200;
	uart.DataWidth = LL_USART_DATAWIDTH_8B;
	uart.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	uart.OverSampling = LL_USART_OVERSAMPLING_16;
	uart.Parity = LL_USART_PARITY_NONE;
	uart.StopBits = LL_USART_STOPBITS_1;
	uart.TransferDirection = LL_USART_DIRECTION_TX_RX;
	LL_USART_Init(UART_BASE, &uart);
	LL_USART_ConfigAsyncMode(UART_BASE);
	LL_USART_Enable(UART_BASE);

	LL_USART_EnableIT_RXNE(UART_BASE);

}
