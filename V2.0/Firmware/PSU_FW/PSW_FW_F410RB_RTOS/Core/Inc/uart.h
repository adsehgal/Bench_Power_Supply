/*
 * uart.h
 *
 *  Created on: Mar 7, 2021
 *      Author: adityasehgal
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "stats.h"

#define UART_RX_CHAR_SIZE 1
#define UART_RX_BUFF_SIZE 128
#define UART_TX_BUFF_SIZE 1024

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

typedef struct uartRxDataStruct {
	char uartRxBuff[UART_RX_BUFF_SIZE];
	char uartRxChar;
} uartRxData;

typedef enum uartRxMsgType {
	MSG_NO_CMD = 0x00,
	MSG_ERR_CMD = 0x02,
	MSG_V_SET = 0x04,
	MSG_I_SET = 0x08,
	MSG_OE_EN = 0x10,
	MSG_OE_NEN = 0x11,
	MSG_VI_V_SEL = 0x12,
	MSG_VI_I_SEL = 0x14,

} uartRxMsg;

/**
 * @brief printf alternative - outputs to com port
 * @param format - printf type string formattign
 * @retval void
 * @note uses HAL Transmit, need to change to DMA transmit
 */
void uartTxString(char *format, ...);

/**
 * @brief printf alternative - outputs to com port
 * @param format - printf type string formattign
 * @retval void
 * @note uses HAL Transmit, need to change to DMA transmit
 */
void uartTxStringHandler(char * str);

/**
 * @brief recieves UART chars and concatenates them into a string
 * @param void
 * @retval void
 * @note changes uartRxData struct and interruptFlags
 */
void uartRxIntHandler(uartRxData *uartRx);

/**
 * @brief parses user strings
 * @param void
 * @retval void
 */
void uartRxStringParser(uartRxData *uartRx, Stats *psuStats);

/**
 * @brief parses user strings
 * @param void
 * @retval void
 */
uartRxMsg uartRxStringDecoder(char *str, Stats *psuStats, uint32_t *valueToSet);

void uartRxConfigSet(Stats *psuStats, uartRxMsg msgType, uint32_t valueToSet);

#endif /* INC_UART_H_ */
