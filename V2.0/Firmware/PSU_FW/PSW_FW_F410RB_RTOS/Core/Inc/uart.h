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

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define UART_RX_CHAR_SIZE 1
#define UART_RX_BUFF_SIZE 128
#define UART_DMA_BUFFER_SIZE 10
#define PARSER_MESSAGE_LIST_SIZE 8
#define PARSER_MESSAGE_SIZE 1024

void txString(char *format, ...);

#endif /* INC_UART_H_ */
