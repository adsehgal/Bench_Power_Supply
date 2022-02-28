/*
 * debug.h
 *
 *  Created on: Nov 25, 2021
 *      Author: adityasehgal
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#include <stdio.h>
#include <string.h>
#include "main.h"

#define USART_GUI 	0	// 1 == GUI used - disables trace
#define TRACE_LEVEL 1	// 0 == no trace, warnings, errors
#define WARN_LEVEL 	1	// 0 == no warnings, errors
#define ERROR_LEVEL 1	// 0 == no errors

/**
 *
 * DO NOT EDIT BELOW THIS LINE!
 *
 */

#define COMPILE_TIME __TIME__
#define COMPILE_DATE __DATE__

#if !USART_GUI && !TRACE_LEVEL
#endif

#if USART_GUI > 0
#undef TRACE_LEVEL
#define TRACE_LEVEL 0
#endif	//USART_GUI

#if TRACE_LEVEL == 0
#define DBG_TRACE(format, ...) do()while(0);
#elif TRACE_LEVEL == 1
#define DBG_TRACE(format, ...) printf("TRACE: %s:%d: " format, __FILE__, __LINE__, __VA_ARGS__);
#endif	//TRACE_LEVEL

#if TRACE_LEVEL == 0
#undef WARN_LEVEL
#define WARN_LEVEL 0

#undef ERROR_LEVEL
#define ERROR_LEVEL 0
#endif	//TRACE_LEVEL

#if WARN_LEVEL == 0
#define DBG_WARN(format, ...) do()while(0);
#elif WARN_LEVEL == 1
#define DBG_WARN(format, ...) printf("WARN: %s:%d: " format, __FILE__, __LINE__, __VA_ARGS__);
#endif	//WARN_LEVEL

#if ERROR_LEVEL == 0
#define DBG_ERROR(format, ...) do()while(0);
#elif ERROR_LEVEL == 1
#define DBG_ERROR(format, ...) printf("ERROR: %s:%d: " format, __FILE__, __LINE__, __VA_ARGS__);
#endif	//WARN_LEVEL

#endif /* INC_DEBUG_H_ */
