/*
 * stats.h
 *
 *  Created on: Mar 7, 2021
 *      Author: adityasehgal
 */

#ifndef INC_STATS_H_
#define INC_STATS_H_

//general bool type defines
#define STATUS_SET 1
#define STATUS_RESET 0

//I2C device scan error codes
#define OLED_FOUND 0x01
#define POT_FOUND 0x02

//interrupt flag defines
#define INT_FLAG_CLEAR 0x00
#define INT_FLAG_UART_RX 0x01
#define INT_FLAG_UART_TX 0x02
#define INT_FLAG_BTN 0x04


typedef struct StatsStruct {
	uint8_t vSet;  //voltage set in a byte for MCP4018
	uint32_t iSet; //current set in mA
	uint8_t iLim;  //bit[0] ? limit reached : limit NOT reached
	uint8_t OE;    //bit[0] ? enabled : disabled
	uint8_t VI;    //bit[0] ? voltage : current
}Stats;

enum enables {
	OE_ENABLED = STATUS_SET,
	OE_DISABLED = STATUS_RESET,
	VI_V_SEL = STATUS_SET,
	VI_I_SEL = STATUS_RESET,
	I_LIM_SET = STATUS_SET,
	I_LIM_NSET = STATUS_RESET,
};

enum defaults {
	V_DEFAULT = 0x3F,
	I_DEFAULT = 1000,
	I_LIM_DEFAULT = I_LIM_NSET,
	OE_DEFAULT = OE_DISABLED,
	VI_DEFAULT = VI_V_SEL,
};



#endif /* INC_STATS_H_ */
