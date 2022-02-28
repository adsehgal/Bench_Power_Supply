/*
 * display.h
 *
 *  Created on: Nov 25, 2021
 *      Author: adityasehgal
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_
#include <stdint.h>
#include "ssd1306.h"
#include "stats.h"

/**
 * @brief initializes SSD1306 display
 * @param void
 * @retval void
 */
void display_init(void);

/**
 * @brief write error message at specified coordinates
 * @param x: x coordinate - starting point
 * @param y: y coordinate - starting point
 * @param str: error string to write
 * @retval void
 */
void display_writeErrorMsg(int16_t x, int16_t y, char *str);

void display_showStats(stats_t stats);
#endif /* INC_DISPLAY_H_ */
