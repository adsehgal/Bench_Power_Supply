/*
 * power.h
 *
 *  Created on: 26 Nov 2021
 *      Author: adityasehgal
 */

#ifndef INC_POWER_H_
#define INC_POWER_H_

#include <stdint.h>

#include "main.h"

#include "stats.h"

void power_init(void);

void power_updateStats(stats_t *data);

#endif /* INC_POWER_H_ */
