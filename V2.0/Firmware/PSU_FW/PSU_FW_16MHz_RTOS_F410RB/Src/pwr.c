/*
 * pwr.c
 *
 *  Created on: Feb 28, 2022
 *      Author: adityasehgal
 */

#include "os.h"
#include "config.h"

#include "pwr.h"

#define VIN_RTOP	29000	// Ohm
#define VIN_RBOT	10000	// Ohm

#define VOUT_RTOP	14000	// Ohm
#define VOUT_RBOT	4700	// Ohm

#define IOUT_RSNS	10		// mOhm
#define IOUT_GAIN	100		// V/V
#define IOUT_CALC(vshunt) \
	( \
			IOUT_GAIN * vshunt\
	)

typedef struct pwrData_s {
	sem_t sem;
	tim_t tim;
	float volt[ANA_COUNT];
	float val[ANA_COUNT];
} pwrData_t;

static pwrData_t pwr = { .tim = { .name = "PWR" }, };

static void callback(void *arg) {
	sem_signal(&pwr.sem);
}
#include <stdio.h>
static void updateVal(anaIdx_t idx, uint32_t volt) {
//	Calculating voltage from divider scaling:
//		voltage = adcVolt * ((TOP + BOT) / BOT);

// Calculating current from diff op-amp/shunt:
//		current = adcVolt[mV] / (gain[V/V] * (sense_r[ohm]));

	switch (idx) {
	case (ANA_IOUT):
		pwr.val[ANA_IOUT] =
				(float) (volt / (IOUT_GAIN * (IOUT_RSNS / 1000.0f)));
		break;
	case (ANA_VOUT):
		pwr.val[ANA_VOUT] =
				(float) (volt * (VOUT_RTOP + VOUT_RBOT)) / VOUT_RBOT;
		break;
	case (ANA_VIN):
		pwr.val[ANA_VIN] = (float) (volt * (VIN_RTOP + VIN_RBOT)) / VIN_RBOT;
		break;
	case (ANA_TEMP):
		pwr.val[ANA_TEMP] = (float) volt;
		break;
	case (ANA_VREF):
		pwr.val[ANA_VREF] = (float) volt;
		break;
	default:
		break;
	}

	printf("%s: %d%s\n", pwr_getName(idx), pwr_getVal(idx), pwr_getUnits(idx));
}

static void thread(void *arg) {

	sem_new(&pwr.sem, 0);
	timer_new(&pwr.tim, callback, osTimerPeriodic, NULL);
	timer_start(&pwr.tim, 50);

	for (;;) {
		sem_wait(&pwr.sem, osWaitForever);
		for (anaIdx_t i = 0; i < ANA_COUNT; i++) {
			pwr.volt[i] = analog_getVolt(i);
			//TODO: Add scaling here
			updateVal(i, pwr.volt[i]);
		}
		printf("\n");
	}
}

uint16_t pwr_getVal(pwrIdx_t idx) {
	if (idx >= ANA_COUNT) {
		return 0;
	}
	return pwr.val[idx];
}

const char* pwr_getName(pwrIdx_t idx) {
	if (idx >= ANA_COUNT) {
		return "";
	}
	return analog_getName(idx);

}

const char* pwr_getUnits(pwrIdx_t idx) {
	if (idx >= ANA_COUNT) {
		return "";
	}
	return analog_getUnits(idx);
}

void pwr_init(void) {
	thread_new("PWR", thread, NULL, DEFAULT_THREAD_STACKSIZE,
	DEFAULT_THREAD_PRIO);
}
