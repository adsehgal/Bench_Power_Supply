/*
 * analog.c
 *
 *  Created on: Nov 25, 2021
 *      Author: adityasehgal
 */

#include <string.h>
#include "stdio.h"

#include "cmsis_os.h"
#include "adc.h"
#include "analog.h"
#include "stats.h"

#define MAX_ADC_VAL 4095.0f		//12bits
#define SYSTEM_VOLTAGE 3.3f		//system voltage
#define I_SENSE_GAIN 100		//gain of diff amp for I
#define I_SENSE_R 10			//mOhms
#define IDLE_I_HIGH 100    		//mA
#define IDLE_I_LOW 100    		//mA
#define VIN_R_TOP 29210    		//Ohm R19
#define VIN_R_BOT 10000    		//Ohm R20
#define VOUT_R_TOP 14010    	//Ohm R22
#define VOUT_R_BOT 4720    		//Ohm R25
#define TYPICAL_V25	760			//used for calc temperature - mV
#define TYPICAL_AVG_SLOPE 2.5	//used for calc temperature - mV/C
#define TYPICAL_VREF 1210		//internal reference = 1210mV

#define BUFF_SIZE 5
/**
 *
 * buff*[0] = Isense
 * buff*[1] = VoutSense
 * buff*[2] = VinSense
 * buff*[3] = vRefInt
 * buff*[4] = tempSense
 */
#define I_SENSE_POS 0
#define V_OUT_POS 	1
#define V_IN_POS 	2
#define V_REF_POS 	3
#define TEMP_POS 	4

uint16_t buffDMA[BUFF_SIZE];
uint16_t buffFinal[BUFF_SIZE];

static osThreadId analogTask_Handle;

static osMutexId statsMutexId;
static osMutexDef(statsMutex);

osSemaphoreDef(adcSem);
osSemaphoreId (adcSemId);

stats_t stats;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	memcpy(buffFinal, buffDMA, sizeof(buffDMA));

	osSemaphoreRelease(adcSemId); // signal thread that conversion is complete

}

/**
 * brief: returns voltage at the ADC pin WRT internal reference
 * @param: pos - index of voltage to get
 * @return: double - voltage calculated - mV
 */
static double getAdcVoltage(uint8_t pos) {
	return ((buffFinal[pos] * 1210) / buffFinal[V_REF_POS]);
}

static void calcVin(void) {
	double pinV = getAdcVoltage(V_IN_POS);
	stats.Vin = pinV * ((VIN_R_TOP + VIN_R_BOT) / VIN_R_TOP);
}

static void calcVout(void) {
	double pinV = getAdcVoltage(V_OUT_POS);
	stats.Vout = pinV * ((VOUT_R_TOP + VOUT_R_BOT) / VOUT_R_TOP);
}

static void calcIout(void) {
	double pinV = getAdcVoltage(I_SENSE_POS);

	stats.Iout = pinV / (I_SENSE_GAIN * (I_SENSE_R / 1000.00));

}

static void calcTemp(void) {
	double pinV = getAdcVoltage(TEMP_POS);
	//equation from reference manual - page 114
	stats.temp = ((pinV - TYPICAL_V25) / TYPICAL_AVG_SLOPE) + 25;
}

static void calcStats(void) {
	calcVin();
	calcVout();
	calcIout();
	calcTemp();

	printf("In: %f\n", stats.Vin);
	printf("Out: %f\n", stats.Vout);
}

static void analogTask(void const *argument) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) buffDMA, BUFF_SIZE);

	for (;;) {
		osSemaphoreWait(adcSemId, osWaitForever);
		osMutexWait(statsMutexId, osWaitForever);
		calcStats();	//update local stats packet
		osMutexRelease(statsMutexId);
	}
	osThreadTerminate(osThreadGetId());
}

void analog_updateStats(stats_t *data) {
	osMutexWait(statsMutexId, osWaitForever);
	data->Vin = stats.Vin;
	data->Vout = stats.Vout;
	data->Iout = stats.Iout;
	data->temp = stats.temp;
	osMutexRelease(statsMutexId);
}

void analog_init(void) {
	adcSemId = osSemaphoreCreate(osSemaphore(adcSem), 1);
	statsMutexId = osMutexCreate(osMutex(statsMutex));

	osThreadDef(analogTask_, analogTask, osPriorityNormal, 0, 512);
	analogTask_Handle = osThreadCreate(osThread(analogTask_), NULL);
}

