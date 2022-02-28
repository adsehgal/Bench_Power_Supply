/*
 * power.c
 *
 *  Created on: 26 Nov 2021
 *      Author: adityasehgal
 */

#include "power.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "i2c.h"

#define MCP4018_I2C_ADDR (0x2F << 1)
#define MCP4018_MAX_VAL 0xFF
#define MCP4018_MIN_VAL 0x00

typedef enum BUTTONS {
	BTN_NONE = 0x00,	//
	BTN_UP = 0x01, 		//
	BTN_DW = 0x02, 		//
	BTN_VI = 0x04, 		//
	BTN_OE = 0x08,		//
	BTN_ALL = 0x0F,		//
} buttons_t;

osThreadId powerTask_Handle;

static osMutexId statsMutexId;
static osMutexDef(statsMutex);

osSemaphoreDef(buttonsSem);
osSemaphoreId (buttonsSemId);

stats_t stats;

static uint8_t btnIntFlag = 0;

//static uint8_t mcp4018ReadVal(void);

static void mcp4018WriteVal(uint8_t val);

static buttons_t scanButtons(void);

static void processButtons(buttons_t buttons);

static void powerTask(void const *argument);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == nSW_INT_Pin) {
		btnIntFlag = 1;
	}
}

//uint8_t mcp4018ReadVal(void) {
//	uint8_t retVal = 0;
//	I2C_safeRead(&hi2c1, MCP4018_I2C_ADDR, &retVal, 1);
//	return retVal;
//}

void mcp4018WriteVal(uint8_t val) {
	I2C_safeWrite(&hi2c1, MCP4018_I2C_ADDR, &val, 1);
}

buttons_t scanButtons(void) {
	buttons_t ret = 0;

	ret |= HAL_GPIO_ReadPin(nSW_UP_GPIO_Port, nSW_UP_Pin) ? BTN_NONE : BTN_UP;
	ret |= HAL_GPIO_ReadPin(nSW_DW_GPIO_Port, nSW_DW_Pin) ? BTN_NONE : BTN_DW;
	ret |= HAL_GPIO_ReadPin(nSW_VI_GPIO_Port, nSW_VI_Pin) ? BTN_NONE : BTN_VI;
	ret |= HAL_GPIO_ReadPin(nSW_OE_GPIO_Port, nSW_OE_Pin) ? BTN_NONE : BTN_OE;

	return ret;
}

void processButtons(buttons_t buttons) {

	if (buttons == BTN_NONE) {
		return;
	}
	if (buttons & BTN_UP) {
		if (stats.selVI == VI_V_SEL) {
			if (stats.vSet < MCP4018_MAX_VAL) {
				stats.vSet++;
			}
		} else {
			stats.iSet += 50;
		}
	}
	if (buttons & BTN_DW) {
		if (stats.selVI == VI_I_SEL) {
			if (stats.vSet > MCP4018_MIN_VAL) {
				stats.vSet--;
			}
		} else {
			stats.iSet -= 50;
		}
	}

	if (buttons & BTN_VI) {

		if (stats.selVI == VI_V_SEL) {
			stats.selVI = VI_I_SEL;
		} else {
			stats.selVI = VI_V_SEL;
		}
	}

	if (buttons & BTN_OE) {

		if (stats.outputEn == OE_ENABLED) {
			stats.outputEn = OE_DISABLED;
		} else {
			stats.outputEn = OE_ENABLED;
		}
	}
}

void powerTask(void const *argument) {

	osMutexWait(statsMutexId, osWaitForever);
	stats.vSet = V_DEFAULT;
	stats.iLim = I_LIM_DEFAULT;
	stats.outputEn = OE_DEFAULT;
	stats.selVI = VI_DEFAULT;
	osMutexRelease(statsMutexId);
	for (;;) {
// wait for any other resources using the struct
		osMutexWait(statsMutexId, osWaitForever);

		if (btnIntFlag) {
			btnIntFlag = 0;
			uint8_t buttons = scanButtons();
			processButtons(buttons);
		}

		mcp4018WriteVal(stats.vSet);

		osMutexRelease(statsMutexId);
	}
}

void power_init(void) {
	buttonsSemId = osSemaphoreCreate(osSemaphore(buttonsSem), 1);
	statsMutexId = osMutexCreate(osMutex(statsMutex));

	osThreadDef(powerTask_, powerTask, osPriorityNormal, 0, 512);
	powerTask_Handle = osThreadCreate(osThread(powerTask_), NULL);
}

void power_updateStats(stats_t *data) {
	osMutexWait(statsMutexId, osWaitForever);
	data->vSet = stats.vSet;
	data->iSet = stats.iSet;
	data->iLim = stats.iLim;
	data->outputEn = stats.outputEn;
	data->selVI = stats.selVI;
	osMutexRelease(statsMutexId);
}
