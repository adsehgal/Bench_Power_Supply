/*
 * oled.c
 *
 *  Created on: Mar 7, 2021
 *      Author: adityasehgal
 */

#include <cmsis_os.h>
#include <display.h>
#include <gfx.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>

void displayStartUpScreen(void) {
	ssd1306_DrawBitMap(0, 0, BOOTSCREEN, 128, 32, SSD1306_WHITE);
	osDelay(2000);
}

void displayInit(void) {
	ssd1306_Init();
	ssd1306_Fill(SSD1306_BLACK);
	displayStartUpScreen();
}

void displayVin(double Vin) {
	char buff[10] = { };

	//Display voltage in
	ssd1306_SetCursor(INFO_X, VIN_Y);
	ssd1306_WriteString("Vin = ", INFO_TEXT_SIZE, SSD1306_WHITE);
	if (Vin > 1000) {
		sprintf(buff, "%4.2f", Vin / 1000);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		ssd1306_WriteString("V", INFO_TEXT_SIZE, SSD1306_WHITE);
	} else {
		sprintf(buff, "%4.2f", Vin);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		ssd1306_WriteString("mV", INFO_TEXT_SIZE, SSD1306_WHITE);
	}
}

void displaySetVoltage(Stats *psuStats) {
	char buff[10] = { };
	//display set voltage
	ssd1306_SetCursor(INFO_X, VSET_Y);
	ssd1306_WriteString("Vset = ", INFO_TEXT_SIZE, SSD1306_WHITE);

	if (displayVSetCalc(psuStats->vSet) >= 1000) {
		sprintf(buff, "%4.2f",
				(double) displayVSetCalc(psuStats->vSet) / 1000.0);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		if (psuStats->VI == VI_V_SEL)
			ssd1306_WriteString("V <<", INFO_TEXT_SIZE, SSD1306_WHITE);
		else
			ssd1306_WriteString("V     ", INFO_TEXT_SIZE, SSD1306_WHITE);
	} else {
		sprintf(buff, "%4.2f", (double) displayVSetCalc(psuStats->vSet));
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		if (psuStats->VI == VI_V_SEL)
			ssd1306_WriteString("mV <<", INFO_TEXT_SIZE, SSD1306_WHITE);
		else
			ssd1306_WriteString("mV     ", INFO_TEXT_SIZE, SSD1306_WHITE);
	}
}

void displayVout(double Vout) {
	char buff[10] = { };
	//display output voltage
	ssd1306_SetCursor(INFO_X, VOUT_Y);
	ssd1306_WriteString("Vout = ", INFO_TEXT_SIZE, SSD1306_WHITE);
	if (Vout >= 1000) {
		sprintf(buff, "%4.2f", Vout / 1000.0);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		ssd1306_WriteString("V", INFO_TEXT_SIZE, SSD1306_WHITE);
	} else {
		sprintf(buff, "%4.2f", Vout);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		ssd1306_WriteString("mV", INFO_TEXT_SIZE, SSD1306_WHITE);
	}

}

void displaySetCurrent(Stats *psuStats) {
	char buff[10] = { };
	//display set current
	ssd1306_SetCursor(INFO_X, ISET_Y);
	ssd1306_WriteString("Iset = ", INFO_TEXT_SIZE, SSD1306_WHITE);
	if (psuStats->iSet >= 1000) {
		sprintf(buff, "%4.2f", (double) psuStats->iSet / 1000.0);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		if (psuStats->VI == VI_I_SEL)
			ssd1306_WriteString("A <<", INFO_TEXT_SIZE, SSD1306_WHITE);
		else
			ssd1306_WriteString("A     ", INFO_TEXT_SIZE, SSD1306_WHITE);
	} else {
		sprintf(buff, "%4.2f", (double) psuStats->iSet);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		if (psuStats->VI == VI_I_SEL)
			ssd1306_WriteString("mA <<", INFO_TEXT_SIZE, SSD1306_WHITE);
		else
			ssd1306_WriteString("mA     ", INFO_TEXT_SIZE, SSD1306_WHITE);
	}
}

void displayIout(double Iout) {
	char buff[10] = { };
	//display output current
	ssd1306_SetCursor(INFO_X, IOUT_Y);
	ssd1306_WriteString("Iout = ", INFO_TEXT_SIZE, SSD1306_WHITE);
	if (Iout >= 1000) {
		sprintf(buff, "%4.2f", Iout / 1000.0);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		ssd1306_WriteString("A", INFO_TEXT_SIZE, SSD1306_WHITE);
	} else {
		sprintf(buff, "%4.2f", Iout);
		ssd1306_WriteString(buff, INFO_TEXT_SIZE, SSD1306_WHITE);
		ssd1306_WriteString("mA", INFO_TEXT_SIZE, SSD1306_WHITE);
	}
}

void displayOnOffStatus(Stats *psuStats) {
	//display whether PSU output is enabled
	ssd1306_SetCursor(ON_OFF_X, VIN_Y);
	if (psuStats->OE == OE_ENABLED)
		ssd1306_WriteString("ON   ", INFO_TEXT_SIZE, SSD1306_WHITE);
	else
		ssd1306_WriteString("OFF", INFO_TEXT_SIZE, SSD1306_WHITE);
}

void displayVoltageCurrent(Stats *psuStats, double Vin, double Vout,
		double Iout) {
	//clear screen
	ssd1306_Fill(SSD1306_BLACK);

	displayVin(Vin);

	displayOnOffStatus(psuStats);

	displaySetVoltage(psuStats);

	displayVout(Vout);

	displaySetCurrent(psuStats);

	displayIout(Iout);

	ssd1306_UpdateScreen();

}

void displayFatalError(void) {
//	disableOutput();
	//clear screen
	ssd1306_Fill(SSD1306_BLACK);

	ssd1306_SetCursor(2, 0);
	ssd1306_WriteString("FATAL", ERROR_TEXT_SIZE, SSD1306_WHITE);
	ssd1306_SetCursor(2, 20);
	ssd1306_WriteString("ERROR!", ERROR_TEXT_SIZE, SSD1306_WHITE);

	//countdown 5 seconds
	ssd1306_SetCursor(2, 40);
	ssd1306_WriteString("Reinit(5)!", INFO_TEXT_SIZE, SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(1000);

	ssd1306_SetCursor(2, 40);
	ssd1306_WriteString("Reinit(4)!.", INFO_TEXT_SIZE, SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(1000);

	ssd1306_SetCursor(2, 40);
	ssd1306_WriteString("Reinit(3)!..", INFO_TEXT_SIZE, SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(1000);

	ssd1306_SetCursor(2, 40);
	ssd1306_WriteString("Reinit(2)!...", INFO_TEXT_SIZE, SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(1000);

	ssd1306_SetCursor(2, 40);
	ssd1306_WriteString("Reinit(1)!....", INFO_TEXT_SIZE, SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(1000);

	ssd1306_SetCursor(2, 40);
	ssd1306_WriteString("Reinit(0)!.....", INFO_TEXT_SIZE, SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(1000);

	//flash screen
	ssd1306_Fill(SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(50);
	ssd1306_Fill(SSD1306_BLACK);
	ssd1306_UpdateScreen();
	osDelay(50);
	ssd1306_Fill(SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(50);
	ssd1306_Fill(SSD1306_BLACK);
	ssd1306_UpdateScreen();
	osDelay(50);
	ssd1306_Fill(SSD1306_WHITE);
	ssd1306_UpdateScreen();
	osDelay(50);

}

uint32_t displayVSetCalc(uint8_t val) {
	double potCalc = (((double) val / 128.0)) * 5000.0;
	double temp = 800.0 * ((4690.0 / potCalc) + 1.0);
	return (uint32_t) (temp);
}
