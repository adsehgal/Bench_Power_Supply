/*
 * analog.h
 *
 *  Created on: Mar 10, 2021
 *      Author: adityasehgal
 */

#ifndef INC_ANALOG_H_
#define INC_ANALOG_H_

//#include <stm32f4xx_hal_def.h>
#include <sys/_stdint.h>


#define ANALOG_DMA_BUFF_SIZE 3



//analogBuff*[0] = Isense
//analogBuff*[1] = VoutSense
//analogBuff*[2] = VinSense
uint16_t analogBuffDMA[ANALOG_DMA_BUFF_SIZE];
uint16_t analogBuffFinal[ANALOG_DMA_BUFF_SIZE];



#define MAX_ADC_VAL 4095.0f	//12bits
#define SYS_VOLTAGE 3.3f	//system voltage
#define SENSE_GAIN 500	//gain of diff amp for I
#define R_SENSE 10		//mOhms
#define IDLE_I_HIGH 100    //mA
#define IDLE_I_LOW 100    //mA
#define RESISTOR_TOP_VIN 29210    //Ohm R19
#define RESISTOR_BOT_VIN 10000    //Ohm R20
#define RESISTOR_TOP_VOUT 14010    //Ohm R22
#define RESISTOR_BOT_VOUT 4720    //Ohm R25

double analogReadVin(uint16_t Vin);

double analogReadVout(uint16_t Vout);

double analogReadIOut(uint16_t Iout);




#endif /* INC_ANALOG_H_ */
