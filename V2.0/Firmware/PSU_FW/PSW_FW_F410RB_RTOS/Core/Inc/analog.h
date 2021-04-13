/*
 * analog.h
 *
 *  Created on: Mar 10, 2021
 *      Author: adityasehgal
 */

#ifndef INC_ANALOG_H_
#define INC_ANALOG_H_

#include <sys/_stdint.h>


#define ANALOG_DMA_BUFF_SIZE 3

//analogBuff*[0] = Isense
//analogBuff*[1] = VoutSense
//analogBuff*[2] = VinSense
#define ANALOG_BUFF_I_SENSE_POS 0
#define ANALOG_BUFF_V_OUT_POS 1
#define ANALOG_BUFF_V_IN_POS 2
uint16_t analogBuffDMA[ANALOG_DMA_BUFF_SIZE];
uint16_t analogBuffFinal[ANALOG_DMA_BUFF_SIZE];

#define ANALOG_MAX_ADC_VAL 4095.0f	//12bits
#define ANALOG_SYSTEM_VOLTAGE 3.3f	//system voltage
#define ANALOG_I_SENSE_GAIN 100	//gain of diff amp for I
#define ANALOG_SENSE_R 10		//mOhms
#define ANALOG_IDLE_I_HIGH 100    //mA
#define ANALOG_IDLE_I_LOW 100    //mA
#define ANALOG_VIN_R_TOP 29210    //Ohm R19
#define ANALOG_VIN_R_BOT 10000    //Ohm R20
#define ANALOG_VOUT_R_TOP 14010    //Ohm R22
#define ANALOG_VOUT_R_BOT 4720    //Ohm R25

double analogReadVin(uint16_t Vin);

double analogReadVout(uint16_t Vout);

double analogReadIOut(uint16_t Iout);




#endif /* INC_ANALOG_H_ */
