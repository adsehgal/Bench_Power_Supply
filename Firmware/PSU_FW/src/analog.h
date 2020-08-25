#include <Arduino.h>
#include <stdint.h>

#define I_PIN A0
#define V_PIN A1

const uint16_t SENSE_GAIN = 500;
const uint8_t R_SENSE = 10;//mOhms
const uint8_t IDLE_I_HIGH = 100;    //mA
const uint8_t IDLE_I_LOW = 100;    //mA
const uint8_t RESISTOR_TOP = 14;    //KOhm
const uint8_t RESISTOR_BOT = 10;    //KOhm

void initAnalog();
double readCurrent();
double readVoltage();
