#include "analog.h"

void initAnalog(){
  pinMode(I_PIN, INPUT);
  pinMode(V_PIN, INPUT);
}

double readCurrent()
{
    int analogI = analogRead(I_PIN);
    if (analogI <= 30)
        analogI = 0;
    double V = ((double)analogI / 1023.00) * (5.00);
    double final_I = (V * 1000) / (SENSE_GAIN * (R_SENSE / 1000.00));
    if ((final_I <= IDLE_I_HIGH) && (final_I >= IDLE_I_LOW))
        return 0;
    return final_I - 2.500;
}

double readVoltage()
{
    int analogV = analogRead(V_PIN);
    if (analogV <= 23)
        analogV = 0;
    double step = ((double)analogV / 1023.00) * (5.00);
    return   (double)((step * (RESISTOR_TOP + RESISTOR_BOT)) / (RESISTOR_BOT)) * 1000.00;
}
