#include <Arduino.h>
#include "MCP4023.h"


void initDiv(){
    pinMode(POT_CS, OUTPUT);
    pinMode(POT_UD, OUTPUT);
    digitalWrite(POT_CS, HIGH);
    digitalWrite(POT_UD, HIGH);

}
void rDivIncrement(){
    delayMicroseconds(1);
    digitalWrite(POT_UD, HIGH);
    delayMicroseconds(1);
    digitalWrite(POT_CS, LOW);
    delayMicroseconds(1);
    digitalWrite(POT_UD, LOW);
    delayMicroseconds(1);
    digitalWrite(POT_UD, HIGH);
    delayMicroseconds(1);
    digitalWrite(POT_CS, HIGH);
    delayMicroseconds(1);
}
void rDivDecrement(){
    delayMicroseconds(1);
    digitalWrite(POT_UD, LOW);
    delayMicroseconds(1);
    digitalWrite(POT_CS, LOW);
    delayMicroseconds(1);
    digitalWrite(POT_UD, HIGH);
    delayMicroseconds(1);
    digitalWrite(POT_UD, LOW);
    delayMicroseconds(1);
    digitalWrite(POT_CS, HIGH);
    delayMicroseconds(1);
}