//library includes
#include <Arduino.h>
#include <stdint.h>
#include "MCP4023.h"
#include "buttons.h"
#include "analog.h"


#define OUT_LED A2
#define CL_LED A3

#define nREG_EN 5

bool V_I_SEL = false; //false = I; true = V


void setup() {

  pinMode(OUT_LED, OUTPUT);
  pinMode(CL_LED, OUTPUT);
  pinMode(nREG_EN, OUTPUT);

  digitalWrite(nREG_EN, HIGH); //start the PSU with enable off
  initDiv();
  initBtns();
  initAnalog();
  
}

void loop() {
  uint8_t btnPress = whichBtn();
  double voltage;
  double current;
  if (btnPress && up){
    rDivIncrement();
  }
  if (btnPress && dw){
    rDivDecrement();
  }
  if (btnPress && oen){
    digitalWrite(nREG_EN, !digitalRead(nREG_EN)); //toggle output enable
  }
  if (btnPress && vi){
    V_I_SEL = !V_I_SEL; //toggle V/I select
  }
}