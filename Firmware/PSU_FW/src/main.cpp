//library includes
#include <Arduino.h>
#include <stdint.h>
#include "MCP4023.h"
#include "buttons.h"
#include "analog.h"
#include "graphics.h"
#include "Adafruit_GFX.h"
#include "Wire.h"
#include "Adafruit_SSD1306.h"

#define OLED_RESET 9
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);


#define OUT_LED A2
#define CL_LED A3

#define nREG_EN 5

#define OLED_ADD 0x3C //or 0x3D
const uint16_t OLED_W = 128;
const uint16_t OLED_H = 32;

bool V_I_SEL = false; //false = I; true = V

void displayLogo(){
  display.clearDisplay();
  display.drawBitmap(0, 0, BOOTSCREEN, OLED_W, OLED_H, WHITE);
  display.display();
}

void displayCLReached(){
  display.clearDisplay(); //for Clearing the display
  display.drawBitmap(0, 0, CL_REACHED_W, OLED_W, OLED_H, WHITE);
  display.display();
  delay(300);
  display.clearDisplay(); //for Clearing the display
  display.drawBitmap(0, 0, CL_REACHED_B, OLED_W, OLED_H, WHITE);
  display.display();
  delay(300);
}

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADD); //or 0x3C
  pinMode(OUT_LED, OUTPUT);
  pinMode(CL_LED, OUTPUT);
  pinMode(nREG_EN, OUTPUT);

  digitalWrite(nREG_EN, HIGH); //start the PSU with enable off
  initDiv();
  initBtns();
  initAnalog();
  displayLogo();
  delay(500);
}

void loop() {
  uint8_t btnPress = whichBtn();
  // double voltage;
  // double current;
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