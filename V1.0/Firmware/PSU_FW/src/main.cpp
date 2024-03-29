//library includes
#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include "MCP4023.h"
#include "buttons.h"
#include "analog.h"
#include "graphics.h"
#include "Adafruit_GFX.h"
#include "Wire.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GrayOLED.h"

#define OUT_LED A2
#define CL_LED A3

#define nREG_EN 5

#define OLED_ADD 0x3D //or 0x3D
const uint16_t OLED_W = 128;
const uint16_t OLED_H = 64;
#define OLED_RESET 10
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

unsigned long lastSampledTime = 0;
unsigned int currentLimit = 0;

bool V_I_SEL = false; //false = I; true = V

void displayLogo();

void displayCLReached();

void displayPrintString(String text, uint16_t coordinateX, uint16_t coordinateY);

void displayVoltageCurrent(double Vin, double V, double I);

//returns true when requested time(ms) has passed for specified button
bool checkTime(unsigned long time, Btns btn);

void setup()
{
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADD); //or 0x3C

  Serial.begin(115200);

  pinMode(OUT_LED, OUTPUT);
  pinMode(CL_LED, OUTPUT);
  pinMode(nREG_EN, OUTPUT);

  digitalWrite(nREG_EN, LOW); //start the PSU with enable off
  digitalWrite(OUT_LED, LOW);
  digitalWrite(CL_LED, HIGH);
  initDiv();
  initBtns();
  initAnalog();
  displayLogo();
  delay(2000);
}

void loop()
{
  uint8_t btnPress = whichBtn();
  double voltageIn = readVoltageIn();
  double voltageOut = readVoltageOut();
  double current = readCurrent();
  displayVoltageCurrent(voltageIn, voltageOut, current);
  if (btnPress & VI)
  {
    V_I_SEL = !V_I_SEL; //toggle V/I select
  }

  if (btnPress & UP)
  {
    if (V_I_SEL)
    {
      rDivIncrement();
      Serial.println("up VI sel\n");
    }
    else
    {
      currentLimit += 10; //inc by 10mA
      Serial.println("up else\n");
    }
  }

  if (btnPress & DW)
  {
    if (V_I_SEL)
    {
      rDivDecrement();
      Serial.println("DW VI sel\n");
    }
    else
    {
      currentLimit -= 10; //dec by 10mA
      Serial.println("DW else\n");
    }
  }

  if (current >= currentLimit)
  {
    // Serial.println("VI cl\n");
    digitalWrite(CL_LED, LOW);
    // digitalWrite(nREG_EN, HIGH); //disable regulator
  }
  else
  {
    // Serial.println("VI else\n");
    digitalWrite(CL_LED, HIGH); //keep LED enabled
    // digitalWrite(nREG_EN, LOW); //keep regulator enabled
  }

  if (btnPress & OEN)
  {
    if (checkTime(2000, OEN)) //check for 2 seconds
    {
      Serial.println("OEN wiperlock\n");
      disableWiperLock();
      displayPrintString("Wiper Lock Disabled", 2, 12);
      delay(500);
    }
    else
    {
      Serial.println("oe else\n");
      digitalWrite(nREG_EN, !digitalRead(nREG_EN)); //toggle output enable
      digitalWrite(OUT_LED, !digitalRead(OUT_LED)); //toggle output enable LED
    }
  }
}

void displayLogo()
{
  display.clearDisplay();
  display.drawBitmap(0, 0, BOOTSCREEN, OLED_W, OLED_H, WHITE);
  display.display();
}

void displayCLReached()
{
  display.clearDisplay(); //for Clearing the display
  display.drawBitmap(0, 0, CL_REACHED_W, OLED_W, OLED_H, WHITE);
  display.display();
  delay(300);
  display.invertDisplay(true);
  delay(300);
  display.invertDisplay(false);
}

void displayPrintString(String text, uint16_t coordinateX, uint16_t coordinateY)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(coordinateX, coordinateY);
  display.println(text);
  display.display();
}

void displayVoltageCurrent(double Vin, double V, double I)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(2, 12);

  display.print("Vin= ");
  if (Vin > 1000)
  {
    display.print(Vin / 1000, 2);
    if (V_I_SEL == true)
      display.println("V");
  }
  else
  {
    display.print(Vin, 2);
    display.println("mV");
  }

  display.setCursor(2, 24);
  display.print("Vout= ");
  if (V > 1000)
  {
    display.print(V / 1000, 2);
    if (V_I_SEL == true)
      display.println("V <<");
    else
      display.println("V");
  }
  else
  {
    display.print(V, 2);
    if (V_I_SEL == true)
      display.println("mV <<");
    else
      display.println("mV");
  }

  display.setCursor(2, 36);
  display.print("Iout= ");
  if (I > 1000)
  {
    display.print(I / 1000, 2);
    if (V_I_SEL == false)
      display.println("A <<");
    else
      display.println("A");
  }
  else
  {
    display.print(I, 2);
    if (V_I_SEL == false)
      display.println("mA <<");
    else
      display.println("mA");
  }

  display.display();
}

//returns true when requested time(ms) has passed for specified button
bool checkTime(unsigned long time, Btns btn)
{
  lastSampledTime = millis();
  while (whichBtn() & btn)
  {
    unsigned long timeNow = millis();
    if ((timeNow - lastSampledTime) >= time)
    {
      lastSampledTime = timeNow;
      return true;
    }
  }
  // lastSampledTime = timeNow;
  return false;
}
