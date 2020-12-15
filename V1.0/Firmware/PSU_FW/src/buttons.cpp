#include "buttons.h"

void initBtns()
{
    pinMode(UP_BTN, INPUT);
    pinMode(DW_BTN, INPUT);
    pinMode(V_I_BTN, INPUT);
    pinMode(O_EN_BTN, INPUT);
}

uint8_t whichBtn()
{
    uint8_t ret = 0b0000;
    while (!digitalRead(UP_BTN))
        ret |= UP;
    while (!digitalRead(DW_BTN))
        ret |= DW;
    while (!digitalRead(V_I_BTN))
        ret |= VI;
    if (!digitalRead(O_EN_BTN))
        ret |= OEN;
    return ret;
}