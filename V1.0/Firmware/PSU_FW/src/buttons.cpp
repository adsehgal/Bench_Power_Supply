#include "buttons.h"


void initBtns(){
    pinMode(UP_BTN,INPUT);
    pinMode(DW_BTN, INPUT);
    pinMode(V_I_BTN, INPUT);
    pinMode(O_EN_BTN,INPUT);
}

uint8_t whichBtn(){
    uint8_t ret = 0b0000;
    if (!digitalRead(UP_BTN))
        ret |= up;
    if (!digitalRead(DW_BTN))
        ret |= dw;
    if (!digitalRead(V_I_BTN))
        ret |= vi;
    if (!digitalRead(O_EN_BTN))
        ret |= oen;
    return ret;
}