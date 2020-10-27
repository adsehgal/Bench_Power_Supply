#include <Arduino.h>
#include <stdint.h>

#define UP_BTN 4
#define DW_BTN 3
#define V_I_BTN 2
#define O_EN_BTN 7

enum Btns{
    up = 0b0001,
    dw = 0b0010,
    vi = 0b0100,
    oen = 0b1000,
};

void initBtns();
uint8_t whichBtn();
