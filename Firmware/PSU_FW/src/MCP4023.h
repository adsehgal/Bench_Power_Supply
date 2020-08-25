#include <Arduino.h>
#include <stdint.h>
#define POT_CS 6
#define POT_UD 14

/**
 *initialize pins and digital values for the MCP4023 
 */
void initDiv();

/**
 * Increment MCP4023 by 1 level
 */
void rDivIncrement();

/**
 * Decrement MCP4023 by 1 level
 */
void rDivDecrement();