/*
 * ssd1306.h
 *
 *  Created on: Feb 14, 2021
 *      Author: adityasehgal
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#include <stddef.h>
#include <_ansi.h>
#include <math.h>
#include <stdlib.h>
#include <string.h> // For memcpy

_BEGIN_STD_C

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "main.h"

#define SSD1306_I2C_ADDR (0x3D << 1)

#define SSD1306_HEIGHT 64
#define SSD1306_WIDTH 128
#define SSD1306_BUFFER_SIZE SSD1306_WIDTH *SSD1306_HEIGHT / 8

//fonts start
typedef struct {
	const uint8_t FontWidth; /*!< Font width in pixels */
	uint8_t FontHeight; /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef;

#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

#ifdef SSD1306_INCLUDE_FONT_6x8
extern FontDef Font_6x8;
#endif
#ifdef SSD1306_INCLUDE_FONT_7x10
extern FontDef Font_7x10;
#endif
#ifdef SSD1306_INCLUDE_FONT_11x18
extern FontDef Font_11x18;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x26
extern FontDef Font_16x26;
#endif
//fonts end

extern I2C_HandleTypeDef hi2c1;

// Enumeration for screen colors
typedef enum {
	SSD1306_BLACK = 0x00, // SSD1306_BLACK color, no pixel
	SSD1306_WHITE = 0x01  // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

typedef enum {
	SSD1306_OK = 0x00, SSD1306_ERR = 0x01 // Generic error.
} SSD1306_Error_t;

// Struct to store transformations
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
	uint8_t DisplayOn;
} SSD1306_t;

typedef struct {
	uint8_t x;
	uint8_t y;
} SSD1306_VERTEX;

// Procedure definitions
void ssd1306_Init(void);

void ssd1306_Fill(SSD1306_COLOR color);

void ssd1306_UpdateScreen(void);

void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);

char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);

char ssd1306_WriteString(char *str, FontDef Font, SSD1306_COLOR color);

void ssd1306_SetCursor(uint8_t x, uint8_t y);

void ssd1306_Line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
		SSD1306_COLOR color);

void ssd1306_DrawArc(uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle,
		uint16_t sweep, SSD1306_COLOR color);

void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r,
		SSD1306_COLOR color);

void ssd1306_Polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size,
		SSD1306_COLOR color);

void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
		SSD1306_COLOR color);
/**
 * @brief Sets the contrast of the display.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void ssd1306_SetContrast(const uint8_t value);

/**
 * @brief Set Display ON/OFF.
 * @param[in] on 0 for OFF, any for ON.
 */
void ssd1306_SetDisplayOn(const uint8_t on);

/**
 * @brief Reads DisplayOn state.
 * @return  0: OFF.
 *          1: ON.
 */
uint8_t ssd1306_GetDisplayOn();

// Low-level procedures
void ssd1306_Reset(void);

void ssd1306_WriteCommand(uint8_t byte);

void ssd1306_WriteData(uint8_t *buffer, size_t buff_size);

SSD1306_Error_t ssd1306_FillBuffer(uint8_t *buf, uint32_t len);

void ssd1306_DrawBitMap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
		int16_t h, uint16_t color);

void ssd1306_WriteErrorMsg(int16_t x, int16_t y, char *str);

_END_STD_C

#endif /* INC_SSD1306_H_ */
