/*
 * ssd1306.h
 *
 *  Created on: Nov 25, 2021
 *      Author: adityasehgal
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#include <stdint.h>

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
	uint16_t currX;
	uint16_t currY;
	uint8_t inverted;
	uint8_t initialized;
	uint8_t displayOn;
} SSD1306_t;

typedef struct {
	uint8_t x;
	uint8_t y;
} SSD1306_VERTEX;

// Procedure definitions
void ssd1306_init(void);

void ssd1306_fill(SSD1306_COLOR color);

void ssd1306_updateScreen(void);

void ssd1306_drawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);

char ssd1306_writeChar(char ch, FontDef Font, SSD1306_COLOR color);

char ssd1306_writeString(char *str, FontDef Font, SSD1306_COLOR color);

void ssd1306_setCursor(uint8_t x, uint8_t y);

void ssd1306_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
		SSD1306_COLOR color);

void ssd1306_polyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size,
		SSD1306_COLOR color);

void ssd1306_drawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2,
		SSD1306_COLOR color);
/**
 * @brief Sets the contrast of the display.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void ssd1306_setContrast(const uint8_t value);

/**
 * @brief Set Display ON/OFF.
 * @param[in] on 0 for OFF, any for ON.
 */
void ssd1306_setDisplayOn(const uint8_t on);

/**
 * @brief Reads DisplayOn state.
 * @return  0: OFF.
 *          1: ON.
 */
uint8_t ssd1306_getDisplayOn();

// Low-level procedures
void ssd1306_reset(void);

void ssd1306_writeCommand(uint8_t byte);

void ssd1306_writeData(uint8_t *buffer, uint32_t buff_size);

SSD1306_Error_t ssd1306_fillBuffer(uint8_t *buf, uint32_t len);

void ssd1306_drawBitMap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
		int16_t h, uint16_t color);

#endif /* INC_SSD1306_H_ */
