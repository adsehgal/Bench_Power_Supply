/*
 * ssd1306_fonts.h
 *
 *  Created on: Feb 14, 2021
 *      Author: adityasehgal
 */
#include <stdint.h>
#ifndef INC_SSD1306_FONTS_H_
#define INC_SSD1306_FONTS_H_

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

#endif /* INC_SSD1306_FONTS_H_ */
