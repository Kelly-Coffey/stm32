#ifndef __LCD_H
#define __LCD_H

#include "main.h"
#include "stm32f3xx_hal.h"

#define LCD_RD_GPIO_Port   A0_GPIO_Port
#define LCD_RD_Pin         A0_Pin
#define LCD_WR_GPIO_Port   A1_GPIO_Port
#define LCD_WR_Pin         A1_Pin
#define LCD_RS_GPIO_Port   A2_GPIO_Port
#define LCD_RS_Pin         A2_Pin
#define LCD_CS_GPIO_Port   A3_GPIO_Port
#define LCD_CS_Pin         A3_Pin
#define LCD_RESET_GPIO_Port A4_GPIO_Port
#define LCD_RESET_Pin       A4_Pin

#define HEIGHT  320
#define WIDTH   240

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

static const unsigned char font[];

void Lcd_Init(void);

void setRotation(uint8_t r);

void setCursor(int16_t x, int16_t y);
void setTextColor(uint16_t c, uint16_t b);
void setTextSize(uint8_t s);

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void fillScreen(uint16_t color);
void drawChar(int16_t x, int16_t y, uint8_t c, uint16_t color, uint16_t bg, uint8_t size);
void writeChar(uint8_t c);
void writeString(uint8_t* str);

#endif

