#include "lcd.h"

int16_t _width = 240;
int16_t _height = 320;
int16_t _cursor_x = 0;
int16_t _cursor_y = 0;
int8_t  _rotation = 0;

uint16_t _textcolor = 0xFFFF;
uint16_t _textbgcolor = 0xFFFF;
uint8_t  _textsize = 1;
uint8_t  _wrap = 1;

int16_t _MC, _MP, _MW, _SC, _EC, _SP, _EP;

void setCursor(int16_t x, int16_t y)
{
  _cursor_x = x;
  _cursor_y = y;
}

void setTextColor(uint16_t c, uint16_t b)
{
  _textcolor = c;
  _textbgcolor = b;
}

void setTextSize(uint8_t s)
{
  _textsize = s;
}

void WriteBus(uint8_t d)
{
  GPIOA->BSRR = 0x0700 << 16;
  GPIOB->BSRR = 0x0438 << 16;
  GPIOC->BSRR = 0x0080 << 16;
  GPIOA->BSRR = (((d) & (1<<0)) << 9) \
              | (((d) & (1<<2)) << 8) \
              | (((d) & (1<<7)) << 1);
  GPIOB->BSRR = (((d) & (1<<3)) << 0) \
              | (((d) & (1<<4)) << 1) \
              | (((d) & (1<<5)) >> 1) \
              | (((d) & (1<<6)) << 4);
  GPIOC->BSRR = (((d) & (1<<1)) << 6);
  
  HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, RESET);
  HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, SET);
}

void WriteCmd(uint16_t cmd)
{
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, RESET);
  WriteBus(cmd >> 8);
  WriteBus(cmd);
}

void WriteCmdData(uint16_t cmd, uint16_t dat)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);

  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, RESET);
  WriteBus(cmd >> 8);
  WriteBus(cmd);
  
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, SET);
  WriteBus(dat>>8);
  WriteBus(dat);

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, SET);
}

void setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
  WriteCmdData(_MC, x);
  WriteCmdData(_MP, y);
  WriteCmdData(_SC, x);
  WriteCmdData(_SP, y);
  WriteCmdData(_EC, x1);
  WriteCmdData(_EP, y1);
}

void vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
  int16_t bfa = HEIGHT - top - scrollines;  // bottom fixed area
  int16_t vsp;
  int16_t sea = top;

  if (offset <= -scrollines || offset >= scrollines) {
    offset = 0; //valid scroll
  }
	vsp = top + offset; // vertical start position

  if (offset < 0) {
    vsp += scrollines;          //keep in unsigned range
  }

  sea = top + scrollines - 1;

  WriteCmdData(0x6A, vsp);        //VL#
}

void setRotation(uint8_t r)
{
  _rotation = r & 3;           // just perform the operation ourselves on the protected variables
  _width = (_rotation & 1) ? HEIGHT : WIDTH;
  _height = (_rotation & 1) ? WIDTH : HEIGHT;

  switch (_rotation) {
  case 0:                    //PORTRAIT:
    WriteCmdData(0x60, 0xA700);
    WriteCmdData(0x01, 0x0100);
    WriteCmdData(0x03, 0x0030);
    _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
  break;
  case 1:                    //LANDSCAPE: 90 degrees
    WriteCmdData(0x60, 0xA700);
    WriteCmdData(0x01, 0x0000);
    WriteCmdData(0x03, 0x1038);
    _MC = 0x21, _MP = 0x20, _MW = 0x22, _SC = 0x52, _EC = 0x53, _SP = 0x50, _EP = 0x51;
    break;
  case 2:                    //PORTRAIT_REV: 180 degrees
    WriteCmdData(0x60, 0x2700);
    WriteCmdData(0x01, 0x0000);
    WriteCmdData(0x03, 0x1030);
    _MC = 0x20, _MP = 0x21, _MW = 0x22, _SC = 0x50, _EC = 0x51, _SP = 0x52, _EP = 0x53;
    break;
  case 3:                    //LANDSCAPE_REV: 270 degrees
    WriteCmdData(0x60, 0x2700);
    WriteCmdData(0x01, 0x0100);
    WriteCmdData(0x03, 0x0038);
    _MC = 0x21, _MP = 0x20, _MW = 0x22, _SC = 0x52, _EC = 0x53, _SP = 0x50, _EP = 0x51;
    break;
  }
  
  setAddrWindow(0, 0, _width-1, _height-1);
  vertScroll(0, HEIGHT, 0);   //reset scrolling after a rotation
}

void Lcd_Init(void)
{
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, SET);
  HAL_Delay(50); 
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, RESET);
  HAL_Delay(150);
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, SET);
  HAL_Delay(150);

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, SET);
  HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, SET);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);

  WriteCmdData(0x0000, 0x0001);
  HAL_Delay(100); //at least 100ms
  WriteCmdData(0x0010, 0x1790);
  WriteCmdData(0x0046, 0x0002);
  WriteCmdData(0x0013, 0x8010);
  WriteCmdData(0x0012, 0x80fe);

  WriteCmdData(0x0092, 0x0200);
  WriteCmdData(0x0093, 0x0303);
    
  WriteCmdData(0x0001, 0x0100); // set SS and SM bit 0100
  WriteCmdData(0x0002, 0x0700); // set 1 line inversion
  WriteCmdData(0x0003, 0x0030); // set GRAM write direction and BGR=0.
  WriteCmdData(0x0004, 0x0000); // Resize register
  WriteCmdData(0x0008, 0x0302); // set the back porch and front porch
  WriteCmdData(0x0009, 0x0000); // set non-display area refresh cycle ISC[3:0]
  WriteCmdData(0x000A, 0x0000);// FMARK function
  WriteCmdData(0x000C, 0x0000); // RGB interface setting
  WriteCmdData(0x000D, 0x0000); // Frame marker Position
  WriteCmdData(0x000F, 0x0000); // RGB interface polarity
  HAL_Delay(120);
  
  WriteCmdData(0x0030, 0x0303);
  WriteCmdData(0x0031, 0x0303);
  WriteCmdData(0x0032, 0x0303);
  WriteCmdData(0x0033, 0x0300);
  WriteCmdData(0x0034, 0x0003);
  WriteCmdData(0x0035, 0x0303);
  WriteCmdData(0x0036, 0x0014);
  WriteCmdData(0x0037, 0x0303);
  WriteCmdData(0x0038, 0x0303);
  WriteCmdData(0x0039, 0x0303);
  WriteCmdData(0x003a, 0x0300);
  WriteCmdData(0x003b, 0x0003);
  WriteCmdData(0x003c, 0x0303);
  WriteCmdData(0x003d, 0x1400);
  
  WriteCmdData(0x0060, 0xA700); // Gate Scan 
  WriteCmdData(0x0061, 0x0001); // NDL,VLE, REV
  WriteCmdData(0x006A, 0x0000); // set scrolling line
  WriteCmdData(0x0090, 0x0033);
  WriteCmdData(0x0095, 0x0110);
  
  WriteCmdData(0x00FF, 0x0001);
  WriteCmdData(0x00FF, 0x000C);
  WriteCmdData(0x00FF, 0x0000);
  HAL_Delay(100);
  WriteCmdData(0x0007, 0x0173);
  
  HAL_Delay(50);

  setRotation(0);
}


void drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if (x < 0 || y < 0 || x >= _width || y >= _height) {
    return;
  }

  WriteCmdData(_MC, x);
  WriteCmdData(_MP, y);
  WriteCmdData(_MW, color);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  int16_t end;
  if (w < 0) {
    w = -w;
    x -= w;
  }                           //+ve w
  end = x + w;
  if (x < 0) {
    x = 0;
  }

  if (end > _width) {
    end = _width;
  }
  w = end - x;
  if (h < 0) {
    h = -h;
    y -= h;
  }                           //+ve h
  end = y + h;
  if (y < 0) {
    y = 0;
  }
  if (end > _height) {
    end = _height;
  }
  h = end - y;

  setAddrWindow(x, y, x + w - 1, y + h - 1);

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, RESET);
  WriteCmd(_MW);
  
  if (h > w) {
    end = h;
    h = w;
    w = end;
  }
  
  uint8_t hi = color >> 8, lo = color & 0xFF;
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, SET);
  while (h-- > 0) {
    end = w;

    do {
      WriteBus(hi);
      WriteBus(lo);
    } while (--end != 0);
  }

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, SET);
}

void fillScreen(uint16_t color)
{
  fillRect(0, 0, _width, _height, color);
}

// Draw a character
void drawChar(int16_t x, int16_t y, uint8_t c,
  uint16_t color, uint16_t bg, uint8_t size)
{
  if ((x >= _width)            || // Clip right
      (y >= _height)           || // Clip bottom
      ((x + 6 * size - 1) < 0) || // Clip left
      ((y + 8 * size - 1) < 0)    // Clip top
      ) {
    return;
  }

  for(int8_t i=0; i<6; i++ ) {
    uint8_t line;
    if (i < 5) {
      line = *(font+(c*5)+i);
    } else {
      line = 0x0;
    }

    for (int8_t j=0; j<8; j++, line >>= 1) {
      if (line & 0x1) {
        if (size == 1) {
          drawPixel(x+i, y+j, color);
        } else {
          fillRect(x+(i*size), y+(j*size), size, size, color);
        }
      } else if (bg != color) {
        if (size == 1) {
          drawPixel(x+i, y+j, bg);
        } else {
          fillRect(x+i*size, y+j*size, size, size, bg);
        }
      }
    }
  }
}

void writeChar(uint8_t c)
{
  if (c == '\n') {
    _cursor_y += _textsize*8;
    _cursor_x  = 0;
  } else if (c == '\r') {
    // skip
  } else {
    if (_wrap && ((_cursor_x + _textsize * 6) > _width)) { // Heading off edge?
      _cursor_x  = 0;            // Reset x to zero
      _cursor_y += _textsize * 8; // Advance y one line
    }
    drawChar(_cursor_x, _cursor_y, c, _textcolor, _textbgcolor, _textsize);
    _cursor_x += _textsize * 6;
  }
}

void writeString(uint8_t* str)
{
  while (*str != 0) {
    writeChar(*str++);
  }
}

