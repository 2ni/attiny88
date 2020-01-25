#ifndef _ST7789_H_
#define _ST7789_H_

#include <stdint.h>

extern void     st7789_init(void);
extern void     st7789_send_data(uint8_t data);
extern void     st7789_send_cmd(uint8_t cmd);
extern uint16_t st7789_convert_color(uint8_t r, uint8_t g, uint8_t b);
extern void     st7789_draw_point(uint8_t x, uint8_t y, uint16_t color);
extern void     st7789_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
extern void     st7789_fill_screen(uint16_t color);
extern void     st7789_on();
extern void     st7789_off();
extern void     st7789_char(char c, uint8_t x, uint8_t y, uint8_t scale, uint16_t fgcolor, uint16_t bgcolor);

// vcc (pin 13)
// gnd  (pin 10)
// port/ddr SPI
// scl -> sck (pin 4)
// sda -> mosi (pin 3)
#define RST      SS  // (pin 5)
#define DC       PD2 // (test pin)

#define PORT_BLK PORT_DIO0
#define DDR_BLK  DDR_DIO0
#define BLK      DIO0 // (pin 14)

#define ST7789_TFTWIDTH 	240
#define ST7789_TFTHEIGHT 	240

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 0

#define ST_CMD_DELAY   0x80    // special signifier for command lists

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

// Color definitions
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#endif
