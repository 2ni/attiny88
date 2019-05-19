/*
 * I2C scanner with hardware twi (eg used in ATtiny88)
 * based on twi library
 * https://github.com/tibounise/SSD1306-AVR
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>

#include "def.h"
#include "uart.h"


// ************************* twi functions *************************

#define SCL_CLOCK 100000L

uint8_t i2c_status_register;

void i2c_init() {
  TWSR = 0;
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
}

uint8_t i2c_start(uint8_t address) {
  uint8_t addr_write = (address << 1) & 0xfe;

  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));

  i2c_status_register = TW_STATUS & 0xF8;
  if ((i2c_status_register != TW_START) && (i2c_status_register != TW_REP_START)) {
      return 1;
  }

  TWDR = addr_write;
  TWCR = (1<<TWINT) | (1<<TWEN);

  while(!(TWCR & (1<<TWINT)));

  i2c_status_register = TW_STATUS & 0xF8;
  if ((i2c_status_register != TW_MT_SLA_ACK) && (i2c_status_register != TW_MR_SLA_ACK)) {
      return 1;
  }

  return 0;
}

uint8_t i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);

    while(!(TWCR & (1<<TWINT)));

    i2c_status_register = TW_STATUS & 0xF8;
    if (i2c_status_register != TW_MT_DATA_ACK) {
        return 1;
    } else {
        return 0;
    }
}

void i2c_stop(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    while(TWCR & (1<<TWSTO));
}


// ************************* ssd1306 functions *************************

#define SSD1306_DEFAULT_ADDRESS 0x3C
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETMULTIPLEX 0xA8
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_SWITCHCAPVCC 0x2
#define SSD1306_NOP 0xE3

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_BUFFERSIZE (SSD1306_WIDTH*SSD1306_HEIGHT)/8

void send_cmd(uint8_t command) {
  i2c_start(SSD1306_DEFAULT_ADDRESS);
  i2c_write(0x80);
  i2c_write(command);
  i2c_stop();
}

void ssd1306_init() {
  send_cmd(SSD1306_DISPLAYOFF);

  send_cmd(SSD1306_SETDISPLAYCLOCKDIV);
  send_cmd(0x80);

  send_cmd(SSD1306_SETMULTIPLEX);
  send_cmd(0x3F);

  send_cmd(SSD1306_SETDISPLAYOFFSET);
  send_cmd(0x00);

  send_cmd(SSD1306_SETSTARTLINE | 0x00);

  // We use internal charge pump
  send_cmd(SSD1306_CHARGEPUMP);
  send_cmd(0x14);

  // Horizontal memory mode
  send_cmd(SSD1306_MEMORYMODE);
  send_cmd(0x00);

  send_cmd(SSD1306_SEGREMAP | 0x1);

  send_cmd(SSD1306_COMSCANDEC);

  send_cmd(SSD1306_SETCOMPINS);
  send_cmd(0x12);

  // Max contrast
  send_cmd(SSD1306_SETCONTRAST);
  send_cmd(0xCF);

  send_cmd(SSD1306_SETPRECHARGE);
  send_cmd(0xF1);

  send_cmd(SSD1306_SETVCOMDETECT);
  send_cmd(0x40);

  send_cmd(SSD1306_DISPLAYALLON_RESUME);

  // Non-inverted display
  send_cmd(SSD1306_NORMALDISPLAY);

  // Turn display back on
  send_cmd(SSD1306_DISPLAYON);
}

void ssd1306_invert(uint8_t inverted) {
  if (inverted) {
    send_cmd(SSD1306_INVERTDISPLAY);
  } else {
    send_cmd(SSD1306_NORMALDISPLAY);
  }
}

void ssd1306_sendframe(uint8_t *buffer) {
  send_cmd(SSD1306_COLUMNADDR);
  send_cmd(0x00);
  send_cmd(0x7F);

  send_cmd(SSD1306_PAGEADDR);
  send_cmd(0x00);
  send_cmd(0x07);

  // We have to send the buffer as 16 bytes packets
  // Our buffer is 1024 bytes long, 1024/16 = 64
  // We have to send 64 packets
  for (uint8_t packet = 0; packet < 64; packet++) {
    i2c_start(SSD1306_DEFAULT_ADDRESS);
    i2c_write(0x40);
    for (uint8_t packet_byte = 0; packet_byte < 16; ++packet_byte) {
      i2c_write(buffer[packet*16+packet_byte]);
    }
    i2c_stop();
  }
}

void ssd1306_pixel(uint8_t *buffer, uint8_t pos_x, uint8_t pos_y, uint8_t pixel_status) {
  if (pos_x >= SSD1306_WIDTH || pos_y >= SSD1306_HEIGHT) {
    return;
  }

  if (pixel_status) {
    buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] |= (1 << (pos_y&7));
  } else {
    buffer[pos_x+(pos_y/8)*SSD1306_WIDTH] &= ~(1 << (pos_y&7));
  }
}

// ************************* main function *************************

int main(void) {
  DINIT(); // enable debug output
  DL("\n\nHello there");

  sei();

  i2c_init();
  ssd1306_init();
  uint8_t buffer[1024];
  ssd1306_sendframe(buffer);
  /*
  ssd1306_pixel(buffer, 5,5,1);
  */


  /*
  DL("Scanning...");
  for (uint8_t addr = 0; addr<127; addr++) {
    uint8_t addrw = (addr << 1) & 0xfe;
    uint8_t ret = i2c_start(addrw);
    if (!ret) {
      DF("found device 0x%02X", addr);
    }
    i2c_stop();
  }
  DL("Done.");
  */

  return 0;
}
