/*
 * I2C scanner with hardware twi (eg used in ATtiny88)
 * based on twi library
 * http://www.chrisherring.net/all/tutorial-interrupt-driven-twi-interface-for-avr-part1/
 * add TWIlib.c and TWIlib.h in lib/ folder
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "def.h"
#include "uart.h"
#include "TWIlib.h"


// oled ssd1306
#define O_ADDR 0x3c
#define O_ADDR_W (O_ADDR << 1) & 0xfe

uint8_t send_cmd(unsigned char cmd) {
  uint8_t data[3];
  data[0] = O_ADDR_W;
  data[1] = 0x80; //  command mode
  data[2] = cmd;

  TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;

  uint8_t timeout = 0;
  while (TWIInfo.errorCode != TWI_SUCCESS && timeout < 2) {
    TWITransmitData(data, 3, 0);
    _delay_ms(1);
    timeout++;
  }
  if (TWIInfo.errorCode == TWI_SUCCESS) {
    return 0; // success
  } else {
    return 1;
  }
}

int main(void) {
  DINIT(); // enable debug output
  DL("\n\nHello there");

  uint8_t txdata[1];
  uint8_t addrw;
  uint16_t timeout;
  sei();
  TWIInit();

  DF("errors: %u", send_cmd(0xaf)); // oled on

  DL("Scanning...");
  for (uint8_t addr = 0; addr<127; addr++) {
    addrw = (addr << 1) & 0xfe;
    txdata[0] = addrw;
    TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;

    timeout = 0;
    while (TWIInfo.errorCode != TWI_SUCCESS && timeout < 2) {
      TWITransmitData(txdata, 1, 0);
      _delay_ms(1);
      timeout++;
    }
    if (TWIInfo.errorCode == TWI_SUCCESS) {
      DF("found device 0x%02X", addr);
    }
  }
  DL("Done.");

  return 0;
}
