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

int main(void) {
  DINIT(); // enable debug output
  DL("\n\nHello there");

  uint8_t txdata[1];
  uint8_t addrw;
  uint16_t timeout;
  sei();
  TWIInit();
  DL("Scanning...");
  for (uint8_t addr = 0; addr<127; addr++) {
    addrw = (addr << 1) & 0xfe;
    txdata[0] = addrw;
    TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;

    timeout = 0;
    while (TWIInfo.errorCode != 0xFF && timeout < 10) {
      TWITransmitData(txdata, 1, 0);
      _delay_ms(1);
      timeout++;
    }
    if (TWIInfo.errorCode == 0xFF) {
      DF("found device 0x%02X", addr);
    }
  }
  DL("Done.");
  return 0;
}
