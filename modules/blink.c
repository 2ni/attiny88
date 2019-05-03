/**
 * Copyright (C) PlatformIO <contact@platformio.org>
 * See LICENSE for details.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#include "avr_print.h"
#include "uart.h"
#include "def.h"

// _BV(3) => 1 << 3 => 0x08
// PORTB = PORTB | (1 << 4);
// set: PORTB |= (1<<BIT2); PORTB |= _BV(1) | BV(2);
// clear: PORTB &= ~(1<<BIT2); PORTB &= _BV(1) & _BV(2);

int main(void) {
  /*
  cli();
  setup_spi();
  sendstr("HELLO\n");
  CLKPR = 0x80;
  CLKPR = 0x00;
  sei();
  */

  DINIT(); // simplex uart setup
  DL("Hello there");

  ledSetup();

  while (1) {
    _delay_ms(500);
    ledToggle('b');
  }

  return 0;
}
