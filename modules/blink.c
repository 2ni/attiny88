/**
 * Copyright (C) PlatformIO <contact@platformio.org>
 * See LICENSE for details.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#include "avr_print.h"
#include "uart.h"

#define DDR_G DDRA
#define PORT_G PORTA
#define LED_G PA3

#define DDR_R DDRB
#define PORT_R PORTB
#define LED_R PB6

#define DDR_B DDRB
#define PORT_B PORTB
#define LED_B PB7


// _BV(3) => 1 << 3 => 0x08
// PORTB = PORTB | (1 << 4);
// set: PORTB |= (1<<BIT2); PORTB |= _BV(1) | BV(2);
// clear: PORTB &= ~(1<<BIT2); PORTB &= _BV(1) & _BV(2);

inline static void ledSetup(){
  DDR_G |= _BV(LED_G); // enable as output (set 1)
  PORT_G &= ~_BV(LED_G); // set low

  DDR_B |= _BV(LED_B); // enable as output (set 1)
  PORT_B &= ~_BV(LED_B); // set low

  DDR_R |= _BV(LED_R); // enable as output (set 1)
  PORT_R &= ~_BV(LED_R); // set low
}

inline static void ledOn(char color) {
  if (color == 'g') {
    PORT_G |= _BV(LED_G); // set high
  } else if (color == 'b') {
    PORT_B |= _BV(LED_B);
  } else if (color == 'r') {
    PORT_R |= _BV(LED_R);
  }
}

inline static void ledOff(char color) {
  if (color == 'g') {
    PORT_G &= ~_BV(LED_G); // set low
  } else if (color == 'b') {
    PORT_B &= ~_BV(LED_B);
  } else if (color == 'r') {
    PORT_R &= ~_BV(LED_R);
  }
}

inline static void ledToggle(char color) {
  if (color == 'g') {
    PORT_G ^= _BV(LED_G); // invert (exclusive or)
  } else if (color == 'b') {
    PORT_B ^= _BV(LED_B);
  } else if (color == 'r') {
    PORT_R ^= _BV(LED_R);
  }
}

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
