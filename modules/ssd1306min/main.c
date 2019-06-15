/*
 * oled based on
 * https://github.com/nicoud/Oled
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <string.h> // memset

#include "def.h"
#include "uart.h"

#include "twi.h"
#include "oled.h"

int main(void) {
  DINIT(); // enable debug output
  DL("\n\nHello there");

  sei();

  oled_setup();
  oled_hline(1);

  oled_set_pos(0, 0);
  oled_text("Welcome!");
  for(uint8_t i=0; i<40; i++) {
    oled_ddot(40+i,40);
  }

  return 0;
}
