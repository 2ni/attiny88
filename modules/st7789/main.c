/**
 * simple lightweight library to control a ST7789 lcd display w/o CS pin
 * BLK off & RST low -> 300uA consumption
 * RST init process needed
 *
 * when RST low -> init() needed again. Screen shows last setup
 *
 */

#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"
#include "st7789.h"
#include "def.h"

// _BV(3) => 1 << 3 => 0x08
// PORTB = PORTB | (1 << 4);
// set: PORTB |= (1<<BIT2); PORTB |= _BV(1) | BV(2);
// clear: PORTB &= ~(1<<BIT2); PORTB &= _BV(1) & _BV(2);

int main(void) {
  DINIT(); // simplex uart setup
  DL("Hello there");

  led_setup();

  st7789_init();

  // blink a couple of times
  uint8_t num=3;
  for (uint8_t c=0; c<num; c++) {
    led_on('r');
    _delay_ms(10);
    led_off('r');
    if (c!= num-1) _delay_ms(200); // no delay if last cycle
  }

  st7789_fill_screen(BLACK);
  st7789_on();

  uint8_t scale = 3, space = 2;
  st7789_char('2', 0, 0, scale, YELLOW, MAGENTA);
  st7789_draw_point(ST7789_TFTWIDTH/2, ST7789_TFTWIDTH/2, GREEN);
  st7789_fill_rect(40, 100, 10, 20, RED);

  st7789_draw_point(10, 30, GREEN);
  char f[3] = "foo";
  st7789_text(f, 3, 0, 50, 3, 2, GREEN, BLACK);

  // test resetting screen
  // after init the screen shows the same content
  /*
  _delay_ms(500);
  PORT_SPI &= ~_BV(RST);
  st7789_init();
  */

  DL("done.");

  while (1) {
  }

  return 0;
}
