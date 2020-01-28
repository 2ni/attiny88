/*
 * measure vcc w/o any gpio pin
 * see http://ww1.microchip.com/downloads/en/Appnotes/00002447A.pdf
 */

#include <avr/io.h>
#include <util/delay.h>
//#include "avr_print.h"
#include "uart.h"
#include "def.h"

// _BV(3) => 1 << 3 => 0x08
// PORTB = PORTB | (1 << 4);
// set: PORTB |= (1<<BIT2); PORTB |= _BV(1) | BV(2);
// clear: PORTB &= ~(1<<BIT2); PORTB &= _BV(1) & _BV(2);

int main(void) {
  DINIT(); // simplex uart setup
  DL("Hello there");

  led_setup();

  // for float output check: https://stackoverflow.com/questions/905928/using-floats-with-sprintf-in-embedded-c
  uint16_t vcc = 0;
  uint16_t vcc_l = 0, vcc_h = 0;

  ADMUX |= ((1<<REFS0)|       // vcc as reference
      (0<<ADLAR)|             // left adjust disabled
      (0xe<<MUX0));           // input is internal reference (MUX3=MUX2=MUX1=1, MUX0=0)

  PRR &= ~(1<<PRADC);         // clear adc power reduction bit
  ADCSRA |= ((1<<ADEN)|       // enable adc
      (1<<ADPS2)|(1<<ADPS1)|  // prescaler 64 -> 8MHz/64 = 125kHz
      (1<<ADSC));             // start conversion

  while (1) {
    if (ADCSRA & (1<<ADIF)) { // adc conversion complete
      led_flash('r', 3);
      vcc_l = ADCL;
      vcc_h = ADCH;
      // vcc = 1024*vbg/resadc in 0.1 resolution
      // 34 -> 3.4v
      vcc = (1024 * 1.1 * 10)/(vcc_l + vcc_h*256);
      _delay_ms(1000);
      DF("vcc: %u", vcc);
      ADCSRA |= (1<<ADSC);    // start conversion
    }
  }
  return 0;
}
