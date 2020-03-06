/**
 * tests to measure current with INA213 sensor
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#include "avr_print.h"
#include "uart.h"
#include "def.h"
#include <avr/sleep.h>

/*
_BV(3) => 1 << 3 => 0x08
PORTB = PORTB | (1 << 4);
set: PORTB |= (1<<BIT2); PORTB |= _BV(1) | BV(2);
clear: PORTB &= ~(1<<BIT2); PORTB &= _BV(1) & _BV(2);

PORTD |= (1 << 7) | (1 << 6);           // set both bits
PORTD &= ~((1 << 7) | (1 << 6));        // clear both bits
PORTD = (PORTD & ~(1 << 7)) | (1 << 6); // set bit 7 and clear bit 6
*/

uint16_t adc_value;

/*
 * interrupt for ad converter
 */
ISR(ADC_vect) {
  cli();
  uint8_t l = ADCL;
  adc_value = ADCH<<8 | l;
  sei();
}

int main(void) {
  DINIT(); // simplex uart setup
  DL("Hello there");

  led_setup();

  DDRC |= (1<<EN_CUR);  // set current measurement to output
  PORTC |= (1<<EN_CUR); // set current measeurement to high

  DDR_O |= (1<<OUT);    // set to output
  PORT_O |= (1<<OUT);   // set high

  ADMUX &= ~(_BV(REFS0) | _BV(ADLAR)); // internal 1.1v reference, left adjusted results
  // clear input channels 1st (corresponds to mode = 2)
  ADMUX &= ~(_BV(MUX3) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0));
  ADMUX |= (1<<MUX1); // ADC2 measure current
  ADCSRA |= _BV(ADEN); // enable adc
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // prescaler 64 -> 8MHz/64 = 125kHz

  sei();

  while (1) {
    ADCSRA |= _BV(ADSC) | _BV(ADIE); // start conversion with interrupt
    // sleep & wait for conversion
    // assume no other interrupt will wake up avr
    set_sleep_mode(SLEEP_MODE_ADC);
    sleep_enable();
    sleep_mode();

    // adc_value: 2.2A = 1.1v, 1bit = 2200mA/1024 ~ 2.148mA
    uint32_t current = adc_value*2200000/1024; // value in 1uA
    DF("current: %luuA", current);

    led_toggle('r');

    _delay_ms(500);
  }

  return 0;
}
