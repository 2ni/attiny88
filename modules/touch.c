/*
 * simple capacitive touch pin / moisture sensor
 * using 500k instead of 1M
 * GND around sensor and on bottom layer
 * without GND sensor does almost not detect water
 *
 *
 * PA1 -----|500k|-----|+
 *       |
 *       |
 *     ----
 * GND |  | GND
 *     |  |
 * GND |  | GND
 *     |  |
 * GND ---- GND
 *     GND
 *
 *
 * based on https://github.com/cnlohr/magfestbadges/blob/master/basictouch/test.c
 * (or https://www.youtube.com/watch?v=BO3umH4Ht8o&t=547s)
 *
 */

#include <avr/io.h>
#include <util/delay.h>
#include "def.h"

#include "uart.h"

#include <avr/interrupt.h>
#include <avr/sleep.h>

#if defined (__AVR_ATtiny84__)
  #define TIMER_vect TIM1_COMPA_vect
  #define INT_SETUP GIMSK
  #define INT_CLEAR GIFR
#elif defined (__AVR_ATtiny88__)
  #define TIMER_vect TIMER1_COMPA_vect
  #define INT_SETUP PCICR
  #define INT_CLEAR PCIFR
#endif


// tr is a temporary variable for storing the value in TCNT1 (timer 1)
volatile uint16_t tr;
volatile uint16_t count;

// isr when the touch pin rises
ISR(PCINT0_vect) {
  tr = TCNT1;
  count++;
  if (TIFR1 & _BV(OCF1A))
    tr = 0xfff0;
}

// isr if the pin takes too long to rise
ISR(TIMER_vect) {
  tr = 0xfff0;
}

/*
 * measure values by blocking CPU
 */
uint16_t measure() {
  DDRA |= _BV(MOIST_A);  // set as output -> low to discharge capacity

  // wait a short time to ensure discharge
  _delay_ms(10);

  INT_CLEAR = (1<<PCIF0); // clear any pending interrupt flag
  PCMSK0 |= _BV(MOIST_A_INT); // set pin change interrupt

  DDRA &= ~_BV(MOIST_A);  // set as input -> capacity is charged via resistor

  // reset the timer
  TCNT1 = 0;

  // put the CPU to sleep until we either get a pin change or overflow compare A.
  sleep_cpu();
  PCMSK0 &= ~_BV(MOIST_A_INT); // disable pin change interrupt

  return tr;
}

int main(void) {
  DINIT();
  uint16_t value;
  unsigned short calibrate = 0;  // initial "zero" value for touch sensor

  // setup timer

  TCCR1B = _BV(CS10);       // use prescalar 2. CS10 = /1, CS11 = /8, ...
  TIMSK1 |= _BV(OCIE1A);  // enable overflow compare A (to detect if we're taking too long)
  OCR1A = 0xfff0;            // set overflow A to be 0xfff0 cycles

  sleep_enable();          // allow the CPU to sleep.

  INT_SETUP |= _BV(PCIE0);      // enable Pin change interrupts.

  TIFR1 |= _BV(OCF1A); // interrupt flag register, compares to OCR1A

  PORTA &= ~_BV(MOIST_A); // set port low as default

  tr = 0x0000;

  DL("Hello there");
  count = 0;
  sei();

  // calibrate sensor
  DL("calibrating. Do not touch sensor");
  for (int i=0; i<10; i++) {
    calibrate = measure();
  }

  while (1) {
    value = measure();
    uint16_t valueCalibrated = value - calibrate;
    if (valueCalibrated > 0xfff0) valueCalibrated = 0x00; // if negative set it 0
    DF("value: %u (uncalibrated: %u)", valueCalibrated, value);

    if (valueCalibrated > 50) {
      ledOn('b');
    } else {
      ledOff('b');
    }
    _delay_ms(500);
  }
  return 0;
}
