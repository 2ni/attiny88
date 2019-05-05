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

// tr is a temporary variable for storing the value in TCNT1 (timer 1)
volatile uint16_t tr;

// isr when the touch pin rises
ISR(TOUCH_INT_VECT) {
  tr = TCNT1;
}

// isr if the pin takes too long to rise
ISR(TOUCH_TIMER_VECT) {
  tr = 0xfff0;
}

/*
 * measure values by blocking CPU
 */
uint16_t measure() {
  TOUCH_DDR |= _BV(MOIST_A);  // set as output -> low to discharge capacity

  // wait a short time to ensure discharge
  _delay_ms(10);

  INT_CLEAR = (1<<TOUCH_PCIF); // clear any pending interrupt flag
  TOUCH_PCMSK |= _BV(MOIST_A_INT); // activate pin change interrupt on pin

  TOUCH_DDR &= ~_BV(MOIST_A);  // set as input -> capacity is charged via resistor

  // reset the timer and start it with prescaler CS10 = /1, CS11 = /8, ...
  TCNT1 = 0;
  TCCR1B = _BV(CS10);

  // put the CPU to sleep until we either get a pin change or overflow compare A.
  sleep_cpu();
  TCCR1B = 0x00; // disable timer
  TOUCH_PCMSK &= ~_BV(MOIST_A_INT); // disable pin change interrupt

  return tr;
}

int main(void) {
  DINIT();                       // enable debug output
  led_setup();

  uint16_t value;
  unsigned short calibrate = 0;  // initial "zero" value for touch sensor

  // setup overflow
  TIMSK1 |= _BV(OCIE1A);         // enable overflow compare A (to detect if we're taking too long)
  OCR1A = 0xfff0;                // set overflow A to be 0xfff0 cycles
  TIFR1 |= _BV(OCF1A);          // interrupt flag register, compares to OCR1A

  sleep_enable();                // allow the CPU to sleep.

  INT_SETUP |= _BV(TOUCH_PCIE); // enable pin change interrupts

  TOUCH_PORT &= ~_BV(MOIST_A);   // set port low as default

  tr = 0x0000;

  DL("Hello there");
  sei();

  // calibrate sensor
  D("calibrating. Do not touch sensor...");
  for (int i=0; i<10; i++) {
    calibrate = measure();
  }
  DL(" done");

  while (1) {
    value = measure();
    uint16_t valueCalibrated = value - calibrate;
    if (valueCalibrated > 0xfff0) valueCalibrated = 0x00; // if negative set it 0
    DF("value: %u (uncalibrated: %u)", valueCalibrated, value);

    if (valueCalibrated > 50) {
      led_on('r');
    } else {
      led_off('r');
    }
    _delay_ms(500);
  }
  return 0;
}
