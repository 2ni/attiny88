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
 *
 * PORTC |= (_BV(0) | _BV(2) | _BV(7));  // Set bits 0,2,7
 * PORTC &= ~(_BV(1) | _BV(2) | _BV(6)); // Clear bits 1,2,6
 * PORTC ^= (_BV(5) | _BV(3));           // Toggle bits 3,5
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
 * measure sensor values (blocks CPU)
 * 0 = MOIST_A
 * 1 = TOUCH1
 * 2 = TOUCH2
 * 3 = TOUCH3
 */
uint16_t measure(int sensor) {
  int input;
  int input_int;

  if (sensor == 0) {
    input = MOIST_A;
    input_int = MOIST_A_INT;
  } else if (sensor == 1) {
    input = TOUCH1;
    input_int = TOUCH1_INT;
  } else if (sensor == 2) {
    input = TOUCH2;
    input_int = TOUCH2_INT;
  } else if (sensor == 3) {
    input = TOUCH3;
    input_int = TOUCH3_INT;
  }

  TOUCH_DDR |= _BV(input);  // set as output -> low to discharge capacity

  // wait a short time to ensure discharge
  _delay_ms(10);

  INT_CLEAR = (1<<TOUCH_PCIF); // clear any pending interrupt flag
  TOUCH_PCMSK |= _BV(input_int); // activate pin change interrupt on pin

  TOUCH_DDR &= ~_BV(input);  // set as input -> capacity is charged via resistor

  // reset the timer and start it with prescaler CS10 = /1, CS11 = /8, ...
  TCNT1 = 0;
  TCCR1B = _BV(CS10);

  // put the CPU to sleep until we either get a pin change or overflow compare A.
  sleep_cpu();
  TCCR1B = 0x00; // disable timer
  TOUCH_PCMSK &= ~_BV(input_int); // disable pin change interrupt

  return tr;
}

int main(void) {
  DINIT();                       // enable debug output
  led_setup();


  // setup overflow
  TIMSK1 |= _BV(OCIE1A);         // enable overflow compare A (to detect if we're taking too long)
  OCR1A = 0xfff0;                // set overflow A to be 0xfff0 cycles
  TIFR1 |= _BV(OCF1A);          // interrupt flag register, compares to OCR1A

  sleep_enable();                // allow the CPU to sleep.

  INT_SETUP |= _BV(TOUCH_PCIE); // enable pin change interrupts

  TOUCH_PORT &= ~(_BV(MOIST_A) | _BV(TOUCH1) | _BV(TOUCH2) | _BV(TOUCH3));   // set low as default

  tr = 0x0000;

  DL("Hello there");
  sei();

  // calibrate touch sensors
  uint16_t offset[4];
  DL("calibrating. Do not touch sensor...");
  for (int sensor=0; sensor<4; sensor++) {
    // take last value (1st ones might be off)
    for (int i=0; i<10; i++) {
      offset[sensor] = measure(sensor);
    }
    DF("sensor %d: %u", sensor, offset[sensor]);
  }
  DL("done");

  uint16_t value[4];
  static int pressed[4];

  while (1) {
    for (int sensor=0; sensor<4; sensor++) {
      value[sensor] = measure(sensor) - offset[sensor];
      if (value[sensor] > 0xfff0) value[sensor] = 0x0000; // if negative set it 0

      // check touch buttons
      if ( sensor != 0) {
        if (pressed[sensor] == 0 && value[sensor] > TOUCH_THRESHOLD) {
          pressed[sensor] = 1;
          DF("pushing touch %i", sensor);

          if (sensor == 1) {
            DF("humidity: %u", value[0]);

            led_off_all();
            if (value[0] > 250) led_on('g');
            if (value[0] > 500) led_on('r');
            if (value[0] > 750) led_on('b');
          }

          if (sensor == 2) {
            led_off_all();
          }
        }

        // release button
        if (pressed[sensor] == 1 && value[sensor] < TOUCH_THRESHOLD) {
          pressed[sensor] = 0;
        }
      }
    }

    //_delay_ms(500);
  }
  return 0;
}
