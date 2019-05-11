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

 * Sleep with bit manipulation
 * SMCR &= ~(_BV(SM1) | _BV(SM0)); // sleep mode "Idle"
 *
 * SMCR |= _BV(SE); // go into sleep
 * __asm__ __volatile__ ("sleep\n\t"::);
 *
 * SMCR &= ~_BV(SE); // clear sleep bit upon waking up
 */

#include <avr/io.h>
#include <util/delay.h>
#include "def.h"

#include "uart.h"

#include <avr/interrupt.h>
#include <avr/sleep.h>

// tr is a temporary variable for storing the value in TCNT1 (timer 1)
volatile uint16_t tr;

volatile uint16_t counter0, counter0_init;
volatile uint8_t counter0_done;

// pin interrupt
ISR(TOUCH_INT_VECT) {
  tr = TCNT1;
}

// timer1 compare A
ISR(TOUCH_TIMER1_VECT) {
  tr = 0xfff0;
}

// timer 0 compare A
ISR(TOUCH_TIMER0_VECT) {
  cli();
  if (--counter0 == 0) {
    counter0 = counter0_init;
    counter0_done = 1;
  }
  sei();
}

void setup_timer0() {
  // setup timer0
  TIMSK0 |= _BV(OCIE0A); // compare match A int enable
  TIFR0 |= _BV(OCF0A); // ISR(TIMER0_COMPA_vect)
  // TIMSK0 |= _BV(TOIE0) // overflow interrupt enable using ISR(TIMER0_OVF_vect)
}

void start_timer0(uint16_t duration) {
  counter0 = counter0_init = duration;
  OCR0A = 8; // 8MHz/1024/8 = 1.024ms
  TCNT0 = 0;
  TCCR0A |= _BV(CTC0) | _BV(CS00) | _BV(CS02); // start timer with prescaler 1024 and CTC (clear timer on compare)
  counter0_done = 0;
}

void stop_timer0() {
  TCCR0A = 0x00; // stop timer 0
}

void setup_timer1() {
  TIMSK1 |= _BV(OCIE1A); // enable overflow compare A (to detect if we're taking too long)
  TIFR1 |= _BV(OCF1A); // interrupt flag register, compares to OCR1A
}

void start_timer1() {
  OCR1A = 0xfff0; // set overflow A to be 0xfff0 cycles
  TCNT1 = 0;
  TCCR1B = _BV(CS10); // start timer with prescaler 1
}

void stop_timer1() {
  TCCR1B = 0x00;
}

void sleep(uint16_t ms) {
  start_timer0(ms);
  while (counter0_done == 0) {
    sleep_mode();
  }
  /* blocked sleeping until waking up */
  counter0_done = 0; // reset counter flag
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
  start_timer1();

  // put the CPU to sleep until we either get a pin change or overflow compare A.
  sleep_mode();
  stop_timer1();
  TOUCH_PCMSK &= ~_BV(input_int); // disable pin change interrupt

  return tr;
}

int main(void) {
  DINIT(); // enable debug output
  DL("Hello there");

  led_setup();

  setup_timer0();
  setup_timer1();

  INT_SETUP |= _BV(TOUCH_PCIE); // enable pin change interrupts

  // set touch pins low
  TOUCH_PORT &= ~(_BV(MOIST_A) | _BV(TOUCH1) | _BV(TOUCH2) | _BV(TOUCH3));

  tr = 0x0000;

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  sei();


  /*
  start_timer0(1000);

  while(1) {
    // avr wakes up on interrupt but we have an additional counter over it
    while (counter0_done == 0) {
      sleep_mode();
    }
    counter0_done = 0; // clear count flag

    if (!led_is_on('g')) {
      led_on('g');
      _delay_ms(20);
      led_off('g');
    }
  }
  */

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

  /*
   * 0: no activity, can sleep
   * 1: new activity, start activity countdown
   * 2: ongoing activity countdown
   */
  uint8_t sensed_activity = 0;

  while (1) {

    for (int sensor=0; sensor<4; sensor++) {
      uint16_t m = measure(sensor);
      value[sensor] = m < offset[sensor] ? 0 : m - offset[sensor];
      // DF("sensor %i: %u, (offset: %u)", sensor, value[sensor], offset[sensor]);

      // check touch buttons ignore sensor0 which is moisture sensor
      if ( sensor != 0) {
        // we do have activity
        if (value[sensor] > TOUCH_THRESHOLD) {
          sensed_activity = 1;
        }

        if (pressed[sensor] == 0 && value[sensor] > TOUCH_THRESHOLD) {
          pressed[sensor] = 1;
          DF("pushed touch %i (value: %u)", sensor, value[sensor]);
        }

        if (sensor == 1 && value[1] > TOUCH_THRESHOLD) {
          DF("humidity: %u", value[0]);

          led_off_all();
          if (value[0] > 250) led_on('g');
          if (value[0] > 500) led_on('r');
          if (value[0] > 750) led_on('b');
        }

        // release button
        if (pressed[sensor] == 1 && value[sensor] < TOUCH_THRESHOLD) {
          pressed[sensor] = 0;
          DF("released touch %i", sensor);
          if (sensor == 1) {
            led_off_all();
          }
        }
      }
    }
    // DF("sensed_activity: %u", sensed_activity);

    // no activity - go to sleep
    if (sensed_activity == 0) {
      led_on('g');
      _delay_ms(10);
      led_off('g');
      sleep(3000);
    // new activity sensed
    // set to ongoing countdown
    // do not block loop on ongoing countdown
    } else if (sensed_activity == 1) {
      sensed_activity = 2;
      start_timer0(3000);
    } else if (sensed_activity == 2 && counter0_done == 1) {
      counter0_done = 0; // reset counter flag
      sensed_activity = 0;
    }

  }
  return 0;
}
