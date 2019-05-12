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
  cli();
  tr = TCNT1;
  sei();
}

// timer1 compare A
// 16bit
ISR(TOUCH_TIMER1_VECT) {
  cli();
  tr = 0xfff0;
  sei();
}

// timer 0 compare A
// 8bit
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

/*
 * 8bit timer for timing work
 *
 * 1ms = 1.024ms
 * -> ms max ~ 67seconds
 */
void start_timer0(uint16_t ms) {
  counter0 = counter0_init = ms;
  OCR0A = 8; // 8MHz/1024/8 = 1.024ms
  TCNT0 = 0;
  TCCR0A |= _BV(CTC0) | _BV(CS00) | _BV(CS02); // start timer with prescaler 1024 and CTC (clear timer on compare)
  counter0_done = 0;
}

void stop_timer0() {
  TCCR0A = 0x00; // stop timer 0
}

/*
 * 16bit timer for measuring capacitive sensors
 */
void setup_timer1() {
  TIMSK1 |= _BV(OCIE1A); // enable overflow compare A (to detect if we're taking too long)
  TIFR1 |= _BV(OCF1A); // interrupt flag register, compares to OCR1A
}

void start_timer1() {
  OCR1A = 0xfff0; // set overflow A to be 0xfff0 cycles
  TCNT1 = 0;
  TCCR1B |= _BV(CS10); // start timer with prescaler 1
}

void stop_timer1() {
  TCCR1B = 0x00;
}

void sleep(uint16_t ms) {
  // turn off anything which is possible
  // consumption ~1.1mA
  PRR |= _BV(PRTWI) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRADC);
  start_timer0(ms);
  while (counter0_done == 0) {
    sleep_mode();
  }
  /* blocked sleeping until waking up */
  counter0_done = 0; // reset counter flag
  stop_timer0();
  PRR &= ~(_BV(PRTWI) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRADC));
}

/*
 * measure sensor values (blocks CPU)
 * 0 = MOIST_A
 * 1 = TOUCH1
 * 2 = TOUCH2
 * 3 = TOUCH3
 */
uint16_t measure(uint8_t sensor) {
  uint8_t input;
  uint8_t input_int;

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
  // do not use _delay_ms or sleep as timers in use
  volatile uint16_t i = 0;
  while ((TOUCH_PORT & _BV(input)) != 0) {
    __asm__ __volatile__ ("nop");
  }

  for (i=0; i<255; i++) {
    __asm__ __volatile__ ("nop");
  }

  INT_SETUP |= _BV(TOUCH_PCIE); // enable pin change interrupts

  INT_CLEAR = (1<<TOUCH_PCIF); // clear any pending interrupt flag
  TOUCH_PCMSK |= _BV(input_int); // activate pin change interrupt on pin

  start_timer1();
  TOUCH_DDR &= ~_BV(input);  // set as input -> capacity is charged via resistor

  // put the CPU to sleep until we either get a pin change or overflow compare A.
  sleep_mode();
  stop_timer1();

  // disable pin change interrupt
  TOUCH_PCMSK &= ~_BV(input_int);
  INT_SETUP &= ~_BV(TOUCH_PCIE); // pcicr

  return tr;
}

uint16_t offset[4];
void calibrate() {
  // calibrate touch sensors
  DL("calibrating. Do not touch sensor...");
  for (int sensor=0; sensor<4; sensor++) {
    // take average values ignoring first ones
    uint16_t sum = 0;
    for (int i=0; i<14; i++) {
      if (i>5) {
        sum += measure(sensor);
      }
    }
    offset[sensor] = sum/8;
    DF("sensor %d: %u", sensor, offset[sensor]);
  }
  DL("done");
}

void show_humidity(uint16_t value) {
  led_off_all();
  char color;
  if (value < 500) color = 'r';
  else if (value < 1000) color = 'g';
  else color = 'b';
  led_on(color);
}

int main(void) {
  DINIT(); // enable debug output
  DL("Hello there");

  led_setup();

  setup_timer1();
  setup_timer0();

  // set touch pins low
  TOUCH_PORT &= ~(_BV(MOIST_A) | _BV(TOUCH1) | _BV(TOUCH2) | _BV(TOUCH3));

  tr = 0x0000;

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  sei();

  // hardcoded calibrations
  offset[0] = 166;
  offset[1] = 57;
  offset[2] = 68;
  offset[3] = 65;

  uint16_t value[4];
  static int pressed[4];

  /*
   * 0: sleep mode with regular wake-ups to check on sensors
   * 1: activity (button pushed) -> do nothing
   * 2: activity (button released) -> start countdown
   * 3: activity countdown running
   */
  uint8_t mode = 0;

  while (1) {

    for (int sensor=0; sensor<4; sensor++) {
      uint16_t m = measure(sensor);
      value[sensor] = m < offset[sensor] ? 0 : m - offset[sensor];
      // DF("sensor %i: %u, (offset: %u)", sensor, value[sensor], offset[sensor]);

      // check touch buttons ignore sensor0 which is moisture sensor
      if ( sensor != 0) {
        // any button pushed
        if (value[sensor] > TOUCH_THRESHOLD) {
          mode = 1;
        }

        if (pressed[sensor] == 0 && value[sensor] > TOUCH_THRESHOLD) {
          pressed[sensor] = 1;
          DF("pushed touch %i (value: %u) (measured: %u)", sensor, value[sensor], m);
        }

        if (sensor == 1 && value[1] > TOUCH_THRESHOLD) {
          DF("humidity: %u", value[0]);

          show_humidity(value[0]);
        }

        // release button
        if (pressed[sensor] == 1 && value[sensor] < TOUCH_THRESHOLD) {
          mode = 2;
          pressed[sensor] = 0;
          DF("released touch %i", sensor);
          if (sensor == 1) {
            led_off_all();
          } else if (sensor == 3) {
            calibrate();
          }
        }
      }
    }

    // DF("\n************** mode: %u", mode);

    if (mode == 0) {
      // no activity - sleep mode with regular wake up to check on sensors
      show_humidity(value[0]);
      sleep(10);
      led_off_all();
      sleep(3000); // timer0
    } else if (mode == 1) {
      // activity (some button pushed)
      // do nothing (let the loop run)
      stop_timer0(); // in case we were coming back from mode 3
    } else if (mode == 2) {
      // activity (button released)
      // let the loop run
      DL("starting countdown");
      mode = 3;
      start_timer0(3000);
    } else if (mode == 3 && counter0_done == 1) {
      // countdown reached -> switch to sleep mode
      stop_timer0();
      counter0_done = 0; // reset counter flag
      mode = 0;
      DL("sleep mode");
    }

  }
  return 0;
}
