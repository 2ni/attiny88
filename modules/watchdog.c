/*
 * based on https://electronics.stackexchange.com/questions/74840/use-avr-watchdog-like-normal-isr
 * power consumption when sleeping: 11.5uA
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "def.h"
#include "uart.h"

void deep_sleep() {
  cli();
  // clear all prescale bits 1st
  uint8_t prescale = 0x00;
  prescale &= ~_BV(WDP3) | ~_BV(WDP2) | ~_BV(WDP1) | ~_BV(WDP0);
  prescale |= _BV(WDP2) | _BV(WDP0);


  MCUSR &= ~_BV(WDRF); // clear wdt reset flag
  WDTCSR |= (_BV(WDCE) | _BV(WDE));  // enable WD change bit
  WDTCSR = _BV(WDIE) | // enable wdt interrupt
  prescale; // timeout

  power_adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();

  DL("going to sleep");
  sleep_mode(); // sleep baby, sleep!
  DL("I'm back");

  sleep_disable();
  power_adc_enable();
}

volatile uint8_t count;
ISR(WDT_vect) {
  sleep_disable();
  PORTB ^= _BV(LED_R);

  count++;

  // stop watchdog
  WDTCSR &= ~_BV(WDIE);

  sleep_enable();
}

int main(void) {

  DINIT();
  DL("Hello.");

  led_setup();
  count = 0;

  sei();

  while (1) {
    if (count<8) {
      DF("count: %u", count);
      deep_sleep();
    }
  }
}
