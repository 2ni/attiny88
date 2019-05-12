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


volatile uint8_t count;
ISR(WDT_vect) {
  sleep_disable();
  PORTB ^= _BV(LED_R);

  // stop watchdog
  if (++count == 8) {
    WDTCSR &= ~_BV(WDIE);
  }
  sleep_enable();
}

int main(void) {

  DINIT();
  DL("Hello.");

  led_setup();
  count = 0;

  MCUSR &= ~_BV(WDRF); // clear wdt reset flag
  WDTCSR |= (_BV(WDCE) | _BV(WDE));  // Enable the WD Change Bit
  WDTCSR = _BV(WDIE) | // Enable WDT Interrupt
  _BV(WDP2) | _BV(WDP1); // Set Timeout to ~1 seconds

  power_adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  sei();

  while (1) {
    DL("going to sleep");
    sleep_mode();
  }
}
