/**
 * use RFM95 to send data to thethingsnetwork.org (ttn)
 * SPI in master mode is needed
 * tinySPI is not working because it depends on USI (ATtiny88 has no USI)
 *
 */
#include "ttn.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#include "avr_print.h"
#include "uart.h"
#include "def.h"

int main(void) {
  DINIT(); // simplex uart setup
  DL("Hello there");

  led_setup();
  rfm_init();

  uint8_t data[2];
  uint8_t length = 2;
  uint16_t frame_counter = 0;

  data[0] = 13; // lsb
  data[1] = 5;  // msb

  while (1) {
    led_toggle('r');
    lora_send_data(data, length, frame_counter);
    frame_counter++;

    _delay_ms(500);
  }

  return 0;
}
