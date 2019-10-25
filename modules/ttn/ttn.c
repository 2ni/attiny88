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

  // needs to be stored consistently see https://www.thethingsnetwork.org/docs/lorawan/security.html
  uint16_t frame_counter = 0;

  data[0] = 13; // lsb
  data[1] = 5;  // msb

  uint16_t counter;
  while (1) {
    counter = 0;
    led_toggle('r');
    lora_send_data(data, length, frame_counter);
    frame_counter++;

    // wait 5min
    while(counter++ < 600) {
      _delay_ms(500);
    }
  }

  return 0;
}
