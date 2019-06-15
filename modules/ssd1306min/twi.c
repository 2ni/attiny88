#include "twi.h"

void twi_setup() {
  TWSR = 1;  // 0 for 400kHz TODO
  TWBR = 0x0C; // (F_CPU/SCL_CLOCK-16)/2 = (8Mhz/200kHz-16)/2 = 12 = 0x0C
  TWCR = (1<<TWEN);
}

void  twi_start () {
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);  // start
  while (!(TWCR & _BV(TWINT))) {}
}

void  twi_stop () {
  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);  // stop
  for (volatile int i=0; i<20; i++) ;
}

void  twi_write (uint8_t data) {
  TWDR = data;
  TWCR = _BV(TWINT) | _BV(TWEN);
  while (!(TWCR & _BV(TWINT))) {}
  // status = TWSR & 0xF8;  // Ack bit
}

uint8_t twi_read_ack () {
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
  while (!(TWCR & _BV(TWINT))) {}
  return TWDR;
}

uint8_t twi_read_nck () {
  TWCR = _BV(TWINT) | _BV(TWEN);
  while (!(TWCR & _BV(TWINT))) {}
  return TWDR;
}


