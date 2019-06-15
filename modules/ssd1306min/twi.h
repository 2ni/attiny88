#ifndef TWI_h
#define TWI_h

#include <avr/io.h>

void twi_setup();
void twi_start();
void twi_stop();
void twi_write(uint8_t data);
uint8_t twi_read_ack();
uint8_t twi_read_nck();

#endif
