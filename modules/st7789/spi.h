#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>
#include <avr/io.h>

extern void spi_init(uint8_t mode);
extern void spi_transfer_sync (uint8_t * dataout, uint8_t * datain, uint8_t len);
extern void spi_transmit_sync (uint8_t * dataout, uint8_t len);
extern uint8_t spi_fast_shift (uint8_t data);

#endif
