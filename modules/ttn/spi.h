#ifndef _SPI_H_
  #define _SPI_H_

  #include <stdint.h>
  #include <avr/io.h>

  #define PORT_SPI    PORTB
  #define DDR_SPI     DDRB
  #define DD_MISO     DDB4
  #define DD_MOSI     DDB3
  #define DD_SCK      DDB5

  // RFM95 specific
  #define DD_SS       DDB2

  #define DDR_DIO0    DDRD
  #define PORT_DIO0   PORTD
  #define DD_DIO0     DDD1

  extern void spi_init();
  extern void spi_transfer_sync (uint8_t * dataout, uint8_t * datain, uint8_t len);
  extern void spi_transmit_sync (uint8_t * dataout, uint8_t len);
  extern uint8_t spi_fast_shift (uint8_t data);

#endif
