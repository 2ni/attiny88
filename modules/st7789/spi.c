/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

    http://www.tinkerer.eu/AVRLib/SPI/
    http://maxembedded.com/2013/11/the-spi-of-the-avr/
*/

#include "spi.h"
#include "uart.h"
#include "def.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/*
 * initialize pins for spi communication
 * TODO pass mode:
 * 0: CPOL = 0, CPHA = 0
 * 1: CPOL = 0, CPHA = 1
 * 2: CPOL = 1, CPHA = 0
 * 3: CPOL = 1, CPHA = 1
 */
void spi_init(uint8_t mode) {
  uint8_t cpol = 0, cpha = 0;
  if (mode == 1) {
    cpha = 1;
  } else if (mode == 2) {
    cpol = 1;
  } else if (mode == 3) {
    cpol = 1; cpha = 1;
  }

  // DDR_SPI &= ~((1<<MOSI)|(1<<MISO)|(1<<SS)|(1<<SCK));
  DDR_SPI |= ((1<<MOSI)|(1<<SS)|(1<<SCK)); // outputs
  PORT_SPI |= (1<<SS);   // set high (disabled)

  SPCR = ((1<<SPE)|                    // enable spi
    (0<<SPIE)|                         // no spi interrupt
    (1<<MSTR)|                         // master
    (0<<DORD)|                         // Data Order (0:MSB first / 1:LSB first)
    (1<<SPI2X)|(0<<SPR1)|(0<<SPR0)|   // clock rate /2
    (cpol<<CPOL)|                      // Clock polarity (0:SCK low / 1:SCK hi when idle)
    (cpha<<CPHA));                     // Clock phase (0:leading / 1:trailing edge sampling)

  SPSR = (1<<SPI2X);        // clock rate x2 (fosc/8, = 1MHz)
}

/*
 * shift full array through target device
 * and return received data
 */
void spi_transfer_sync (uint8_t *dataout, uint8_t *datain, uint8_t len) {
  uint8_t i;
  for (i = 0; i < len; i++) {
    SPDR = dataout[i];
    while (!(SPSR & (1<<SPIF)));
    datain[i] = SPDR;
  }
}

/*
 * send full array ignore reception
 */
void spi_transmit_sync (uint8_t *dataout, uint8_t len) {
  uint8_t i;
  for (i = 0; i < len; i++) {
    SPDR = dataout[i];
    while (!(SPSR & (1<<SPIF)));
  }
}

/*
 * send one byte to target device and return the received one
 */
uint8_t spi_fast_shift (uint8_t data) {
  SPDR = data;
  while(!(SPSR & (1<<SPIF)));
  return SPDR;
}
