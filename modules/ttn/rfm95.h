#ifndef _RFM95_H_
  #define _RFM95_H_

  #include <stdint.h>
  #include <util/delay.h>
  #include "spi.h"
  #include "aes.h"

  void rfm_init(void);
  void rfm_write(unsigned char addr, unsigned char data);
  void rfm_send_package(unsigned char *package, uint8_t length);
  void lora_send_data(unsigned char *data, uint8_t length, unsigned int counter);

#endif
