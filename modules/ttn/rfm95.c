/**
 * with help from https://www.thethingsnetwork.org/forum/t/attiny85-rfm95-temperature-sensor/11211
 */

#include "rfm95.h"

void rfm_init() {
  spi_init();
  rfm_write(0x01, 0x00); // switch rfm to sleep
  rfm_write(0x01, 0x80); // LoRa mode
  rfm_write(0x01, 0x81); // standby mode wait on mode ready

  _delay_ms(10);

  rfm_write(0x06, 0xD9); // set carrier frequency: 868.100MHz / 61.035Hz = 14222987 = 0xD9068B
  rfm_write(0x07, 0x06);
  rfm_write(0x08, 0x8B);

  rfm_write(0x09, 0xFF); // max power
  rfm_write(0x1D, 0x72); // BW = 125kHz, coding rate 4/5, explicit header mode
  rfm_write(0x1E, 0xB4); // spreading factor 7, payloadcrc on
  rfm_write(0x1F, 0x25); // rx timeout to 37 symbols
  rfm_write(0x20, 0x00); // preample length to 8 symbols (0x0008 + 4 = 12)
  rfm_write(0x21, 0x08);
  rfm_write(0x26, 0x0C); // low datarate optimization off, AGC auto on
  rfm_write(0x39, 0x34); // set LoRa sync word
  rfm_write(0x33, 0x27); // set IQ to normal values
  rfm_write(0x3B, 0x1D);
  rfm_write(0x0E, 0x80); // set FIFO pointers, TX base addr
  rfm_write(0x0F, 0x00); // RX base addr
  rfm_write(0x01, 0x00); // swtich rfm to sleep
}

void rfm_write(unsigned char addr, unsigned char data) {
  DDR_SPI |= (1 << DD_SS);            // nss low starts communication
  unsigned char cmd = addr | 0x80;
  spi_transmit_sync(&cmd, 1);         // send addr with msb 1 to make it a write cmd
  spi_transmit_sync(&data, 1);        // send data
  DDR_SPI &= ~(1 << DD_SS);          // nss high ends communication
}

void rfm_send_package(unsigned char *package, uint8_t length) {
  rfm_write(0x01, 0x81); // standby mode wait on mode ready
  _delay_ms(10);

  rfm_write(0x40, 0x40); // switch DIO0 (PD1) to TX done
  rfm_write(0x06, 0xD9); // set carrier frequency: 868.100MHz / 61.035Hz = 14222987 = 0xD9068B
  rfm_write(0x07, 0x06);
  rfm_write(0x08, 0x8B);

  rfm_write(0x1E, 0x74); // SF7 CRC on
  rfm_write(0x1D, 0x72); // 125kH, coding rate 4/5, explicit header mode
  rfm_write(0x26, 0x04); // low datarate optimization off, AGC auto on

  rfm_write(0x33, 0x27); // set IQ to normal values
  rfm_write(0x3B, 0x1D);

  rfm_write(0x22, length); // set payload length

  rfm_write(0x0D, 0x80); // set fifo location according to RFM95 specs

  // write payload to fifo
  for(uint8_t i=0; i<length; i++) {
    rfm_write(0x00, *package);
    package++;
  }

  rfm_write(0x01, 0x83); // switch rfm95 to TX

  // wait for TX to be done (DIO0 == LOW)
  while(!(PORT_DIO0 & (1<<DD_DIO0))) {
  }

  rfm_write(0x01, 0x00); // switch rfm95 to sleep
}

void lora_send_data(unsigned char *data, uint8_t length, unsigned int frame_counter) {
  unsigned char direction = 0;
  unsigned char rfm_data[64];
  unsigned char rfm_package_length;
  unsigned char mic[4];
  unsigned char mac_header = 0x40; // unconfirmed data up
  unsigned char frame_control = 0;
  unsigned char frame_port = 0x01;

  aes_encrypt_payload(data, length, frame_counter, direction);

  // buidl radio package
  rfm_data[0] = mac_header;
  rfm_data[1] = dev_addr[3];
  rfm_data[2] = dev_addr[2];
  rfm_data[3] = dev_addr[1];
  rfm_data[4] = dev_addr[0];
  rfm_data[5] = frame_control;
  rfm_data[6] = (frame_counter & 0x00FF);
  rfm_data[7] = ((frame_counter>>8) & 0x00FF);
  rfm_data[8] = frame_port;

  rfm_package_length = 9;

  // load data
  for (uint8_t i=0; i<length; i++) {
    rfm_data[rfm_package_length+i] = data[i];
  }

  rfm_package_length = rfm_package_length+ length;
  aes_calculate_mic(rfm_data, mic, rfm_package_length, frame_counter, direction);

  // load mic in package
  for(uint8_t i=0; i<4; i++) {
    rfm_data[rfm_package_length+i] = mic[i];
  }

  // add mic length to rfm_package_length
  rfm_package_length = rfm_package_length+4;

  rfm_send_package(rfm_data, length);
}
