#ifndef _AES_H_
  #define _AES_H_

  #include <stdint.h>
  #include <string.h> // needed for memcpy
  #include <avr/pgmspace.h>
  #include "keys.h"

  static const unsigned char PROGMEM S_Table[16][16];
  static const unsigned char nw_k_skey[16];
  static const unsigned char app_skey[16];
  static const unsigned char dev_addr[4];

  void aes_encrypt_payload(unsigned char *data, uint8_t length, uint8_t frame_counter, uint8_t direction);
  void aes_calculate_mic(unsigned char *data, unsigned char *mic, uint8_t length, uint8_t frame_counter, uint8_t direction);
  void aes_encrypt(unsigned char *data, const unsigned char *key);
  void aes_generate_keys(unsigned char *k1, unsigned char *k2);
  void aes_calculate_round_key(unsigned char round, unsigned char *round_key);
  void aes_add_round_key(unsigned char *round_key, unsigned char (*State)[4]);
  void aes_shift_rows(unsigned char (*State)[4]);
  void aes_mix_columns(unsigned char (*State)[4]);
  void aes_shift_left(unsigned char *data);
  void aes_xor(unsigned char *new_data, unsigned char *old_data);
  unsigned char aes_sub_byte(unsigned char byte);

#endif
