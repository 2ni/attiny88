/**
 * adapted from https://www.thethingsnetwork.org/forum/t/attiny85-rfm95-temperature-sensor/11211
 *
 * network session key (used to check validity of message MIC check)
 * application session key
 * unique device address
 * OTAA: over the air activation
 * ABP: activation by personalization
 *
 * 1) use ABP, set network session key, app session key in console, frame_counter: 16bit
 * 2) device address is generated
 *
 * the frame_counter is not reset on ttn! Therefore the device must be re-registered
 * https://www.thethingsnetwork.org/docs/lorawan/security.html
 *
 *
 * include your keys in a file keys.h:
 * #ifndef _KEYS_H_
 *   #define _KEYS_H_
 *   static const unsigned char PROGMEM S_Table[16][16] = {
 *     {0x63,0x7C,0x77,0x7B,0xF2,0x6B,0x6F,0xC5,0x30,0x01,0x67,0x2B,0xFE,0xD7,0xAB,0x76},
 *     {0xCA,0x82,0xC9,0x7D,0xFA,0x59,0x47,0xF0,0xAD,0xD4,0xA2,0xAF,0x9C,0xA4,0x72,0xC0},
 *     {0xB7,0xFD,0x93,0x26,0x36,0x3F,0xF7,0xCC,0x34,0xA5,0xE5,0xF1,0x71,0xD8,0x31,0x15},
 *     {0x04,0xC7,0x23,0xC3,0x18,0x96,0x05,0x9A,0x07,0x12,0x80,0xE2,0xEB,0x27,0xB2,0x75},
 *     {0x09,0x83,0x2C,0x1A,0x1B,0x6E,0x5A,0xA0,0x52,0x3B,0xD6,0xB3,0x29,0xE3,0x2F,0x84},
 *     {0x53,0xD1,0x00,0xED,0x20,0xFC,0xB1,0x5B,0x6A,0xCB,0xBE,0x39,0x4A,0x4C,0x58,0xCF},
 *     {0xD0,0xEF,0xAA,0xFB,0x43,0x4D,0x33,0x85,0x45,0xF9,0x02,0x7F,0x50,0x3C,0x9F,0xA8},
 *     {0x51,0xA3,0x40,0x8F,0x92,0x9D,0x38,0xF5,0xBC,0xB6,0xDA,0x21,0x10,0xFF,0xF3,0xD2},
 *     {0xCD,0x0C,0x13,0xEC,0x5F,0x97,0x44,0x17,0xC4,0xA7,0x7E,0x3D,0x64,0x5D,0x19,0x73},
 *     {0x60,0x81,0x4F,0xDC,0x22,0x2A,0x90,0x88,0x46,0xEE,0xB8,0x14,0xDE,0x5E,0x0B,0xDB},
 *     {0xE0,0x32,0x3A,0x0A,0x49,0x06,0x24,0x5C,0xC2,0xD3,0xAC,0x62,0x91,0x95,0xE4,0x79},
 *     {0xE7,0xC8,0x37,0x6D,0x8D,0xD5,0x4E,0xA9,0x6C,0x56,0xF4,0xEA,0x65,0x7A,0xAE,0x08},
 *     {0xBA,0x78,0x25,0x2E,0x1C,0xA6,0xB4,0xC6,0xE8,0xDD,0x74,0x1F,0x4B,0xBD,0x8B,0x8A},
 *     {0x70,0x3E,0xB5,0x66,0x48,0x03,0xF6,0x0E,0x61,0x35,0x57,0xB9,0x86,0xC1,0x1D,0x9E},
 *     {0xE1,0xF8,0x98,0x11,0x69,0xD9,0x8E,0x94,0x9B,0x1E,0x87,0xE9,0xCE,0x55,0x28,0xDF},
 *     {0x8C,0xA1,0x89,0x0D,0xBF,0xE6,0x42,0x68,0x41,0x99,0x2D,0x0F,0xB0,0x54,0xBB,0x16}
 *   };
 *   static const unsigned char nw_k_skey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
 *   static const unsigned char app_skey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
 *   static const unsigned char dev_addr[4] = { 0x00, 0x00, 0x00, 0x00 };
 * #endif
 *
 */

#include "aes.h"

#if !defined(_KEYS_H_)
#warning "pls define your keys in keys.h"
#endif

void aes_encrypt_payload(unsigned char *data, uint8_t length, uint8_t frame_counter, uint8_t direction) {
  unsigned char number_of_blocks = 0;
  unsigned char incomplete_block_size = 0;
  unsigned char block_a[16];
  unsigned char i;

  // calculate number of blocks
  number_of_blocks = length/17;
  incomplete_block_size = length%16;
  if (incomplete_block_size != 0) number_of_blocks++;

  for (i=1; i<=number_of_blocks; i++) {
    block_a[0] = 0x01;
    block_a[1] = 0x00;
    block_a[2] = 0x00;
    block_a[3] = 0x00;
    block_a[4] = 0x00;
    block_a[5] = direction;
    block_a[6] = dev_addr[3];
    block_a[7] = dev_addr[2];
    block_a[8] = dev_addr[1];
    block_a[9] = dev_addr[0];
    block_a[10] = (frame_counter & 0x00FF);
    block_a[11] = ((frame_counter >> 8) & 0x00FF);
    block_a[12] = 0x00; //Frame counter upper Bytes
    block_a[13] = 0x00;
    block_a[14] = 0x00;
    block_a[15] = i;

    aes_encrypt(block_a, app_skey);

    // verify last block
    if (i != number_of_blocks) {
      for (uint8_t j=0; j<16; j++) {
        *data = *data ^block_a[j];
        data++;
      }
    } else {
      if (incomplete_block_size == 0) {
        incomplete_block_size = 16;
      }
      for (uint8_t j=0; j<incomplete_block_size; j++) {
        *data = *data ^ block_a[j];
        data++;
      }
    }
  }
}

/**
 * Calculate_MIC
 */
void aes_calculate_mic(unsigned char *data, unsigned char *mic, uint8_t length, uint8_t frame_counter, uint8_t direction) {
  unsigned char block_b[16];

  unsigned char key_k1[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  unsigned char key_k2[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  unsigned char old_data[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  unsigned char new_data[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };


  unsigned char number_of_blocks = 0x00;
  unsigned char incomplete_block_size = 0x00;
  unsigned char block_counter = 0x01;

  //Create block_b
  block_b[0] = 0x49;
  block_b[1] = 0x00;
  block_b[2] = 0x00;
  block_b[3] = 0x00;
  block_b[4] = 0x00;

  block_b[5] = direction;

  block_b[6] = dev_addr[3];
  block_b[7] = dev_addr[2];
  block_b[8] = dev_addr[1];
  block_b[9] = dev_addr[0];

  block_b[10] = (frame_counter & 0x00FF);
  block_b[11] = ((frame_counter >> 8) & 0x00FF);

  block_b[12] = 0x00; //Frame counter upper bytes
  block_b[13] = 0x00;

  block_b[14] = 0x00;
  block_b[15] = length;

  // calculate number of blocks and blocksize of last block
  number_of_blocks = length/16;
  incomplete_block_size = length%16;

  if (incomplete_block_size != 0) number_of_blocks++;

  aes_generate_keys(key_k1, key_k2);
  aes_encrypt(block_b, nw_k_skey);

  // copy block_b to old_data
  for(uint8_t i=0; i<16; i++) {
    old_data[i] = block_b[i];
  }

  // perform full calculation until n-1 message blocks
  while (block_counter<number_of_blocks) {
    // copy data into array
    for(uint8_t i=0; i<16; i++) {
      new_data[i] = *data;
      data++;
    }

    // perform xor with old_data
    aes_xor(new_data, old_data);
    aes_encrypt(new_data, nw_k_skey);

    // copy new_data to old_data
    for(uint8_t i=0; i<16; i++) {
      old_data[i] = new_data[i];
    }

    block_counter++;
  }

  // perform calculation on last block
  // check if length multiple of 16
  if (incomplete_block_size == 0) {
    // copy last data into array
    for(uint8_t i=0; i<16; i++) {
      new_data[i] = *data;
      data++;
    }

    aes_xor(new_data, key_k1);
    aes_xor(new_data, old_data);
    aes_encrypt(new_data, nw_k_skey);
  } else {
    // copy the remaining data and fill up the rest
    for (uint8_t i=0; i<16; i++) {
      if (i<incomplete_block_size) {
        new_data[i] = *data;
        data++;
      }
      if (i==incomplete_block_size) {
        new_data[i] = 0x80;
      }
      if (i>incomplete_block_size) {
        new_data[i] = 0x00;
      }
    }
    aes_xor(new_data, key_k2);
    aes_xor(new_data, old_data);
    aes_encrypt(new_data, nw_k_skey);
  }

  for(uint8_t i=0; i<4; i++) {
    mic[i] = new_data[i];
  }
}

/**
 * AES_Encrypt
 * TODO: do not predefine round, but call on 2nd aes_calculate_round_key(10, round_key)
 */
void aes_encrypt(unsigned char *data, const unsigned char *key) {
  unsigned char round = 0;
  unsigned char round_key[16];
  unsigned char state[4][4];

  // copy input to state
  for (uint8_t column=0; column<4; column++) {
    for (uint8_t row=0; row<4; row++) {
      state[row][column] = data[row+(column<<2)];;
    }
  }

  // copy key to round_key
  memcpy(&round_key[0], &round_key[0], 16);

  // add round_key
  aes_add_round_key(round_key, state);

  // preform 9 ful rounds with mixed columns
  for (round=1; round<10; round++) {
    // perform byte substitution with S_Table
    for (uint8_t column=0; column<4; column++) {
      for (uint8_t row=0; row<4; row++) {
        state[row][column] = aes_sub_byte(state[row][column]);
      }
    }
    // perform row shift
    aes_shift_rows(state);

    // mix columns
    aes_mix_columns(state);

    // calculate new round key
    aes_calculate_round_key(round, round_key);

    // add the round key to the state
    aes_add_round_key(round_key, state);
  }

  // perform byte substitution with S_Table without mixiing columns
  for (uint8_t column=0; column<4; column++) {
    for (uint8_t row=0; row<4; row++) {
      state[row][column] = aes_sub_byte(state[row][column]);
    }
  }

  // shift rows
  aes_shift_rows(state);

  // calculate new round key
  aes_calculate_round_key(round, round_key);

  // add round key to the state
  aes_add_round_key(round_key, state);

  // copy the state into the data array
  for (uint8_t column=0; column<4; column++) {
    for(uint8_t row=0; row<4; row++) {
      data[row+(column<<2)] = state[row][column];
    }
  }
}

/**
 * Generate_Keys
 */
void aes_generate_keys(unsigned char *k1, unsigned char *k2) {
  unsigned char msb_key = 0;

  // encrypt the zeros in k1 with nw_k_skey
  aes_encrypt(k1, nw_k_skey);

  if ((k1[0] & 0x80) == 0x80) msb_key = 1;

  aes_shift_left(k1);

  if (msb_key == 1) k1[15] = k1[15]^0x87;

  // copy k1 to k2
  for (uint8_t i=0; i<16; i++) {
    k2[i] = k1[i];
  }

  msb_key = 0;
  if ((k2[0] & 0x80) == 0x80) msb_key = 1;

  aes_shift_left(k2);

  if (msb_key == 1) k2[15] = k2[15] ^ 0x87;
}

/**
 * AES_Calculate_Round_Key
 */
void aes_calculate_round_key(unsigned char round, unsigned char *round_key) {
  uint8_t b, r_con;
  uint8_t temp[4];
  r_con = 0x01;
  while (round != 1) {
    b = r_con & 0x80;
    r_con = r_con<<1;

    if (b==0x80) {
      r_con ^= 0x1b;
    }
    round--;
  }
  // calculate first temp
  // copy laste byte from previous key and subsitute the byte, but shift the array contents around by 1.
  temp[0] = aes_sub_byte(round_key[12 + 1]);
  temp[1] = aes_sub_byte(round_key[12 + 2]);
  temp[2] = aes_sub_byte(round_key[12 + 3]);
  temp[3] = aes_sub_byte(round_key[12 + 0]);

  temp[0] ^= r_con;

  for (uint8_t i=0; i<4; i++) {
    for (uint8_t j=0; j<4; j++) {
      round_key[j+(i<<2)] ^= temp[j];
      temp[j] = round_key[j+(i<<2)];
    }
  }
}

/**
 * AES_Add_Round_Key
 */
void aes_add_round_key(unsigned char *round_key, unsigned char (*state)[4]) {
  for (uint8_t column=0; column<4; column++) {
    for (uint8_t row=0; row<4; row++) {
      state[row][column] ^= round_key[row + (column<<2)];
    }
  }
}

/**
 * AES_Shift_Rows
 */
void aes_shift_rows(unsigned char (*state)[4]) {
  unsigned char buffer;

  //Store firt byte in buffer
  buffer      = state[1][0];
  //Shift all bytes
  state[1][0] = state[1][1];
  state[1][1] = state[1][2];
  state[1][2] = state[1][3];
  state[1][3] = buffer;

  buffer      = state[2][0];
  state[2][0] = state[2][2];
  state[2][2] = buffer;
  buffer      = state[2][1];
  state[2][1] = state[2][3];
  state[2][3] = buffer;

  buffer      = state[3][3];
  state[3][3] = state[3][2];
  state[3][2] = state[3][1];
  state[3][1] = state[3][0];
  state[3][0] = buffer;
}

/**
 * AES_Mix_Columns
 */
void aes_mix_columns(unsigned char (*state)[4]) {
  unsigned char a[4], b[4];

  for(uint8_t column=0; column<4; column++) {
    for(uint8_t row=0; row<4; row++) {
      a[row] = state[row][column];
      b[row] = (state[row][column]<<1);

      if ((state[row][column] & 0x80) == 0x80) b[row] ^= 0x1B;
    }
    state[0][column] = b[0]^a[1]^b[1]^a[2]^a[3];
    state[1][column] = a[0]^b[1]^a[2]^b[2]^a[3];
    state[2][column] = a[0]^a[1]^b[2]^a[3]^b[3];
    state[3][column] = a[0]^b[0]^a[1]^a[2]^b[3];
  }
}

/**
 * Shift_Left
 * TODO: simplify by setting overflow = 0 in loop and remove else's
 */
void aes_shift_left(unsigned char *data) {
  unsigned char overflow = 0;
  for (uint8_t i=0; i<16; i++) {
    // check for overflow on next byte except for last byte
    if (i<15) {
      // check if upper bit is one
      if ((data[i+1] & 0x80) == 0x80) {
        overflow = 1;
      } else {
        overflow = 0;
      }
    } else {
      overflow = 0;
    }
    // shift one left
    data[i] = (data[i] << 1) + overflow;
  }
}

/**
 * XOR
 */
void aes_xor(unsigned char *new_data, unsigned char *old_data) {
  for (uint8_t i=0; i<16; i++) {
    new_data[i] = new_data[i] ^ old_data[i];
  }
}

/**
 * AES_Sub_Byte
 *
 * S_Row    = ((Byte >> 4) & 0x0F);
 * S_Collum = ((Byte >> 0) & 0x0F);
 * S_Byte   = S_Table [S_Row][S_Collum];
 */
unsigned char aes_sub_byte(unsigned char byte) {
  return pgm_read_byte(&(S_Table[((byte>>4) & 0x0F)][((byte>>0) & 0x0F)]));
}
