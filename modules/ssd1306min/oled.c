#include <avr/pgmspace.h>
#include <string.h>

#include "oled.h"
#include "twi.h"
#include "fonts.h"

uint8_t oled_init_cmds[] = { 0xae, 0xd5, 0x80, 0xa8, 63, \
        0xD3, 0x0, 0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, \
        0xc8, 0xDA, 0x12, 0x81, 0xcf, 0xd9, 0xf1, 0xdb, \
        0x40, 0xa4, 0xa6, 0x2e, 0xaf };

uint8_t oled_init_transfer_cmds[] = { OLED_ADDR, 0, 0x21, 0x0, 0x7f, 0x22, 0x0, 0x7F };

void oled_start_data() {
  twi_start();
  twi_write(OLED_ADDR);
  twi_write(0x40);
}

void oled_clear() {  //nn<128x8=1024
  oled_init_transfer();
  oled_start_data();
  for (int i=0; i<1024; i++) {
    twi_write(0);
  }
  twi_stop();
}

void oled_setup() {
  for (uint8_t i=0; i<sizeof(oled_init_cmds); i++) {
    oled_start_cmd();
    twi_write(oled_init_cmds[i]);
    twi_stop();
    oled_start_data();
    twi_stop();
  }
  oled_clear();
}

void oled_start_cmd() {
  twi_start();
  twi_write(OLED_ADDR);
  twi_write(0);
}

void oled_cmd(uint8_t cmd) {
  oled_start_cmd();
  twi_write(cmd);
  twi_stop();
}

void oled_init_transfer () {
  for (uint8_t i=0; i<sizeof(oled_init_transfer_cmds); i++) {
    oled_start_cmd();
    twi_write(oled_init_transfer_cmds[i]);
    twi_stop();
  }
}

void oled_set_line(int8_t line) {
  current_line = line;
  line--;
  if (line < 0) {
    line = 7;
  }
  oled_start_cmd();
  twi_write(0x22);
  twi_write(line);
  twi_write(7);
  twi_stop();
}

void oled_set_col(uint8_t col) {
  oled_start_cmd();
  twi_write(0x21);
  twi_write(col);
  twi_write(127);
  twi_stop();
  current_col = col;
}

/*
 * line: 0-7
 * col: 0-124 (or 123 depending on char size)
 */
void oled_set_pos(uint8_t line, uint8_t col) {
  oled_set_line(line);
  oled_set_col(col);
//  ptMap = (128*current_line)+current_col;
}

void oled_char(char cc) {
  oled_start_data();
  for (uint8_t i=0; i<5; i++) {
    twi_write(pgm_read_byte(&font5x7[((cc-32)*5)+i])); // start at pos 32 ascii table, 5 bytes
  }
  twi_write(0);
  twi_stop();
}

/*
void oled_char(char cc) {
  cc &= 0x7F; // char 0-127
  switch (cc/32) {
    case 0:
      // next line
      if(cc==13) {
        oled_set_line(current_line+1);
        oled_set_col(0);
      }
      //else Error();
      break;
    case 1:  // codes 32-
      oled_start_data();
      for (uint8_t i=0; i<4; i++) {
        twi_write(pgm_read_byte(&taNum[((cc-32)*4)+i])); // start at pos 32 ascii table, 4 bytes
      }
      twi_write(0);
      twi_stop();
      break;
    case 2:  // codes 64-
      oled_start_data();
      for (uint8_t i=0; i<5; i++) {
        twi_write(pgm_read_byte(&taMaj[((cc-64)*5)+i])); // start at pos 64 ascii table, 5 bytes
      }
      twi_write(0);
      twi_stop();
      break;
    case 3:  //codes 96-
      oled_start_data();
      for (uint8_t i=0; i<4; i++) {
        twi_write(pgm_read_byte(&taMin[((cc-96)*4)+i]));
      }
      twi_write(0);
      twi_stop();
      break;
  }
}
*/

void oled_text(const char str[]) {
  for (uint8_t i=0; i< strlen(str); i++) {
    oled_char(str[i]);
  }
}

// yy 0-64 -> 0-7
void oled_dot(uint8_t xx,uint8_t yy) {
  oled_set_pos(yy/8,xx);
  oled_start_data();
  twi_write(1<<yy%8); // deletes all other bits
  twi_stop();
}

//ddot  2points superposés si (yy/8!=7)
// si =7 il faut agit sur le bit0 de la ligne suiv si !=7
// yy0-64 --> 0-7
void oled_ddot(uint8_t xx, uint8_t yy) {
  uint8_t tmp=(1<<yy%8);
  if (yy%8!=7) {
    tmp+=(1<<(yy%8+1));
  }
  oled_set_pos(yy/8, xx);
  oled_start_data ();
  twi_write(tmp);
  twi_stop();
}

void oled_hline(uint8_t yy) {
  for (uint8_t i=0; i<128; i++) {
    oled_dot(i,yy);
  }
}

void oled_vline (uint8_t xx) {
  for (uint8_t i=0; i<8; i++) {
    oled_set_pos(i, xx);
    oled_start_data ();
    twi_write(0xFF);
    twi_stop();
  }
}

void oled_doubleh() {
  oled_cmd(0xda);
  oled_cmd(0x02);
}

void oled_simpleh() {
  oled_cmd(0xda);
  oled_cmd(0x12);
}
