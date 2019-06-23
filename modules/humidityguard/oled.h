#ifndef OLED_h
#define OLED_h

#define OLED_ADDR 0x78  //(0x3C*2)

#include <avr/pgmspace.h>

extern uint8_t oled_init_cmds[];
extern uint8_t oled_init_transfer_cmds[];

uint8_t current_line, current_col;

void oled_clear();
void oled_on();
void oled_off();
void oled_start_data();
void oled_setup();
void oled_start_cmd();
void oled_cmd (uint8_t cmd);
void oled_init_transfer();
void oled_set_line(int8_t line);
void oled_set_col(uint8_t col);
void oled_set_pos(uint8_t line, uint8_t col);
uint8_t oled_char_small(char cc);
uint8_t oled_char_large(char cc);
void oled_clear_part(uint8_t width, uint8_t line, uint8_t col, char size);
void oled_sprite(uint8_t cc[]);
uint8_t oled_text(const char str[], char size);
void oled_dot(uint8_t xx, uint8_t yy);
void oled_ddot(uint8_t xx, uint8_t yy);
void oled_hline(uint8_t yy);
void oled_vline(uint8_t xx);
void oled_doubleh();
void oled_simpleh();

// used as define as can't pass pointer of array and use sizeof
#define oled_sprite(tt) \
  oled_start_data(); \
  for (uint8_t i=0; i<sizeof tt; i++) { \
  twi_write(pgm_read_byte(&tt[i])); } twi_stop()

/*
#define MySprite(tt) \
  oled_start_data(); \
  for (uint8_t i=0; i<sizeof tt; i++) { \
  Write (tt[i]); } Write(0); Stop();
*/

#endif
