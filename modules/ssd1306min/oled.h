#ifndef OLED_h
#define OLED_h

#define OLED_ADDR 0x78  //(0x3C*2)

#include <avr/pgmspace.h>

extern uint8_t oled_init_cmds[];
extern uint8_t oled_init_transfer_cmds[];

uint8_t current_line, current_col;

void oled_clear();
void oled_start_data();
void oled_setup();
void oled_start_cmd();
void oled_cmd (uint8_t cmd);
void oled_init_transfer();
void oled_set_line(int8_t line);
void oled_set_col(uint8_t col);
void oled_set_pos(uint8_t line, uint8_t col);
void oled_char(char cc);
void oled_text(const char str[]);
void oled_dot(uint8_t xx, uint8_t yy);
void oled_ddot(uint8_t xx, uint8_t yy);
void oled_hline(uint8_t yy);
void oled_vline(uint8_t xx);


void oled_doubleh();
void oled_simpleh();

#define  Sprite(tt) \
  WrStaData(); \
  for (uint8_t i=0; i<sizeof tt; i++) { \
  Write (pgm_read_byte(&tt[i])); } Stop()
#define MySprite(tt) \
  WrStaData(); \
  for (uint8_t i=0; i<sizeof tt; i++) { \
  Write (tt[i]); } Write(0); Stop();

#endif
