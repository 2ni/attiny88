/*
 * oled based on
 * https://github.com/nicoud/Oled
 *
 * convert -resize 128x64\! -monochrome 1.png 1.bmp
 * then use http://javl.github.io/image2cpp/ or
 * http://en.radzio.dxp.pl/bitmap_converter/
 * to convert image
 *
 * or use http://dotmatrixtool.com and https://fontdrop.info/ to generate it
 */
#include <avr/io.h>

#include "def.h"
#include "uart.h"

#include "twi.h"
#include "oled.h"
#include "font_small.h" // needed for squar

int main(void) {
  DINIT(); // enable debug output
  DL("\n\nHello there");
  oled_setup();
  oled_set_pos(0, 0);
  oled_text("humidity", 's');
  oled_set_pos(1, 0);
  oled_text("20.3%", 'l');

  oled_set_pos(3, 0);
  oled_text("a", 's');
  oled_set_pos(3, 122);
  oled_text("a", 's');
  oled_set_pos(4, 0);
  oled_sprite(squar);

  for(uint8_t i=0; i<40; i++) {
    oled_ddot(40+i,40);
  }

  return 0;
}
