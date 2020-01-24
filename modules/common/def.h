#ifndef __DEF_H__
#define __DEF_H__

#include <avr/io.h>

// led's
#define PORT_G         PORTA
#define DDR_G          DDRA
#define LED_G          PA3

#define PORT_R         PORTB
#define DDR_R          DDRB
#define LED_R          PB6

#define PORT_B         PORTB
#define DDR_B          DDRB
#define LED_B          PB7

// spi
#define PORT_SPI       PORTB
#define DDR_SPI        DDRB
#define MISO           PB4
#define MOSI           PB3
#define SCK            PB5

// rfm
#define SS             PB2

#define PORT_DIO0      PORTD
#define DDR_DIO0       DDRD
#define DIO0           PD1

// moisture
/*
#define MOIST_A       PD4      // 1touch, qtouch
#define MOIST_A_INT   PCINT20
*/

#define MOIST_A       PD0      // 1touch, qtouch
#define MOIST_A_INT   PCINT16
#define MOIST_B       PA2      // qtouch

// periphery
#define TEMP          PA0      // ADC6, 0°=0.81v, 10°=0.55v, 25°=0.3v, 50°=0.11v
#define BAT           PA1      // ADC7, battery (0.76v = 4.2v, 0.5v = 2.8v)
#define PWM           PB1      // buzzer
#define CLKO          PB0      // clock out
#define EN_LIGHT      PC1
#define LIGHT         PC0      // ADC0, dark: 1.03v, light: 0.03v

// current measurement
#define EN_CUR        PC7      // enable current measurement
#define CUR           PC2      // 2.15mA/bit, 2.2A=1.1v

// out
#define PORT_O        PORTC
#define DDR_O         DDRC
#define OUT           PC3

// i2c
#define SDA           PC4
#define SCL           PC5

// debug
// connect tx pin to debug
#define DBG_PORT    PORTD
#define DBG_DDR     DDRD
#define DBG         PD3

// touch and moisture sensor settings
#if defined (__AVR_ATtiny84__)
  #define TOUCH_TIMER0_VECT TIM0_COMPA_vect
  #define TOUCH_TIMER1_VECT TIM1_COMPA_vect
  #define INT_SETUP GIMSK
  #define INT_CLEAR GIFR
#elif defined (__AVR_ATtiny88__)
  #define TOUCH_TIMER0_VECT TIMER0_COMPA_vect
  #define TOUCH_TIMER1_VECT TIMER1_COMPA_vect
  #define INT_SETUP PCICR
  #define INT_CLEAR PCIFR
#endif

#define TOUCH_INT_VECT PCINT2_vect
#define TOUCH_PCIE PCIE2 // PCICR bit setting for PCINT16..23
#define TOUCH_PCIF PCIF2 // PCIFR bit setting for PCINT16..23
#define TOUCH_PCMSK PCMSK2 //PCMSK bit setting for PCINT16..23
#define TOUCH_PORT PORTD
#define TOUCH_DDR DDRD

#define TOUCH1 PD5
#define TOUCH1_INT PCINT21
#define TOUCH2 PD6
#define TOUCH2_INT PCINT22
#define TOUCH3 PD7
#define TOUCH3_INT PCINT23
#define TOUCH_THRESHOLD 50

inline static void led_setup() {
  DDR_G |= _BV(LED_G); // enable as output (set 1)
  PORT_G &= ~_BV(LED_G); // set low

  DDR_B |= _BV(LED_B); // enable as output (set 1)
  PORT_B &= ~_BV(LED_B); // set low

  DDR_R |= _BV(LED_R); // enable as output (set 1)
  PORT_R &= ~_BV(LED_R); // set low
}

inline static void out_setup() {
  DDR_O |= _BV(OUT); // define as output
  PORT_O &= ~_BV(OUT); // set low
}

inline static void out_on() {
  PORT_O |= _BV(OUT);
}

inline static void out_off() {
  PORT_G &= ~_BV(OUT);
}

inline static int out_is_on() {
  return PORT_G & _BV(OUT);
}

inline static void out_toggle() {
  PORT_O ^= _BV(OUT);
}

inline static void led_on(char color) {
  if (color == 'g') {
    PORT_G |= _BV(LED_G); // set high
  } else if (color == 'b') {
    PORT_B |= _BV(LED_B);
  } else if (color == 'r') {
    PORT_R |= _BV(LED_R);
  }
}

inline static void led_on_all() {
  led_on('r');
  led_on('g');
  led_on('b');
}

inline static int led_is_on(char color) {
  uint8_t val;
  if (color == 'g') {
    val = PORT_G & _BV(LED_G);
  } else if (color == 'b') {
    val = PORT_B & _BV(LED_B);
  } else if (color == 'r') {
    val = PORT_R & _BV(LED_R);
  }

  return val;
}

inline static void led_off(char color) {
  if (color == 'g') {
    PORT_G &= ~_BV(LED_G); // set low
  } else if (color == 'b') {
    PORT_B &= ~_BV(LED_B);
  } else if (color == 'r') {
    PORT_R &= ~_BV(LED_R);
  }
}

inline static void led_off_all() {
  led_off('r');
  led_off('g');
  led_off('b');
}

inline static void led_toggle(char color) {
  if (color == 'g') {
    PORT_G ^= _BV(LED_G); // invert (exclusive or)
  } else if (color == 'b') {
    PORT_B ^= _BV(LED_B);
  } else if (color == 'r') {
    PORT_R ^= _BV(LED_R);
  }
}

#endif
