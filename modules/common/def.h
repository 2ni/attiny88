#ifndef __DEF_H__
#define __DEF_H__

#define DDR_G DDRA
#define PORT_G PORTA
#define LED_G PA3

#define DDR_R DDRB
#define PORT_R PORTB
#define LED_R PB6

#define DDR_B DDRB
#define PORT_B PORTB
#define LED_B PB7

// set tx pin to debug
#define DEBUG_PIN PD3
#define DEBUG_PORT PORTD
#define DEBUG_DDR DDRD

// touch and moisture sensor settings
#if defined (__AVR_ATtiny84__)
  #define TOUCH_TIMER_VECT TIM1_COMPA_vect
  #define TOUCH_INT_VECT PCINT2_vect
  #define INT_SETUP GIMSK
  #define INT_CLEAR GIFR
#elif defined (__AVR_ATtiny88__)
  #define TOUCH_TIMER_VECT TIMER1_COMPA_vect
  #define TOUCH_INT_VECT PCINT2_vect
  #define INT_SETUP PCICR
  #define INT_CLEAR PCIFR
#endif

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

#define MOIST_A PD4   // 1touch, qtouch
#define MOIST_A_INT  PCINT20

#define MOIST_B PA2   // qtouch

#define TEMP PA0      // 0°=0.81v, 10°=0.55v, 25°=0.3v, 50°=0.11v
#define BAT PA1       // battery (0.76v = 4.2v, 0.5v = 2.8v)
#define PWM PB1       // buzzer
#define CLKO PB0      // clock out
#define EN_LIGHT PC1
#define LIGHT PC0     // dark: 1.03v, light: 0.03v

#define EN_CUR PC7    // enable current measurement
#define CUR PC2       // 2.15mA/bit, 2.2A=1.1v
#define OUT PC3

#define SDA PC4
#define SCL PC5

inline static void led_setup() {
  DDR_G |= _BV(LED_G); // enable as output (set 1)
  PORT_G &= ~_BV(LED_G); // set low

  DDR_B |= _BV(LED_B); // enable as output (set 1)
  PORT_B &= ~_BV(LED_B); // set low

  DDR_R |= _BV(LED_R); // enable as output (set 1)
  PORT_R &= ~_BV(LED_R); // set low
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

inline static int led_is_on(char color) {
  int val;
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
