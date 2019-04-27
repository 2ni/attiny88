/**
 * Copyright (C) PlatformIO <contact@platformio.org>
 * See LICENSE for details.
 */

#include <avr/io.h>
#include <util/delay.h>

#define DDR DDRA
#define PORT PORTA
#define LED_G PA3


// _BV(3) => 1 << 3 => 0x08
// PORTB = PORTB | (1 << 4);
// set: PORTB |= (1<<BIT2);
// clear: PORTB &= ~(1<<BIT2);

inline static void ledSetup(){
    DDR |= _BV(LED_G); // enable as output (set 1)
    PORT &= ~_BV(LED_G); // set low
}

inline static void ledOn() {
    PORT |= _BV(LED_G); // set high
}

inline static void ledOff() {
    PORT &= ~_BV(LED_G); // set low
}

inline static void ledToggle() {
    PORT ^= _BV(LED_G); // invert (exclusive or)
}

int main(void) {
    ledSetup();

    while (1) {
        _delay_ms(500);
        ledToggle();
    }

    return 0;
}
