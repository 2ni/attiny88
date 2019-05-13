#ifndef HumidityGuard_h
#define HumidityGuard_h

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include "def.h"
#include "uart.h"

// value we get from a timer to charge a capacity
volatile uint16_t capacitive_value;

// timer0 needs several isr calls to reach longer wait/sleep
// _counter: current amount of isr calls which need to be called to reach desired wait
// _counter_init: initial amount of necessary isr calls to reach desired wait
// _is_done: desired wait achieved. Flag needs to be reset when consuming!
volatile uint16_t timer_isr_call_counter;
volatile uint16_t timer_isr_call_counter_init;
volatile uint16_t timer_is_done;

// timer0 used for wait/sleep function
void setup_timer0();
void start_timer0(uint16_t ms);
void stop_timer0();

void sleep(uint16_t ms);

// timer1 used to measure capacitive sensors
void setup_timer1();
void start_timer1();
void stop_timer1();

// put avr in POWER_DOWN mode
void deep_sleep(uint16_t mode);

// sensor0: capacitive humidity sensor
// sensor1-3: capactive touch sensors
uint16_t measure(uint8_t sensor);

// calibrate sensor and save values to offset
// hardcode values as default
uint16_t offset[4] = {166, 57, 68, 65};
uint16_t humidity_optimum = 1000;
uint16_t get_average(uint8_t sensor);
void calibrate();

// show humidity on led
void show_humidity(uint16_t value);

int main(void);

#endif
