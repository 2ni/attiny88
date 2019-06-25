#ifndef HumidityGuard_h
#define HumidityGuard_h

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>

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
uint16_t get_sensor_data_raw(uint8_t sensor);
uint16_t get_sensor_data_calibrated(uint8_t sensor);

// calibrate sensor and save values to offset
// hardcode values as default
uint16_t EEMEM ee_offset[4];
uint16_t offset[4];
uint16_t humidity_optimum;
uint16_t EEMEM ee_humidity_optimum;
uint16_t get_average(uint8_t sensor);
void calibrate();

uint16_t value;
#ifdef OLED
  uint16_t cycle = 0;
#endif
uint8_t pressed = {0}; // bit 1 set if sensor 1 set, ...
uint8_t pressed_prev = {0}; // state one cycle before
uint8_t pressed_changed = {0}; // state if some sensor changed
uint8_t countdown_started;
uint8_t sleep_count;
uint16_t humidity;

typedef struct {
  int16_t temp;
  uint16_t adc;
} adc_vector;
// based on https://datasheet.lcsc.com/szlcsc/Nanjing-Shiheng-Elec-MF52A1104F3950-P209-15A_C13424.pdf
// https://docs.google.com/spreadsheets/d/1a3KXmudcpWfhWYVWFE6oMQExikhBC05OGW9-anG_0Dc/edit#gid=0
// MF52 LDR temperature resistor
// 0.1°: adc value
static uint8_t temp_vector_size = 13;
static adc_vector temp_vector[] = {
  {-80, 1018},
  {-50,  913},
  {0,    756},
  {50,   623},
  {100,  510},
  {150,  418},
  {200,  341},
  {250,  279},
  {300,  229},
  {350,  188},
  {400,  155},
  {450,  128},
  {500,  106},
};
// based on https://datasheet.lcsc.com/szlcsc/Shenzhen-Jing-Chuang-He-Li-Tech-GL3528-10-20K_C10082.pdf
// GL3528 light resistor
// in series with 220k on 3.3v (1.1v reference)
// lux: adc value
static uint8_t light_vector_size = 5;
static adc_vector light_vector[] = {
  {1,  735}, // 69k  -> 0.79v  -> 735
  {3,  298}, // 24k  -> 0.32v  -> 298
  {10, 186}, // 14k  -> 0.20v  -> 186
  {30,  61}, // 4.5k -> 0.066v -> 61
  {90,   3}, // 2k   -> 0.03v  -> 3
};
uint16_t adc_value;
uint16_t get_analog(char mode);
int16_t convert_adc(uint16_t adc, adc_vector *characteristics, uint8_t size);
int16_t convert_adc_voltage(uint16_t adc);
void convert_int_to_char(char *buf, uint16_t measure);

// show humidity on led
void show_humidity_led(uint16_t value);
void show_measure_oled(uint16_t value, uint8_t line, uint8_t col);
uint16_t convert_hum_to_relative(uint16_t raw);
uint16_t norm_adc_voltage(uint16_t adc, uint16_t voltage);

// sensor handling
uint8_t sensor_released(uint8_t sensor);
uint8_t sensor_initial_push(uint8_t sensor);
uint8_t sensor_pushing(uint8_t sensor);

int main(void);

#endif
