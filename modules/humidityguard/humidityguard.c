/*
 * simple capacitive touch pin / moisture sensor
 * using 500k instead of 1M
 * GND around sensor and on bottom layer
 * without GND sensor does almost not detect water
 *
 *
 * PA1 -----|500k|-----|+
 *       |
 *       |
 *     ----
 * GND |  | GND
 *     |  |
 * GND |  | GND
 *     |  |
 * GND ---- GND
 *     GND
 *
 *
 * based on https://github.com/cnlohr/magfestbadges/blob/master/basictouch/test.c
 * (or https://www.youtube.com/watch?v=BO3umH4Ht8o&t=547s)
 *
 *
 * PORTC |= (_BV(0) | _BV(2) | _BV(7));  // Set bits 0,2,7
 * PORTC &= ~(_BV(1) | _BV(2) | _BV(6)); // Clear bits 1,2,6
 * PORTC ^= (_BV(5) | _BV(3));           // Toggle bits 3,5
 * if (PORTC & _BV(5))                   // is bit 5 set on PORTC

 * Sleep with bit manipulation
 * SMCR &= ~(_BV(SM1) | _BV(SM0)); // sleep mode "Idle"
 *
 * SMCR |= _BV(SE); // go into sleep
 * __asm__ __volatile__ ("sleep\n\t"::);
 *
 * SMCR &= ~_BV(SE); // clear sleep bit upon waking up
 * / TODO green led = optimum, blue = too much water, red = not enough water
 * / TODO check deep_sleep power consumption ~19uA
 * / TODO save calibration(s) to sram
 * / TODO calibrate only happens if TOUCH2 pressed longer than 2sec
 * / TODO status LED's are shown until device goes to sleep or any other button pressed
 * / TODO attiny + oled
 * / TODO light/temperature sensors adc
 * / TODO minimize font size
 * / TODO update README
 * / TODO implement manual pump (on-off)
 * TODO graphically show humidity
 * TODO show battery status graphically
 * TODO waterproof capacitive touches
 *
 * https://github.com/tibounise/SSD1306-AVR
 * https://github.com/Dygmalab/avr-keyscanner
 */

#include "humidityguard.h"
#include "oled.h"

// pin interrupt
ISR(TOUCH_INT_VECT) {
  cli();
  capacitive_value = TCNT1;
  sei();
}

// timer1 compare A
// 16bit
ISR(TOUCH_TIMER1_VECT) {
  cli();
  capacitive_value = 0xfff0;
  sei();
}

// timer 0 compare A
// 8bit
ISR(TOUCH_TIMER0_VECT) {
  cli();
  if (--timer_isr_call_counter == 0) {
    timer_isr_call_counter = timer_isr_call_counter_init;
    timer_is_done = 1;
  }
  sei();
}

/*
 * watchdog interrupt is only called once when calling deep_sleep
 */
ISR(WDT_vect) {
  sleep_disable();
  WDTCSR &= ~_BV(WDIE); // disable wdt interrupt
  sleep_enable();
}

/*
 * interrupt for ad converter
 */
ISR(ADC_vect) {
  cli();
  uint8_t l = ADCL;
  adc_value = ADCH<<8 | l;
  sei();
}

void setup_timer0() {
  // setup timer0
  TIMSK0 |= _BV(OCIE0A); // compare match A int enable
  TIFR0 |= _BV(OCF0A); // ISR(TIMER0_COMPA_vect)
  // TIMSK0 |= _BV(TOIE0) // overflow interrupt enable using ISR(TIMER0_OVF_vect)
}

/*
 * 8bit timer for timing work
 *
 * 1ms = 1.024ms
 * -> ms max ~ 67seconds
 */
void start_timer0(uint16_t ms) {
  timer_isr_call_counter = timer_isr_call_counter_init = ms;
  OCR0A = 8; // 8MHz/1024/8 = 1.024ms
  TCNT0 = 0;
  TCCR0A |= _BV(CTC0) | _BV(CS00) | _BV(CS02); // start timer with prescaler 1024 and CTC (clear timer on compare)
  timer_is_done = 0;
}

void stop_timer0() {
  TCCR0A = 0x00; // stop timer 0
}

/*
 * 16bit timer for measuring capacitive sensors
 */
void setup_timer1() {
  TIMSK1 |= _BV(OCIE1A); // enable overflow compare A (to detect if we're taking too long)
  TIFR1 |= _BV(OCF1A); // interrupt flag register, compares to OCR1A
}

void start_timer1() {
  OCR1A = 0xfff0; // set overflow A to be 0xfff0 cycles
  TCNT1 = 0;
  TCCR1B |= _BV(CS10); // start timer with prescaler 1
}

void stop_timer1() {
  TCCR1B = 0x00;
}

void sleep(uint16_t ms) {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  // turn off anything which is possible
  // consumption ~1.1mA
  PRR |= _BV(PRTWI) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRADC);
  start_timer0(ms);
  while (timer_is_done == 0) {
    sleep_mode();
  }
  /* blocked sleeping until waking up */
  timer_is_done = 0; // reset counter flag
  stop_timer0();
  PRR &= ~(_BV(PRTWI) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRADC));

  sleep_disable();
}

/*
 * use watchdog timer
 * can be used when we just sleep for a predefined time w/o prio wake-up
 * only specific times can be achieved for simplicity
 * 16ms, 32ms, 64ms, 125ms, 250ms, 500ms, 1s, 2s, 4s, 8s
 */
void deep_sleep(uint16_t mode) {
  uint8_t prescale = 0x00;
  // clear all prescale bits first
  prescale &= ~_BV(WDP3) | ~_BV(WDP2) | ~_BV(WDP1) | ~_BV(WDP0);

  if (mode == 32) {
    prescale |= _BV(WDP0);
  } else if (mode == 64) {
    prescale |= _BV(WDP1);
  } else if (mode == 125) {
    prescale |= _BV(WDP1) | _BV(WDP0);
  } else if (mode == 250) {
    prescale |= _BV(WDP2);
  } else if (mode == 500) {
    prescale |= _BV(WDP2) | _BV(WDP0);
  } else if (mode == 1000) {
    prescale |= _BV(WDP2) | _BV(WDP1);
  } else if (mode == 2000) {
    prescale |= _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
  } else if (mode == 4000) {
    prescale |= _BV(WDP3);
  } else if (mode == 8000) {
    prescale |= _BV(WDP3) | _BV(WDP0);
  }

  cli();
  MCUSR &= ~_BV(WDRF); // clear wdt reset flag
  WDTCSR |= (_BV(WDCE) | _BV(WDE));  // enable WD change bit
  WDTCSR = _BV(WDIE) | // enable wdt interrupt
  prescale; // timeout

  power_adc_disable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();

  // DL("sleep");
  sleep_mode(); // sleep baby, sleep!
  // DL("back");

  sleep_disable();
  power_adc_enable();
}

/*
 * measure sensor values (blocks CPU)
 * 0 = MOIST_A
 * 1 = TOUCH1
 * 2 = TOUCH2
 * 3 = TOUCH3
 */
uint16_t get_sensor_data_calibrated(uint8_t sensor) {
  // sometimes 1st values are crap
  get_sensor_data_raw(sensor);
  uint16_t m = get_sensor_data_raw(sensor);
  return m < offset[sensor] ? 0 : m - offset[sensor];
}

/*
 * measure sensor values (blocks CPU)
 * 0 = MOIST_A
 * 1 = TOUCH1
 * 2 = TOUCH2
 * 3 = TOUCH3
 */
uint16_t get_sensor_data_raw(uint8_t sensor) {
  uint8_t input;
  uint8_t input_int;

  if (sensor == 0) {
    input = MOIST_A;
    input_int = MOIST_A_INT;
  } else if (sensor == 1) {
    input = TOUCH1;
    input_int = TOUCH1_INT;
  } else if (sensor == 2) {
    input = TOUCH2;
    input_int = TOUCH2_INT;
  } else if (sensor == 3) {
    input = TOUCH3;
    input_int = TOUCH3_INT;
  }

  TOUCH_DDR |= _BV(input);  // set as output -> low to discharge capacity

  // wait a short time to ensure discharge
  // do not use _delay_ms or sleep as timers in use
  volatile uint16_t i = 0;
  while ((TOUCH_PORT & _BV(input)) != 0) {
    __asm__ __volatile__ ("nop");
  }

  for (i=0; i<255; i++) {
    __asm__ __volatile__ ("nop");
  }

  INT_CLEAR = (1<<TOUCH_PCIF); // clear any pending interrupt flag
  TOUCH_PCMSK |= _BV(input_int); // activate pin change interrupt on pin
  INT_SETUP |= _BV(TOUCH_PCIE); // enable pin change interrupts

  start_timer1();
  TOUCH_DDR &= ~_BV(input);  // set as input -> capacity is charged via resistor

  // put the CPU to sleep until we either get a pin change or overflow compare A.
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();

  sleep_mode();
  stop_timer1();
  sleep_disable();

  // disable pin change interrupt
  TOUCH_PCMSK &= ~_BV(input_int);
  INT_SETUP &= ~_BV(TOUCH_PCIE); // pcicr

  return capacitive_value;
}

/*
 * get average from 6-13
 * ignoring 1st values
 */
uint16_t get_average(uint8_t sensor) {
  uint16_t sum = 0;
  for (uint8_t i=0; i<14; i++) {
    if (i>5) {
      sum += get_sensor_data_raw(sensor);
    }
  }
  return sum/8;
}

/*
 * calibrate all sensors
 * the sensor should be untouched and unused while processing
 * (min values)
 */
void calibrate() {
  DL("calibrating. Do not touch sensor...");
  for (uint8_t sensor=0; sensor<=3; sensor++) {
    offset[sensor] = get_average(sensor);
    DF("sensor %d: %u", sensor, offset[sensor]);
  }
  DL("done");
}

/*
 * calibrate humidity sensor
 * (optimal value)
 */
void calibrate_humidity() {
  DL("calibrating hum");
  humidity_optimum = get_average(0) - offset[0];
  if (humidity_optimum < 0) humidity_optimum = 0;
  eeprom_write_word(&ee_humidity_optimum, humidity_optimum);
  DF("hum: %u‰", convert_hum_to_relative(humidity_optimum));
  #ifdef OLED
    show_measure_oled(convert_hum_to_relative(humidity_optimum), 1, 67);
  #endif
}

/*
 * show humidity
 * on led if show_led set
 * on oled if show_oled set and available
 *
 */
char show_humidity(uint8_t show_led, uint8_t show_oled) {
  char color;

  humidity = get_sensor_data_calibrated(0);

  if (show_led) {
    led_off_all();
    if (humidity < humidity_optimum*.7) color = 'r';
    else if (humidity > humidity_optimum*1.3) color = 'b';
    else color = 'g';
    led_on(color);
  }

  #ifdef OLED
    if (show_oled) {
      if (count_oled-- == 0) {
        count_oled = 50;
        // show humidity
        show_measure_oled(convert_hum_to_relative(humidity), 1, 0);

        // show voltage
        show_measure_oled(convert_adc_voltage(get_analog('v')), 6, 67);

        // show temperature
        show_measure_oled(convert_adc(get_analog('t'), temp_vector, temp_vector_size), 6, 0);
      }
    }
  #endif

  return color;
}

/*
 * voltage ADC7, mode = 'v'
 * 220k/1220k @ 1.1v
 * 1023 = 6.1v
 * returns volts/10
 *
 * light ADC0, mode = 'l'
 *
 * temp ADC6, mode = 't'
 */
uint16_t get_analog(char mode) {
  ADMUX &= ~(_BV(REFS0) | _BV(ADLAR)); // internal 1.1v reference, left adjusted results

  // clear input channels 1st (corresponds to mode = 2)
  ADMUX &= ~(_BV(MUX3) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0)); // ADC0, light
  if (mode == 'l') {
    // do nothing, already setup
  } else if (mode == 't') {
    ADMUX |= _BV(MUX2) | _BV(MUX1); // ADC6, temp
  } else if (mode == 'v') {
    ADMUX |= _BV(MUX2) | _BV(MUX1) | _BV(MUX0); // ADC7, voltage
  } else {
    return -1;
  }

  ADCSRA |= _BV(ADEN) | _BV(ADIE); // enable adc and interrupt
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // prescaler 64 -> 8MHz/64 = 125kHz
  ADCSRA |= _BV(ADSC); // start conversion

  // sleep & wait for conversion
  // assume no other interrupt will wake up avr
  set_sleep_mode(SLEEP_MODE_ADC);
  sleep_enable();
  sleep_mode();

  ADCSRA &= ~_BV(ADEN); // disable adc
  return adc_value;
}

int16_t convert_adc_voltage(uint16_t adc) {
  return adc*61/1024;
}

/*
 * normalize adc value to given power voltage
 * ie 3.3v/x
 * use 33 for precision
 */
uint16_t norm_adc_voltage(uint16_t adc, uint16_t voltage) {
  return adc;
  // return adc*33/voltage;
}

/*
 * conversion for temperature and light
 * based on characteristics with interpolation
 */
int16_t convert_adc(uint16_t adc, adc_vector *characteristics, uint8_t size) {
  uint8_t match = size-1;
  for (uint8_t i=0; i<size; i++) {
    if (adc > characteristics[i].adc) {
      match = i;
      break;
    }
  }

  if (match==0) return characteristics[0].temp;
  else if (match==size-1) return characteristics[size-1].temp;

  uint16_t temp_start = characteristics[match-1].temp;
  uint16_t temp_end = characteristics[match].temp;
  int16_t adc_start = characteristics[match-1].adc;
  int16_t adc_end = characteristics[match].adc;

  int16_t r = (temp_end-temp_start) * (adc-adc_start);
  int16_t d = adc_end-adc_start;
  r /= d;

  return temp_start+r;
}

/*
 * 1000% = 2000
 * returns permille
 */
uint16_t convert_hum_to_relative(uint16_t hum) {
  return hum/2;
}

/*
 * convert uint16_t 203 to char "20.3"
 * buf must be buf[5]
 *
 */
void convert_int_to_char(char *buf, uint16_t measure) {
  // show leading 0 if < 10
  if (measure < 10) {
    buf[0] = '0';
    buf[1] = '.';
    buf[2] = measure+'0'; // convert to char
    return;
  }

  sprintf(buf, "%u", measure);
  for (uint8_t i=0; i<4; i++) {
    if (buf[i]==0) {
      buf[i] = buf[i-1];
      buf[i-1] = '.';
      break;
    }
  }
}

/*
 * show value at position line/col
 * value is factor 10, ie 12 is shown as 1.2
 */
void show_measure_oled(uint16_t value, uint8_t line, uint8_t col) {
  char buf[5] = {0};
  convert_int_to_char(buf, value);
  oled_clear_part(40, line, col, 'l');
  oled_set_pos(line, col);
  oled_text(buf, 'l');
}

/*
 * return true if sensor pushed
 * 0: if any sensor pushed
 *
 */
uint8_t sensor_active(uint8_t sensor) {
  if (sensor) return pressed & _BV(sensor);
  else return pressed;
}

/*
 * only triggers once when sensor is pushed
 */
uint8_t sensor_pushed(uint8_t sensor) {
  return (pressed_changed & _BV(sensor)) && (pressed & _BV(sensor));
}

int main(void) {
  DINIT(); // enable debug output
  DL("\n\nHello there");

  // reading variables from eeprom
  humidity_optimum = eeprom_read_word(&ee_humidity_optimum);
  // 1st time call, save defaults to eeprom
  if (humidity_optimum == 0xffff) {
    DL("write hum opt");
    humidity_optimum = 1000;
    eeprom_write_word(&ee_humidity_optimum, humidity_optimum);
  }

  for (uint8_t i=0; i<4; i++) {
    offset[i] = eeprom_read_word(&ee_offset[i]);
  }
  // 1st time call, save defaults to eeprom
  if (offset[0] == 0xffff) {
    DL("write offsets");
    uint16_t o[4] = {166, 57, 68, 65};
    for (uint8_t i=0; i<4; i++) {
      offset[i] = o[i];
      eeprom_write_word(&ee_offset[i], offset[i]);
    }
  }

  led_setup();

  setup_timer1();
  setup_timer0();

  // set touch pins low
  TOUCH_PORT &= ~(_BV(MOIST_A) | _BV(TOUCH1) | _BV(TOUCH2) | _BV(TOUCH3));

  capacitive_value = 0x0000;

  out_setup();
  out_off();

  sei();

  for (uint8_t i=0; i<4; i++) {
    DF("sensor %u: %u (%u)", i, offset[i], get_sensor_data_raw(i));
  }

  humidity = get_sensor_data_calibrated(0);

  // base display data / grid
  #ifdef OLED
    oled_setup();

    oled_hline(32);
    oled_vline(64);

    oled_set_pos(0, 0);
    oled_text("HUMIDITY", 's');
    show_measure_oled(convert_hum_to_relative(humidity), 1, 0);

    oled_set_pos(0, 67);
    oled_text("THRESHOLD", 's');
    show_measure_oled(convert_hum_to_relative(humidity_optimum), 1, 67);

    oled_set_pos(5, 0);
    oled_text("TEMP", 's');
    oled_set_pos(5, 67);
    oled_text("BAT", 's');

    // show voltage
    show_measure_oled(convert_adc_voltage(get_analog('v')), 6, 67);

    // show temp
    show_measure_oled(convert_adc(get_analog('t'), temp_vector, temp_vector_size), 6, 0);

  #endif

  DF("hum opt: %u, cur: %u", humidity_optimum, humidity);
  DL("\n");

  while (1) {
    // check touch sensors
    pressed = 0;
    for (uint8_t sensor=1; sensor<4; sensor++) {
      value = get_sensor_data_calibrated(sensor);

      if (value > TOUCH_THRESHOLD) {
        pressed |= _BV(sensor);
      }
    }

    pressed_changed = pressed ^ pressed_prev;
    pressed_prev = pressed;

    last_mode = mode;
    mode = new_mode;

    // toggle OUT no matter in which mode we are
    if (sensor_pushed(3)) {
      out_toggle();
    }

    switch(mode) {
      // sleep
      case 0:

        // turn off oled when entering mode 0
        if (last_mode != 0) {
          led_off_all();
          #ifdef OLED
           oled_off();
          #endif
        }

        // flash + sleep
        // show humidity every 8x500ms = 4sec
        if (count_sleep-- == 0) {
          char color = show_humidity(1, 0);
          deep_sleep(16);
          led_off_all();
          count_sleep = 8;

          // flash twice if battery getting empty
          if (convert_adc_voltage(get_analog('v')) < 39) {
            deep_sleep(125);
            led_on(color);
            deep_sleep(16);
            led_off_all();
          }
        }
        deep_sleep(500);

        if (sensor_active(0)) new_mode = 1;
        break;
      // measure
      case 1:
        show_humidity(1, 1);

        // turn on oled when coming from sleep mode
        #ifdef OLED
          if (last_mode == 0) oled_on();
        #endif

        if (last_mode == 3 || last_mode == 2) stop_timer0();

        if (sensor_active(2) && !sensor_active(1) && !sensor_active(3)) new_mode = 3;
        else if (!sensor_active(0)) new_mode = 2;
        break;
      // countdown to sleep
      case 2:
        show_humidity(1, 1);

        // start timer when entering mode 3
        if (last_mode != 2) start_timer0(3000);

        if (timer_is_done) {
          timer_is_done = 0;
          new_mode = 0;
          oled_off();
        }
        else if (sensor_active(0)) new_mode = 1;
        break;
      // countdown to calibration
      case 3:
        show_humidity(1, 1);

        // start timer when entering mode 3
        if (last_mode != 3) start_timer0(2000);

        if (timer_is_done) {
          DL("timer done");
          new_mode = 4;
          timer_is_done = 0;
        }
        else if (!sensor_active(2)) new_mode = 1;
        break;
      // calibrating
      case 4:
        // show on oled only
        show_humidity(0,1);

        // stop timer when entering mode 4
        if (last_mode != 4) {
          stop_timer0();
          led_on_all();
        }

        if (sensor_active(1) ||sensor_active(3)) new_mode = 1;
        else if (!sensor_active(2)) new_mode = 5;
        break;
      // save calibration data
      case 5:
        calibrate_humidity();
        led_off_all();
        new_mode = 2;
        break;
    }

  }
  return 0;
}
