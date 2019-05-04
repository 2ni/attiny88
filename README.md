## Debugging with additional simplex serial
- [using simplex uart](http://www.justgeek.de/a-simple-simplex-uart-for-debugging-avrs)
- connect uart to ATtiny: GND-GND, RXD-DEBUG (PD3)
- connect uart to additional USB on computer
- connect spi programmer to USB on computer

To run serial console:
```
make debug
```

### Setup USBtiny and flash custom tinyispterm code to programmer

Unforatunately it's not yet [working](https://github.com/cnlohr/tinyisp-micro/issues/1#issuecomment-489075296)
Some information on [USBtinyISP with ATtiny24](https://github.com/julianschuler/USBtinyISP) might help

What's needed:
- custom firmware ([tinyisp_micro](https://github.com/cnlohr/tinyisp-micro))) on the programmer
- [tinyispterm](https://github.com/cnlohr/tinyispterm) programm
- patching avrdude


I used a standard USBtiny and a USBasp to flash it
Be sure to solder jumper SJFab 1st.
Do not compile code, as it won't compile or work

Pinout is according to http://fab.cba.mit.edu/content/archive/projects/fabisp/fabisp.pdf
D-   PA0 (13)
D+   PA7 (6) / PB2 (5)

RST  PA3 (10)
MOSI PA6 (7)
MISO PA5 (8)
SCK  PA4 (9)

- change in spi/usbtiny.h
#define USBTINY_PORT   A
#define USBTINY_DPLUS  7 (was 2)
#define USBTINY_DMINUS 0 (was 3)
#define USBTINY_INT    0

- change in spi/main.c
#define RESET         PA3 (was PA7)
#define LED           PA2 (was PA0) // there is no LED on the board, set it to unused pin
#define POWER_PULLUP  PA1
#define MOSI          PA6
#define SCK           PA4
#define MISO          PA5 (was 5)

- change in spi/Makefile
usbtiny with usbasp


```
git clone git@github.com:cnlohr/tinyispterm.git or https://github.com/cnlohr/tinyisp-micro
cd tinyispterm/tinyisp_micro/firmware_mod/spi/
avrdude -c usbasp -p t44 -U hfuse:w:0xdf:m -U lfuse:w:0xef:m
avrdude -c usbasp -p t44 -U flash:w:main.hex
```

Unsolder SJFab when flashing done.
Fallback: http://fab.cba.mit.edu/content/archive/projects/fabisp/firmware.zip

### Compiling avrdude (as it might need to be patched)
```
wget http://download.savannah.gnu.org/releases/avrdude/avrdude-6.3.tar.gz
tar xfz avrdude-6.3.tar.gz
cd avrdude-6.3
./configure
make
make install
```
