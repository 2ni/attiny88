.PHONY: compile upload check fuse debug
.DEFAULT_GOAL := compile

ft: flash terminal

compile:
	pio run

clean:
	platformio run --target clean

flash:
	@avrdude -p t88 -c usbtiny -U flash:w:.pioenvs/attiny88/firmware.hex:i -F -P usb
	#pio run -t program doesn't work anymore

check:
	@avrdude -c usbtiny -p t88 -P usb -v

show_fuses:
	@avrdude -c usbtiny -p t88 -P usb 2>&1 |grep Fuses

raw:
	@avr-gcc -g -Os -mmcu=attiny88 -c -DF_CPU=8000000UL blink.c
	@avr-size -C blink.o
	@avr-objcopy -j .text -j .data -O ihex blink.o blink.hex
	@avrdude -p t88 -c usbtiny -U flash:w:blink.hex:i -F -P usb

# https://eleccelerator.com/fusecalc/fusecalc.php?chip=attiny88
# no div8, clk to pb0
fuse:
	avrdude -c usbtiny -p t88 -P usb -U lfuse:w:0xAE:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m

# Connect PD3 to RXD of UART. GND programmer to GND UART.
# Connect UART to computer via USB
# Connect SPI programmer to computer via additional USB
debug:
	@echo "quit with: ctrl-a; ctrl-$$ "
	@sleep 1
	@screen /dev/cu.usbserial-AH06TDUZ

terminals:
	@./handle_serial.py --list --port=$$port

terminal:
	@./handle_serial.py --monitor --port=$$port

terminalt:
	@./handle_serial.py --monitor --port=$$port | ./ts
