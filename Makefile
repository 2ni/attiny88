.PHONY: compile upload check fuse debug

compile:
	pio run

flash:
	pio run -t program

# see https://stackoverflow.com/questions/25591406/how-to-make-mac-detect-avr-board-using-usbasp-and-burn-program-to-it
check:
	avrdude -c usbasp -p t88 -P usb -v

show_fuses:
	@avrdude -c usbasp -p t88 -P usb 2>&1 |grep Fuses

# https://eleccelerator.com/fusecalc/fusecalc.php?chip=attiny88
# no div8, clk to pb0
fuse:
	avrdude -c usbasp -p t88 -P usb -U lfuse:w:0xAE:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m

# Connect PD3 to RXD of UART. GND programmer to GND UART.
# Connect UART to computer via USB
# Connect SPI programmer to computer via additional USB
debug:
	@echo "quit with: ctrl-a; ctrl-$$ "
	@sleep 1
	@screen /dev/cu.usbserial-AH06TDUZ
