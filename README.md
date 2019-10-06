# What's this all about?
Software for my ATtiny88 based humidity sensor.
The board is also some kind of development board to play around with the ATtiny88,
but mainly focused on humidity.

The humidty and touch sensors are built on capacitive touch technology described in
https://www.youtube.com/watch?v=BO3umH4Ht8o&t=547s

It consists basically of loading a capacitor and measuring the tics until the input is read as high.

The software works as a [state machine](humidityguard-state-machine.jpg) and can do the following:
- measure humidity on humidity sensor and show if too humid/dry on 3 LEDs
- handle 3 touch button
- touch1/2/3 activates the display
- long press on touch2 sets the threshold humidity
- display shows humidity, humidity threshold, temperature and battery voltage

See the [pcb folder](pcb) for schematics.

# Setup software
```
brew install platformio avrdude
pyenv 3.7.4 attiny88
pyenv local attiny88
pip install -r requirments.txt
```

Run the following to activate the necessary files.
This command symlinks the files to the folder src/
```
./activate.py modules/humidityguard
```

# Flash software
Connect board with USBTinyISP to computer and run
```
make flash
```

# Debugging
Ensure tart serial console
```
make debug
```

# Important files
- modules/common/def.h: IO definitions for the board and common functions
- modules/common/avr_print.*: Debug software to print to serial terminal
