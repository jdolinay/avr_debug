Examples for ATmega2560 and AtMega1280 (Arduino Mega).
The projects are configured for Arduino mega with ATmega2560 MCU.
To change to ATmega1280, go to project properties > AVR > Target hardware 
and change the MCU Type to ATmega1280.
Clock is 16000000 (16 MHz) for both.

To upload to the board use this avrdude programmer configuration:
Mega1280 use Programmer hardware: Arduino, baudrate 57600
Mega2560 use Programmer hardware: Wiring, baudrate 115200 
