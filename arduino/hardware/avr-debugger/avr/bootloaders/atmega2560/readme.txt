
This is Optiboot bootloader binary for Arduino Mega 2560 (Atmega2560, 16 MHz clock, 115200 baud rate).
Optiboot version 8.1

This bootloader can be used with avr_debug stub to support flash breakpoints (optiboot mode).

Note that this is Optiboot binary built from sources available here: 
https://github.com/Optiboot/optiboot

The binary is included here for convenience because the Optiboot release archive does not contain binary 
for Atmega2560 and building it from sources can be complicated for many users (the original instructions seem outdated).

For fuse setting see the boards.txt settings.

IMPORTANT: To upload programs using this bootloader you need to use Arduino protocol; not the Wiring protocol which is used with the default bootloader!

