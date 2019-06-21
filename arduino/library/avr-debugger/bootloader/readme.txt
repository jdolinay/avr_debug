Bootloader for source level debugger for Arduino avr_debug

Works for Arduino Uno (ATmega328).

optiboot.hex is the binary file which you can burn into your Arduino Uno to use flash breakpoints in the debugger.

The original of this file together with sources and instructions can be found in avr_debug/bootloader folder. 

Usage
-------
To use this bootloader, flash your Arduino with the optiboot.hex provided here. To do this, you will need 
an AVR-ISP programmer.

Fuse settings
--------------
IMPORTANT: The size of the bootloader is different than the original optiboot for Arduino.
   You need to program different bootloader size for this bootloader.
   Set BOOTSZ to 1024 w (bootloader address 3c00)
   You don't need to change any other fuses in Arduino.

   For those interested, here is complete fuse info:
   Raw fuses value:
   EXTENDED: 0xFD
   HIGH: 0xD2
   LOW: 0xFF

   Fuse settings for humans:
   Enable SPIEN
   Enable BOOTRST
   Enable EESAVE
   Set BOOTSZ to 1024 words (bootloader start address 0x3c00)
   Set clock to EXT OSC 8 or 16 MHz

