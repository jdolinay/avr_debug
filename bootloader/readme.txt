Bootloader for source level debugger for Arduino avr_debug

Works for Arduino Uno (ATmega328).

Introduction
-------------
This project is based on optiboot bootloader, which is used by default in Arduino. 
This is modified version of the bootloader which makes it possible for the debugger
to write breakpoints to flash memory.


Usage
-------
To use this bootloader, flash your Arduino with optiboot.hex from optiboot/debug.

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


Building the bootloader
-------------------------
If you want to make changes to the sources and build your own bootloader, you can
use the eclipse project provided here. The project can be built in Eclipse with AVR Eclipse plugin.

Files needed to build this bootloader:
optiboot.c
bootapi.c
stub.c
and the headers included by these files.

If you build the project in different IDE or from command line, you need to
1) make sure the main starts at adress 0x7800 - this is where the CPU jumps after reset 
and it must execute the bootloader.

2) add to linker options the sections 
.version=0x7ffe 
.opti_api=0x7ff0
.mystrings=0x7fc0 

3) and also 
-Wl,--undefined=api_functions -Wl,--undefined=optiboot_version
The undefined switch prevent linker from optimizing out the sections .opti_api and .version
which contain variables never referenced in the code.
For details see optiboot.c.



Revision History
----------------

July 2018
+ Fixed bug - there were const data at the start address instead of main. Loading via avrdude didn't work.

June 2017
+ First version finished. Supports flash breakpoints and binary load from GDB.

March 2017
+ started


