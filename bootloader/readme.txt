Bootloader for source level debugger for Arduino avr_debug

Works for Arduino Uno (ATmega328).

Introduction
-------------
This project is based on optiboot bootloader, which is used by default in Arduino. 
This is modified version of the bootloader which makes it possible for the debugger
to write breakpoints to flash memory.


Support for other boards than Uno
---------------------------------
This modified optiboot bootloder works with Arduino Uno and possibly other boards based on Atmega328 MCU.

As on June 2020 there is an option to use standard Optiboot bootloader verson 8.0 or above instead of this
modified bootloader. It is because the new version of Optiboot supports writing to flash memory (do_spm function).

So for Atmega328 boards you can decide whether to use the modified bootloader provided here or the standard Optiboot.
For other boards, like Arduino Mega there is only the original Optiboot option.
For more information please see the manual in /doc/ folder.


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
   
Lock bits
----------
   For the debugging to work the application code must be able to read the bootloader memory.
   This is enabled by default if you erase the chip. 
   If you encounter problems with debugging your app after updating the bootloader, make sure the
   lock bits do not prohibit reading Boot section (LPM instruction in Boot section must not be disabled).     
   The correct lock bit value is 0xFF or 0xEF in the programmer GUI in Atmel Studio. 
    


Building the bootloader
-------------------------
If you want to make changes to the sources and build your own bootloader, 
you can use Eclipse IDE with AVR Eclipse plugin (eclipse project provided).
You can also build the project with PlatformIO - see the readme_platformio.txt in the optiboot subfolder. 

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
--------------

May 2020
+ Added support for building the bootloader with PlatformIO, thanks to msquirogac, https://github.com/msquirogac/avr-debugger-bootloader.
The sources were moved to src subfolder to support PlatformIO project structure. Eclipse project modified to refer to the files in new location.

Sept. 2019
+ Changed number of LED blinks to 3 (LED_START_FLASHES=3). Was 2 but on some computers the upload from Arduino IDE failed.
The standard Arduino implementation uses LED_START_FLASHES=3.

July 2018
+ Fixed bug - there were const data at the start address instead of main. Loading via avrdude didn't work.

June 2017
+ First version finished. Supports flash breakpoints and binary load from GDB.

March 2017
+ started


