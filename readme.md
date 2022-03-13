# Source level debugger for Arduino
Created by Jan Dolinay, June 2015<br/>
Works for Arduino Uno, Nano, Pro Mini (ATmega328), Arduino Mega, as well as
for AtMega1284(P).

### ARDUINO LIBRARY NOTE
To use this debugger as Arduino library, go to the arduino/library sub-folder. The avr-debugger is the Arduino library which you can copy to your Arduino libraries folders to use it. For more information please read the readme.txt file in arduino/library/avr-debugger.

## Introduction
This is debugger for Arduino based on GNU Debugger (GDB). The debugger is implemented using GDB stub mechanism. This means a piece of code (stub) is added to your Arduino program. This code then communicates with the GDB debugger. No external programmer or modification of the Arduino board is required. Eclipse can be used as a graphical frontend for debugging. For more information and tutorial please see the manual in /doc directory.

## Revision History
**March 2022**
+ Added support for other UART than the default UART0 for Arduino Mega. (see AVR8_UART_NUMBER in avr8-stub.h).
 Now it is possible to use any one of the available UARTs to communicate with GDB and the UART0 can be used by Arduino Serial.

**April 2021**
(thanks to Bernhard Nebel, https://github.com/felias-fogg/avr_debug)
+ Added code to use TIMER0 as an alternative to using the watchdog
timer (AVR8\_USE\_TIMER0\_INSTEAD\_OF\_WDT = 1).
+ Added a new possibility for generating a software interrupt, which does not use any external pin (AVR8\_SWINT\_SOURCE = -1).
+ Shortened the runtime of the debugging ISR from 40 microseconds to
to less than 4 microseconds.
+ Fixed a bug that sometimes caused to show the wrong value of a local
variable.
+ Fixed a bug that sometimes led to ignoring a breakpoint.
+ Added a message that tells the user when s/he set too many
breakpoints.
+ The stub handles now resets caused by the debugger correctly in that it
  removes all breakpoints from flash memory before issuing the reset.


**April 2021**
(thanks to Bernhard Nebel, https://github.com/felias-fogg/avr_debug)
+ Added code for ATMega1284(P). 


**March 2021**
+ Fixed debug_message function; it hung the debugged program, now it prints properly even when debugged program is running.
+ Doc updated with info about using millis and micros Arduino functions while debugging
+ Doc updated about using the debugger in PlatformIO.

**July 2020**
+ Added support for writing breakpoints to flash memory using do_spm function exported by Optiboot bootloader. Now it is possible to use flash breakpoints also with Arduino Mega.

**May 2020**
+ Added support for PlatformIO based on contribution from msquirogac (https://github.com/msquirogac/avr-debugger-bootloader)
 This includes instructions in the doc to use the debugger in PlatformIO IDE (VSCode) and changes in the bootloader subfolder
 to support building the bootloader with PlatformIO.
 
**July 2019**
+ Created Arduino hardware configuration to make it possible to burn the bootloader from Arduino IDE.

**June 2019**
+ Created Arduino library to make it easier to use this debugger.

**July 2018**
+ Fixed bug in bootloader - it really works now with avrdude.
+ Fixed bug in stub which prevented GDB from stopping running program in some cases.

**January 2018**
+ Code tested and updated to work with current toolchain and eclipse.
+ Documentation updated.

**June 2017**
+ Added support for writing breakpoints to flash memory including special bootloader.
+ Added support for loading the program via debugger - without AVRDude.

**January 2017**
+ Added support for Arduino Mega board with ATmega1280 and ATmega2560 MCUs.
+ Example programs reorganized and renamed. The name now contains Arduino variant so that example projects for different variants can be imported into single eclipse workspace.
+ Fixed bug for ATmega328 (Uno) - the debugger now works for programs larger than 16 kB.
+ Documentation updated to describe also direct serial communication with the debugger (without the TCP-to-COM proxy) which seems to work on Windows 10 and with some boards also on Windows 7.

**June 2015**
+ First release

## Contents of this package:
Tool | Description
--- | ---
avr8-stub	| source code of the debug driver (gdb stub for ATmega328 used in Arduino Uno)
arduino | arduino library and some other code.
doc | documentation with tutorials.
examples | example projects which can be imported into your eclipse workspace. See documentation for instructions on use.
hub4com-2.1.0.0-386 | hub2com tcp-serial proxy from http://sourceforge.net/p/com0com/news/2012/06/hub4com-v2100-released
start_proxy.bat | convenience script to start the com2tcp proxy.

## Notes
You can also start the tcp-serial proxy directly from command line:
```
com2tcp.bat --baud 115200 \\.\COM1 11000
```
(The example command assumes Arduino on port COM1; GDB connecting to localhost at port 11000).

## License
This is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.
