Source level debugger for Arduino
Created by Jan Dolinay, June 2015
Works for Arduino Uno (ATmega328).

Introduction
-------------
This is debugger for Arduino based on GNU Debugger (GDB). 
The debugger is implemented using GDB stub mechanism. This means a piece of code (stub) is added 
to your Arduino program. This code then communicates with the GDB debugger. 
No external programmer or modification of the Arduino board is required. 
Eclipse can be used as a graphical frontend for debugging.
For more information and tutorial please see the manual in /doc directory.


Revision History
----------------

January 2017
+ Added support for Arduino Mega board with ATmega1280 and ATmega2560 MCUs.
+ Example programs reorganized and renamed. The name now contains Arduino variant so that example 
  projects for different variants can be imported into single eclipse workspace.
+ Fixed bug for ATmega328 (Uno) - the debugger now works for programs larger than 16 kB.
+ Documentation updated to describe also direct serial communication with the debugger (without the TCP-to-COM proxy) 
  which seems to work on Windows 10 and with some boards also on Windows 7.


June 2015
First release


Contents of this package:
-------------------------
avr8-stub	- source code of the debug driver (gdb stub for ATmega328 used in Arduino Uno)
doc 		- documentation with tutorials.
examples 	- example projects which can be imported into your eclipse workspace. See documentation for instructions on use.
hub4com-2.1.0.0-386 - hub2com tcp-serial proxy from http://sourceforge.net/p/com0com/news/2012/06/hub4com-v2100-released
start_proxy.bat - convenience script to start the com2tcp proxy.


Notes
------
You can also start the tcp-serial proxy directly from command line:
com2tcp.bat --baud 115200 \\.\COM1 11000
(The example command assumes Arduino on port COM1; GDB connecting to localhost at port 11000).


License
--------
This is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either version 3 of the License, or (at your option) any later version.