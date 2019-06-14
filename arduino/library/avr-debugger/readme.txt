avr_debug

This library provides source level debugger for Arduino Uno and Mega. 
This debugger can be used to step through your code, place breakpoints, view variables etc. 
No external hardware is needed. 
Supports Arduino UNO and MEGA.
Created by Jan Dolinay.

This library is part of larger project, see https://www.codeproject.com/Articles/1037057/Debugger-for-Arduino
and https://github.com/jdolinay/avr_debug


Requirements
--------------------------------------------------------------------------------
Debugging requires IDE with debugging capabilities, for example, Visual Studio Code. 
This cannot be used with Arduino IDE! 
But you do need to have Arduino IDE installed unless you use your own toolchain to build the program.

Installation
--------------------------------------------------------------------------------
To install this library, just place this entire folder as a subfolder in your
Arduino/libraries folder (This folder is in Documents on Windows)


Usage
--------------------------------------------------------------------------------
Note: This library cannot be used with Arduino IDE. It requires IDE with debugging capabilities, for example, Visual Studio Code or eclipse.

Windows instructions:
- Add this library to your sketch
- Put call to debug_init(); into your setup() function.
- In VS Code copy the provided launch.json file to your-sketsch-location/.vscode folder. 
- Change the number of COM port to the number where your arduino is connected - see "miDebuggerServerAddress":
- Change the locaion of avr-gdb.exe. The default will work only if you installed Arduino IDE to default location.   
Default path: "c:\\Program Files (x86)\\Arduino\\hardware\\tools\\avr\\bin\\avr-gdb.exe"
see "miDebuggerPath":

Optional, but recommended step: 
- copy platform.local.txt to your-arduino-directory/hardware/arduino/avr
Default location: c:\Program Files (x86)\Arduino\hardware\arduino\avr\
Important note: this affects all programs you build in Arduino IDE or VS code! 
This will modify the build settings so that the program is easier to debug (turn of compiler optimizations).
The code will also be larger. For building the final program remove or rename the platform.local.txt file. 
This will make your program smaller and faster.


Linux and Mac instructions:
- to be done :) 
I hope you can figure it out based on the instructions above. 

 
