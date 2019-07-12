This folder contains 

1. library for Arduino - makes it easy to use the debugger in Visual Studio Code.
You can copy the avr-debugger folder to your Libraries forder (in your Documents/Arduino on Windows)
and use it as any other library. See readme.txt in the library for more info.

Note: the source files .c and .h in this folder/src are copies of the files in avr8-stub folder.
This is not the best solution - the same files at two places - but I want to have the library complete, ready to just copy and use.
Make any code changes in the avr8-stub folder; then copy changed files to this arduino library src folder to keep the files in sync.

2. hardware platform files to allow burning the debugger-enabled bootloader from Arduino IDE.
 Copy the hardware folder into your Arduino sketchbook folder, for example, Documents/Arduino.
 You should then see new board - avr-debugger Atmega328 bootloader in the Tools menu in Arduino IDE.
 Select this board to burn the bootloader with debug support to your Arduino.


3. modified file(s) from the Arduino package - can be useful if you use eclipse IDE.
You can replace the original files with those files to make it easier to enable the debugger support in your program.

Arduino package used: 1.6.5-r5, 1.8.1

Your project should define AVR_DEBUG symbol in project properties > C/C++ Build > Settings > AVR Compiler  > Symbols.
This way only projects with AVR_DEBUG symbol defined will exclude this code. Other project will not be affected. 

For more information, please see the documentation in /doc folder.

Files:
-------
File: WInterrupts.c   
Modification: section for INT0, INT1,... interrupt placed into #if AVR_DEBUG section.
Reason: This allows building the project with this debugger which uses INTx interrupt itself.



