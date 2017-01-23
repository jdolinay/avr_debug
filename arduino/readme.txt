This folder contains modified file(s) from the Arduino package.

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



