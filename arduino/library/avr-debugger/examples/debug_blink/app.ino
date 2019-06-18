// Example program for avr-debugger library.
// IMPORTANT: 
// These instructions are for Visual Studio Code. For other use cases see the readme.txt.
// To start debugging:
// - Add "output": ".\\build" to file arduino.json (in .vscode folder)
// - Change the COM port number to your port number in launch.json, see "miDebuggerServerAddress": "\\\\.\\COM7"
// - Change the path to point to avr-gdb.exe in your Arduino installation, see launch.json,  "miDebuggerPath": 
//
// - Place breakpoint to loop, e.g. on digitalWrite - by clicking into the left margin of the file - red dot should appear
// - Upload the program to your Arduino
// - Switch to debug view
// - Click Start debugging button (Play icon) in the top
//

#include <app_api.h>
#include <avr8-stub.h>

int globalCounter = 0;

void setup()
{
    debug_init();	// initialize the debugger
	
    pinMode(13, OUTPUT);    
}

void loop()
{
	int localCounter = 0;
	localCounter++;
   
	digitalWrite(13, HIGH);
	delay(100);
	digitalWrite(13, LOW);
	delay(200);
	
	globalCounter++;
	localCounter++;
	
}
