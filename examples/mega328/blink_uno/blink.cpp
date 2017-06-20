/*
 * blink.cpp
 * Example project for the Arduino debugger.
 * Blinks the LED on Arduino Uno board.
 * For instructions on use please see doc\avr_debug.pdf
 *
 *  Created on: 11. 6. 2015
 *  Author: Jan Dolinay
 */
#include "Arduino.h"

/* relative path for GDB stub valid only if this file is in the examples subfolder */
#include "../../../avr8-stub/avr8-stub.h"

void setup(void)
{
	debug_init();	// initialize the debugger
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	breakpoint();
	//while(1) ;
}

void loop(void)
{
	//breakpoint();		// stop execution here
	digitalWrite(10, HIGH);
	delay(200);
	digitalWrite(10, LOW);
	delay(500);
}
