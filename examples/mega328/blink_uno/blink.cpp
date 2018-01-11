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
	//breakpoint();   // stop execution here
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
}

void loop(void)
{
	//breakpoint();		// stop execution here
	digitalWrite(13, HIGH);
	delay(500);
	digitalWrite(13, LOW);
	delay(1000);
}
