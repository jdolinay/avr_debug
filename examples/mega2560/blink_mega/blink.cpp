/*
 * blink.cpp
 * Example project for the Arduino debugger.
 * Blinks the LED on Arduino Mega board.
 * For instructions on use please see doc\avr_debug.pdf
 *
 *  Created on: 19. 1. 2017
 *  Author: Jan Dolinay
 */
#include "Arduino.h"
/* relative path for GDB stub valid only if this file is in the examples subfolder */
#include "../../../avr8-stub/avr8-stub.h"

void setup(void)
{
	debug_init();	// initialize the debugger
	pinMode(13, OUTPUT);
}

void loop(void)
{
	breakpoint();		// stop execution here
	digitalWrite(13, HIGH);
	delay(200);
	digitalWrite(13, LOW);
	delay(500);
}




