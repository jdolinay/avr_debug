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

// 0e 94 b4 06 	call	0xd68	; 0xd68 <digitalWrite>
// adresa je 06b4 wordove coz je 0d68 bajtove
#define TRAP_OPCODE 0x0000940e
uint32_t trap_opcode = (TRAP_OPCODE | ((uint32_t)((uint16_t)breakpoint) << 16));
uint16_t addr = (uint16_t)breakpoint;

uint32_t op;

void setup(void)
{

	debug_init();	// initialize the debugger
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	//breakpoint();
	//op = trap_opcode;
	//while(1) ;
	//trap_opcode |= (uint32_t)addr << 16;
	//asm("call	trap_opcode");
	//asm("call	addr");
	 /*asm volatile (
	        "call %0" "\n"
	        :
	        : "i" (addr)
	    );*/
}

void loop(void)
{
	//breakpoint();		// stop execution here
	digitalWrite(13, HIGH);
	delay(200);
	digitalWrite(13, LOW);
	delay(500);
}
