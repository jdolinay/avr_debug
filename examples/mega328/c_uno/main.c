/*
 * main.c
 *
 *
 *  Example for debugging with avr8-stub.c in plain C language,
 *  without Arduino libraries.
 *  For instructions on use please see doc\avr_debug.pdf
 *
 *  Author: Jan Dolinay
 */

#include <avr/io.h>
#include <avr/interrupt.h>

/* relative path for GDB stub valid only if this file is in the examples subfolder */
#include "../../../avr8-stub/avr8-stub.h"


uint16_t cnt = 0;
uint16_t result;
uint16_t function(uint16_t a);

int main(void)
{
    debug_init();
    DDRB |= _BV(5);	// pin PB5 to output (LED)
    sei();			// enable interrupts
    breakpoint();
    while(1)
    {
    	PORTB |= _BV(5);	// LED on
    	cnt++;
    	result = function(cnt);
    	PORTB &= ~_BV(5);	// LED off
    	result++;
    }
    return 0;
}

uint16_t function(uint16_t a)
{
	uint16_t n;
	n = 2*a;
	return n;
}

