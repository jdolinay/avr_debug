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
#include "avr8-stub.h"

int cnt = 0;
int function(int a);

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
    	cnt = function(cnt);
    	PORTB &= ~_BV(5);	// LED off
    	cnt++;
    }
    return 0;
}

int function(int a)
{
	int n;
	n = 2*a;
	return n;
}

