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
#include "../../../avr8-stub/avr8-stub.h"	/* relative path for GDB stub valid only if this file is in the examples subfolder */

// LED is on pin PB7 on Arduino Mega (Arduino pin 13)
#define		LED_PIN		(7)

int cnt = 0;
int function(int a);

int main(void)
{
    debug_init();
    DDRB |= _BV(LED_PIN);	// pin mode to output for driving the LED
    sei();			// enable interrupts
   // breakpoint();
    while(1)
    {
    	PORTB |= _BV(LED_PIN);	// LED on
    	cnt++;
    	cnt = function(cnt);
    	PORTB &= ~_BV(LED_PIN);	// LED off
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

