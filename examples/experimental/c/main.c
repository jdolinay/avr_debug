/*
 * main.c
 *
 * Experimental example for debugging with avr8-stub.c in plain C language,
 *  without Arduino libraries.
 *  For instructions on use please see doc\avr_debug.pdf
 *
 *  This project can be used for various experiments, e.g. running in linux/Windows,
 *  filling-up program memory...
 *  It can be easily changed for Atmega328/1280/2560:
 *  - In project properties > AVR change target hardware (e.g. ATmega2560)
 *  and programmer (avrdude programmer configuration).
 *  - To debug use/create debug configuration as needed (direct serial/tcp/linux dev/windows COM,...).
 *
 *  Author: Jan Dolinay
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "../../../avr8-stub/avr8-stub.h"	/* relative path for GDB stub valid only if this file is in the examples subfolder */

/* See the fillmem.h file for option to fill up the program memory to test
 * the debugger with larger programs. */
#include "fillmem.h"

// LED is on pin PB7 on Arduino Mega, PD5 on Arduino Uno (Arduino pin 13)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	#define		LED_PIN		(7)
#else
	#define		LED_PIN		(5)
#endif

uint16_t cnt = 0;
uint16_t result;
uint16_t function(uint16_t a);
void mydelay(void);


int main(void)
{
    debug_init();
    DDRB |= _BV(LED_PIN);	// pin mode to output for driving the LED
    sei();			// enable interrupts
   // breakpoint();
    while(1)
    {
    	PORTB |= _BV(LED_PIN);	// LED on

    	mydelay();

    	cnt++;
    	cnt = function(cnt);
    	PORTB &= ~_BV(LED_PIN);	// LED off
    	cnt++;

    	mydelay();
    }
    return 0;
}

uint16_t function(uint16_t a)
{
	uint16_t n;
	n = 2*a;
	return n;
}

void mydelay(void)
{
	unsigned long cnt = 50000;
	while ( cnt > 0 )
		cnt--;
}

