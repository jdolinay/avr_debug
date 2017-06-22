/*
 * main.c
 *
 * Experimental program for testing bootloader API in avr_debug debugger.
 * Plain C language, without Arduino libraries.
 *  Author: Jan Dolinay
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "../../../avr8-stub/avr8-stub.h"	/* relative path for GDB stub valid only if this file is in the examples subfolder */
#include "../../../avr8-stub/app_api.h"	/* only needed for testing bootloader api */



// LED is on pin PB7 on Arduino Mega, PD5 on Arduino Uno (Arduino pin 13)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#error This MCU is not supported.
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
	// test bootloader api
	// Write to flash - the address is in words, so if you call
	// dboot_safe_prg_write with 0x1000, you will find the result in binary
	// editor at address 0x2000.
/*
	const uint16_t address = 0x3800;	// word addr: 0x3800 > byte addr. 0x7000
	uint8_t data[] = {0xBE, 0xEF, 0x00, 0x00, 0xBE, 0xEF, };
	dboot_safe_pgm_write(data, address, 6 );
	dboot_led_init();
	while(1) {
		dboot_led_toggle();
		mydelay();
		mydelay();
		mydelay();
	}
*/
	// Test program for flash breakpoints
    debug_init();
   // BREAKPOINT_HERE();
    DDRB |= _BV(LED_PIN);	// pin mode to output for driving the LED
    sei();			// enable interrupts
    //breakpoint();
    while(1)
    {
    	PORTB |= _BV(LED_PIN);	// LED on
    	//asm("sbi	0x05, 5");

    	mydelay();

    	cnt++;
    	cnt = function(cnt);
    	PORTB &= ~_BV(LED_PIN);	// LED off
    	//asm("cbi	0x05, 5");

    	cnt++;
    	mydelay();
    	//breakpoint();
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
	unsigned long i = 10000;
	while ( i > 0 )
		i--;
}

