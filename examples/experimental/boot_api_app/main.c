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
//#include <avr/wdt.h>
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
void mylongdelay(void);


/* Watchdog settings */
#define WATCHDOG_OFF    (0)	/* this off means no reset but interrupt is on*/
#define WATCHDOG_16MS   ((_BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_32MS   ((_BV(WDP0) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_64MS   ((_BV(WDP1) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_125MS  ((_BV(WDP1) | _BV(WDP0) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_250MS  ((_BV(WDP2) | _BV(WDE)) & (~_BV(WDE)))
#define WATCHDOG_500MS  ((_BV(WDP2) | _BV(WDP0) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_1S     ((_BV(WDP2) | _BV(WDP1) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_2S     ((_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_4S     ((_BV(WDP3) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_8S     ((_BV(WDP3) | _BV(WDP0) | _BV(WDIE)) & (~_BV(WDE))))


void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

/* must be optimized or in asm to do in 4 cycles*/
__attribute__((optimize("-Os")))
void watchdogConfig(uint8_t x) {
	uint8_t cSREG;
	cSREG = SREG; /* store SREG value */
	/* disable interrupts during timed sequence */
	cli();
	/* Enable watchdog in interrupt mode */
	/* must write WDE = 1 first together with WDCE to enable changes */
	WDTCSR = _BV(WDCE) | _BV(WDE);
	 /* now write desired value of prescaler and WDE with WDCE cleared within 4 cycles */
	WDTCSR = x;
	SREG = cSREG; /* restore SREG value (I-bit) */
}

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

	// test watchdog
	DDRB |= _BV(LED_PIN);	// pin mode to output for driving the LED
	/* First blink with the LED to see that the program is starting */
	PORTB |= _BV(LED_PIN);	// LED on
	mylongdelay();
	PORTB &= ~_BV(LED_PIN);	// LED off
	mylongdelay();
	sei();			// enable interrupts
	watchdogConfig(WATCHDOG_500MS);
	while(1)
		;


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

void mylongdelay(void) {
	uint8_t cnt = 100;
	while ( cnt-- )
		mydelay();
}

/* Watchdog interrupt vector */
ISR(WDT_vect)
{
	PORTB ^= _BV(LED_PIN);	// toggle LED
	/* Re-enable watchdog interrupt */
	watchdogConfig(WATCHDOG_500MS);
}

