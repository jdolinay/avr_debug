/*
a * mega2560.c
 *
 *  Created on: 5. 12. 2016
 *      Author: Jan Dolinay
 *
 *  GDB Debugging Agent (GDB stub) for ATMega2560
 *  This is device-specific code for ATmega2560.
 *
 * This is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "avr8-stub.h"


/* Configuration */

/* Serial port baudrate */
/* Note that we need to use the double UART speed option for the 115200 baudrate. */
#define GDB_USART_BAUDRATE 115200

/* For double UART speed (U2X0 bit) use this macro: */
TODO:
#define GDB_BAUD_PRESCALE (((( F_CPU / 8) + ( GDB_USART_BAUDRATE / 2) ) / ( GDB_USART_BAUDRATE )) - 1)
/* For normal UART speed use: (usable for speeds up to 57600) */
/*
#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2) ) / ( USART_BAUDRATE )) - 1)
*/

/*
 * Macros used in this file which change value based on options set in header
 */

TODO:
/* Symbols:
 * AVR8_SWINT_PIN 		- the pin used for generating SW interrupt
 * AVR8_SWINT_INTMASK 	- mask used in EIMSK register to enable the interrupt and in EIFR
 * 						register to clear the flag.
 */
#if AVR8_SWINT_SOURCE == 0
	#define	AVR8_SWINT_PIN		(PD2)
	#define AVR8_SWINT_INTMASK	(INT0)

#elif AVR8_SWINT_SOURCE == 1
	#define	AVR8_SWINT_PIN		(PD3)
	#define AVR8_SWINT_INTMASK	(INT1)

#else
	#error SW Interrupt source not valid. Please define in avr8-stub.h
#endif


/*
 * Other defines and macros
 */

/* Reason the program stopped sent to GDB:
 * These are similar to unix signal numbers.
   See signum.h on unix systems for the values. */
#define GDB_SIGINT  2      /* Interrupt (ANSI). */
#define GDB_SIGTRAP 5      /* Trace trap (POSIX). */

TODO:
#define MEM_SPACE_MASK 0x00ff0000
#define FLASH_OFFSET   0x00000000
#define SRAM_OFFSET    0x00800000

/* AVR puts garbage in high bits of return address on stack.
   Mask them out */
#define RET_ADDR_MASK  0x1f

TODO: move to common
/* To insert size of the buffer into PacketSize reply*/
#define STR(s) #s
#define STR_VAL(s) STR(s)


#if AVR8_FLASH_BREAKPOINTS == 1
/* information about breakpoint in flash */
struct gdb_break
{
	uint16_t addr; /* in words */
	uint16_t opcode;
};
#endif


/**
 * Data used by this driver.
 */
struct gdb_context
{

	uint16_t sp;
	uint16_t pc;
#if AVR8_FLASH_BREAKPOINTS == 1
	struct gdb_break breaks[AVR8_MAX_BREAKS];
#elif AVR8_RAM_BREAKPOINTS == 1
	uint16_t breaks [AVR8_MAX_BREAKS];	/* Breakpoints */
	uint8_t singlestep_enabled;
	uint8_t breakpoint_enabled;		/* At least one BP is set */
#else
	#error BREAKPOINT configuration is not valid.
#endif
	uint8_t breaks_cnt;		/* number of valid breakpoints */
	uint8_t buff[AVR8_MAX_BUFF+1];
	uint8_t buff_sz;
};



/* EOF */




