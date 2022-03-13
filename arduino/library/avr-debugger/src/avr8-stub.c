/*
 * avr8-stub.c
 *
 *  Created on: 25. 3. 2015
 *      Author: Jan Dolinay
 *
 *  GDB Debugging Agent (GDB stub) for ATMega328
 *
 *  Note: This file contains stub for ATMega328 and ATMega 1280 (2560). Since April 2021 
 *  It also can cope with ATMega1284(P). Since it is very similar to ATMega1280, only a 
 *  few special had to be added.
 *  Code for ATMega 1280 is selected automatically based on define in the avr includes.
 *  ATMega328 is the default option otherwise; the code is:
 *  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
 *   - code for Arduino Mega
 *  #else
 *   - code for Arduino Uno
 *  #endif
 *  Define __AVR_ATmega328__ could be used for ATMega328 if needed.
 *  It is used since 3/2022 in the conditional code for UART selection.
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
#include <avr/wdt.h>
#include <util/delay.h>

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "avr8-stub.h"



/* Check for invalid configuration */
#if ( (AVR8_BREAKPOINT_MODE != 0) && (AVR8_BREAKPOINT_MODE != 1) && (AVR8_BREAKPOINT_MODE != 2) )
  #error Please select valid value of AVR8_BREAKPOINT_MODE in avr-stub.h
#endif

/* It's possible to support load with RAM BP, but not with Optiboot Flash BP, because
 * the Optiboot doesn't provide function for loading program from GDB. */
#if (AVR8_LOAD_SUPPORT == 1) && (AVR8_BREAKPOINT_MODE == 2)
  #error AVR8_LOAD_SUPPORT not available with the Optiboot bootloader. Please use AVR8_BREAKPOINT_MODE 0 or 1.
#endif

#if (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT == 1)
    /* Flash BP using our bootloader, so include our API */
	#include "app_api.h"	/* bootloader API used for writing to flash  */
#endif


#if (AVR8_BREAKPOINT_MODE == 2)
	/* Flash BP using Optiboot bootloader */

/*
 * Definitions below are from optiboot.h file provided with Optiboot test spm example.
 */

/*
 * Main 'magic' function - enter to bootloader do_spm function
 *
 * address - address to write (in bytes) but must be even number
 * command - one of __BOOT_PAGE_WRITE, __BOOT_PAGE_ERASE or __BOOT_PAGE_FILL
 * data - data to write in __BOOT_PAGE_FILL. In __BOOT_PAGE_ERASE or
 *          __BOOT_PAGE_WRITE it control if boot_rww_enable is run
 *         (0 = run, !0 = skip running boot_rww_enable)
 *
 */
// 'typedef' (in following line) and 'const' (few lines below)
//   are a way to define external function at some arbitrary address
typedef void (*do_spm_t)(uint16_t address, uint8_t command, uint16_t data);


/*
 * Devices with more than 64KB of flash:
 * - have larger bootloader area (1KB) (they are BIGBOOT targets)
 * - have RAMPZ register :-)
 * - need larger variable to hold address (pgmspace.h uses uint32_t)
 */
#ifdef RAMPZ
  typedef uint32_t optiboot_addr_t;
#else
  typedef uint16_t optiboot_addr_t;
#endif

#if FLASHEND > 65534
  const do_spm_t do_spm = (do_spm_t)((FLASHEND-1023+2)>>1);
#else
  const do_spm_t do_spm = (do_spm_t)((FLASHEND-511+2)>>1);
#endif

static void do_spm_cli(optiboot_addr_t address, uint8_t command, uint16_t data);
static void optiboot_page_erase(optiboot_addr_t address);
static void optiboot_page_fill(optiboot_addr_t address, uint16_t data);
static void optiboot_page_write(optiboot_addr_t address);

/* END of definitions from optiboot.h file*/


#define SPM_PAGESIZE_W (SPM_PAGESIZE>>1)
#define ROUNDUP(x, s) (((x) + (s) - 1) & ~((s) - 1))
#define ROUNDDOWN(x, s) ((x) & ~((s) - 1))

static uint8_t get_optiboot_major(void);
static void optiboot_safe_pgm_write(const void *ram_addr, optiboot_addr_t rom_addr, uint16_t sz);
static void optiboot_page_read_raw(optiboot_addr_t address, uint8_t output_data[]);
#endif

/* Define name of the flash writing routine */
#if (AVR8_BREAKPOINT_MODE == 0)
	/* out bootloader routine, see app_api.h */
	#define	flash_memory_write dboot_safe_pgm_write
#elif  (AVR8_BREAKPOINT_MODE == 2)
	/* routine in this file which calls Optiboot function from optiboot.h */
	#define	flash_memory_write optiboot_safe_pgm_write
#endif


typedef uint8_t bool_t;
#define FALSE 0
#define TRUE 1

/* For Arduino Mega flash breakpoints are only supported with the optiboot bootloader.
 * And load from debugger is not supported for Mega, so report this to the user */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
#if (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT==1)
	#error Flash breakpoints mode 0 and loading program from the debugger is not supported for Arduino Mega.
#endif
#endif

/* Configuration */

/* Baudrate configuration - based on Optiboot code */

/* Set default value if baudrate is not provided */
#ifndef AVR8_USER_BAUDRATE
#if F_CPU >= 8000000L
	#define GDB_USART_BAUDRATE   115200L
#elif F_CPU >= 1000000L
	#define GDB_USART_BAUDRATE   9600L
#elif F_CPU >= 128000L
	#define GDB_USART_BAUDRATE   4800L
#else
	#define GDB_USART_BAUDRATE 	1200L
#endif
#else
	#define GDB_USART_BAUDRATE AVR8_USER_BAUDRATE
#endif

/* Calculate the prescaler value for UART */
#define GDB_BAUD_PRESCALE (( (F_CPU + GDB_USART_BAUDRATE * 4L) / ((GDB_USART_BAUDRATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((GDB_BAUD_PRESCALE)+1)))

/* Check if the baudrate will work */
#if BAUD_ACTUAL <= GDB_USART_BAUDRATE
  #define BAUD_ERROR (( 100*(GDB_USART_BAUDRATE - BAUD_ACTUAL) ) / GDB_USART_BAUDRATE)
  #if BAUD_ERROR >= 5
    #error Baudrate off by more than -5%
  #elif BAUD_ERROR >= 3
    #warning Baud rate off by more than -3%
  #endif
#else
  #define BAUD_ERROR (( 100*(BAUD_ACTUAL - GDB_USART_BAUDRATE) ) / GDB_USART_BAUDRATE)
  #if BAUD_ERROR >= 5
    #error Baud rate off by more than 5%
  #elif BAUD_ERROR >= 3
    #warning Baud rate off by more than 3%
  #endif
#endif

/* check for slow baudrate */
#if GDB_BAUD_PRESCALE > 250
	#error Unachievable baud rate (too slow)
#endif
/* check for fast baudrate */
#if (GDB_BAUD_PRESCALE - 1) < 3
#if BAUD_ERROR != 0 	/* permit high bitrates (i.e. 1Mbps@16MHz) if error is zero */
	#error Unachievable baud rate (too fast)
#endif
#endif


/*
 * Macros used in this file which change value based on options set in header
 */

/* Symbols:
 * AVR8_SWINT_PIN 		- the pin used for generating SW interrupt
 * AVR8_SWINT_INTMASK 	- mask used in EIMSK register to enable the interrupt and in EIFR
 * 				  register to clear the flag.
 * AVR8_SWINT_SC        - sense control bits to enable low level interrupt
 * AVR8_SWINT_EICR      - the external interrupt control register
 * AVR8_SWINT_DDR       - data direction register
 * AVR8_SWINT_PORT      - port register
 * AVR8_SWINT_VECT      - the particular interrupt vector
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) 
	/* Arduino Mega configuration */
	#if AVR8_SWINT_SOURCE == 0
		#define	AVR8_SWINT_PIN		(PD0)
		#define AVR8_SWINT_INTMASK	(INTF0)
                #define AVR8_SWINT_SC           (~(_BV(ISC01) | _BV(ISC00)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRD
                #define AVR8_SWINT_PORT         PORTD
                #define AVR8_SWINT_VECT         INT0_vect
	#elif AVR8_SWINT_SOURCE == 1
		#define	AVR8_SWINT_PIN		(PD1)
		#define AVR8_SWINT_INTMASK	(INTF1)
                #define AVR8_SWINT_SC           (~(_BV(ISC11) | _BV(ISC10)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRD
                #define AVR8_SWINT_PORT         PORTD
                #define AVR8_SWINT_VECT         INT1_vect
	#elif AVR8_SWINT_SOURCE == 2
		#define	AVR8_SWINT_PIN		(PD2)
		#define AVR8_SWINT_INTMASK	(INTF2)
                #define AVR8_SWINT_SC           (~(_BV(ISC21) | _BV(ISC20)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRD
                #define AVR8_SWINT_PORT         PORTD
                #define AVR8_SWINT_VECT         INT2_vect
	#elif AVR8_SWINT_SOURCE == 3
		#define	AVR8_SWINT_PIN		(PD3)
		#define AVR8_SWINT_INTMASK	(INTF3)
                #define AVR8_SWINT_SC           (~(_BV(ISC31) | _BV(ISC30)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRD
                #define AVR8_SWINT_PORT         PORTD
                #define AVR8_SWINT_VECT         INT3_vect
	#elif AVR8_SWINT_SOURCE == 4
		#define	AVR8_SWINT_PIN		(PE4)
		#define AVR8_SWINT_INTMASK	(INTF4)
                #define AVR8_SWINT_SC           (~(_BV(ISC41) | _BV(ISC40)))
                #define AVR8_SWINT_EICR         EICRB
                #define AVR8_SWINT_DDR          DDRE
                #define AVR8_SWINT_PORT         PORTE
                #define AVR8_SWINT_VECT         INT4_vect
	#elif AVR8_SWINT_SOURCE == 5
		#define	AVR8_SWINT_PIN		(PE5)
		#define AVR8_SWINT_INTMASK	(INTF5)
                #define AVR8_SWINT_SC           (~(_BV(ISC51) | _BV(ISC50)))
                #define AVR8_SWINT_EICR         EICRB
                #define AVR8_SWINT_DDR          DDRE
                #define AVR8_SWINT_PORT         PORTE
                #define AVR8_SWINT_VECT         INT5_vect
	#elif AVR8_SWINT_SOURCE == 6
		#define	AVR8_SWINT_PIN		(PE6)
		#define AVR8_SWINT_INTMASK	(INTF6)
                #define AVR8_SWINT_SC           (~(_BV(ISC61) | _BV(ISC60)))
                #define AVR8_SWINT_EICR         EICRB
                #define AVR8_SWINT_DDR          DDRE
                #define AVR8_SWINT_PORT         PORTE
                #define AVR8_SWINT_VECT         INT6_vect
	#elif AVR8_SWINT_SOURCE == 7
		#define	AVR8_SWINT_PIN		(PE7)
		#define AVR8_SWINT_INTMASK	(INTF7)
                #define AVR8_SWINT_SC           (~(_BV(ISC71) | _BV(ISC70)))
                #define AVR8_SWINT_EICR         EICRB
                #define AVR8_SWINT_DDR          DDRE
                #define AVR8_SWINT_PORT         PORTE
                #define AVR8_SWINT_VECT         INT7_vect
	#elif AVR8_SWINT_SOURCE == -1
		#define AVR8_SWINT_INTMASK	(OCIE0A)
                #define AVR8_SWINT_VECT         TIMER0_COMPA_vect
	#else
		#error SW Interrupt source not valid. Please define AVR8_SWINT_SOURCE value 0 thru 7 or -1 in avr8-stub.h
	#endif
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 
	#if AVR8_SWINT_SOURCE == 0
		#define	AVR8_SWINT_PIN		(PD2)
		#define AVR8_SWINT_INTMASK	(INTF0)
                #define AVR8_SWINT_SC           (~(_BV(ISC01) | _BV(ISC00)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRD
                #define AVR8_SWINT_PORT         PORTD
                #define AVR8_SWINT_VECT         INT0_vect
	#elif AVR8_SWINT_SOURCE == 1
		#define	AVR8_SWINT_PIN		(PD3)
		#define AVR8_SWINT_INTMASK	(INTF1)
                #define AVR8_SWINT_SC           (~(_BV(ISC11) | _BV(ISC10)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRD
                #define AVR8_SWINT_PORT         PORTD
                #define AVR8_SWINT_VECT         INT1_vect
	#elif AVR8_SWINT_SOURCE == 2
		#define	AVR8_SWINT_PIN		(PB2)
		#define AVR8_SWINT_INTMASK	(INTF2)
                #define AVR8_SWINT_SC           (~(_BV(ISC21) | _BV(ISC20)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRB
                #define AVR8_SWINT_PORT         PORTB
                #define AVR8_SWINT_VECT         INT2_vect
	#elif AVR8_SWINT_SOURCE == -1
		#define AVR8_SWINT_INTMASK	(OCIE0A)
                #define AVR8_SWINT_VECT         TIMER0_COMPA_vect
        #else
		#error SW Interrupt source not valid. Please define AVR8_SWINT_SOURCE value 0 thru 2 or -1 in avr8-stub.h
	#endif
#else	/* Arduino Uno */
	#if AVR8_SWINT_SOURCE == 0
		#define	AVR8_SWINT_PIN		(PD2)
		#define AVR8_SWINT_INTMASK	(INT0)
                #define AVR8_SWINT_SC           (~(_BV(ISC01) | _BV(ISC00)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRD
                #define AVR8_SWINT_PORT         PORTD
                #define AVR8_SWINT_VECT         INT0_vect
	#elif AVR8_SWINT_SOURCE == 1
		#define	AVR8_SWINT_PIN		(PD3)
		#define AVR8_SWINT_INTMASK	(INT1)
                #define AVR8_SWINT_SC           (~(_BV(ISC11) | _BV(ISC10)))
                #define AVR8_SWINT_EICR         EICRA
                #define AVR8_SWINT_DDR          DDRD
                #define AVR8_SWINT_PORT         PORTD
                #define AVR8_SWINT_VECT         INT1_vect
	#elif AVR8_SWINT_SOURCE == -1
		#define AVR8_SWINT_INTMASK	(OCIE0A)
                #define AVR8_SWINT_VECT         TIMER0_COMPA_vect
	#else
#error SW Interrupt source not valid. Please define AVR8_SWINT_SOURCE 0, 1 or -1 in avr8-stub.h
	#endif
#endif

#if (AVR8_BREAKPOINT_MODE == 1) && (AVR8_SWINT_SOURCE == -1)
        #error When you use RAM breakpoints, you have to use one of the external interrupt pins as the software interrupt source
#endif

/*
 * Other defines and macros
 */
#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))
#define MIN(i1, i2) (i1 < i2 ? i1 : i2);

/** Size of the buffer we use for receiving messages from gdb in HEX.
  IMPORTANT: There is a decimal value below which must be in sync!
  AVR8_MAX_BUFF_HEX MUST BE IN HEX!!!
  because it is used in a macro and sent directly to GDB.
  The size must not be lower than 79 bytes (see gdb_read_registers)*/
#define AVR8_MAX_BUFF_HEX	60
/* This is decimal equivalent of AVR8_MAX_BUFF_HEX, which we used in code to allocate buffers etc.
   Keep this in sync with AVR8_MAX_BUFF_HEX!!! */
#define	AVR8_MAX_BUFF		(96)
/* Note:
 For binary load from GDB it is good to report the PacketSize 60 (96 bytes). Then GDB loads the
 program in packets of 0x50 bytes (80 B) which means each flash page suffers 2 erase cycles per load.
 If the packet size is smaller than half the page, each page will likely suffer 3 cycles per load.
 To get 1 erase per load gdb would need to send packet of exactly the page size but there are escaped chars
 so even if we tune the packet size to page size, less bytes will often be written.
*/



/* Reason the program stopped sent to GDB:
 * These are similar to unix signal numbers.
   See signum.h on unix systems for the values. */
#define GDB_SIGINT  2      /* Interrupt (ANSI). - user interrupted the program (UART ISR) */
#define GDB_SIGTRAP 5      /* Trace trap (POSIX). - stopped by this stub, e.g. on a breakpoint */


/* SRAM_OFFSET is hard-coded in GDB, it is there to map the separate address space of AVR (Harvard)
* to linear space used by GDB. So when GDB wants to read from RAM address 0 it asks our stub for
* address 0x00800000.
* FLASH_OFFSET is always 0
* MEM_SPACE_MASK is used to clear the RAM offset bit. It should not affect the highest possible
* address in flash which is 17-bit for Atmega2560, that is why 0xfffE0000.
*  */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 

	#define MEM_SPACE_MASK 0x00fe0000
	#define FLASH_OFFSET   0x00000000
	#define SRAM_OFFSET    0x00800000
	
	/* AVR puts garbage in high bits of return address on stack.
   	   Mask them out */
	// Atmega 1280 PC is 16 bit according to data sheet; Program memory is addressed by words, not bytes.
	// Atmega 2560 PC is 17 bits. We mask the 3rd (HIGH-HIGH) byte of PC in this case, not the 2nd (HIGH) byte.
#if defined(__AVR_ATmega2560__)
	#define RET_ADDR_MASK  0x01
#else
	#define RET_ADDR_MASK  0xff
#endif

#else
	/* This is for ATmega328 */
	#define MEM_SPACE_MASK 0x00ff0000
	#define FLASH_OFFSET   0x00000000
	#define SRAM_OFFSET    0x00800000	/* GDB works with linear address space; RAM address from GBD will be (real addresss + 0x00800000)*/

	/* AVR puts garbage in high bits of return address on stack.
   	   Mask them out
 	 The original version used mask 0x1f, which does not work for code above 16 kB!
	 In 1/2017 changed to 0x3f. The PC is 14 bits on Atmega328 (13 bits on Atmega168), we must
	 out the higher byte, so the mask is actually: 0x3fFF to keep 14 bits.
	 In the code this is used when reading 2 B from RAM to mask the first byte adr+0; adr+1 is not masked
	*/
	#define RET_ADDR_MASK  0x3f
#endif

/* UART definitions */

/* The number in AVR8_UART_NUMBER is used to generate generic registry names for the USART.
   The code in this stub then uses these generic names and the USART used by this stub
   can be changed simply by changing the AVR8_UART_NUMBER value in avr8-stub.h
*/

/* Check if AVR8_UART_NUMBER is valid for selected MCU */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) 
	/* ATmega1280 and ATmega2560 have 4 USARTs */
	#if AVR8_UART_NUMBER < 0 || AVR8_UART_NUMBER > 3
		#error For ATmega1280 or ATmega2560 only UART 0 to 3 can be used. Plese change the AVR8_UART_NUMBER to a number between 0 and 3.
	#endif
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 
	/* ATmega1284 has 2 USARTs */
	#if AVR8_UART_NUMBER != 0 && AVR8_UART_NUMBER != 1
		#error For ATmega1284 only UART 0 or 1 can be used. Plese change the AVR8_UART_NUMBER to 0 or 1.
	#endif
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
	/* There is only USART0 on ATmega328*/
	#if AVR8_UART_NUMBER != 0
		#error For ATmega328 only UART0 can be used. Plese change the AVR8_UART_NUMBER to 0.
	#endif
#else
	#error Selected MCU is not supported by this stub
#endif


/* A helper macro is needed to expand AVR8_UART_NUMBER macro to the actual number,
   because macro arguments used with ## operator are not expanded, so 
   we can't pass AVR8_UART_NUMBER directly to this macro, but must do so via the 
   next macro - this expands the AVR8_UART_NUMBER to its value. */
#define 	MAKE_UART_NAME_(name1, number, name2) name1 ## number ## name2
/* create name of the register or bit based on */
#define		MAKE_UART_NAME(name1 , num, name2 ) MAKE_UART_NAME_(name1, num, name2 )

/* Create the actual registry and bit names for UART */
#define 	UCSRA	MAKE_UART_NAME(UCSR, AVR8_UART_NUMBER, A)
#define 	UCSRB	MAKE_UART_NAME(UCSR, AVR8_UART_NUMBER, B)
#define		UCSRC	MAKE_UART_NAME(UCSR, AVR8_UART_NUMBER, C)
#define		UBRRH	MAKE_UART_NAME(UBRR, AVR8_UART_NUMBER, H)
#define		UBRRL	MAKE_UART_NAME(UBRR, AVR8_UART_NUMBER, L)
#define		RXEN	MAKE_UART_NAME(RXEN, AVR8_UART_NUMBER, )
#define		TXEN	MAKE_UART_NAME(TXEN, AVR8_UART_NUMBER, )
#define		UCSZ0	MAKE_UART_NAME(UCSZ, AVR8_UART_NUMBER, 0)
#define		UCSZ1	MAKE_UART_NAME(UCSZ, AVR8_UART_NUMBER, 1)
#define		RXCIE	MAKE_UART_NAME(RXCIE, AVR8_UART_NUMBER, )
#define		UDRE	MAKE_UART_NAME(UDRE, AVR8_UART_NUMBER, )
#define		UDR		MAKE_UART_NAME(UDR, AVR8_UART_NUMBER, )
#define		RXC		MAKE_UART_NAME(RXC, AVR8_UART_NUMBER, )
#define		U2X		MAKE_UART_NAME(U2X, AVR8_UART_NUMBER, )

/* Define the USART vector name */
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) 
	#define	UART_ISR_VECTOR	USART_RX_vect
#else
	/* the other MCUs have USART number in the ISR name */
	#define	MAKE_UART_VECT_(number)		USART ## number ## _RX_vect
	#define	MAKE_UART_VECT(num)			MAKE_UART_VECT_(num)
	#define	UART_ISR_VECTOR	MAKE_UART_VECT(AVR8_UART_NUMBER)	
#endif


/* To insert size of the buffer into PacketSize reply*/
#define STR(s) #s
#define STR_VAL(s) STR(s)

/* Data type to hold address of breakpoint.
   Devices with up to 16 bit PC will use uint16_t for breakpoint but Atmega 2560
   needs bigger data type */
#if defined(__AVR_ATmega2560__)
typedef uint32_t Address_t;
#else
typedef uint16_t Address_t;
#endif


#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )	/* Flash BP */
/* The opcode of instruction(s) we use for stopping the program at breakpoint.
 Instruction at the BP location is replaced by this opcode.
 To stop the program  we use RJMP on itself, i.e. endless loop,
 1100 kkkk kkkk kkkk, where 'k' is a -1 in words.
 #define TRAP_OPCODE 0xcfff
 To learn that BP was hit we will use periodic interrupt from watchdog.
 */
#define TRAP_OPCODE 0xcfff



/**
 Structure to hold information about a breakpoint in flash.
 */
struct gdb_break
{
	Address_t addr; /* in words */
	uint16_t opcode;
	uint8_t status;	/* status of the breakpoint in flash, see below */
	/*uint16_t opcode2;*/
};

/*
 The flags used for gdb_break.status:
 bit 0: is BP in flash? 1 = yes, 0 = no
 bit 1: is BP enabled now? 1 =yes, 0 = no
 To work with status use the macros below!
 */

/* Helper macros to manipulate breakpoint status */
/** mark this breakpoint as written to flash */
#define GDB_BREAK_SET_INFLASH(gdb_struct_ma)	(gdb_struct_ma).status |= 0x01;
/** mark this breakpoint as not written to flash */
#define GDB_BREAK_CLEAR_INFLASH(gdb_struct_ma)	(gdb_struct_ma).status &= ~0x01;
/** Test if breakpoint is in flash. Evaluates to true if BP is in flash. */
#define GDB_BREAK_IS_INFLASH(gdb_struct_ma)		((gdb_struct_ma).status & 0x01)
/** mark this breakpoint as currently enabled (GBD inserted it ) */
#define GDB_BREAK_ENABLE(gdb_struct_ma)		(gdb_struct_ma).status |= 0x02;
/** mark this breakpoint as disabled ( GDB removed it) */
#define GDB_BREAK_DISABLE(gdb_struct_ma)	(gdb_struct_ma).status &= ~0x02;
/** Test if breakpoint is enabled. Evaluates to true if BP is enabled */
#define GDB_BREAK_IS_ENABLED(gdb_struct_ma)		((gdb_struct_ma).status & 0x02)
/** Reset the status of the breakpoint - make it disabled and not in flash */
#define GDB_BREAK_CLEAR_STATUS(gdb_struct_ma) (gdb_struct_ma).status = 0;

/* Watchdog settings */
#define WATCHDOG_OFF    (0)	/* this off means no reset but interrupt is on*/
#define WATCHDOG_16MS   ((_BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_32MS   ((_BV(WDP0) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_64MS   ((_BV(WDP1) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_125MS  ((_BV(WDP1) | _BV(WDP0) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_250MS  ((_BV(WDP2) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_500MS  ((_BV(WDP2) | _BV(WDP0) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_1S     ((_BV(WDP2) | _BV(WDP1) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_2S     ((_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_4S     ((_BV(WDP3) | _BV(WDIE)) & (~_BV(WDE)))
#define WATCHDOG_8S     ((_BV(WDP3) | _BV(WDP0) | _BV(WDIE)) & (~_BV(WDE))))

/* One of the values above to really use in the code.
 This defines the delay between the target program hitting a breakpoint in flash
 and reporting it to GDB, so the shorter the better. But the shorter the delay
 the more is the program run influenced by the debugger.  */
#define	GDB_WATCHDOG_TIMEOUT	WATCHDOG_500MS


void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

/* Enable watchdog interrutp (but not system reset, just interrupt like another timer)
 * Function must be optimized or in asm to do the change in 4 cycles! */
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

#endif	/* AVR8_BREAKPOINT_MODE == 0 or 2 */

/* In order to avoid internal WDT interrupts/resets when NOT using the watchdog timer: */
#if  (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 0) || (AVR8_BREAKPOINT_MODE == 1)
#define WDTRESET()
#else 
#define WDTRESET() wdt_reset();
#endif

/**
 * Data used by this driver.
 */
struct gdb_context
{
    uint8_t singlestep_enabled; // made it the first field so that addressing this field is easy!
	uint16_t sp;
	Address_t pc; /* PC is 17-bit on ATmega2560*/


#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )
	/* Flash breakpoints */
    struct gdb_break breaks[AVR8_MAX_BREAKS*2+1]; /* so that we can choose AVR8_MAX_BREAKS new bps */
	//uint8_t breakpoint_step;		/* Indicates special step to skip breakpoint */
	//uint8_t skip_step;				/* Indicates special step from a breakpoint */

#elif ( AVR8_BREAKPOINT_MODE == 1 )
	/* RAM only breakpoints */

	/* On ATmega2560 the PC is 17-bit. We could use uint32_t for all MCUs, but
	 * that would be a waste of RAM and also all the manipulation with 32-bit will
	 * be much slower than with 16-bit variable, to we use Address_t which changes
	 * to 32 bits for ATmega2560.  */
	Address_t breaks [AVR8_MAX_BREAKS+1];	/* Breakpoints */
#endif

	uint8_t breakpoint_enabled;		/* At least one BP is set. This could be RAM only but it makes code easier to read if it exists in flash bp mode too. */
	uint8_t breaks_cnt;				/* number of valid breakpoints inserted */
	uint8_t buff[AVR8_MAX_BUFF+1];
	uint8_t buff_sz;
	uint8_t target_running;		/* 0 if target program is stopped */
};


/* Convert number 0-15 to hex */
#define nib2hex(i) (uint8_t)((i) > 9 ? 'a' - 10 + (i) : '0' + (i))


/*  Prototypes of internal functions */

/* UART routines
 * Names taken from GDB documentation for stub; int replaced by uint8_t */
static uint8_t getDebugChar(void);		/* Read a single character from the serial port */
static void putDebugChar(uint8_t c);	/* Write a single character to serial port */
static void uart_init(void);			/* Our function to initialize UART */
static void handle_exception(void);	/* Function called when the program stops */
static inline void gdb_enable_swinterrupt();
static inline void gdb_disable_swinterrupt();

static uint8_t hex2nib(uint8_t hex);
static uint8_t parse_hex(const uint8_t *buff, uint32_t *hex);
static void gdb_send_buff(const uint8_t *buff, uint8_t sz);
static void gdb_send_reply(const char *reply);
static bool_t gdb_parse_packet(const uint8_t *buff);
static void gdb_send_state(uint8_t signo);
static void gdb_write_registers(const uint8_t *buff);
static void gdb_read_registers(void);
static void gdb_write_memory(const uint8_t *buff);
static void gdb_read_memory(const uint8_t *buff);
static void gdb_insert_remove_breakpoint(const uint8_t *buff);
static bool_t gdb_insert_breakpoint(Address_t rom_addr);
static void gdb_remove_breakpoint(Address_t rom_addr);


#if ( AVR8_BREAKPOINT_MODE == 0 || (AVR8_BREAKPOINT_MODE == 2) )	/* code is for flash BP only */
	static void gdb_update_breakpoints(void);
#endif



static inline void restore_regs (void);
static inline void save_regs1 (void);
static inline void save_regs2 (void);
static inline void quickcheck (void);

static uint8_t safe_pgm_read_byte(uint32_t rom_addr_b);

/* Helpers for determining required size of our stack and for printing to console*/
#ifdef AVR8_STUB_DEBUG
#define		GDB_STACK_CANARY	(0xBA)
uint8_t test_check_stack_usage(void);
static void wfill_stack_canary(uint8_t* buff, uint16_t size);
static uint8_t wcheck_stack_usage(uint8_t* buff, uint16_t size );	/* returns how many bytes are used from given buffer */
/* Helper for writing debug message to console when debugging this debugger */
static void test_print_hex(const char* text, uint16_t num);
#endif	/* AVR8_STUB_DEBUG */


#ifdef AVR8_STUB_DEBUG
/* You can use these variables to debug this stub.
   Display them in the Expressions window in eclipse
*/
uint8_t G_Debug_INTxCount = 0;	/* How many times external INTx ISR was called*/
uint8_t G_BpEnabledINTx = 0;	/* How many times external INTx ISR was called and bp were enabled*/
uint16_t G_LastPC = 0;			/* Value of PC register in INTx ISR */
uint32_t G_BreakpointAdr = 0;	/* Address of the breakpoint last set/written to flash */
uint16_t G_StepCmdCount = 0;	/* Counter for step commands from GDB */
uint8_t G_ContinueCmdCount = 0;	/* Counter for continue commands from gdb */
uint32_t G_RestoreOpcode = 0;	/* Opcode(s) which were restored in flash when removing breakpoint */
uint8_t	G_StackUnused = 0;		/* used for testing stack size only*/

/* Helper macros to work with LED*/
#if 0
/* LED is on pin PB7 on Arduino Mega, PD5 on Arduino Uno (Arduino pin 13) */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 
#define		AVR_LED_PIN		(7)
#else
#define		AVR_LED_PIN		(5)
#endif
#define	AVR8_LEDINIT()  DDRB |= _BV(AVR_LED_PIN);	/* pin to output mode */
#define	AVR8_LEDON() PORTB |= _BV(AVR_LED_PIN)
#define	AVR8_LEDOFF() PORTB &= ~_BV(AVR_LED_PIN)

static void avr8_led_blink(void) {
	uint16_t n = 10000;
	AVR8_LEDINIT();
	AVR8_LEDON();
	while(n--) ;
	AVR8_LEDOFF();
	n = 10000;
	while(n--) ;
}
#endif

#if 0
/* Simple trace for logging the execution path. Can be used to debug the stub.
Call start_trace() before the problematic code.
Call stop_trace() e.g. in handle_exception for the case 0x03 - debugger break.
Call add_trace('some unique symbol') into the code where you suspect the problem.
While debugging the target, look into the variable gtrace_buffer to see the execution
path. Perhaps you will find out where the program hangs....
*/
void add_trace(char info);
#define		TRACE_MAX_DATA		(128)
char gtrace_buffer[TRACE_MAX_DATA];
uint8_t gcurrent_trace;
uint8_t genable_trace;
void start_trace() { 
	memset(gtrace_buffer, ' ', sizeof(gtrace_buffer)); 
	gcurrent_trace = 0;
	genable_trace = 1;
	add_trace('S');
}
void stop_trace() {
	add_trace('X');
	genable_trace = 0;
}
void add_trace(char info) {
	if ( genable_trace && gcurrent_trace < TRACE_MAX_DATA ) {
		gtrace_buffer[gcurrent_trace] = info;
		gcurrent_trace++;
	}
}
#endif

#endif  /* AVR8_STUB_DEBUG */

/* Debugging using the logic analyzer */
#ifdef AVR8_LA_DEBUG
void la_init(void)
{
  DDRB |= 0x1F; //PC4 is clock, PC0-PC3 is parallel data
  PORTB |= 0x10; // clock line to high
}

void la_send(uint8_t digit)
{
  _delay_us(5);
  PORTB = (0x10 | digit);
  _delay_us(20);
  PORTB &= ~_BV(PC4);
  _delay_us(25);
  PORTB |= _BV(PC4);
}

void la_pause(void)
{
  PORTB = 0x10;
  _delay_us(200);
}
#endif

#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )		/* Flash breakpoints */
	static uint16_t safe_pgm_read_word(uint32_t rom_addr_b);
	static struct gdb_break *gdb_find_break(Address_t rom_addr);
	static void gdb_remove_breakpoint_ptr(struct gdb_break *breakp);
#endif

/* Code used only if flash writing is needed */
#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT == 1) || (AVR8_BREAKPOINT_MODE == 2))
	static void gdb_no_bootloder_prep(void);
#endif

/* Global variables */

/* Our context and pointer to this context.
 * The pointer is used in the original code which "creates" the struct on stack in ISR.
 * I keep it so as not to change all the functions even though the context is saved to regs array. */
static struct gdb_context ctx;
static struct gdb_context *gdb_ctx;

/* String for PacketSize reply to gdb query.
 * Note: if running out of RAM the reply to qSupported packet can be removed. */
static char* gdb_str_packetsz = "PacketSize=" STR_VAL(AVR8_MAX_BUFF_HEX);

/* PC is 17-bit on ATmega2560; we need 1 more byte for registers but since we will
 * work with the PC as uint32 we need one extra byte; the code then assumes this byte
 * is always zero. */
#if defined(__AVR_ATmega2560__)
	#define GDB_NUMREGBYTES	(39)			/* Total bytes in registers */
#else
	#define GDB_NUMREGBYTES	(37)
#endif

/* GDB_STACKSIZE - size of the internal stack used by this stub.
 * According to tests in 5/2017 with flash breakpoints
 * there are 78 bytes of stack used. With RAM breakpoints 49 B used.
 * It worked OK with stack size 96 for flash BP and 72 for RAM BP.
 * But in 1/2018 with new avr toolchain with load via debugger enabled the stack size
 * was not enough. 116 B was required for flash BP and 124 for RAM BP.
 * So the size was changed to be extra large if load-via-debugger is enabled and
 * smaller if load-via-debugger is not enabled.
 * For Flash mode using optiboot make the stack larger, see Flash page buffer below,
 */
#if (AVR8_LOAD_SUPPORT == 1)
	#define GDB_STACKSIZE 	(144)
#else
	#if (AVR8_BREAKPOINT_MODE == 1)
		#define GDB_STACKSIZE 	(80)
		/* Internal stack size for RAM only BP */
	#elif (AVR8_BREAKPOINT_MODE == 0)
		#define GDB_STACKSIZE 	(104)
	#else
		/* stack size for writing to flash with Optiboot, see note below */
		#define GDB_STACKSIZE 	((uint16_t)(140 + SPM_PAGESIZE))
	#endif
#endif

/* Note on stack size with writing to flash using Optiboot do_spm().
 * SPM_PAGESIZE is flash page size in bytes.
 * We call Optiboot function to handle erase and write to flash memory
 * but our code is located in the user-app memory section called RWW and that's why it
 * cannot execute while flash programming is in progress. So we have to use the sequence
 * erase-fill-write. (fill means fill built-in buffer for writing to flash).
 *   We cannot use sequence fill-erase-write because after erase the CPU needs to read the next
 * command (write) from memory - which is not possible without re-enabling access to the flash memory.
 * But re-enabling access resets the fill buffer and we lose what we filled into it.
 * So to preserve the original content and just change one word to set the breakpoint
 * we need to read the page into some RAM buffer before erasing it.
 *
 * If the code handling the erase and write was in NRWW section, we could do
 * fill-erase-write and would't need this RAM buffer. This is the case with our bootloader which provides
 * complete function to set the breakpoint but not with Optiboot which only provides function for
 * one step like write, erase or fill.
 *
 * Now where should we place the RAM buffer?
 * This stub uses it's own stack so if we define the buffer as a local variable, we need to increase
 * our stack - this is what I do.
 * If we define the buffer as global variable, we don't need bigger stack but the RAM is used up.
 * So global or local makes no difference.
 * I choose local, that is increase the stack because this way all the code can benefit from
 * larger stack. If I create separate global variable for the buffer, the RAM usage is the same
 * and I don't get the larger stack.
 * Note that our stack is just a global variable "stack" defined here, so making the buffer
 * global variable would mean just having two global smaller variables instead of one bigger.
 *
 */

static char stack[GDB_STACKSIZE];			/* Internal stack used by the stub */
static unsigned char regs[GDB_NUMREGBYTES];	/* Copy of all registers */

#if defined(__AVR_ATmega2560__)
	#define R_PC	*(uint32_t*)(regs+35)	/* Copy of PC register */
#else
	#define R_PC	*(uint16_t*)(regs+35)	/* Copy of PC register */
#endif
#define	R_PC_H  *(uint8_t*)(regs+36)	/* High byte of the Copy of the PC */
#if defined(__AVR_ATmega2560__)	/* PC is 17-bit on ATmega2560*/
	#define	R_PC_HH  *(uint8_t*)(regs+37)	/* High-High byte of the Copy of the PC */
#endif
#define R_SP	*(uint16_t*)(regs+33)	/* Copy of SP register */
#define R_SREG	*(uint8_t*)(regs+32)	/* Copy of SREG register */

#if FLASHEND > 0x1FFFF
#       define ASM_GOTO "jmp "
#elif FLASHEND > 0x2000
#       define ASM_GOTO "jmp "
#else
#       define ASM_GOTO "rjmp "
#endif



/* ---------- User interface (API) for this driver ------------- */

/* Initialize the debug driver. */
void debug_init(void)
{
	/* Init gdb context */
	gdb_ctx = &ctx;
	gdb_ctx->sp = 0;
	gdb_ctx->breaks_cnt = 0;
	gdb_ctx->buff_sz = 0;
	gdb_ctx->singlestep_enabled = 0;	/* single step is used also in Flash BP mode */
	gdb_ctx->target_running = 0;

	/* Init breaks */
	memset(gdb_ctx->breaks, 0, sizeof(gdb_ctx->breaks));


#if (AVR8_BREAKPOINT_MODE == 1)		/* RAM BP */
	gdb_ctx->breakpoint_enabled = 0;
#endif

#ifdef AVR8_STUB_DEBUG
	/* For testing stack usage only - fill stack with canary values  */
	wfill_stack_canary((uint8_t*)stack, GDB_STACKSIZE);
#endif

	/* Initialize serial port */
	uart_init();

#if ((AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT == 1)) && (AVR8_BREAKPOINT_MODE != 2)
	/* Flash BP or load binary using our bootloader */

	/* Initialize bootloader API */
	uint8_t result = dboot_init_api();
	/* If there's an error, hang the app here and the user will see it in the debugger */
	if ( result != BOOT_OK ) {
		gdb_no_bootloder_prep();
		while(1) ;	/* Bootloader API not found. Do you have the bootloader with avr_debug support in your board? */
	}

	uint8_t version;
	dboot_get_api_version(&version);
	if ( version < BOOT_API_VERSION) {
		gdb_no_bootloder_prep();
		while(1) ;	/* Bootloader API is too old. Please update the bootloader in your board. */
	}

#elif (AVR8_BREAKPOINT_MODE == 2)
	/* Flash BP using Optiboot bootloader */

	/* Check for do_spm() support in Optiboot - added in version 8 */
	/* Arduino Mega: The default bootloader is not optiboot and this check will pass
	 * without error; there is probably 0xff where we check the version. Do NOT assume that
	 * You do need to replace the bootloader with optiboot even if your program is not stopped here.
	 * There will be no error but the debugging will not work - it just hangs in stepping. */
	uint8_t optiboot_major = get_optiboot_major();
	if ( optiboot_major < 8 ) {
		/* Writing to flash in bootloader is not available.
		   Note that this check is not foolproof because if the Optiboot is not
		   official but some custom version, the number can be higher and still based
		   on older version without the do_spm support.  */
		gdb_no_bootloder_prep();
		while(1) ;   /* Bootloader too old; no support for writing to flash. */
		/* Please update bootloader in your board to Optiboot version 8.0 or newer. */
	}
#endif

#if ((AVR8_USE_TIMER0_INSTEAD_OF_WDT == 1) || (AVR8_SWINT_SOURCE == -1)) && (AVR8_BREAKPOINT_MODE != 1)
 	/* initialize the timer0 OC0A interrupt */
 	OCR0A = 0x7F; /* raise an interrupt between two "millis" interrupts */
 	TIMSK0 |= _BV(OCIE0A); /* enable the interrupt */
#endif

}

#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT == 1) || (AVR8_BREAKPOINT_MODE == 2) )
/* Flash BP or load binary supported */

/* This is used to report to the user that flash breakpoints or load are enabled but
   the bootloader does not support this.
   For Arduino code, which uses timer interrupts before our debug_init is called
   we cannot simply fall into endless loop and let the user see the situation in debugger
   because the program will likely stop in timer ISR handlers. So we disable timer
   interrupts and then wait. */
__attribute__((optimize("-Os")))
static void gdb_no_bootloder_prep(void) {

	/* IMPORTANT: if you find yourself here it means you have enabled flash breakpoints and/or
	   load via debugger but your board does not have the bootloader to support this.
	   Please burn the bootloader provided for this debugger to use the flash breakpoints and/or load.
	 */
	cli();

	/* disable all timer interrupts */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 

	TIMSK0 &= ~(_BV(TOIE0) | _BV(OCIE0A) | _BV(OCIE0B));
	TIMSK2 &= ~(_BV(TOIE2) | _BV(OCIE2A) | _BV(OCIE2B));

	TIMSK1 &= ~(_BV(TOIE1) | _BV(OCIE1A) | _BV(OCIE1B) | _BV(OCIE1B) | _BV(ICIE1));
	TIMSK3 &= ~(_BV(TOIE3) | _BV(OCIE3A) | _BV(OCIE3B) | _BV(OCIE3B) | _BV(ICIE3));
	
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	TIMSK4 &= ~(_BV(TOIE4) | _BV(OCIE4A) | _BV(OCIE4B) | _BV(OCIE4B) | _BV(ICIE4));
	TIMSK5 &= ~(_BV(TOIE5) | _BV(OCIE5A) | _BV(OCIE5B) | _BV(OCIE5B) | _BV(ICIE5));
#endif

#else

	TIMSK0 &= ~(_BV(TOIE0) | _BV(OCIE0A) | _BV(OCIE0B));
	TIMSK1 &= ~(_BV(TOIE1) | _BV(OCIE1A) | _BV(OCIE1B) | _BV(ICIE1));
	TIMSK2 &= ~(_BV(TOIE2) | _BV(OCIE2A) | _BV(OCIE2B));
#endif
	sei();  /* enable interrupts to allow communication with us */
}
#endif /* AVR8_BREAKPOINT_MODE == 0... */


/* ------------ User interface (API) for this driver ---------------- */


/* ---------- UART communication routines  ------------- */

/* Initialize UART */
static void uart_init(void)
{
	/* Init UART */
	UCSRA = _BV(U2X);		/* double UART speed */
	UCSRB = (1 << RXEN ) | (1 << TXEN );		/* enable RX and Tx */
	UCSRC =  (1 << UCSZ0 ) | (1 << UCSZ1 ); /* Use 8- bit character sizes */
	UBRRH = ( GDB_BAUD_PRESCALE >> 8);
	UBRRL = GDB_BAUD_PRESCALE;
	UCSRB |= (1 << RXCIE ); /* Enable the USART Recieve Complete interrupt ( USART_RXC ) */

}

/* Read a single character from the serial port */
static uint8_t getDebugChar(void)
{
	/* wait for data to arrive */
	while ( !(UCSRA & (1<<RXC)) )
		WDTRESET();

	return (uint8_t)UDR;
}

/* Write a single character to serial port */
static void putDebugChar(uint8_t c)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) )
		WDTRESET();

	/* Put data into buffer, sends the data */
	UDR = c;
}
/* ---------------- end UART communication routines ---------------------------------- */



/* ---------- Debugging driver routines  ------------- */
/**
 * Stimulate interrupt;
 * Used for single stepping or when any breakpoint in RAM is set to stop the
 * program after every instruction.
 * The AVR core will always execute one instruction in the main code before
 * jumping into ISR even if interrupt is pending. We set the INT0 to trigger when
 * pin is low and set the pin low. If Flash breakpoints are used and we use Timer0 for
 * periodic interrupts instead of WDT, then we will utilize the output compare 
 * interrupts as software interrupts, so we do not need any interrupt pin in this case. */
__attribute__((always_inline))
static inline void gdb_enable_swinterrupt()
{

#if (AVR8_SWINT_SOURCE == -1) /* Use TIMER0_COMPA interrupt */
        OCR0A = TCNT0;
	/* The counter might have been advanced while writing to the register. So in order to
         * gurantee an immediate match, one has to write again to the register */
        OCR0A = TCNT0; 
#else
	AVR8_SWINT_EICR &= AVR8_SWINT_SC;
	/* The pin needs to be configured as output to allow us to set the
	 * level on the pin and thus generate the interrupt*/
	AVR8_SWINT_DDR |= _BV(AVR8_SWINT_PIN);		/* set pin to output mode */
	EIFR |= _BV(AVR8_SWINT_INTMASK);	/* clear INTx flag */
	EIMSK |= _BV(AVR8_SWINT_INTMASK);	/* enable INTx interrupt */
	AVR8_SWINT_PORT &= ~_BV(AVR8_SWINT_PIN);		/* make sure the pin is low */
#endif 
}

/** Disable the interrupt used for single stepping and RAM breakpoints. */
__attribute__((always_inline))
static inline void gdb_disable_swinterrupt()
{
#if (AVR8_SWINT_SOURCE != -1)
	EIMSK &= ~_BV(AVR8_SWINT_INTMASK);
#else
	OCR0A = TCNT0 + 0x7F; /* next IRQ in 500 usec */
	TIFR0 |= _BV(OCF0A);  /* clear compare flag */ 
#endif
}

/** Macro which is true if there is a pending interrupt from UART Rx
 * */
#define	UART_RXINT_PENDING()  (UCSRA & (1<<RXC))

/* handle_exception
 * Called when the program stops; communicates with the GDB.
 * The name follows the GDB stub documentation:
 * This is the central workhorse.
 * This is where the communications protocol is implemented;
 * handle_exception acts as the gdb representative on the target machine.
 * It begins by sending summary information on the state of your program,
 * then continues to execute, retrieving and transmitting any information gdb needs,
 * until you execute a gdb command that makes your program resume;
 * at that point, handle_exception returns control to your own code on the target machine.
 * */
__attribute__((optimize("-Os")))
static void handle_exception(void)
{
	uint8_t checksum, pkt_checksum;
	uint8_t b;

	gdb_ctx->singlestep_enabled = 0;		/* stepping by single instruction is enabled below for each step */


	while (1) {
	    WDTRESET();

		b = getDebugChar();
		
		switch(b) {
		case '$':
			/* Read everything to buffer */
			gdb_ctx->buff_sz = 0;
			for (pkt_checksum = 0, b = getDebugChar();
				 b != '#'; b = getDebugChar())
			{
				gdb_ctx->buff[gdb_ctx->buff_sz++] = b;
				pkt_checksum += b;
				WDTRESET();
			}
			gdb_ctx->buff[gdb_ctx->buff_sz] = 0;

			checksum  = hex2nib(getDebugChar()) << 4;
			checksum |= hex2nib(getDebugChar());

			/* send nack in case of wrong checksum  */
			if (pkt_checksum != checksum) {
				putDebugChar('-');
				continue;
			}

			/* ack */
			putDebugChar('+');

			/* parse already read buffer */
			if (gdb_parse_packet(gdb_ctx->buff))
				continue;

			/* If too many breakpoints, we stop immediately */
			/* This is alternative version of reporting the situation to user instead of
			  returning error code from gdb_insert_remove_breakpoint(). It requires that the
			  gdb_insert_breakpoint allows inserting one breakpoint over the limit of AVR8_MAX_BREAKS.
			if (gdb_ctx->breaks_cnt > AVR8_MAX_BREAKS) {
			  do {
			    debug_message("Too many breakpoints in use\n");
			    WDTRESET();
			    b = getDebugChar();
			  } while (b == '-');
			  gdb_send_state(GDB_SIGTRAP);
			  gdb_ctx->target_running = 0;
			  gdb_ctx->singlestep_enabled = FALSE;
			  break;
			}*/
			 

			if(gdb_ctx->singlestep_enabled || gdb_ctx->breakpoint_enabled)
			{
				/* this will generate interrupt after one instruction in main code */
				WDTRESET();
				gdb_enable_swinterrupt();

			}
			else
			{
				/* Do not go to ext int ISR after every instruction. For RAM breakpoints this is
				  the case if the program is let run and it can be only stopped by break command (pause).
				  For flash breakpoints it is the same plus the program can also break itself when it encounters
				  breakpoint in flash.	*/
				WDTRESET();
				gdb_disable_swinterrupt();
			}

			/* leave the trap, continue execution */
			return;

		case '-':  /* NACK, repeat previous reply */
			gdb_send_buff(gdb_ctx->buff, gdb_ctx->buff_sz);
			WDTRESET();
			break;
		
		case '+':  /* ACK, great */
			/* there is special case we need to handle - if we send something to the GDB while the 
			target is running, and GDB returns ACK, we need to let the target run instead of 
			waiting here for commands from GDB - because there will be none and the target hangs.
			This happens with the debug_message() which can be sent while target is running. 
			Probably it would be safe to always return from here as the UART ISR would call us again when
			another char arrives, but the GDB stub documentation above states that we should wait for
			communication until there is command to run the target... */
			if ( gdb_ctx->target_running ) 
				return;				
			break;
		
		case 0x03:					
			/* user interrupt by Ctrl-C, send current state and
			   continue reading */
			WDTRESET();
			gdb_ctx->target_running = 0;	/* stopped by debugger break */
			gdb_send_state(GDB_SIGINT);
			break;

		default:
			WDTRESET();
			gdb_send_reply(""); /* not supported */
			break;
		}	// switch
	}	// while 1

	/* should never get here, see while(1) above. */		

}

/* This is the main "dispatcher" of commands from GDB
 * If returns false, the debugged program continues execution */
__attribute__((optimize("-Os")))
static bool_t gdb_parse_packet(const uint8_t *buff)
{
#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )	/* code is for flash BP only */
	Address_t pc = gdb_ctx->pc;	// PC with word address
	struct gdb_break* pbreak;
#endif

#ifdef AVR8_STUB_DEBUG
	G_StackUnused = test_check_stack_usage();
#endif
	
	switch (*buff) {
	case '?':               /* last signal */
		gdb_send_reply("S05"); /* signal # 5 is SIGTRAP */
		break;
	case 'H':               /* Set thread, always OK */
		gdb_send_reply("OK");
		break;
	case 'T':               /* Is thread alive, always OK */
		gdb_send_reply("OK");
		break;
	case 'g':               /* read registers */
		gdb_read_registers();
		break;
	case 'G':               /* write registers */
		gdb_write_registers(buff + 1);
		break;

	case 'm':               /* read memory */
		gdb_read_memory(buff + 1);
		break;
	case 'M':               /* write memory */
		gdb_write_memory(buff + 1);
		break;

		/* Support loading program from gdb */
#if (AVR8_LOAD_SUPPORT == 1)
	case 'X':
		/* Call bootloader which will receive the data and then restart the app including us.
		 GDB first sends packet X with 0 size to test whether the stub supports binary load.
		 If we do support it we reply OK, we call the bootloader code to handle it.
		 If we do not support it, GDB will try to load using memory write.
		*/
		gdb_send_reply("OK");
		dboot_handle_xload();
		break;
#endif

	case 'D':               /* detach the debugger */
	case 'k':               /* kill request */
#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )	/* code is for flash BP only */
		/* Update the flash so that the program can run after reset without breakpoints */
		gdb_update_breakpoints();
#endif
		gdb_send_reply("OK");
		gdb_ctx->target_running = 1;
		return FALSE;

	case 'c':               /* continue */

#ifdef AVR8_STUB_DEBUG
		G_ContinueCmdCount++;
#endif /* AVR8_STUB_DEBUG */

#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )	/* code is for flash BP only */
		/* Enable watchdog interrupt. It is automatically disabled when the ISR
		 is called so it must re-enable itself if needed */
		gdb_update_breakpoints();
		/* todo: could enable only if at least one BP is set */
  #if (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 0)
		watchdogConfig(GDB_WATCHDOG_TIMEOUT);
  #endif
#endif
		gdb_ctx->target_running = 1;
		return FALSE;

	case 'C':               /* continue with signal */
	case 'S':               /* step with signal */
		gdb_send_reply(""); /* not supported */
		break;

	case 's':               /* step */

#ifdef AVR8_STUB_DEBUG
		G_StepCmdCount++;
#endif /* AVR8_STUB_DEBUG */

#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )	/* code is for flash BP only */
		/* Updating breakpoints is needed, otherwise it would not be
		 possible to step out from breakpoint. We need to replace our trap-code with the
		 original instruction and execute it when stepping from breakpoint.

		 Stepping from breakpoint can occur not only after the program stops on BP,
		 but also if we step through the code and step onto breakpoint. */

		/* If we stopped on break, update breakpoints but do not update if we step from
		 non-breakpoint position. */
		pbreak = gdb_find_break(pc);
		if ( pbreak ) {
			gdb_update_breakpoints();
			/* todo: not needed?
			 If we step from flash breakpoint (which is call instruction) from main loop code,
			  then EXT INT ISR will be executed right after we return and we would stop the program
			  at the same address. We just set flag for the ISR not to signal state  */
		//	gdb_ctx->skip_step = 1;
		} else {
		//	gdb_ctx->skip_step = 0;
		}
#endif

		gdb_ctx->singlestep_enabled = 1;		
		return FALSE;

	case 'z':               /* remove break/watch point */
	case 'Z':               /* insert break/watch point */
		gdb_insert_remove_breakpoint(gdb_ctx->buff);
		break;

	case 'q':               /* query requests */
		if(memcmp_PF(buff, (uintptr_t)PSTR("qSupported"), 10) == 0) {
			/* plain send, from ram */
			gdb_send_buff((uint8_t*)gdb_str_packetsz, strlen(gdb_str_packetsz));
		}
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qC"), 2) == 0) {
			/* current thread is always 1 */
			gdb_send_reply("QC01");
		}
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qfThreadInfo"), 12) == 0) {
			/* always 1 thread*/
			gdb_send_reply("m1");
	    }
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qsThreadInfo"), 12) == 0) {
			/* send end of list */
			gdb_send_reply("l");
		}
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qRcmd,7265736574"), 16) == 0) {
			/* reset target */
#if (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2)
		        gdb_update_breakpoints(); // needed to gid rid of breakpoints
#endif
			gdb_send_reply("OK");
			/* recommended way to reset - activate watchdog */
			/* note: on newer ARVs including ATmega328 once enabled, the watchdog will stay 
			 enabled even after reset; the software should disable it. It is disabled by 
			 the optiboot bootloader on Arduino so all works fine. If you use your own hardware
			 without optiboot you may need to disable the watchdog or it will keep reseting your
			 program after the monitor reset restart. */
			wdt_enable(WDTO_15MS);  
    		while(1) ;		
		}
		else {
			gdb_send_reply("");  /* not supported */
		}

		break;

	default:
		gdb_send_reply("");  /* not supported */
		break;
	}
	
	return TRUE;
}


#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )	/* code is for flash BP only */

#ifdef AVR8_LA_DEBUG
void show_la_breaks(uint8_t typ)
{
  la_pause();
  la_pause();
  la_send(typ);
  la_pause();
  la_send(gdb_ctx->breaks_cnt);
  la_pause();
  for (uint8_t i=0; i < AVR8_MAX_BREAKS*2+1; i++) {
    la_send(gdb_ctx->breaks[i].addr ? 1 : 0);
    la_send(gdb_ctx->breaks[i].status);
    la_pause();
  }
  la_pause();
  la_pause();
}
#endif

/**
 Called before the target starts to run in order to write/remove breakpoints in flash.
 Note that the breakpoints are inserted to gdb_ctx->breaks at the first free position
 (free means the addr is 0). And a breakpoint is permanently removed by setting the addr to 0.
 A breakpoint is marked as disabled when the debugger issues a remove breakpoint command
 after the target has stopped. We leave the trap opcode in the flash memory in order to minimize
 flash wear. The trap opcode is only removed, when the breakpoint is not re-enabled at the next start.
 
 Note that in order to be able to detect that more breakpoints than allowed are used, we have to give
 room for one more entry above the maximum number of entries. Further, because we may want to set the maximum
 number of new breakpoints (although the entries are taken up by breakpoints that are still in flash, but disabled),
 we need double as many entries, i.e., all in all we need 4*2+1 entries. Note that the maximum number of
 disabled breakpoints still in flash is 4. It cannot be 5, because in this case we would not start to write 
 breakpoints in flash (see variable 'enabled' below). In order to guarantee that there is always a free entry
 when a breakpoint is inserted (and we have not more than the maximum number + 1), we have to make sure
 that a breakpoint entry is freed if it is not in flash and disabled (see remove_breakpoint).
 
 Note that breaks_cnt counts only the enabled breakpoints. So the disabled breakpoints, which are still 
 in flash, are not counted (but we know that there cannoz be more than 4).

 In a first run over the list of breakpoints, we check whether there are too many enabled breakpoints. 
 If this is the case, we will not write enabled breakpoints to flash, because the target won't start 
 in any case.
 */
__attribute__((optimize("-Os")))
static void gdb_update_breakpoints(void)
{
	uint16_t trap_opcode = TRAP_OPCODE;
	uint8_t i;
	/*uint8_t enabled = 0;*/

#ifdef AVR8_LA_DEBUG
	la_init();
	show_la_breaks(1);
#endif
	/* insert_breakpoint function should not allow enabling more than max breakpoints
	 so no need to check it here
	for (i=0; i < AVR8_MAX_BREAKS*2+1; i++)
	  if (GDB_BREAK_IS_ENABLED(gdb_ctx->breaks[i])) enabled++; */

	for (i = 0; i < AVR8_MAX_BREAKS * 2 + 1; i++) {
		WDTRESET();

		/* Ignore free breakpoint structs */
		if (!gdb_ctx->breaks[i].addr)
			continue;

		/* Possible cases:
		 1) BP is enabled and already in flash > do nothing
		 2) BP is enabled but not in flash > write to flash, set IN_FLASH flag (if not too many BPs)
		 3) BP is disabled but written in flash > remove from flash,free BP struct
		 4) BP is disabled and not in flash > free only BP struct
		 */
		if (GDB_BREAK_IS_ENABLED(gdb_ctx->breaks[i])) {
			/* BP should be inserted... */
			if (!GDB_BREAK_IS_INFLASH(gdb_ctx->breaks[i]) /*&& (enabled <= AVR8_MAX_BREAKS)*/) {
				/* ...and it is not in flash, so write it (2) */
				gdb_ctx->breaks[i].opcode = safe_pgm_read_word((uint32_t) (gdb_ctx->breaks[i].addr << 1));
				/*gdb_ctx->breaks[i].opcode2 = safe_pgm_read_word((uint32_t)((gdb_ctx->breaks[i].addr << 1)+2));*//* opcode replaced by our infinite loop trap */
				flash_memory_write(&trap_opcode, gdb_ctx->breaks[i].addr, sizeof(trap_opcode));
				GDB_BREAK_SET_INFLASH(gdb_ctx->breaks[i]);
			} /* else do nothing (1)*/

		} else {
			/* BP should be removed... */
			if (GDB_BREAK_IS_INFLASH(gdb_ctx->breaks[i])) {
				/* ...and it is in flash, so remove it, also free the struct  (3) */
				gdb_remove_breakpoint_ptr(&gdb_ctx->breaks[i]);
			} else {
				/* If it is not in flash, just free the struct (4) */
				gdb_ctx->breaks[i].addr = 0;
				/* The status is actually 0 in this case but it won't hurt to reset it. */
				GDB_BREAK_CLEAR_STATUS(gdb_ctx->breaks[i]);
			}
		}
	} /* for */
#ifdef AVR8_LA_DEBUG
	show_la_breaks(0x0F);
#endif
}
#endif	/* AVR8_BREAKPOINT_MODE == 0 or 2 */

__attribute__((optimize("-Os")))
static void gdb_insert_remove_breakpoint(const uint8_t *buff)
{
	uint32_t rom_addr_b, sz;
	uint8_t len;
	/* skip 'z0,' */
	len = parse_hex(buff + 3, &rom_addr_b);
	/* skip 'z0,xxx,' */
	parse_hex(buff + 3 + len + 1, &sz);

	/* get break type */
	switch (buff[1]) {
	case '0': /* software breakpoint */
		if (buff[0] == 'Z') {
			if ( gdb_insert_breakpoint(rom_addr_b >> 1) )
				gdb_send_reply("OK");
			else {
				gdb_send_reply("E50");	/* Unable to set breakpoint */
				/* The error code does not seem to make any difference... */
			}
		}
		else {
			gdb_remove_breakpoint(rom_addr_b >> 1);
			gdb_send_reply("OK");
		}
		break;

	default:
		/* we do not support other breakpoints, only software */
		gdb_send_reply("");
		break;
	}
}

/* rom_addr is in words */
__attribute__((optimize("-Os")))
static bool_t gdb_insert_breakpoint(Address_t rom_addr)
{
#ifdef AVR8_STUB_DEBUG
	G_BreakpointAdr = rom_addr << 1;	/* convert to byte address */
#endif

#if (AVR8_BREAKPOINT_MODE == 1)		/* RAM only breakpoints */
	uint8_t i;
	Address_t* p = gdb_ctx->breaks;
	/* First look if the breakpoint already exists */
	for (i = gdb_ctx->breaks_cnt; i > 0; i--)
	{
		if (*p++ == rom_addr)
			return TRUE;
	}

	if ( gdb_ctx->breaks_cnt >= AVR8_MAX_BREAKS)
		return FALSE;	/* no more breakpoints available */

	gdb_ctx->breaks[gdb_ctx->breaks_cnt++] = rom_addr;
	gdb_ctx->breakpoint_enabled = 1;	/* at least one breakpoint exists */
	return TRUE;

#else
	/* Breakpoints in flash */

	/* Note: GDB will always delete all breakpoints (BP) when the program stops
	 * on a BP or after step. It will then set them again before continue (c)
	 * or step.
	 * With flash BP we should buffer the requests and only rewrite flash if
	 * any BP really changed...see gdb_update_breakpoints
	 * */

	/* original code for flash breakpoints */
	uint8_t i;

	/* Check number of enabled breakpoints first, so we never allow more than
	 AVR8_MAX_BREAKS breakpoints enabled. */
	if (gdb_ctx->breaks_cnt >= AVR8_MAX_BREAKS)
		return FALSE; /* we already have more than the maximum number of enabled BPs */

	/* First, try to find the breakpoint if it is already inserted */
	struct gdb_break *breakp = gdb_find_break(rom_addr);
	if (breakp) {
		/* Breakpoint exists but it may be disabled, so enable it*/
		if (!GDB_BREAK_IS_ENABLED(*breakp)) {
			GDB_BREAK_ENABLE((*breakp));
			/* Increment counter of enabled bp only it was not already enabled  */
			gdb_ctx->breaks_cnt++;
		}
		/* else - it was already enabled so just return OK. */
		return TRUE;
	}

	/* If breakpoint is not found, add a new one */

	/* find first BP struct which is free - that is addr is 0 and store the BP there */
	/* Although there might be AVR8_MAX_BREAKS disabled BPs in flash, there must still be a free one! */
	for (i = 0; i < AVR8_MAX_BREAKS * 2 + 1; i++) {
		if (!gdb_ctx->breaks[i].addr) {
			gdb_ctx->breaks_cnt++;
			gdb_ctx->breaks[i].addr = rom_addr;
			GDB_BREAK_ENABLE(gdb_ctx->breaks[i]);
			return TRUE;
		}
	}

	return FALSE; /* should never get here but play it safe*/


#endif	/* AVR8_BREAKPOINT_MODE */
}

static void gdb_remove_breakpoint(Address_t rom_addr)
{
#if (AVR8_BREAKPOINT_MODE == 1)  /* RAM only BP*/
	uint8_t i, j;

	for (i = 0, j = 0; j < gdb_ctx->breaks_cnt; i++, j++)
	{
		/* i is target, j is source index */
		if ( gdb_ctx->breaks[i] == rom_addr )
		{
			/* this is the BP to remove */
			j++;
			if ( j >= gdb_ctx->breaks_cnt )
			{
				gdb_ctx->breaks[i] = 0;
				break;		/* removing the last BP in the array */
			}
		}

		/* normally, this will do nothing but after the breakpoint-to-be-removed is found and
		 * j is incremented, it will shift the remaining breakpoints. */
		gdb_ctx->breaks[i] = gdb_ctx->breaks[j];

	}	// end for

	if ( j > i )	{/* if we found the BP to be removed, there is now one less */
		gdb_ctx->breaks_cnt--;
		gdb_ctx->breaks[i] = 0;
	}

	if ( gdb_ctx->breaks_cnt == 0 )
		gdb_ctx->breakpoint_enabled = 0;	/* if there are no breakpoints */

#else
	/* Combined mode - BPs in flash */
	struct gdb_break *breakp = gdb_find_break(rom_addr);
	/* Just mark the breakpoint for removal but do not update flash */
	if (breakp) {

		if (GDB_BREAK_IS_ENABLED((*breakp))) {
			GDB_BREAK_DISABLE((*breakp));
			gdb_ctx->breaks_cnt--;	/* we count number of enabled breakpoints */
		}

		if (!GDB_BREAK_IS_INFLASH((*breakp))) {
			/* if not in flash, free structure and reset the status */
			breakp->addr = 0;
			GDB_BREAK_CLEAR_STATUS((*breakp));
		}
	}
	/*gdb_remove_breakpoint_ptr(breakp);*/
#endif
}

/* ----------------- Functions for flash breakpoints support ------------------- */
#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )

static uint16_t safe_pgm_read_word(uint32_t rom_addr_b)
{
#ifdef pgm_read_word_far
	if (rom_addr_b >= (1l<<16))
		return pgm_read_word_far(rom_addr_b);
	else
#endif
		return pgm_read_word(rom_addr_b);
}

static void gdb_remove_breakpoint_ptr(struct gdb_break *breakp)
{
	/*
	uint32_t opcode;
	opcode = (uint32_t)breakp->opcode2 << 16;
	opcode |=  breakp->opcode;
	*/
	uint16_t opcode = breakp->opcode;

#ifdef AVR8_STUB_DEBUG
	G_RestoreOpcode = opcode;
#endif /* AVR8_STUB_DEBUG */

	flash_memory_write(&opcode, breakp->addr, sizeof(opcode));
	breakp->addr = 0;
	GDB_BREAK_CLEAR_STATUS((*breakp));

}

/* rom_addr is in words */
static struct gdb_break *gdb_find_break(Address_t rom_addr)
{
  uint8_t i = 0, sz = AVR8_MAX_BREAKS*2+1;
	/* do search */
	for (; i < sz; ++i)
		if (gdb_ctx->breaks[i].addr == rom_addr)
			return &gdb_ctx->breaks[i];

	return NULL;
}

#endif	/*  AVR8_FLASH_BREAKPOINTS */

/* END Functions for flash breakpoints version only */



/* GDB needs the 32 8-bit, gpw registers (r00 - r31), the
   8-bit SREG, the 16-bit SP (stack pointer) and the 32-bit PC
   (program counter). Thus need to send a reply with
   r00, r01, ..., r31, SREG, SPL, SPH, PCL, PCH,
   low bytes before high since AVR is little endian.
   This routine requires (32 gpwr, SREG, SP, PC) * 2 hex bytes
   space of buffer, i.e. min (32 + 1 + 2 + 4) * 2 = 78 */
__attribute__((optimize("-Os")))
static void gdb_read_registers(void)
{
	uint8_t a;
	uint16_t b;
	char c;
	uint32_t pc = (uint32_t)gdb_ctx->pc << 1;	/* convert word address to byte address used by gdb */
	uint8_t i = 0;

	a = 32;	/* in the loop, send R0 thru R31 */
	b = (uint16_t) &regs;

	do {
		c = *(char*)b++;
		gdb_ctx->buff[i++] = nib2hex((c >> 4) & 0xf);
		gdb_ctx->buff[i++] = nib2hex((c >> 0) & 0xf);

	} while (--a > 0);

	/* send SREG as 32 register */
	gdb_ctx->buff[i++] = nib2hex((R_SREG >> 4) & 0xf);
	gdb_ctx->buff[i++] = nib2hex((R_SREG >> 0) & 0xf);

	/* send SP as 33 register */
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 4)  & 0xf);
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 0)  & 0xf);
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 12) & 0xf);
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 8)  & 0xf);

	/* send PC as 34 register
	 gdb stores PC in a 32 bit value.
	 gdb thinks PC is bytes into flash, not in words. */
	gdb_ctx->buff[i++] = nib2hex((pc >> 4)  & 0xf);
	gdb_ctx->buff[i++] = nib2hex((pc >> 0)  & 0xf);
	gdb_ctx->buff[i++] = nib2hex((pc >> 12) & 0xf);
	gdb_ctx->buff[i++] = nib2hex((pc >> 8)  & 0xf);
#if defined(__AVR_ATmega2560__)
	gdb_ctx->buff[i++] = nib2hex((pc >> 20) & 0xf);
#else
	gdb_ctx->buff[i++] = '0'; /* For AVR with up to 16-bit PC */
#endif
	gdb_ctx->buff[i++] = nib2hex((pc >> 16) & 0xf);
	gdb_ctx->buff[i++] = '0'; /* gdb wants 32-bit value, send 0 */
	gdb_ctx->buff[i++] = '0'; /* gdb wants 32-bit value, send 0 */

	gdb_ctx->buff_sz = i;
	gdb_send_buff(gdb_ctx->buff, gdb_ctx->buff_sz);

}

__attribute__((optimize("-Os")))
static void gdb_write_registers(const uint8_t *buff)
{

	uint8_t a;
	uint32_t pc;

	a = 32;	/* in the loop, receive R0 thru R31 */
	uint8_t *ptr = regs;

	do {
		*ptr  = hex2nib(*buff++) << 4;
		*ptr |= hex2nib(*buff++);
	} while (--a > 0);

	/* receive SREG as 32 register */
	R_SREG = hex2nib(*buff++) << 4;
	R_SREG |= hex2nib(*buff++);

	/* receive SP as 33 register */
	gdb_ctx->sp  = hex2nib(*buff++) << 4;
	gdb_ctx->sp |= hex2nib(*buff++);
	gdb_ctx->sp |= hex2nib(*buff++) << 12;
	gdb_ctx->sp |= hex2nib(*buff++) << 8;

	/* receive PC as 34 register
	   gdb stores PC in a 32 bit value.
	   gdb thinks PC is bytes into flash, not in words. */
	pc  = hex2nib(*buff++) << 4;
	pc |= hex2nib(*buff++);
	pc |= hex2nib(*buff++) << 12;
	pc |= hex2nib(*buff++) << 8;
	pc |= (uint32_t)hex2nib(*buff++) << 20;
	pc |= (uint32_t)hex2nib(*buff++) << 16;
	pc |= (uint32_t)hex2nib(*buff++) << 28;
	pc |= (uint32_t)hex2nib(*buff++) << 24;
	gdb_ctx->pc = pc >> 1;	/* drop the lowest bit; PC addresses words */

	gdb_send_reply("OK");

}

__attribute__((optimize("-Os")))
static void gdb_read_memory(const uint8_t *buff)
{
	uint32_t addr, sz;
	uint8_t i;

	buff += parse_hex(buff, &addr);
	/* skip 'xxx,' */
	parse_hex(buff + 1, &sz);

	if ((addr & MEM_SPACE_MASK) == SRAM_OFFSET) {
		addr &= ~MEM_SPACE_MASK;
		uint8_t *ptr = (uint8_t*)(uintptr_t)addr;
		for (i = 0; i < sz; ++i) {
			uint8_t b = ptr[i];
			/* XXX: this is ugly kludge, but what can I do?
					AVR puts return address on stack with garbage in high
					bits (they say you should mask out them, see Stack Pointer
					section at every AVR datasheet), but how can I understand
					that this word is ret address? To have valid backtrace in
					gdb, I'am required to mask every word, which address belongs
					to stack. */
#if 0 // leads to wrong values of local variables!
#if defined(__AVR_ATmega2560__)
			/* TODO: for ATmega2560 the 3rd byte of PC should be masked out, but
			 * how do we know when the 3rd byte is read?
			 * The code for other derivatives can mask every word, but for 2560 we need to mask
			 * only one byte of one of the two words that GDB reads...or does GDB read 3 bytes?
			 * If yes, the code should be: */
			 if (i == 0 && sz == 3 && addr >= gdb_ctx->sp)
				b &= RET_ADDR_MASK;
#else
			if (i == 0 && sz == 2 && addr >= gdb_ctx->sp)
				b &= RET_ADDR_MASK;
#endif
#endif
			gdb_ctx->buff[i*2 + 0] = nib2hex(b >> 4);
			gdb_ctx->buff[i*2 + 1] = nib2hex(b & 0xf);
		}
	}
	else if ((addr & MEM_SPACE_MASK) == FLASH_OFFSET){
		addr &= ~MEM_SPACE_MASK;
		for (i = 0; i < sz; ++i) {
			uint8_t byte = safe_pgm_read_byte(addr + i);
			gdb_ctx->buff[i*2 + 0] = nib2hex(byte >> 4);
			gdb_ctx->buff[i*2 + 1] = nib2hex(byte & 0xf);
		}
	}
	else {
		/* posix EIO error */
		gdb_send_reply("E05");
		return;
	}
	gdb_ctx->buff_sz = sz * 2;
	gdb_send_buff(gdb_ctx->buff, gdb_ctx->buff_sz);
}

__attribute__((optimize("-Os")))
static void gdb_write_memory(const uint8_t *buff)
{
	uint32_t addr, sz;
	uint8_t i;

	buff += parse_hex(buff, &addr);
	/* skip 'xxx,' */
	buff += parse_hex(buff + 1, &sz);
	/* skip , and : delimiters */
	buff += 2;

	if ((addr & MEM_SPACE_MASK) == SRAM_OFFSET) {
		addr &= ~MEM_SPACE_MASK;
		uint8_t *ptr = (uint8_t*)(uintptr_t)addr;
		for ( i = 0; i < sz; ++i) {
			ptr[i]  = hex2nib(*buff++) << 4;
			ptr[i] |= hex2nib(*buff++);
		}
	}
	else if ((addr & MEM_SPACE_MASK) == FLASH_OFFSET){
		/* posix EIO error */
		gdb_send_reply("E05");
		return;
#if 0	/* writing to flash not supported - but it could be as of 4/2017, use flash_memory_write
 	 	 warning: if enabled, fix the code in parse_packet which handles binary load (X). If case we
 	 	 report we do not support binary load, GDB would load the program using memory write, that is this
 	 	 code. But we cannot overwrite ourselves so we cannot support this.
 	 	 So in case writing to flash is desired feature, enable it here by calling bootloader api, but for
 	 	 X packet report supported and then errors oif hand the app to let user know something is wrong if he
 	 	 tries to load the app from GDB but does not enable support for it in this stub.
 	 	 */
		addr &= ~MEM_SPACE_MASK;
		/* to words */
		addr >>= 1;
		/* we assume sz is always multiple of two, i.e. write words */
		for (uint8_t i = 0; i < sz/2; ++i) {
			uint16_t word;
			word  = hex2nib(*buff++) << 4;
			word |= hex2nib(*buff++);
			word |= hex2nib(*buff++) << 12;
			word |= hex2nib(*buff++) << 8;
			safe_pgm_write(&word, addr + i, sizeof(word));
		}
#endif
	}
	else {
		/* posix EIO error */
		gdb_send_reply("E05");
		return;
	}
	gdb_send_reply("OK");
}

#if 0
/** Support for binary load of program */
uint8_t tmp_buff[128];	/* atmega328 has 128 B page. GDM max for binary write is 256 but we do not support it?*/

__attribute__((optimize("-Os")))
static void gdb_write_binary(const uint8_t *buff) {
	uint32_t addr, sz;
	uint8_t i;

	buff += parse_hex(buff, &addr);
	/* skip 'xxx,' */
	buff += parse_hex(buff + 1, &sz);
	/* skip , and : delimiters */
	buff += 2;

	if ((addr & MEM_SPACE_MASK) == SRAM_OFFSET) {
		addr &= ~MEM_SPACE_MASK;
		gdb_send_reply("E05"); /* do not support binary write */
		return;
	} else if ((addr & MEM_SPACE_MASK) == FLASH_OFFSET) {
		/* Write to flash. GDB sends binary data; not characters in hex */
		bin2mem(buff, (unsigned char *)tmp_buff, sz);
		addr &= ~MEM_SPACE_MASK;
		/* to words */
		addr >>= 1;

#if 0	/* writing to flash not supported - but it could be as of 4/2017, use flash_memory_write */
		addr &= ~MEM_SPACE_MASK;
		/* to words */
		addr >>= 1;
		/* we assume sz is always multiple of two, i.e. write words */
		for (uint8_t i = 0; i < sz/2; ++i) {
			uint16_t word;
			word = hex2nib(*buff++) << 4;
			word |= hex2nib(*buff++);
			word |= hex2nib(*buff++) << 12;
			word |= hex2nib(*buff++) << 8;
			safe_pgm_write(&word, addr + i, sizeof(word));
		}
#endif
	} else {
		/* posix EIO error */
		gdb_send_reply("E05");
		return;
	}
	gdb_send_reply("OK");

}

/* Convert the binary stream in BUF to memory.

   Gdb will escape $, #, and the escape char (0x7d).
   COUNT is the total number of bytes to write into
   memory.
   Code from gdb sample m32r-stub.c */
__attribute__((optimize("-Os")))
static unsigned char *bin2mem(unsigned char *buf, unsigned char *mem, int count) {
	int i;
	//unsigned char ch;

	for (i = 0; i < count; i++) {
		/* Check for any escaped characters. Be paranoid and
		 only unescape chars that should be escaped. */
		if (*buf == 0x7d) {
			switch (*(buf + 1)) {
			case 0x3: /* # */
			case 0x4: /* $ */
			case 0x5d: /* escape char */
				buf++;
				*buf |= 0x20;
				break;
			default:
				/* nothing */
				break;
			}
		}

		*mem = *buf++;
		mem++;
	}

	return mem;
}
#endif

/* ------------------------------------------------------------- */



/* ---------- Interrupt handlers ------------- */
/* Note: GDB stub is normally expected to implement
 * void exceptionHandler (int exception_number, void *exception_address)
 * to install exception handler for given exception type (e.g. Illegal opcode),
 * but we do not need this.*/

/*
 * Interrupt handler for received character from serial port.
 * Having this allows breaking the program during execution from GDB. */
ISR(UART_ISR_VECTOR, ISR_BLOCK ISR_NAKED)
{
	save_regs1 ();
	/* save_regs1 loads SREG into r29 */

	/* Sets interrupt flag = interrupts enabled; but not in real, just in the stored value */
	/* asm volatile ("ori r29, 0x80");	*/ /* user must see interrupts enabled */
	/* Disabled - I don't know why th euser should see it enabled... */
	save_regs2 ();

	
#if defined(__AVR_ATmega2560__)
	R_PC_HH &= 0x01;		/* there is only 1 bit used in the highest byte of PC (17-bit PC) */
	/* No need to mask R_PC_H */
#else
	R_PC_H &= RET_ADDR_MASK;
#endif
	gdb_ctx->pc = R_PC;
	gdb_ctx->sp = R_SP;	
	
	/* Communicate with gdb */
	handle_exception();	

	asm volatile (
		      "restore_registers:");
	restore_regs ();

	asm volatile (
			"out	__SREG__, r31\n"	/* restore SREG */
			"lds	r31, regs+31\n"		/* real value of r31 */
			"reti \n");
	/* simplified exit code - just restore SREG, interrupts will be enabled because we execute RETI */

#if 0
	asm volatile (
		"sbrs	r31, 7\n"		/* test I flag */
		"rjmp	1f\n"
		"andi	r31, 0x7f\n"		/* clear I flag */
		"out	__SREG__, r31\n"	/* restore SREG */
		"lds	r31, regs+31\n"		/* real value of r31 */
		"reti\n"			/* exit with interrupts enabled */
	"1:	out	__SREG__, r31\n"	/* exit with interrupts disabled */
		"lds	r31, regs+31\n"	/* real value of r31 */
		"ret\n");			/* exit with interrupts disabled (reti would enable them) */
#endif
}


/*
 * Interrupt handler for the interrupt which allows single-stepping using the
 * feature of AVR that after each interrupt, one instruction of main program
 * is executed before any pending interrupt service routine is called.
 * Names such as INT0_vect are the same for Atmega328 and Atmega 1280/2560
 *
 * It is not used if:
 * TIMER0 is used both to check for breakpoints (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 1)
 * and for single-stepping (AVR8_SWINT_SOURCE == -1), because in that case
 * the TIMER0 ISR is used for both purposes.
 * #if !( (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 1) && (AVR8_SWINT_SOURCE == -1)) */
#if (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 0) || (AVR8_SWINT_SOURCE != -1)

ISR(AVR8_SWINT_VECT, ISR_BLOCK ISR_NAKED )
{
#if (AVR8_BREAKPOINT_MODE == 1)/* RAM only BPs */
	static uint8_t ind_bks;
#endif

	save_regs1 ();
	/*asm volatile ("ori r29, 0x80");*/	/* user must see interrupts enabled */
	/* removed enabling the interrupt for user - don't know why they should see it enabled... */

	save_regs2 ();
#if defined(__AVR_ATmega2560__)
	R_PC_HH &= RET_ADDR_MASK;		/* there is only 1 bit used in the highest byte of PC (17-bit PC) */
#else
	R_PC_H &= RET_ADDR_MASK;
#endif
	/* Note that for ATmega2560 we allocate 4 bytes for PC in the regs array so that it is safe to
	 cast to uint32 by the R_PC macro. */
	gdb_ctx->pc = R_PC;
	gdb_ctx->sp = R_SP;


#ifdef AVR8_STUB_DEBUG
	G_Debug_INTxCount++;
	G_LastPC = gdb_ctx->pc << 1;	/* convert to byte address */
#endif

	/* if single-stepping, go to trap */
	if ( gdb_ctx->singlestep_enabled)
		goto trap;


#if (AVR8_BREAKPOINT_MODE == 1)/* RAM only BPs */
	if ( gdb_ctx->breakpoint_enabled )
	{
#ifdef AVR8_STUB_DEBUG
		G_BpEnabledINTx++;
#endif

		/*Fix 2018-03-24 - if char received in UART handle it or it may be interrupt request */
		if ( UART_RXINT_PENDING() ) {
			/* option 1 - just disable sw interrupt - but then any char received on uart while
			 * target is running would stop it and stub would wait for communication */
			/* gdb_disable_swinterrupt();*/
			/* option 2 - check if the char received is ctrl+c and if yes, send signal and go handle it */
			ind_bks = getDebugChar();
			if ( ind_bks == 0x03 ) {
				gdb_ctx->target_running = 0;	/* stopped on a breakpoint or after step */
				/* need to send state as we already read the command */
				gdb_send_state(GDB_SIGINT);
				/* UART ISR will be executed when we exit and handle further communication */
				gdb_disable_swinterrupt();				
				goto out;
			}			
		}

		/* find breakpoint */
		for (ind_bks = 0; ind_bks < gdb_ctx->breaks_cnt; ind_bks++)
		{
			if (gdb_ctx->breaks[ind_bks] == gdb_ctx->pc)
				goto trap;
		}
	}
#endif 	/* AVR8_BREAKPOINT_MODE */

	/* Nothing */
	goto out;

trap:

	
	gdb_ctx->target_running = 0;	/* stopped on a breakpoint or after step */
	gdb_send_state(GDB_SIGTRAP);	
	handle_exception();

out:

	/* If there is pending interrupt from UART (char received), handle it - it may be
	 * request to interrupt the target. Because INT0 has higher
	 * priority, it will not allow the UART ISR to execute and this the user is not able
	 * to pause the debugged program if breakpoint is set, has to wait for the breakpoint to
	 * be hit...and if it is never hit, unreachable, there's no way to get back to debugger.
	 * 2018-03-24 Realized we DO need to handle UART here. It was not handled since 20.6.2017,
	 * see comment below. But we need to do this only for RAM breakpoints, not for single step,
	 * In single step the control goes to UART handler after each instruction.
	 *  */
#if 0  /* seems safer not to disable here (20.6.2017) */
	if ( UART_RXINT_PENDING() ) {
		gdb_disable_swinterrupt();
	}
#endif


	restore_regs ();
	asm volatile (
			"out	__SREG__, r31\n"	/* restore SREG */
			"lds	r31, regs+31\n"		/* real value of r31 */
			"reti \n");

#if 0
	asm volatile (
			"sbrs	r31, 7\n"		/* test I flag; skip if bit set = skip if interrupts enabled */
			"rjmp	1f\n"
			"andi	r31, 0x7f\n"		/* clear I flag */
			"out	__SREG__, r31\n"	/* restore SREG */
			"lds	r31, regs+31\n"		/* real value of r31 */
			"reti\n"			/* exit with interrupts enabled */
			"1:	out	__SREG__, r31\n"	/* exit with interrupts disabled */
			"lds	r31, regs+31\n"	/* real value of r31 */
			"ret\n");			/* exit with interrupts disabled (reti would enable them) */
#endif
}
#endif /* (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 0) */


/* This function must be naked, otherwise the stack can be corrupted
 * and save_regs1() assumes stack is not touched. */
__attribute__((naked))
void breakpoint(void)
{
	save_regs1 ();		/* also saves SREG */
	save_regs2 ();

#if defined(__AVR_ATmega2560__)
	R_PC_HH &= RET_ADDR_MASK;		/* there is only 1 bit used in the highest byte of PC (17-bit PC) */
#else
	R_PC_H &= RET_ADDR_MASK;
#endif


	gdb_ctx->pc = R_PC;
	//gdb_ctx->pc = R_PC;
	gdb_ctx->sp = R_SP;

	gdb_ctx->target_running = 0;
	gdb_send_state(GDB_SIGTRAP);
	handle_exception();

	/* jump without return */
	/*asm volatile (ASM_GOTO " restore_registers"); */
	/* cannot just jump to shared code because it ends with RETI which would enable interrupts
	 which is not OK as we may be called with disabled interrupts */

	restore_regs ();	/* loads SREG to r31 */
	asm volatile (
		"out	__SREG__, r31\n"	/* restore SREG */
		"lds	r31, regs+31\n"		/* real value of r31 */
		"ret \n");
}


/*
 * Send text message to gdb console.
 * Note: GDB will queue the messages until "\n" (0x0a) is received; then
 * it displays the messages in console.
 */
void debug_message(const char* msg)
{
	uint8_t cSREG;
	char c;
	uint8_t i = 0;
	cSREG = SREG; /* store SREG value */
	cli();	/* disable interrupts */


	gdb_ctx->buff[i++] = 'O';	/* message to display in gdb console */
	while ( *msg && i < (AVR8_MAX_BUFF-4) )
	{
		c = *msg++;
		gdb_ctx->buff[i++] = nib2hex((c >> 4) & 0xf);
		gdb_ctx->buff[i++] = nib2hex((c >> 0) & 0xf);
	}

	/* Add "\n" */
	gdb_ctx->buff[i++] = '0';
	gdb_ctx->buff[i++] = 'a';

	gdb_send_buff(gdb_ctx->buff , i);
	
	SREG = cSREG; /* restore SREG value (I-bit) */
	/* Example packet we send:
	 $O48656c6c6f2c20776f726c64210a#55 = Hello, world!\n
	 */
}



/*
 * Interrupt handler for watchdog which allows checking whether the program
 * is stopped on a breakpoint (inserted into the code as RJMP -1 (while(1))
 * instruction instead of the original instruction.
 */
/* Watchdog/timer interrupt vector
 * When using TIMER0 instead of watchdog this ISR can also handle single-stepping
 * instead of the separate single-stepping ISR defined above, if AVR8_SWINT_SOURCE == -1 */
#if ( (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2) )	/* code is for flash BP only */
#if (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 1)
ISR(TIMER0_COMPA_vect, ISR_BLOCK ISR_NAKED)
#else
ISR(WDT_vect, ISR_BLOCK ISR_NAKED)
#endif
{
    quickcheck(); /* Do a quick check and RETI if no need to go through entire ISR */
	save_regs1();
	save_regs2 ();

#if defined(__AVR_ATmega2560__)
	R_PC_HH &= 0x01;		/* there is only 1 bit used in the highest byte of PC (17-bit PC) */
	/* No need to mask R_PC_H */
#else
	R_PC_H &= RET_ADDR_MASK;
#endif
	gdb_ctx->pc = R_PC;
	gdb_ctx->sp = R_SP;


	/* Check breakpoint */
	if (gdb_find_break(gdb_ctx->pc))
		goto trap;

#if (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 1) /* could probably always be done */
	if ( gdb_ctx->singlestep_enabled)
		goto trap;
#endif

	/* Nothing */
	/* Re-enable watchdog interrupt as it is disabled when ISR is run.
	  Do this only if we are not on a breakpoint yet, so we can check again next time.
	  If we are on a BP, no need to run this ISR again. It will be re-enabled when the program
	  continues running.  */
#if (AVR8_USE_TIMER0_INSTEAD_OF_WDT == 0)
	watchdogConfig(GDB_WATCHDOG_TIMEOUT);
#endif
	goto out;

trap:
	/* Set correct interrupt reason */
	gdb_ctx->target_running = 0;	/* stopped on a breakpoint */
	gdb_send_state(GDB_SIGTRAP);	
	handle_exception();

out:
	/* this saves memory, jump to the same code instead of repeating it here */
	
asm volatile (ASM_GOTO " restore_registers");
#if 0
	restore_regs ();
	asm volatile (
			"out	__SREG__, r31\n"	/* restore SREG */
			"lds	r31, regs+31\n"		/* real value of r31 */
			"reti \n");
#endif
}
#endif /*  (AVR8_BREAKPOINT_MODE == 0) || (AVR8_BREAKPOINT_MODE == 2)   */
/* ------------------------------------------------------------- */

/* ---------- GDB RCP packet processing  ------------- */

/* Simpler version */
__attribute__((optimize("-Os")))
static void gdb_send_buff(const uint8_t *buff, uint8_t sz)
{
	uint8_t sum = 0;

	putDebugChar('$');
	while ( sz-- > 0)
	{
		putDebugChar(*buff);
		sum += *buff;
		buff++;
	}

	putDebugChar('#');
	putDebugChar(nib2hex((sum >> 4) & 0xf));
	putDebugChar(nib2hex(sum & 0xf));
}

__attribute__((optimize("-Os")))
static void gdb_send_reply(const char *reply)
{
	gdb_ctx->buff_sz = strlen(reply);
	if ( gdb_ctx->buff_sz > (AVR8_MAX_BUFF - 4))
		gdb_ctx->buff_sz = AVR8_MAX_BUFF - 4;

	memcpy(gdb_ctx->buff, reply, gdb_ctx->buff_sz);
	gdb_send_buff(gdb_ctx->buff, gdb_ctx->buff_sz);

}

/* Note: we send "expedited response T" which contains also the SREG, SP and PC
 * to speed things up.
 * Minimal response is the "last signal" response, for example $S05#b8, but then
 * GDB needs to read registers by another message.. */
__attribute__((optimize("-Os")))
static void gdb_send_state(uint8_t signo)
{
	uint32_t pc = (uint32_t)gdb_ctx->pc << 1;


	/* thread is always 1 */
	/* Real packet sent is e.g. T0520:82;21:fb08;22:021b0000;thread:1;
	 05 - last signal response
	 Numbers 20, 21,... are number of register in hex in the same order as in read registers
	 20 (32 dec) = SREG
	 21 (33 dec) = SP
	 22 (34 dec) = PC
	 */
	memcpy_P(gdb_ctx->buff,
			 PSTR("TXX20:XX;21:XXXX;22:XXXXXXXX;thread:1;"),
			 38);
	gdb_ctx->buff_sz = 38;

	/* signo */
	gdb_ctx->buff[1] = nib2hex((signo >> 4)  & 0xf);
	gdb_ctx->buff[2] = nib2hex(signo & 0xf);

	/* sreg */
	gdb_ctx->buff[6] = nib2hex((R_SREG >> 4)  & 0xf);
	gdb_ctx->buff[7] = nib2hex(R_SREG & 0xf);
	//gdb_ctx->buff[6] = nib2hex((gdb_ctx->regs->sreg >> 4)  & 0xf);
	//gdb_ctx->buff[7] = nib2hex(gdb_ctx->regs->sreg & 0xf);

	/* sp */
	gdb_ctx->buff[12] = nib2hex((gdb_ctx->sp >> 4)  & 0xf);
	gdb_ctx->buff[13] = nib2hex((gdb_ctx->sp >> 0)  & 0xf);
	gdb_ctx->buff[14] = nib2hex((gdb_ctx->sp >> 12) & 0xf);
	gdb_ctx->buff[15] = nib2hex((gdb_ctx->sp >> 8)  & 0xf);

	/* pc */
	gdb_ctx->buff[20] = nib2hex((pc >> 4)  & 0xf);
	gdb_ctx->buff[21] = nib2hex((pc >> 0)  & 0xf);
	gdb_ctx->buff[22] = nib2hex((pc >> 12) & 0xf);
	gdb_ctx->buff[23] = nib2hex((pc >> 8)  & 0xf);
#if defined(__AVR_ATmega2560__)
	gdb_ctx->buff[24] = nib2hex((pc >> 20) & 0xf);
#else
	gdb_ctx->buff[24] = '0';
#endif
	gdb_ctx->buff[25] = nib2hex((pc >> 16) & 0xf);
	gdb_ctx->buff[26] = '0'; /* gdb wants 32-bit value, send 0 */
	gdb_ctx->buff[27] = '0'; /* gdb wants 32-bit value, send 0 */

	/* not in hex, send from ram */
	gdb_send_buff(gdb_ctx->buff, gdb_ctx->buff_sz);
}



/* Note: The save and restore functions must be always inline,
 otherwise the debug version will call these functions and
 destroy the stack; The functions assume the stack is not touched.
 */
__attribute__((always_inline))
static inline void save_regs1 (void)
{
	/*  20-6-2017
	 New version: gdb.c says disable interrupts as soon as possible */
	asm volatile (
				"push	r0	\n"
				"in		r0, __SREG__ \n"
				"cli	\n"			/* disable interrupts */
				"sts regs+32, r0\n"	/* save SREG to its place */
				"pop r0 \n"		/* restore r0 from stack */
				"sts	regs+31, r31\n"		/* save R31 */
				"sts	regs+30, r30\n"		/* save R30 */
				"ldi	r31, hi8(regs)\n"	/* Z points to regs */
				"ldi	r30, lo8(regs)\n"
				"std	Z+29, r29\n");		/* save R29 */
				/*"in	r29, __SREG__\n");*/	/* get SREG */

	/* Original version, unsafe if called with interrupts enabled because
	  ISR can damage registers r31,r30...
	  This is not problem in ISRs because interrupts are disabled by default
	  but in breakpoint() function this could cause troubles. */
#if 0
	asm volatile (
	"sts	regs+31, r31\n"		/* save R31 */
	"sts	regs+30, r30\n"		/* save R30 */
	"ldi	r31, hi8(regs)\n"	/* Z points to regs */
	"ldi	r30, lo8(regs)\n"
	"std	Z+29, r29\n"		/* save R29 */
	"in	r29, __SREG__\n");	/* get SREG */
#endif
}

__attribute__((always_inline))
static inline void save_regs2 (void)
{
	asm volatile (
	/* SREG save removed 20-6-2017 */
	/*"std	Z+32, r29\n" */		/* put SREG value to his place */
	"std	Z+28, r28\n"		/* save R28 */
	"ldi	r29, 0\n"		/* Y points to 0 */
	"ldi	r28, 0\n"

	/* R0 address is 0; this loop saves from 0 to Z, which is
	 a "pointer" to "regs" variable .	 */
	"std	Z+27, r27\n"		/* save R27 */
"1:	ld	r27, Y+\n"		/* load register 0..26 */
	"st	Z+, r27\n"		/* save it */
	"cpi	r28, 27\n"
	"brne	1b\n"


    /* Get return address from stack; if we are in ISR, this will be the address
     * where the main loop was interrupted. If we are in breakpoint() this will be
     * address of the breakpoint() function.
     * Debugger must think we are in the main loop, so we need to pop the return address
     * from the stack which it will see.
     * But we need to save this address and restore it later so that the program can
     * continue normally.
     */
#if defined(__AVR_ATmega2560__)
	"pop 	r25\n"			/* pop return address, high-high byte for AVRs with 3-byte PC */
#endif
	"pop	r27\n"			/* pop return address, high byte */
	"pop	r26\n"			/* pop return address, low byte */
	/* Disabled dropping the breakpoint() return address.
	 It seems better to return to breakpoint() normally. It would make sense if breakpoint()
	 would be called from ISR which we do not want the debugger to see */
#if 0
	"cpi	r27, hi8 (pm (breakpoint))\n"
	"ldi	r24, lo8 (pm (breakpoint))\n"
	"cpc	r26, r24\n"	/* if called from breakpoint function, save the real return address of bp caller */
	"brne	1f\n"		/* branch if ret. address is not breakpoint*/
	/* If we are in breakpoint() then we drop the return address and pop the address of
	 the caller of breakpoint() - to return to the caller. */
	"pop	r27\n"		/* pop return address of caller of breakpoint */
	"pop	r26\n"
#endif

	/* now save the return address - where we will really return */
"1:	 std	Z+35-27, r26\n"		/* save return address (PC) */
	"std	Z+36-27, r27\n"		/* -27 because from previous code Z points to regs+27*/
#if defined(__AVR_ATmega2560__)
	"std	Z+37-27, r25\n"
#endif
	"in	r26, __SP_L__\n"
	"in	r27, __SP_H__\n"
	"std	Z+33-27, r26\n"		/* save SP */
	"std	Z+34-27, r27\n"
	"clr	__zero_reg__\n");

	/* Setup internal stack */
	asm volatile (
	"out __SP_H__, %B0\n"
	"out __SP_L__, %A0"
	: : "r" (stack+GDB_STACKSIZE-1));
}


__attribute__((always_inline))
static inline void restore_regs (void)
{
	asm volatile (
	"ldi	r31, hi8(regs)\n"	/* Z points to regs */
	"ldi	r30, lo8(regs)\n"
	"ldi	r29, 0\n"		/* Y points to 0 */
	"ldi	r28, 0\n"
"1:	ld	r27, Z+\n"
	"st	Y+, r27\n"		/* restore register 0..27 */
	"cpi	r28, 28\n"
	"brne	1b\n"

	"ldd	r29, Z+33-28\n"		/* SP low */
	"out	__SP_L__, r29\n"
	"ldd	r29, Z+34-28\n"		/* SP high */
	"out	__SP_H__, r29\n"
	"ldd	r29, Z+35-28\n"		/* PC low */
	"push	r29\n"
	"ldd	r29, Z+36-28\n"		/* PC high */
	"push	r29\n"
#if defined(__AVR_ATmega2560__)
	"ldd	r29, Z+37-28\n"		/* PC high-high */
	"push	r29\n"
#endif
	"ldd	r28, Z+28-28\n"		/* restore R28 */
	"ldd	r29, Z+29-28\n"		/* restore R29 */
	"ldd	r30, Z+30-28\n"		/* restore R30 */
	"lds	r31, regs+32\n");	/* r31 = sreg */
}

/* ------------------------------------------------------------- */

__attribute__((always_inline))
static inline void quickcheck (void)
{
    asm volatile(
//	"sbi    %0, 0\n"                                                         //DEBUG
	"sts    regs, r0\n"      /* save r0 */
	"in     r0, __SREG__\n"  /* save sreg to r0 */
	"sts    regs+32, r0\n"	 /* save SREG to its place */
	"sts    regs+31, r31\n"  /* save r31 */
	"sts    regs+30, r30\n"  /* save r30 */
#if defined(__AVR_ATmega2560__)
	"pop    r0\n"            /* get highest byte of return address for ATmega2560 */
#endif
	"pop	r31\n"           /* get  (rest of) return address from stack */
    "pop	r30\n"
	"push   r30\n"           /* and push it back onto the stack */
	"push   r31\n"
#if defined(__AVR_ATmega2560__)
	"push    r0\n"           /* also the highest byte for ATmega2560 */
#else
	"clr    r0\n"            /* for all other MCUs clear r0 */
#endif
	"lsl    r30\n"           /* convert word address to byte address */
	"rol    r31\n"
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) ||  defined(__AVR_ATmega2560__)
    "rol    r0\n"            /* move highest bit(s) into r0 */
	"out    %1, r0\n"        /* and load it into RAMPZ */
	"elpm   r0,Z+\n"         /* load first byte of instruction */
	"inc    r0\n"            /* check for 0xFF! */
	"brne   _testsingle\n"   /* not equal, test single step */
	"elpm   r30, Z\n"        /* get 2nd byte of instruction */
	"subi   r30, 0xCF\n"     /* check for 0xCF */
	"breq   _longtest\n"     /* It's the trap opcode, so we have to do the long test */
#else
	"lpm    r0,Z+\n"         /* load first byte of instruction */
	"inc    r0\n"            /* check for 0xFF! */
	"brne   _testsingle\n"   /* not equal, test single step */
	"lpm    r30,Z\n"
	"subi   r30, 0xCF\n"     /* check for 0xCF */
	"breq   _longtest\n"     /* It's the trap opcode, so we have to do the long test */
#endif
	"_testsingle: lds    r30, ctx\n"       /* check single step flag */
	"tst    r30\n"           /* activated? */
	"brne   _longtest\n"     /* yes, do a long test */
	"lds    r31, regs+31\n"  /* restore r31 */
	"lds    r30, regs+30\n"  /* restore r30 */
	"lds    r0, regs+32\n"   /* fetch sreg */
	"out    __SREG__, r0\n"
	"lds    r0, regs\n"      /* restore r0 */
//  "cbi    %0, 0\n"                                                          // DEBUG
	"reti\n"                 /* and continue with the execution */
	"_longtest: lds    r31, regs+31\n"   /* restore r31 */
	"lds    r30, regs+30\n"  /* restore r30 */
	"lds    r0, regs+32\n"   /* fetch sreg */
	"out    __SREG__, r0\n"
	"lds    r0, regs\n"      /* restore r0, and fall through */
//	"cbi    %0, 0\n"                                                           // DEBUG 
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) ||  defined(__AVR_ATmega2560__)
        : :  "I" (_SFR_IO_ADDR(PORTB)), "I" (_SFR_IO_ADDR(RAMPZ))	 );   
#else
	: :  "I" (_SFR_IO_ADDR(PORTB))	 );   
#endif
	    
}


/* ---------- Helpers ------------- */

/* Convert a hexadecimal digit to a 4 bit nibble. */
__attribute__((optimize("-Os")))
static uint8_t hex2nib(uint8_t hex)
{
	if (hex >= 'A' && hex <= 'F')
		return 10 + (hex - 'A');
	else if (hex >= 'a' && hex <= 'f')
		return 10 + (hex - 'a');
	else if (hex >= '0' && hex <= '9')
		return hex - '0';

	return 0xff;
}

__attribute__((optimize("-Os")))
static uint8_t parse_hex(const uint8_t *buff, uint32_t *hex)
{
	uint8_t nib, len;
	for (*hex = 0, len = 0; (nib = hex2nib(buff[len])) != 0xff; ++len)
		*hex = (*hex << 4) + nib;
	return len;
}


static uint8_t safe_pgm_read_byte(uint32_t rom_addr_b)
{
#ifdef pgm_read_byte_far
	if (rom_addr_b >= (1l<<16))
		return pgm_read_byte_far(rom_addr_b);
	else
#endif
		return pgm_read_byte(rom_addr_b);
}



#if ( AVR8_BREAKPOINT_MODE == 2 )	/* Flash BP using optiboot */

/* Custom function not available in optiboot.h which reads also 0 and 255.
 * The optiboot_readPage will not store byte which is 0 or 255 into the buffer;
 * instead it will not change the buffer (skip it).
 */
__attribute__((optimize("-Os")))
static void optiboot_page_read_raw(optiboot_addr_t address, uint8_t output_data[])
{
  for(uint16_t j = 0; j < SPM_PAGESIZE; j++)
  {
    output_data[j] = safe_pgm_read_byte(address + j);
  }
}


/* Write to program memory using functions from Optiboot bootloader (in opt_api.h).
 * Uses temp RAM buffer as big as the flash page.
 * Supports writing multiple pages; it's possible to write virtually buffer of any size.
 * rom_addr - in words,
 * sz - in bytes and must be multiple of two.
 *
 * Note that this function is placed into separate page and section
 * at the end of the program so that it is not erased when setting
 * a breakpoint into user code which would be in the same flash page.
 * We do erase-fill-write because this code is not in the NRWW section of flash.
 * If we erase the page with this function itself, the fill-write cannot run.
 * To prevent this we align the function to page size so that it starts in new page
 * and we also put it into separate section. The section is not defined elsewhere,
 * gcc will just automatically put this section after the rest of the code, so the
 * critical flash-writing functions are at higher address than the user code and there
 * is never a breakpoint inserted into them.
 */
__attribute__((optimize("-Os")))
__attribute__((section(".avrdbg_flashwr")))
__attribute__ (( aligned(SPM_PAGESIZE) ))
static void optiboot_safe_pgm_write(const void *ram_addr,
							optiboot_addr_t rom_addr,
							uint16_t sz)
 {
	uint16_t *ram = (uint16_t*) ram_addr;
	uint16_t page_data[SPM_PAGESIZE_W];

	/* Sz must be valid and be multiple of two */
	if (!sz || (sz & 1))
		return;

	/* to words */
	sz >>= 1;

	for (optiboot_addr_t page = ROUNDDOWN(rom_addr, (optiboot_addr_t)SPM_PAGESIZE_W), end_page =
			ROUNDUP(rom_addr + sz, (optiboot_addr_t)SPM_PAGESIZE_W), off = rom_addr
			%  (optiboot_addr_t)SPM_PAGESIZE_W; page < end_page;
			page += SPM_PAGESIZE_W, off = 0) {

		/* page to bytes */
		optiboot_addr_t page_b = (uint32_t) page << 1;
		uint16_t* pFillData = page_data;

		optiboot_page_read_raw(page_b, (uint8_t*)pFillData);

		optiboot_page_erase(page_b);

		/* Fill temporary page */
		for (optiboot_addr_t page_off = 0; page_off < SPM_PAGESIZE_W; ++page_off) {
			/* to bytes */
			optiboot_addr_t rom_addr_b = (page + page_off) << 1;

			/* Fill with word from ram */
			if (page_off == off) {
				optiboot_page_fill(rom_addr_b, *ram);
				if (sz -= 1) {
					off += 1;
					ram += 1;
				}
			}
			/* Fill with word from flash */
			else {
				optiboot_page_fill(rom_addr_b, *pFillData);
			}

			pFillData++;

		}

		/* Write page and wait until done. */
		optiboot_page_write(page_b);
	}
}

/**
 * Return the major version of optiboot bootloader.
 * It should be 8 or higher for write-to-flash (SPM) support.
 * Note that is may be higher number if custom version of optiboot is used; it is defined like this:
 * unsigned const int __attribute__((section(".version")))
 * optiboot_version = 256*(OPTIBOOT_MAJVER + OPTIBOOT_CUSTOMVER) + OPTIBOOT_MINVER;
 */
__attribute__((optimize("-Os")))
static uint8_t get_optiboot_major()
{
	/* Get the optiboot version
	The address of the version is the address of the .version section; find it in the
	optiboot makefiles, e.g. in makefile.2560 you'll find: -Wl,--section-start=.version=0x3fffe */
#if defined(__AVR_ATmega328P__)
	uint16_t ver = safe_pgm_read_word(0x7ffe);

#elif defined(__AVR_ATmega2560__)
	/* Note that the default bootloader for Arduino Mega is not Optiboot, there may be anything at the address
	   where Optiboot stores the version */
	uint16_t ver = safe_pgm_read_word(0x3fffe );
	if ( ver == 0xfffe )
		ver = 0;	/* This is the value found in default Arduino Mega bootloader which is stk500boot_v2_mega2560.hex,
					 so report it as wrong version, we need Optiboot. */
#elif  defined(__AVR_ATmega1280__) ||  defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) 
	uint16_t ver = safe_pgm_read_word(0x1fffe );
#else
	/* This MCU is not supported; just return 0 which means invalid version */
	uint16_t ver = 0;
#endif

	return (uint8_t)((ver & 0xff00) >> 8);
}

/* Functions to access the do_spm function in Optiboot.
 * From the optiboot.h file provided with Optiboot test spm example.
*/

/*
 * The same as do_spm but with disable/restore interrupts state
 * required to succesfull SPM execution
 *
 * On devices with more than 64kB flash, 16 bit address is not enough,
 * so there is also RAMPZ used in that case.
 */
__attribute__((section(".avrdbg_flashwr")))
/*__attribute__ (( aligned(SPM_PAGESIZE) )) */
static void do_spm_cli(optiboot_addr_t address, uint8_t command, uint16_t data)
{
  uint8_t sreg_save;

  sreg_save = SREG;  // save old SREG value
  asm volatile("cli");  // disable interrupts
#ifdef RAMPZ
  RAMPZ = (address >> 16) & 0xff;  // address bits 23-16 goes to RAMPZ
#ifdef EIND
  uint8_t eind = EIND;
  EIND = FLASHEND / 0x20000;
#endif
  // do_spm accepts only lower 16 bits of address
  do_spm((address & 0xffff), command, data);
#ifdef EIND
  EIND = eind;
#endif
#else
  // 16 bit address - no problems to pass directly
  do_spm(address, command, data);
#endif
  SREG = sreg_save; // restore last interrupts state
}


/**
 * Erase page in FLASH
 * Note that this function is placed into separate page and section
 * at the end of the program so that it is not erased when setting
 * a breakpoint into user code which would be in the same flash page.
 */
__attribute__((section(".avrdbg_flashwr")))
/* __attribute__ (( aligned(SPM_PAGESIZE) )) */
static void optiboot_page_erase(optiboot_addr_t address)
{
  do_spm_cli(address, __BOOT_PAGE_ERASE, 0);
}


/** Write word into temporary buffer.
 * Note that this function is placed into separate page and section
 * at the end of the program so that it is not erased when setting
 * a breakpoint into user code which would be in the same flash page.
 */
__attribute__((section(".avrdbg_flashwr")))
/* __attribute__ (( aligned(SPM_PAGESIZE) )) */
static void optiboot_page_fill(optiboot_addr_t address, uint16_t data)
{
  do_spm_cli(address, __BOOT_PAGE_FILL, data);
}


/** Write temporary buffer into FLASH.
 * Note that this function is placed into separate page and section
 * at the end of the program so that it is not erased when setting
 * a breakpoint into user code which would be in the same flash page.
 */
__attribute__((section(".avrdbg_flashwr")))
/* __attribute__ (( aligned(SPM_PAGESIZE) )) */
static void optiboot_page_write(optiboot_addr_t address)
{
  do_spm_cli(address, __BOOT_PAGE_WRITE, 0);
}


#endif


#ifdef AVR8_STUB_DEBUG
/* Helper for tests...
  Returns how many bytes are NOT used from given stack buffer.
  Starts from the end (buffer[size-1]).
  How to use: Call this function e.g. in parse_packet and save the return value in global
  variable. When debugging, display the variable in Expressions window in eclipse. The value you
  will see after some debugging will be the number of unused bytes on stack.
  Note that when any byte on stack is used, it no longer contains the canary code, so even if
  stack is freed and used in different part of the code we will get the maximum usage which ever happens.
  BUT it may happen that the canary value is written to stack as normal data and in such case the function
  will report incorrect values. The best way is to see the "stack" variable in Expressions window and see
  how many bytes from the beginning contain canary value. */
uint8_t test_check_stack_usage(void)
{
	return wcheck_stack_usage((uint8_t*)stack, GDB_STACKSIZE);
}

static uint8_t wcheck_stack_usage(uint8_t* buff, uint16_t size )
{
	uint16_t i;
	for ( i = size-1; i>0; i--)
	{
		/* look for untouched byte of memory with canary value */
		if ( buff[i] == GDB_STACK_CANARY )
			return i;
	}

	return size;
}

static void wfill_stack_canary(uint8_t* buff, uint16_t size)
{
	uint16_t i;
	for (i=0; i<size; i++ )
		buff[i] = GDB_STACK_CANARY;
}

/* Print text and number to debug console */
__attribute__((optimize("-Os")))
static void test_print_hex(const char* text, uint16_t num){
	char buff[6];

	debug_message(text);
	buff[0] = ' ';	// space
	buff[1] = nib2hex((num >> 4)  & 0xf);
	buff[2] = nib2hex((num >> 0)  & 0xf);
	buff[3] = nib2hex((num >> 12) & 0xf);
	buff[4] = nib2hex((num >> 8)  & 0xf);
	buff[5] = 0;
	debug_message(buff);
}

#endif	/* AVR8_STUB_DEBUG */



#if 0
/* Unuser timer code.
 * This is code for flash breakpoints inserted as endless loop and testing the app for such
 a breakpoint periodically from timer ISR.
 Not used since we use external INTx to jump into debugger from breakpoint. */
#if ( AVR8_BREAKPOINT_MODE == 0 )	/* Flash BP */
/*
 Arduino timer usage:
 Timer0 - delay, millis, etc.
 Timer1 - Servo  (On Mega servo uses Timer5)
 Timer2 - tone()
 analogWrite can use different timers depending on the pin.
*/
static void init_timer(void)
{
	/* How many times per second an interrupt is generated. Original value: 1000
	   At 16 MHz clock with prescaler 1 the minimum is about 250 - result F_CPU/TIMER_RATE must
	   fit into 16-bit compare register.
	   With prescaler 1024 the timer rate will not be "per second" but per 1024 seconds!
	   So TIMER_RATE 1024 is about 1x per second. */
#define TIMER_RATE 10240	/* 10240 is about 10 times per sec with prescaler 1024 */
/*#define TIMER_RATE 1000 */

#if defined(__AVR_ATmega328P__)
	TCCR1A = 0;
	/* Set CTC mode */
	TCCR1B = 0;
	TCCR1B |= (1 << WGM12);
	/* Prescaler = 1*/
	/*TCCR1B |= (1 << CS10);*/
	/* Prescaler = 1024*/
	TCCR1B |= (1 << CS10) | (1 << CS12);

	TCCR1C = 0;
	/* Set the compare register */
	OCR1A = F_CPU / TIMER_RATE - 1;
	/* Enable Output Compare Match Interrupt */
	TIMSK1 = 0;
	TIMSK1 |= (1 << OCIE1A);
#else
	/* possible support for atmega2560 */
#error Unsupported AVR device

#endif	/* AVR variant */
}
#endif	/* AVR8_BREAKPOINT_MODE */

#if  (AVR8_BREAKPOINT_MODE == 0 )  /* Combined mode flash + RAM BPs */
/* Interrupt handler for timer - for flash breakpoints */
ISR(TIMER1_COMPA_vect, ISR_BLOCK ISR_NAKED)
{
	save_regs1();
	/* save_regs1 loads SREG into r29 */

	/* Sets interrupt flag = interrupts enabled; but not in real, just in the stored value */
	asm volatile ("ori r29, 0x80");	/* user must see interrupts enabled */
	save_regs2 ();
#if defined(__AVR_ATmega2560__)
	R_PC_HH &= 0x01;		/* there is only 1 bit used in the highest byte of PC (17-bit PC) */
	/* No need to mask R_PC_H */
#else
	R_PC_H &= RET_ADDR_MASK;
#endif
	gdb_ctx->pc = R_PC;
	gdb_ctx->sp = R_SP;


	/* Check breakpoint */
	for (uint8_t i = 0; i < ARRAY_SIZE(gdb_ctx->breaks); ++i)
		if (gdb_ctx->pc == gdb_ctx->breaks[i].addr)
			goto trap;

	/* Nothing */
	goto out;

trap:
	/* Set correct interrupt reason */
	gdb_send_state(GDB_SIGTRAP);
	handle_exception();

out:
	restore_regs ();

	asm volatile (
		"sbrs	r31, 7\n"		/* test I flag */
		"rjmp	1f\n"
		"andi	r31, 0x7f\n"		/* clear I flag */
		"out	__SREG__, r31\n"	/* restore SREG */
		"lds	r31, regs+31\n"		/* real value of r31 */
		"reti\n"			/* exit with interrupts enabled */
	"1:	out	__SREG__, r31\n"	/* exit with interrupts disabled */
		"lds	r31, regs+31\n");	/* real value of r31 */

#if 0	/* Flash only version */
asm volatile (
			"restore_registers:");
	restore_regs ();

	asm volatile (
			"sbrs	r31, 7\n"		/* test I flag; skip if bit set = skip if interrupts enabled */
			"rjmp	1f\n"
			"andi	r31, 0x7f\n"		/* clear I flag */
			"out	__SREG__, r31\n"	/* restore SREG */
			"lds	r31, regs+31\n"		/* real value of r31 */
			"reti\n"			/* exit with interrupts enabled */
			"1:	out	__SREG__, r31\n"	/* exit with interrupts disabled */
			"lds	r31, regs+31\n");	/* real value of r31 */
#endif
}
#endif  /* AVR8_BREAKPOINT_MODE */
#endif	/* if 0 */



/* EOF */




