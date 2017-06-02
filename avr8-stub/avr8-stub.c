/*
a * avr8-stub.c
 *
 *  Created on: 25. 3. 2015
 *      Author: Jan Dolinay
 *
 *  GDB Debugging Agent (GDB stub) for ATMega328
 *
 *  Note: This file contains stub for ATMega328 and ATMega 1280 (2560)
 *  Code for ATMega 1280 is selected automatically based on define in the avr includes.
 *  ATMega328 is the default option otherwise; the code is:
 *  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
 *   - code for Arduino Mega
 *  #else
 *   - code for Arduino Uno
 *  #endif
 *  Define __AVR_ATmega328__ could be used for ATMega328 if needed.
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

/* Define this to enable some global variables to make it easier to debug this stub */
//#define AVR8_DEBUG_MODE


#if (AVR8_BREAKPOINT_MODE == 0)
	#include "app_api.h"	/* bootloader API used for writing to flash  */
#endif


typedef uint8_t bool_t;
#define FALSE 0
#define TRUE 1

/* Configuration */

/* Serial port baudrate */
/* Note that we need to use the double UART speed option (U2X0 bit = 1) for the 115200 baudrate on Uno.
 * Use the double speed always! For Arduino Mega it has lower error both for 57600 and 115200 */
/* For Arduino Mega 1280 there is error in baud 2.1% for 115200. For 57600 the error is -0,8%.
 * Debugging seems to work better (sometimes only) for 57600.  */
#if defined(__AVR_ATmega1280__)
	/* For ATmega1280 baudrate the debuger communicates at 57600... */
	#define GDB_USART_BAUDRATE 57600
#else
	#define GDB_USART_BAUDRATE 115200
#endif


/* For double UART speed (U2X0 bit = 1) use this macro: */
#define GDB_BAUD_PRESCALE (((( F_CPU / 8) + ( GDB_USART_BAUDRATE / 2) ) / ( GDB_USART_BAUDRATE )) - 1)

/* For normal UART speed use: (usable for speeds up to 57600 on ATmega328) */
/*
#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2) ) / ( USART_BAUDRATE )) - 1)
*/

/*
 * Macros used in this file which change value based on options set in header
 */

/* Symbols:
 * AVR8_SWINT_PIN 		- the pin used for generating SW interrupt
 * AVR8_SWINT_INTMASK 	- mask used in EIMSK register to enable the interrupt and in EIFR
 * 						register to clear the flag.
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	/* Arduino Mega configuration */
	#if AVR8_SWINT_SOURCE == 0
		#define	AVR8_SWINT_PIN		(PD0)
		#define AVR8_SWINT_INTMASK	(INTF0)
	#elif AVR8_SWINT_SOURCE == 1
		#define	AVR8_SWINT_PIN		(PD1)
		#define AVR8_SWINT_INTMASK	(INTF1)
	#elif AVR8_SWINT_SOURCE == 2
		#define	AVR8_SWINT_PIN		(PD2)
		#define AVR8_SWINT_INTMASK	(INTF2)
	#elif AVR8_SWINT_SOURCE == 3
		#define	AVR8_SWINT_PIN		(PD3)
		#define AVR8_SWINT_INTMASK	(INTF3)
	#elif AVR8_SWINT_SOURCE == 4
		#define	AVR8_SWINT_PIN		(PE4)
		#define AVR8_SWINT_INTMASK	(INTF4)
	#elif AVR8_SWINT_SOURCE == 5
		#define	AVR8_SWINT_PIN		(PE5)
		#define AVR8_SWINT_INTMASK	(INTF5)
	#elif AVR8_SWINT_SOURCE == 6
		#define	AVR8_SWINT_PIN		(PE6)
		#define AVR8_SWINT_INTMASK	(INTF6)
	#elif AVR8_SWINT_SOURCE == 7
		#define	AVR8_SWINT_PIN		(PE7)
		#define AVR8_SWINT_INTMASK	(INTF7)
	#else
		#error SW Interrupt source not valid. Please define AVR8_SWINT_SOURCE value 0 thru 7 in avr8-stub.h
	#endif

#else	/* Arduino Uno */
	#if AVR8_SWINT_SOURCE == 0
		#define	AVR8_SWINT_PIN		(PD2)
		#define AVR8_SWINT_INTMASK	(INT0)
	#elif AVR8_SWINT_SOURCE == 1
		#define	AVR8_SWINT_PIN		(PD3)
		#define AVR8_SWINT_INTMASK	(INT1)
	#else
		#error SW Interrupt source not valid. Please define AVR8_SWINT_SOURCE 0 or 1 in avr8-stub.h
	#endif
#endif


/*
 * Other defines and macros
 */
#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))
#define MIN(i1, i2) (i1 < i2 ? i1 : i2);

/** Size of the buffer we use for receiving messages from gdb.
 *  must be in hex, and not fewer than 79 bytes,
    see gdb_read_registers for details */
#define AVR8_MAX_BUFF   	(80)


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
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

	#define MEM_SPACE_MASK 0x00fe0000
	#define FLASH_OFFSET   0x00000000
	#define SRAM_OFFSET    0x00800000

	#define	UART_ISR_VECTOR	USART0_RX_vect

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
	#define MEM_SPACE_MASK 0x00ff0000
	#define FLASH_OFFSET   0x00000000
	#define SRAM_OFFSET    0x00800000	/* GDB works with linear address space; RAM address from GBD will be (real addresss + 0x00800000)*/

	#define	UART_ISR_VECTOR	USART_RX_vect

	/* AVR puts garbage in high bits of return address on stack.
   	   Mask them out
 	 The original version used mask 0x1f, which does not work for code above 16 kB!
	 In 1/2017 changed to 0x3f. The PC is 14 bits on Atmega328 (13 bits on Atmega168), we must
	 out the higher byte, so the mask is actually: 0x3fFF to keep 14 bits.
	 In the code this is used when reading 2 B from RAM to mask the first byte adr+0; adr+1 is not masked
	*/
	#define RET_ADDR_MASK  0x3f
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


#if ( AVR8_BREAKPOINT_MODE == 0 )	/* Flash BP */

/* The opcode of instruction(s) we use for stgopping the program at breakpoint.
  Instruction at the BP location is replaced by this opcode.
  To stop the program  we use RJMP on itself, i.e. endless loop,
   1100 kkkk kkkk kkkk, where 'k' is a -1 in words.
   #define TRAP_OPCODE 0xcfff
  To learn that BP was hit we would have to use timer and look if th eprogram is not looping at breakpoint address.
  To avoid this we use external INT - the trap opcode will enable the INT.
  Opcode for set/bit instruction SBI is 1001 1010 AAAA Abbb  (A is address, b is bit number)
  for EIMSK = 0x1d bit 0 AAAAA = 11101, bbb = 000 > 1001 1010 1110 1000 = 0x9ae8
  EIMSK |= _BV(INT0);	enable INT0 interrupt
  e8 9a       	sbi	0x1d, 0
  So the opcode would be:
  #define TRAP_OPCODE 0x9ae8
  But enabling the INT will not stop the program at the next instruction, so we insert both enable INT and loop,
  Just FYI opcode to set pin low:
  PORTD &= ~_BV(PD2);
  5a 98       	cbi	0x0b, 2
  To enable INT1 interrupt:
  e9 9a  sbi	0x1d, 1
  So the opcode is 9ae9
 */
#if AVR8_SWINT_SOURCE == 0
	#define TRAP_OPCODE 0xcfff9ae8
#elif AVR8_SWINT_SOURCE == 1
	#define TRAP_OPCODE 0xcfff9ae9
#else
	#error The value of AVR8_SWINT_SOURCE is not supported. Define valid opcode for this value here.
#endif

/**
 Structure to hold information about a breakpoint in flash.
 */
struct gdb_break
{
	Address_t addr; /* in words */
	uint16_t opcode;
	uint8_t status;	/* status of the breakpoint in flash, see below */
	uint16_t opcode2;
};

/*
 The flags used for gdb_break.status:
 bit 0: is BP in flash? 1 = yes, 0 = no
 bit 1: is BP enabled now? 1 =yes, 0 = no
 To work with status use the macros below!
 */

/* Helper macros to manipulate breakpoint status */
/** mark this breakpoint as written to flash */
#define GDB_BREAK_SET_INFLASH(gdb_struct_ma)	gdb_struct_ma.status |= 0x01;
/** mark this breakpoint as not written to flash */
#define GDB_BREAK_CLEAR_INFLASH(gdb_struct_ma)	gdb_struct_ma.status &= ~0x01;
/** Test if breakpoint is in flash. Evaluates to true if BP is in flash. */
#define GDB_BREAK_IS_INFLASH(gdb_struct_ma)		(gdb_struct_ma.status & 0x01)
/** mark this breakpoint as currently enabled (GBD inserted it ) */
#define GDB_BREAK_ENABLE(gdb_struct_ma)	gdb_struct_ma.status |= 0x02;
/** mark this breakpoint as disabled ( GDB removed it) */
#define GDB_BREAK_DISABLE(gdb_struct_ma)	gdb_struct_ma.status &= ~0x02;
/** Test if breakpoint is enabled. Evaluates to true if BP is enabled */
#define GDB_BREAK_IS_ENABLED(gdb_struct_ma)		(gdb_struct_ma.status & 0x02)


#endif	/* AVR8_BREAKPOINT_MODE == 0 */


/**
 * Data used by this driver.
 */
struct gdb_context
{
	uint16_t sp;
	Address_t pc; /* PC is 17-bit on ATmega2560*/


#if ( AVR8_BREAKPOINT_MODE == 0 )
	/* Flash breakpoints */
	struct gdb_break breaks[AVR8_MAX_BREAKS];
	uint8_t breakpoint_step;		/* Indicates special step to skip breakpoint */

#elif ( AVR8_BREAKPOINT_MODE == 1 )
	/* RAM only breakpoints */

	/* On ATmega2560 the PC is 17-bit. We could use uint32_t for all MCUs, but
	 * that would be a waste of RAM and also all the manipulation with 32-bit will
	 * be much slower than with 16-bit variable.  */
	Address_t breaks [AVR8_MAX_BREAKS];	/* Breakpoints */
#endif

	uint8_t breakpoint_enabled;		/* At least one BP is set. This could be RAM only but it makes code easier to read if it exists in flash bp mode too. */
	uint8_t singlestep_enabled;
	uint8_t breaks_cnt;				/* number of valid breakpoints inserted */
	uint8_t buff[AVR8_MAX_BUFF+1];
	uint8_t buff_sz;
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
#if 0
static void gdb_write_binary(const uint8_t *buff);
static unsigned char *bin2mem(unsigned char *buf, unsigned char *mem, int count);
#endif

#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
	static void gdb_update_breakpoints(void);
#endif

static inline void restore_regs (void);
static inline void save_regs1 (void);
static inline void save_regs2 (void);

static uint8_t safe_pgm_read_byte(uint32_t rom_addr_b);

/* Helpers for determining required size of our stack and for printing t console*/
#ifdef AVR8_DEBUG_MODE
#define		GDB_STACK_CANARY	(0xBA)
uint8_t test_check_stack_usage(void);
static void wfill_stack_canary(uint8_t* buff, uint8_t size);
static uint8_t wcheck_stack_usage(uint8_t* buff, uint8_t size );	/* returns how many bytes are used from given buffer */
/* Helper for writing debug message to console when debugging this debugger */
static void test_print_hex(const char* text, uint16_t num);
#endif	/* AVR8_DEBUG_MODE */

/* Print debug messages for debugging this debugger
 * Use only as a last resort. It affects the program in strange ways, confuses GDB and the messages
 * can point you in the wrong direction rather than help.
 * It is better to use global variables and watch their values in Expressions window when debugging,
 * see the variables like G_Debug_INTxCount.  */
#define	DEBUG_PRINT	0

#if (DEBUG_PRINT == 1 )
	#define	DBG_TRACE(text)	debug_message(text)
	#define	DBG_TRACE1(text, number)	test_print_hex(text, number)
#else
	#define	DBG_TRACE(text)
	#define	DBG_TRACE1(text, number)
#endif


#ifdef AVR8_DEBUG_MODE
/* You can use these variables to debug this stub. Display them in the Expressions window in eclipse */
uint8_t G_Debug_INTxCount = 0;	/* How many times external INTx ISR was called*/
uint8_t G_BpEnabledINTx = 0;	/* How many times external INTx ISR was called and bp were enabled*/
uint16_t G_LastPC = 0;			/* Value of PC register in INTx ISR */
uint32_t G_BreakpointAdr = 0;	/* Address of the breakpoint last set/written to flash */
uint16_t G_StepCmdCount = 0;	/* Counter for step commands from GDB */
uint8_t G_ContinueCmdCount = 0;	/* Counter for continue commands from gdb */
uint32_t G_RestoreOpcode = 0;	/* Opcode(s) which were restored in flash when removing breakpoint */
uint8_t	G_StackUnused = 0;	/* used for testing stack size only*/
#endif  /* AVR8_DEBUG_MODE */



#if (AVR8_BREAKPOINT_MODE == 0)		/* Flash breakpoints */
	static uint16_t safe_pgm_read_word(uint32_t rom_addr_b);
	static struct gdb_break *gdb_find_break(uint16_t rom_addr);
	static void gdb_remove_breakpoint_ptr(struct gdb_break *breakp);
	/* static void init_timer(void); */
#endif

/* Global variables */

/* Our context and pointer to this context.
 * The pointer is used in the original code which "creates" the struct on stack in ISR.
 * I keep it so as not to change all the functions even though the context is saved to regs array. */
static struct gdb_context ctx;
static struct gdb_context *gdb_ctx;

/* String for PacketSize reply to gdb query.
 * Note: if running out of RAM the reply to qSupported packet can be removed. */
static char* gdb_str_packetsz = "PacketSize=" STR_VAL(AVR8_MAX_BUFF);

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
 * there are 78 bytes of stack used. Set size to 96 to be on the safe side.
 * RAM breakpoints version: 49 B of stack used, set to 72 to be on the safe side.
 */
#if (AVR8_BREAKPOINT_MODE == 1 )
	#define GDB_STACKSIZE 	(72)			/* Internal stack size for RAM only BP */
#else
	#define GDB_STACKSIZE 	(96)			/* Internal stack size for FLASH BP */
#endif


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

	/* Init breaks */
	memset(gdb_ctx->breaks, 0, sizeof(gdb_ctx->breaks));


#if (AVR8_BREAKPOINT_MODE == 1)		/* RAM BP */
	gdb_ctx->breakpoint_enabled = 0;
#elif (AVR8_BREAKPOINT_MODE == 0)		/* Flash BP */
	gdb_ctx->breakpoint_step = 0;
	/* Initialize timer - not needed since we use external INT */
	/* init_timer(); */
#endif

#ifdef AVR8_DEBUG_MODE
	/* For testing stack usage only - fill satack with canary values  */
	wfill_stack_canary(stack, GDB_STACKSIZE);
#endif

	/* Initialize serial port */
	uart_init();
}

/* ------------ User interface (API) for this driver ---------------- */


/* ---------- UART communication routines  ------------- */

/* Initialize UART */
static void uart_init(void)
{
	/* Init UART */
	UCSR0A = _BV(U2X0);		/* double UART speed */
	UCSR0B = (1 << RXEN0 ) | (1 << TXEN0 );		/* enable RX and Tx */
	UCSR0C =  (1 << UCSZ00 ) | (1 << UCSZ01 ); /* Use 8- bit character sizes */
	UBRR0H = ( GDB_BAUD_PRESCALE >> 8) ;
	UBRR0L = GDB_BAUD_PRESCALE ;
	UCSR0B |= (1 << RXCIE0 ); /* Enable the USART Recieve Complete interrupt ( USART_RXC ) */
}

/* Read a single character from the serial port */
static uint8_t getDebugChar(void)
{
	/* wait for data to arrive */
	while ( !(UCSR0A & (1<<RXC0)) )
		;

	return (uint8_t)UDR0;
}

/* Write a single character to serial port */
static void putDebugChar(uint8_t c)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
		;

	/* Put data into buffer, sends the data */
	UDR0 = c;
}
/* ---------------- end UART communication routines ---------------------------------- */

/* ---------------- Timer for flash breakpoints ------------------ */
#if ( AVR8_BREAKPOINT_MODE == 0 )	/* Flash BP */
#if 0	/* Not used since we use external INTx */
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
#define TIMER_RATE 10240	/* 10240 is about 10 timer per sec with prescaler 1024 */
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
#endif 	/* #if 0 */

#endif	/* AVR8_BREAKPOINT_MODE */
/* ---------------- end Timer for flash breakpoints ------------------ */

/* ---------- Debugging driver routines  ------------- */
/**
 * Stimulate interrupt;
 * Used for single stepping or when any breakpoint in RAM is set to stop the
 * program after every instruction.
 * The AVR core will always execute one instruction in the main code before
 * jumping into ISR even if interrupt is pending. We set the INT0 to trigger when
 * pin is low and set the pin low. */
__attribute__((always_inline))
static inline void gdb_enable_swinterrupt()
{

#if AVR8_SWINT_SOURCE == 0
	/* Set the sense for the INT0 or INT1 interrupt to trigger it when the pin is low */
	EICRA &= ~(_BV(ISC01) | _BV(ISC00));
#elif AVR8_SWINT_SOURCE == 1
	EICRA &= ~(_BV(ISC11) | _BV(ISC10));
#elif AVR8_SWINT_SOURCE == 2
	EICRA &= ~(_BV(ISC21) | _BV(ISC20));
#elif AVR8_SWINT_SOURCE == 3
	EICRA &= ~(_BV(ISC31) | _BV(ISC30));
#elif AVR8_SWINT_SOURCE == 4
	EICRB &= ~(_BV(ISC41) | _BV(ISC40));
#elif AVR8_SWINT_SOURCE == 5
	EICRB &= ~(_BV(ISC51) | _BV(ISC50));
#elif AVR8_SWINT_SOURCE == 6
	EICRB &= ~(_BV(ISC61) | _BV(ISC60));
#elif AVR8_SWINT_SOURCE == 7
	EICRB &= ~(_BV(ISC71) | _BV(ISC70));
#else
	#error SW Interrupt source not valid. Please define in avr8-stub.h
#endif

	/* The pin needs to be configured as output to allow us to set the
	 * level on the pin and thus generate the interrupt*/
#if AVR8_SWINT_SOURCE < 4
	DDRD |= _BV(AVR8_SWINT_PIN);		/* set pin to output mode */
	EIFR |= _BV(AVR8_SWINT_INTMASK);	/* clear INTx flag */
	EIMSK |= _BV(AVR8_SWINT_INTMASK);	/* enable INTx interrupt */
	PORTD &= ~_BV(AVR8_SWINT_PIN);		/* make sure the pin is low */
#else
	/* INT4 - INT7 pins are on port E */
	DDRE |= _BV(AVR8_SWINT_PIN);
	EIFR |= _BV(AVR8_SWINT_INTMASK);
	EIMSK |= _BV(AVR8_SWINT_INTMASK);
	PORTE &= ~_BV(AVR8_SWINT_PIN);
#endif
}

/** Disable the interrupt used for single stepping and RAM breakpoints. */
__attribute__((always_inline))
static inline void gdb_disable_swinterrupt()
{
	EIMSK &= ~_BV(AVR8_SWINT_INTMASK);
}

/** Macro which is true if there is a pending interrupt from UART Rx
 * */
#define	UART_RXINT_PENDING()  (UCSR0A & (1<<RXC0))

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

#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
	/* Special case steeping after breakpoint... */
	if ( gdb_ctx->breakpoint_step ) {
		gdb_ctx->breakpoint_step = 0;
		gdb_update_breakpoints();
		gdb_disable_swinterrupt();
		return;
	}
#endif

	while (1) {
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

			if(gdb_ctx->singlestep_enabled || gdb_ctx->breakpoint_enabled)
			{
				/* this will generate interrupt after one instruction in main code */
				gdb_enable_swinterrupt();

			}
			else
			{
				gdb_disable_swinterrupt();
				/* Clear flag for the external INT. Added for no-timer flash BP. Probably not needed... */
				EIFR |= _BV(AVR8_SWINT_INTMASK);
			}

			/* leave the trap, continue execution */
			return;

		case '-':  /* NACK, repeat previous reply */
			gdb_send_buff(gdb_ctx->buff, gdb_ctx->buff_sz);
			break;
		case '+':  /* ACK, great */
			break;
		case 0x03:
			/* user interrupt by Ctrl-C, send current state and
			   continue reading */
			gdb_send_state(GDB_SIGINT);
			break;
		default:
			gdb_send_reply(""); /* not supported */
			break;
		}
	}

}

/* This is the main "dispatcher" of commands from GDB
 * If returns false, the debugged program continues execution */
__attribute__((optimize("-Os")))
static bool_t gdb_parse_packet(const uint8_t *buff)
{
#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
	Address_t pc = gdb_ctx->pc;	// PC with word address
	struct gdb_break* pbreak;
#endif

#ifdef AVR8_DEBUG_MODE
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
#if 0
	case 'X':
		gdb_write_binary(buff + 1);
		break;
#endif

	case 'D':               /* detach the debugger */
	case 'k':               /* kill request */
#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
		/* Update the flash so that the program can run after reset without breakpoints */
		gdb_update_breakpoints();
#endif
		gdb_send_reply("OK");
		return FALSE;

	case 'c':               /* continue */

#ifdef AVR8_DEBUG_MODE
		G_ContinueCmdCount++;
#endif /* AVR8_DEBUG_MODE */

#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
		/* We need to handle a special case: if we just stepped from BP on a 1 word instruction,
		we may be standing on the instruction which will be replaced by infinite loop when BP is inserted,
		so we cannot update (insert) the BP now but only after this instruction is executed.
		So we need to let the program do one more step before updating BPs. */
		pbreak = gdb_find_break(pc - 1);
		if ( pbreak )  {
			/* this is the special case - we are one word after breakpoint */
			gdb_ctx->singlestep_enabled = 1;
			gdb_ctx->breakpoint_step = 1;
			return FALSE;
		}

		gdb_update_breakpoints();
#endif

		return FALSE;

	case 'C':               /* continue with signal */
	case 'S':               /* step with signal */
		gdb_send_reply(""); /* not supported */
		break;

	case 's':               /* step */

#ifdef AVR8_DEBUG_MODE
		G_StepCmdCount++;
#endif /* AVR8_DEBUG_MODE */

#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
		/* Updating breakpoints is needed, otherwise it would not be
		 possible to step out from breakpoint. We need to replace our trap-code with the
		 original instruction and execute it when stepping from breakpoint.

		 Stepping from breakpoint can occur not only after the program stops on BP,
		 but also if we step through the code and step onto breakpoint. */

		/* If we stopped on break, update breakpoints but do not update if we step from
		 non-breakpoint position. */
		pbreak = gdb_find_break(pc);
		if ( pbreak )
			gdb_update_breakpoints();
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
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qC"), 2) == 0)
			/* current thread is always 1 */
			gdb_send_reply("QC01");
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qfThreadInfo"), 12) == 0)
			/* always 1 thread*/
			gdb_send_reply("m1");
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qsThreadInfo"), 12) == 0)
			/* send end of list */
			gdb_send_reply("l");
		else
			gdb_send_reply("");  /* not supported */

		break;

	default:
		gdb_send_reply("");  /* not supported */
		break;
	}

	return TRUE;
}


#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
/**
 Called before the target starts to run to write/remove breakpoints in flash.
 Note that the breakpoints are inserted to gdb_ctx->breaks at the first free position
 (free means the addr is 0). And removed by setting the addr to 0.
 */
__attribute__((optimize("-Os")))
static void gdb_update_breakpoints(void)
{

	//uint16_t trap_opcode = TRAP_OPCODE;
	uint32_t trap_opcode = TRAP_OPCODE;
	uint8_t i;

	for (i=0; i < AVR8_MAX_BREAKS; i++) {

		/* Ignore free breakpoint structs */
		if (!gdb_ctx->breaks[i].addr)
			continue;

		/* Possible cases:
		 1) BP is enabled and already in flash > do nothing
		 2) BP is enabled but not in flash > write to flash, set IN_FLASH flag
		 3) BP is disabled but written in flash > remove from flash,free BP struct
		 4) BP is disabled and not in flash > free BP struct
		 */
		if ( GDB_BREAK_IS_ENABLED(gdb_ctx->breaks[i]) ) {
			/* BP should be inserted... */
			if ( !GDB_BREAK_IS_INFLASH(gdb_ctx->breaks[i]) ) {
				/* ...and it is not in flash, so write it (2) */
				gdb_ctx->breaks[i].opcode = safe_pgm_read_word((uint32_t)(gdb_ctx->breaks[i].addr << 1));
				gdb_ctx->breaks[i].opcode2 = safe_pgm_read_word((uint32_t)((gdb_ctx->breaks[i].addr << 1)+2));	/* opcode replaced by our infinite loop trap */
				// todo: need to support 32 bit address in dboot_safe_pgm_write
				dboot_safe_pgm_write(&trap_opcode, gdb_ctx->breaks[i].addr, sizeof(trap_opcode));
				GDB_BREAK_SET_INFLASH(gdb_ctx->breaks[i]);
			} /* else do nothing (1)*/

		} else {
			/* BP should be removed... */
			if (GDB_BREAK_IS_INFLASH(gdb_ctx->breaks[i])) {
				/* ...and it is in flash, so remove it, also free the struct  (3) */
				gdb_remove_breakpoint_ptr(&gdb_ctx->breaks[i]);
				GDB_BREAK_CLEAR_INFLASH(gdb_ctx->breaks[i]);	/* not really needed - the struct is freed */

			}  else {
				/* If it is not in flash, just free the struct (4) */
				gdb_ctx->breaks[i].addr = 0;
				gdb_ctx->breaks_cnt--;
			}
		}
	}	/* for */
}
#endif	/* AVR8_BREAKPOINT_MODE == 0 */

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
		if (buff[0] == 'Z')
			gdb_insert_breakpoint(rom_addr_b >> 1);
		else
			gdb_remove_breakpoint(rom_addr_b >> 1);

		gdb_send_reply("OK");
		break;

	default:
		/* we do not support other breakpoints, only software */
		gdb_send_reply("");
		break;
	}
}

__attribute__((optimize("-Os")))
static bool_t gdb_insert_breakpoint(Address_t rom_addr)
{
#ifdef AVR8_DEBUG_MODE
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

	/* First, try to find the breakpoint if it is already inserted */
	struct gdb_break *breakp = gdb_find_break(rom_addr);
	if ( breakp ) {
		GDB_BREAK_ENABLE((*breakp));
		return TRUE;
	}

	/* If breakpoint is not found, add a new one */
	if (gdb_ctx->breaks_cnt >= AVR8_MAX_BREAKS)
			return FALSE;
	gdb_ctx->breaks_cnt++;

	/* find first BP struct which is free - that is addr is 0 and store the BP there */
	for (i=0; i < AVR8_MAX_BREAKS; i++) {
		if (!gdb_ctx->breaks[i].addr) {
			gdb_ctx->breaks[i].addr = rom_addr;
			GDB_BREAK_ENABLE(gdb_ctx->breaks[i]);
			break;
		}
	}

	return TRUE;

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

	if ( j > i )	/* if we found the BP to be removed, there is now one less */
		gdb_ctx->breaks_cnt--;

	if ( gdb_ctx->breaks_cnt == 0 )
		gdb_ctx->breakpoint_enabled = 0;	/* if there are no breakpoints */

#else
	/* Combined mode - BPs in flash */
	struct gdb_break *breakp = gdb_find_break(rom_addr);
	/* Just mark the breakpoint for removal but do not update flash */
	if ( breakp )
		GDB_BREAK_DISABLE((*breakp));
	/*gdb_remove_breakpoint_ptr(breakp);*/
#endif
}

/* ----------------- Functions for flash breakpoints support ------------------- */
#if ( AVR8_BREAKPOINT_MODE == 0 )

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
	uint32_t opcode;
	opcode = (uint32_t)breakp->opcode2 << 16;
	opcode |=  breakp->opcode;

#ifdef AVR8_DEBUG_MODE
	G_RestoreOpcode = opcode;
#endif /* AVR8_DEBUG_MODE */

	dboot_safe_pgm_write(&opcode, breakp->addr, sizeof(opcode));
	//dboot_safe_pgm_write(&breakp->opcode, breakp->addr, sizeof(breakp->opcode));
	breakp->addr = 0;

	/*if (!gdb_ctx->in_stepi)*/
	gdb_ctx->breaks_cnt--;



}

static struct gdb_break *gdb_find_break(Address_t rom_addr)
{
	uint8_t i = 0, sz = AVR8_MAX_BREAKS;
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
		/* jd: for now not implemented */
		/* posix EIO error */
		/* todo: testing only - load from gdb with write memory.
		 * gdb_send_reply("E05");
		return;
		*/
#if 0	/* writing to flash not supported - but it could be as of 4/2017, use dboot_safe_pgm_write */
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
// todo: optimize / temporary buffer for bin2mem..
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

#if 0	/* writing to flash not supported - but it could be as of 4/2017, use dboot_safe_pgm_write */
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

	/* Communicate with gdb */
	handle_exception();

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

}



/*
 * Interrupt handler for the interrupt which allows single-stepping using the
 * feature of AVR that after each interrupt, one instruction of main program
 * is executed before any pending interrupt service routine is called.
 * The interrupt options are:
 * INT0, INT1 or INT2 external interrupts
 * Analog comparator interrupt
 * Names such as INT0_vect are the same for Atmega328 and Atmega 1280/2560 */
#if AVR8_SWINT_SOURCE == 0
ISR ( INT0_vect, ISR_BLOCK ISR_NAKED )
#elif AVR8_SWINT_SOURCE == 1
ISR ( INT1_vect, ISR_BLOCK ISR_NAKED )
#elif AVR8_SWINT_SOURCE == 2
ISR ( INT2_vect, ISR_BLOCK ISR_NAKED )
#elif AVR8_SWINT_SOURCE == 3
ISR ( INT3_vect, ISR_BLOCK ISR_NAKED )
#elif AVR8_SWINT_SOURCE == 4
ISR ( INT4_vect, ISR_BLOCK ISR_NAKED )
#elif AVR8_SWINT_SOURCE == 5
ISR ( INT5_vect, ISR_BLOCK ISR_NAKED )
#elif AVR8_SWINT_SOURCE == 6
ISR ( INT6_vect, ISR_BLOCK ISR_NAKED )
#elif AVR8_SWINT_SOURCE == 7
ISR ( INT7_vect, ISR_BLOCK ISR_NAKED )
#endif

{
#if (AVR8_BREAKPOINT_MODE == 1)/* RAM only BPs */
	static uint8_t ind_bks;
#endif

	save_regs1 ();
	asm volatile ("ori r29, 0x80");	/* user must see interrupts enabled */
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


#ifdef AVR8_DEBUG_MODE
	G_Debug_INTxCount++;
	G_LastPC = gdb_ctx->pc << 1;	// convert to byte address
#endif

	/* if single-stepping, go to trap */
	if ( gdb_ctx->singlestep_enabled)
		goto trap;

#if (AVR8_BREAKPOINT_MODE == 0) /* FLASH Breakpoints code only */


  /* Go straight to trap if using flash breakpoints because this ISR is only
    called if singlestep is enabled (code above handles this for both RAM and flash version)
    OR if breakpoint is encountered in the program memory (which enables the interrupt)
    Note: If we wanted to compare PC address with breakpoints here, we'd need to compare
     gdb_ctx->pc-1 because the PC already points past the breakpoint when this ISR is executed.
  */

  /* Move PC back one word (the size of our trapcode) "on the stack" which we restore when
     returning from ISR */
  (R_PC)--;	/* this is safe also for 32 bit PC, see the note above - we allocate 4 bytes in regs array in this case */
  gdb_ctx->pc = R_PC;
  goto trap;


#if 0
	/* If stopped on a breakpoint, go to trap... */
	for (uint8_t i = 0; i < ARRAY_SIZE(gdb_ctx->breaks); ++i) {
			if (gdb_ctx->pc == gdb_ctx->breaks[i].addr) {
				G_Debug_INTxHitCount++;
				gdb_patch_pc = 1;	/* need to move pc back if we stopped on flash breakpoint */
				gdb_disable_swinterrupt();
				/*EIFR |= _BV(AVR8_SWINT_INTMASK);	*/ /* no need to clear INTx flag, it's cleared automatically */
				goto trap;
			}
	}
#endif

#endif	/* AVR8_BREAKPOINT_MODE == 0 */

#if (AVR8_BREAKPOINT_MODE == 1)/* RAM only BPs */
	if ( gdb_ctx->breakpoint_enabled )
	{
#ifdef AVR8_DEBUG_MODE
		G_BpEnabledINTx++;
#endif

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


#if (AVR8_BREAKPOINT_MODE == 0) /* FLASH only BPs */
	if ( gdb_ctx->breakpoint_step ) {
		/* Special case of continue - we stepped one more step internally and now should continue
		 after updating breakpoints  - we should not send state to GDB, this all happens without GDB knowing.*/
		handle_exception();
		goto out;
	}
#endif

	gdb_send_state(GDB_SIGTRAP);
	handle_exception();

out:

	/* If there is pending interrupt from UART (char received) then we
	 * do not need to generate interrupt from here. Also because INT0 has higher
	 * priority, it will not allow the UART ISR to execute.
	 * Note: all works OK also if we do not disable the interrupt here. */
	if ( UART_RXINT_PENDING() ) {
		gdb_disable_swinterrupt();
	}


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
}



#if  (AVR8_BREAKPOINT_MODE == 0 )  /* Combined mode flash + RAM BPs */
/*
 * Interrupt handler for timer - for flash breakpoints */
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

#endif


/* This function must be naked, otherwise the stack can be corrupted
 * and save_regs1() assumes stack is not touched. */
__attribute__((naked))
void breakpoint(void)
{
	save_regs1 ();
	asm volatile ("cli");		/* disable interrupts */
	save_regs2 ();

#if defined(__AVR_ATmega2560__)
	R_PC_HH &= RET_ADDR_MASK;		/* there is only 1 bit used in the highest byte of PC (17-bit PC) */
#else
	R_PC_H &= RET_ADDR_MASK;
#endif
	gdb_ctx->pc = R_PC;
	gdb_ctx->sp = R_SP;

	gdb_send_state(GDB_SIGTRAP);
	handle_exception();

	/* jump without return */
	asm volatile (ASM_GOTO " restore_registers");

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
	while ( *msg )
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
 * Interrupt handler for timer which allows checking whether the program
 * is stopped on a breakpoint (inserted into the code as RJMP -1 (while(1))
 * instruction instead of the original instruction.
 * This requires rewriting flash when breakpoint is set/cleared.
 * Not implemented! */

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
	asm volatile (
	"sts	regs+31, r31\n"		/* save R31 */
	"sts	regs+30, r30\n"		/* save R30 */
	"ldi	r31, hi8(regs)\n"	/* Z points to regs */
	"ldi	r30, lo8(regs)\n"
	"std	Z+29, r29\n"		/* save R29 */
	"in	r29, __SREG__\n");	/* get SREG */
}

__attribute__((always_inline))
static inline void save_regs2 (void)
{
	asm volatile (
	"std	Z+32, r29\n"		/* put SREG value to his place */
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

#ifdef AVR8_DEBUG_MODE
/* Helper for tests...
  Returns how many bytes are NOT used from given stack buffer.
  Starts from the end (buffer[size-1]).
  How to use: Call this function e.g. in parse_packet and save the return value in global
  variable. When debugging, display the variable in Expressions window in eclipse. The value you
  will see after some debugging will be the number of unused bytes on stack.
  Note that when any byte on stack is used, it no longer contains the canary code, so even if
  stack is freed and used in different part of the code we will get the maximum usage which ever happens. */
uint8_t test_check_stack_usage(void)
{
	return wcheck_stack_usage(stack, GDB_STACKSIZE);
}

static uint8_t wcheck_stack_usage(uint8_t* buff, uint8_t size )
{
	uint8_t i;
	for ( i = size-1; i>0; i--)
	{
		/* look for untouched byte of memory with canary value */
		if ( buff[i] == GDB_STACK_CANARY )
			return i;
	}

	return size;
}

static void wfill_stack_canary(uint8_t* buff, uint8_t size)
{
	uint8_t i;
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

#endif	/* AVR8_DEBUG_MODE */


/* rom_addr - in words, sz - in bytes and must be multiple of two.
   NOTE: interrupts must be disabled before call of this func */
#if 0
// __attribute__ ((section(".nrww"),noinline))
static void safe_pgm_write(const void *ram_addr,
						   uint16_t rom_addr,
						   uint16_t sz)
{

	uint16_t *ram = (uint16_t*)ram_addr;

	/* Sz must be valid and be multiple of two */
	if (!sz || sz & 1)
		return;

	/* Avoid conflicts with EEPROM */
	eeprom_busy_wait();

	/* to words */
	sz >>= 1;

	for (uint16_t page = ROUNDDOWN(rom_addr, SPM_PAGESIZE_W),
		 end_page = ROUNDUP(rom_addr + sz, SPM_PAGESIZE_W),
		 off = rom_addr % SPM_PAGESIZE_W;
		 page < end_page;
		 page += SPM_PAGESIZE_W, off = 0) {

		/* page to bytes */
		uint32_t page_b = (uint32_t)page << 1;

		/* Fill temporary page */
		for (uint16_t page_off = 0;
			 page_off < SPM_PAGESIZE_W;
			 ++page_off) {
			/* to bytes */
			uint32_t rom_addr_b = ((uint32_t)page + page_off) << 1;

			/* Fill with word from ram */
			if (page_off == off) {
				boot_page_fill(rom_addr_b,  *ram);
				if (sz -= 1) {
					off += 1;
					ram += 1;
				}
			}
			/* Fill with word from flash */
			else
				boot_page_fill(rom_addr_b, safe_pgm_read_word(rom_addr_b));
		}

		/* Erase page and wait until done. */
		boot_page_erase(page_b);
		boot_spm_busy_wait();

		/* Write page and wait until done. */
		boot_page_write(page_b);
		boot_spm_busy_wait();
	}

	/* Reenable RWW-section again to jump to it */
	boot_rww_enable ();

}
#endif
/* ------------------------------------------------------------- */

/* EOF */




