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


#if (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT == 1)
	#include "app_api.h"	/* bootloader API used for writing to flash  */
#endif


typedef uint8_t bool_t;
#define FALSE 0
#define TRUE 1

/* Flash writing not supported yet for Arduino Mega, so report this to the user */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#if (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT==1)
#error Flash breakpoints and loading program from the debugger is not supported for Arduino Mega yet.
#endif
#endif

/* Configuration */

/* Support for user-selected baudrate 
Because in some cases the baudrate cannot be derived from AVR MCU type; e.g. Arduino Nano
can use 57600 (old bootloader) or 115200 baudrate and both varians use ATmega328P MCU.
So, if user defines baudrate, we use that; if not, then we select baudrate based on MCU.
The baudrate shloud be defined at compiler level, otherwise it will not get into the library.
For example, add -D flag to compiler options: -DAVR8_USER_BAUDRATE=57600
Note: The user-defined baudrate should be 115200 or 57600, other values may work but are were not tested.
*/
#ifdef AVR8_USER_BAUDRATE
	#define GDB_USART_BAUDRATE AVR8_USER_BAUDRATE	
	#pragma message "Using user-defined baudrate" 
#else
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

#endif	/* AVR8_USER_BAUDRATE */


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
 so even if we tune the packet size to page size less bytes will often be written.
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
/* The opcode of instruction(s) we use for stopping the program at breakpoint.
 Instruction at the BP location is replaced by this opcode.
 To stop the program  we use RJMP on itself, i.e. endless loop,
 1100 kkkk kkkk kkkk, where 'k' is a -1 in words.
 #define TRAP_OPCODE 0xcfff
 To learn that BP was hit we will use periodic interrupt from watchdog.
 */
#define TRAP_OPCODE 0xcfff

  /* todo: remove old comment
  Could use timer but that's in conflict with arduino usage.
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
   New trap opcode - just call our breakpoint function.
   we use call instruction.
   Example:
   0e 94 b4 06 	call	0xd68	; 0xd68 <digitalWrite>
   Address of digitalWrite is 0x06b4 in words which is 0x0d68 in bytes.
   We need to construct our trapcode as a call to breakpoint function.
   Note that gcc automatically uses word address for reference to function name
   so we do not have to convert the address of breakpoint to words.
   #define TRAP_OPCODE  (0x0000940e | ((uint32_t)((uint16_t)breakpoint) << 16))
   This results in,
   Example (breakpoint function located at 0x2090 byte addr which is 0x1048 in words:
   0e 94 48 10 	call	0x2090	;  0x2090
	this only works for our breakpoint function located within 16-bit word address,
    that is the function must be within 128 kB of flash.. which is always true for atmega328
    and probably also on Atmega2560 be in most cases.

 trap for calling breakpoint function
#define TRAP_OPCODE  (0x0000940e | ((uint32_t)((uint16_t)breakpoint) << 16))

trap for ext interupt enable with infinite loop after this
#if 0
#if AVR8_SWINT_SOURCE == 0
	#define TRAP_OPCODE 0xcfff9ae8
#elif AVR8_SWINT_SOURCE == 1
	#define TRAP_OPCODE 0xcfff9ae9
#else
	#error The value of AVR8_SWINT_SOURCE is not supported. Define valid opcode for this value here.
#endif
#endif
*/

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
	uint8_t skip_step;				/* Indicates special step from a breakpoint */

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


#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
	static void gdb_update_breakpoints(void);
#endif



static inline void restore_regs (void);
static inline void save_regs1 (void);
static inline void save_regs2 (void);

static uint8_t safe_pgm_read_byte(uint32_t rom_addr_b);

/* Helpers for determining required size of our stack and for printing to console*/
#ifdef AVR8_STUB_DEBUG
#define		GDB_STACK_CANARY	(0xBA)
uint8_t test_check_stack_usage(void);
static void wfill_stack_canary(uint8_t* buff, uint8_t size);
static uint8_t wcheck_stack_usage(uint8_t* buff, uint8_t size );	/* returns how many bytes are used from given buffer */
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
// todo: remove uint16_t G_skipPC = 0;

/* Helper macros to work with LED*/
#if 0
/* LED is on pin PB7 on Arduino Mega, PD5 on Arduino Uno (Arduino pin 13) */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
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


#endif  /* AVR8_STUB_DEBUG */


#if (AVR8_BREAKPOINT_MODE == 0)		/* Flash breakpoints */
	static uint16_t safe_pgm_read_word(uint32_t rom_addr_b);
	static struct gdb_break *gdb_find_break(Address_t rom_addr);
	static void gdb_remove_breakpoint_ptr(struct gdb_break *breakp);
#endif

/* Code used only if flash qwriting is needed */
#if (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT == 1)
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
 */
#if (AVR8_LOAD_SUPPORT == 1)
	#define GDB_STACKSIZE 	(144)
#else
	#if (AVR8_BREAKPOINT_MODE == 1)
		#define GDB_STACKSIZE 	(80)			/* Internal stack size for RAM only BP */
	#else
		#define GDB_STACKSIZE 	(104)			/* Internal stack size for FLASH BP */
	#endif
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
	/* todo: remove code
	 Enable watchdog. The ISR will do nothing if the program is not stopped on a BP */
	/* not needed here */
	/* watchdogConfig(GDB_WATCHDOG_TIMEOUT); */
#endif

#ifdef AVR8_STUB_DEBUG
	/* For testing stack usage only - fill satack with canary values  */
	wfill_stack_canary((uint8_t*)stack, GDB_STACKSIZE);
#endif

	/* Initialize serial port */
	uart_init();

#if (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT == 1)		/* Flash BP or load binary supported */
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
#endif

}

#if (AVR8_BREAKPOINT_MODE == 0) || (AVR8_LOAD_SUPPORT == 1)		/* Flash BP or load binary supported */
/* This is used to report to the user that flash breakpoints or load are enabled but
   the bootloader does not support this.
   For Arduino code, which uses timer interrupts before our debug_init is called
   we cannot simply fall into endless loop and let the user see the situation in debugger
   because the program will likely stop in handlers. So we disable timer interrupts and
   then wait. */
__attribute__((optimize("-Os")))
static void gdb_no_bootloder_prep(void) {

	/* IMPORTANT: if you find yourself here it means you have enabled flash breakpoints and/or
	   load via debugger but your board does not have the bootloader to support this.
	   Please burn the bootloader provided for this debugger to use the flash breakpoints and/or load.
	 */
	cli();
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	// todo: disable timers for arduino mega

#else
	/* disable all timer interrupts */
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
				/* Do not go to ext int ISR after every instruction. For RAM breakpoints this is
				  the case if the program is let run and it can be only stopped by break command (pause).
				  For flash breakpoints it is the same plus the program can also break itself when it encounters
				  breakpoint in flash.	*/
				gdb_disable_swinterrupt();
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
#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
		/* Update the flash so that the program can run after reset without breakpoints */
		gdb_update_breakpoints();
#endif
		gdb_send_reply("OK");
		return FALSE;

	case 'c':               /* continue */

#ifdef AVR8_STUB_DEBUG
		G_ContinueCmdCount++;
#endif /* AVR8_STUB_DEBUG */

#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
		/* Enable watchdog interrupt. It is automatically disabled when the ISR
		 is called so it must re-enable itself if needed */
		gdb_update_breakpoints();
		/* todo: could enable only if at least one BP is set */
		watchdogConfig(GDB_WATCHDOG_TIMEOUT);
#endif
		return FALSE;

	case 'C':               /* continue with signal */
	case 'S':               /* step with signal */
		gdb_send_reply(""); /* not supported */
		break;

	case 's':               /* step */

#ifdef AVR8_STUB_DEBUG
		G_StepCmdCount++;
#endif /* AVR8_STUB_DEBUG */

#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
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
	/*uint32_t trap_opcode = TRAP_OPCODE; */
	uint16_t trap_opcode = TRAP_OPCODE;
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
				/*gdb_ctx->breaks[i].opcode2 = safe_pgm_read_word((uint32_t)((gdb_ctx->breaks[i].addr << 1)+2));*/	/* opcode replaced by our infinite loop trap */
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
	/*
	uint32_t opcode;
	opcode = (uint32_t)breakp->opcode2 << 16;
	opcode |=  breakp->opcode;
	*/
	uint16_t opcode = breakp->opcode;

#ifdef AVR8_STUB_DEBUG
	G_RestoreOpcode = opcode;
#endif /* AVR8_STUB_DEBUG */

	dboot_safe_pgm_write(&opcode, breakp->addr, sizeof(opcode));
	breakp->addr = 0;

	gdb_ctx->breaks_cnt--;
}

/* rom_addr is in words */
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
		/* posix EIO error */
		gdb_send_reply("E05");
		return;
#if 0	/* writing to flash not supported - but it could be as of 4/2017, use dboot_safe_pgm_write
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
	/* asm volatile ("ori r29, 0x80");	*/ /* user must see interrupts enabled */
	/* zruseno povolovani pro user, nevim proc ma videt enabled... */
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
			"out	__SREG__, r31\n"	/* restore SREG */
			"lds	r31, regs+31\n"		/* real value of r31 */
			"reti \n");
	/* zjednoduseny exit kod - proste obnovim SREG, preruseni budou povolena protoze vykonam RETI */

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
	/*asm volatile ("ori r29, 0x80");*/	/* user must see interrupts enabled */
	/* zruseno povolovani pro user, nevim proc ma videt enabled... */

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


#if (AVR8_BREAKPOINT_MODE == 0) /* FLASH only BPs */
	/*
	if ( gdb_ctx->skip_step ) {
		handle_exception();
		goto out;
	}*/
#endif

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

	asm volatile (
			"restore_registers:");
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
 * Interrupt handler for watchdog which allows checking whether the program
 * is stopped on a breakpoint (inserted into the code as RJMP -1 (while(1))
 * instruction instead of the original instruction.
 */
/* Watchdog interrupt vector */
#if (AVR8_BREAKPOINT_MODE == 0 )	/* code is for flash BP only */
ISR(WDT_vect, ISR_BLOCK ISR_NAKED)
{
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

	/* Nothing */
	/* Re-enable watchdog interrupt as it is disabled when ISR is run.
	  Do this only if we are'n on a breakpoint yet, so we can  check again next time.
	  If we are on a BP, no need to run this ISR again.  */
	watchdogConfig(GDB_WATCHDOG_TIMEOUT);
	goto out;

trap:
	/* Set correct interrupt reason */
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
#endif /* AVR8_BREAKPOINT_MODE == 0  */
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
	 Nova verze, dle gdb.c co nejdrive zakazat preruseni */
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

	/* Original verze, nebezpecna pokud se vola bez zakazanych preruseni, protoze
	  ISR muze poskodit obsah registru r31, r30...
	  Pro pouziti v ISR to neni problem, preruseni jsou zazakana ale pro breakpoint() by
	  to mohlo delat problemy... */
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




