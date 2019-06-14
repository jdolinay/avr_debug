/*
 * avr8-stub.h
 *
 *  Created on: 25. 3. 2015
 *      Author: Jan Dolinay
 *
 *  Header file for
 *  GDB Debugging Agent (GDB stub) for ATMega328 and ATMega1280 and ATMega2560
 * This code is actually "gdb stub" - code which runs in the debugged MCU and
 * communicates with the gdb debugger. It uses gdb RCP protocol.
 * The code needs to handle UART Receive interrupt and an interrupt for simulating
 * software interrupt (EXT INT).
 *
 * Configuration options (for details see the defines in the code below):
 * AVR8_BREAKPOINT_MODE - how to implement breakpoints and stepping.
 * AVR8_SWINT_SOURCE - which external interrupt is used by the debugger (the corresponding pin cannot be used by user program!)
 * AVR8_LOAD_SUPPORT - whether the stub should support loading the program by GDB. Then you can upload the program
 *  and start debugging with single click in eclipse IDE.
 *
 *
 * If needed, the UART communication speed can be changed in avr8-stub.c, see GDB_USART_BAUDRATE.
 *
 *
 * The following project were used (and combined) to create this stub:
 * 1) AVR-GDBServer (https://github.com/rouming/AVR-GDBServer)
 * 2) avr-stub from uOS - embedded operating system (https://code.google.com/p/uos-embedded/)
 * 3) AVR gdbstub (http://zevv.nl/play/code/avr-gdbstub/)
 *
 * The API is inspired by the GDB documentation for stubs: (quote)
 * The debugging stub for your architecture supplies these three subroutines:
 * set_debug_traps
 * This routine arranges for handle_exception to run when your program stops. You must call this subroutine explicitly in your program's startup code.
 * handle_exception
 * This is the central workhorse, but your program never calls it explicitly—the setup code arranges for handle_exception to run when a trap is triggered.
 * handle_exception takes control when your program stops during execution (for example, on a breakpoint), and mediates communications with gdb on the host machine. This is where the communications protocol is implemented; handle_exception acts as the gdb representative on the target machine. It begins by sending summary information on the state of your program, then continues to execute, retrieving and transmitting any information gdb needs, until you execute a gdb command that makes your program resume; at that point, handle_exception returns control to your own code on the target machine.
 * breakpoint
 * Use this auxiliary subroutine to make your program contain a breakpoint. Depending on the particular situation, this may be the only way for gdb to get control. For instance, if your target machine has some sort of interrupt button, you won't need to call this; pressing the interrupt button transfers control to handle_exception—in effect, to gdb. On some machines, simply receiving characters on the serial port may also trigger a trap; again, in that situation, you don't need to call breakpoint from your own program—simply running ‘target remote’ from the host gdb session gets control.
 * Call breakpoint if none of these is true, or if you simply want to make certain your program stops at a predetermined point for the start of your debugging session.
 * (end quote)
 *
 * Note about porting to other AVR MCUs:
 * You need to change:
 * 1) UART code in uart_init, putChar and getChar
 * 2) Code for software interrupt emulation in gdb_enable_swinterrupt and gdb_disable_swinterrupt
 *
 * Port to Atmega 1280 (2560), Arduino Mega
 * USART routines work, ISR name changes.
 * PC is 3 bytes on Mega2560 and 2 bytes on 1280 as follows from Stack Pointer chapter in the datasheet:
 * The ATmega48A/PA/88A/PA/168A/PA/328/P Program Counter (PC) is 11/12/13/14 bits wide
 * The ATmega640/1280/1281/2560/2561 Program Counter (PC) is 15/16/17 bits wide
 *
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
 *
 */

#ifndef DRV_DEBUG_H_
#define DRV_DEBUG_H_


#include <avr/io.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/*  --------- Configuration ------------ */
/** AVR8_BREAKPOINT_MODE
 * Select the mode for handling breakpoints and step command.
 * Options:
 *  0 - Combined - FLASH breakpoints, stepping using RAM.
 *  1 - RAM only - RAM breakpoints and stepping
 *
 * More info:
 * RAM breakpoints - use external interrupt to stop the program after each instruction and compare PC
 * 	with breakpoints. If any BP address is reached, the program is halted. Program runs slow with BP enabled.
 * (+) No wear of flash memory
 * (-) Debugged program runs slowly - interrupt occurs after every instruction
 * 		and in the ISR there are perhaps 100 cycles needed to save and restore context.
 * 		How much the debugged program is affected depends also on the design of the program,
 * 		for example, blinking LED with delays based of poling timer value may be affected
 * 		little, while delays using busy loop will be much much longer than expected.
 *
 * Flash breakpoints - writes special instruction at the position where program should stop.
 * (-) Flash memory is overwritten often during debug session. It survives 10 000 erase-write cycles.
 * (+) Debugged program runs at normal (full) speed between breakpoints.
 *
 *
 * Note: FLASH only breakpoints and stepping by writing to flash is not supported.
 * this option would wear the flash memory very fast. There are about 10 overwrites
 * for a single step in the debugger.
 *
 * */
#define		AVR8_BREAKPOINT_MODE	(1)


/**
 * AVR8_SWINT_SOURCE
 * Source for software interrupt.
 * Valid values are listed below. The pin selected here will be used for generating SW interrupt
 * by the debugger and cannot be used for anything else - it becomes reserved for the debugger.
 *
 * Supported values for Atmega328 (UNO):
 * 0 - use INT0 (pin PD2, Arduino pin 2)
 * 1 - use INT1 (pin PD3, Arduino pin 3)
 *
 * Supported values for Atmega2560 and Atmega1280:
 * TIP: Use value 6 or 7; the pins for these INTs are not connected on Arduino
 * mega board, so you will not waste any pin.
 * 0 - INT0 (pin PD0, Arduino pin 21)
 * 1 - INT1 (pin PD1, Arduino pin 20)
 * 2 - INT2 (pin PD2, Arduino pin 19)
 * 3 - INT3 (pin PD3, Arduino pin 18)
 * 4 - INT4 (pin PE4, Arduino pin 2)
 * 5 - INT5 (pin PE5, Arduino pin 3)
 * 6 - INT6 (pin PE6, not used on Arduino)
 * 7 - INT7 (pin PE7, not used on Arduino)
 * Note to implementors
 *  INT0-3 uses EICRA reg. and port D
 *  INT4 - 7 uses EICRB reg. and port E
 *  Pins PE6 and PE7 are not connected on Arduino Mega boards.
 * Note: Probably Pin Change Interrupt (PCINT) could be used also.
 * It could be one of the Arduino analog pins (PC0 - PC5) which are less likely
 * to be used by the user program.
 * Note that if PCINT is used, then INT0 and INT1 used by the user program
 * could cause troubles, because they have higher priority than PCINT and could prevent
 * the debugger from catching breakpoints properly...this needs to be verified.
 */
#define	AVR8_SWINT_SOURCE	(0)


/**
  AVR8_LOAD_SUPPORT
  Enable or disable support for loading the program from the debugger.
  If enabled you can just click the Debug button in IDE to upload new program
  and debug it.
  IMPORTANT: requires support in bootloader so the bootloader in Arduino must
   be replaced with the bootloader provided in this package.

   Options:
    0 - load from GDB disabled. Load the program using avrdude as usual.
    1 - load from GDB enabled.
*/
#define	AVR8_LOAD_SUPPORT	(0)


/**
 * Maximum number of breakpoints supported.
 * Note that gdb will set temporary breakpoint, for example, for step into function
 * or the Run to line command, so the actual number of breakpoints user can set will
 * be lower.
 * For flash breakpoints it is better to use lower number - each breakpoint means
 * writing to flash, user will often forget about old breakpoints and these will be
 * rewritten to flash with no need...So it is better to limit the number and make
 * the user delete breakpoints when not needed. It is quite possible to debug
 * a program with single breakpoint.
 */
#if (AVR8_BREAKPOINT_MODE == 1)	/* RAM only breakpoints; recommended 8 breakpoints */
	#define AVR8_MAX_BREAKS       (8)
#else							/* Flash breakpoints, RAM stepping; recommended 4 breakpoints */
	#define	AVR8_MAX_BREAKS       (4)
#endif


/**
 * Define this to enable some global variables to make it easier to debug this stub.
 * This is for advanced users who need to debug the debugger (gbd stub) itself.
 */
/* #define AVR8_STUB_DEBUG */




/**
 * Initialize the debugger driver
 * ? Is this our version of set_debug_traps ? (see above)
 * But the interrupts are re-directed in compile time, so we do not need such function.
 */
void debug_init(void);

/**
 * Insert breakpoint at the position where this function is called.
 * To set-up breakpoints before compilation.
*/
void breakpoint(void);


/**
 * Send text message to gdb console.
 * The function appends "\n" to the msg.
 * Note that GDB will queue the messages until "\n" (0x0a) is received; then
 * it displays the messages in console.
 */
void debug_message(const char* msg);



#ifdef __cplusplus
}
#endif


#endif /* DRV_DEBUG_H_ */
