/*
 * stub.c
 * Code for the GDB stub avr8-stub which needs to be in the bootloader section.
 * This code is part of the bootloader.
 *
 *  Created on: 2. 6. 2017
 *      Author: jan dolinay
 */
#include <inttypes.h>
#include <avr/io.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "bootapi.h"

typedef uint8_t bool_t;
#define FALSE 0
#define TRUE 1


/* Memory definitions from avr8-stub.c - could be merged to header but not worth complication the
  file structure. This will never change. */
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

#else
	#define MEM_SPACE_MASK 0x00ff0000
	#define FLASH_OFFSET   0x00000000
	#define SRAM_OFFSET    0x00800000	/* GDB works with linear address space; RAM address from GBD will be (real addresss + 0x00800000)*/
#endif

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
void watchdogConfig(uint8_t x);	/* defined in optiboot.c */

/** Size of the buffer we use for receiving messages from gdb. */
#define AVR8_MAX_BUFF   	(160)


/* Buffer for messages from GDB */
/* ATmega328 has 128 B page. GDB max for binary write is 256.
   How much bytes GDB sends depends on what we return in PacketSize whis is in the app stub.
   It makes no sense to tune the size so that gdb sends 128 B chunks of data because some bytes are escaped
   so we will write over page boundary anyway.
*/
uint8_t G_buff[AVR8_MAX_BUFF];

/* temporary buffer for write to binary
  Do not worry about RAM usage here in bootloader, when the user app runs it
  uses all the RAM */
uint8_t tmp_buff[AVR8_MAX_BUFF];

/* Convert number 0-15 to hex */
#define nib2hex(i) (uint8_t)((i) > 9 ? 'a' - 10 + (i) : '0' + (i))

uint8_t G_buff_sz;


/* Cannot use PROGMEM because this places the variables into program section before main
 and we need the code of main to start exactly at bootloader start address.
 So place them into separate section.
const char FlashOk[] PROGMEM = "OK";
const char FlashE05[] PROGMEM = "E05";
const char FlashEmpty[] PROGMEM = "";
*/

/* This places the variables into program memory into special section.
  This section location is defined in linker flags:
  -Wl,-section-start=.mystrings=0x7fc0
  But it is optimized out by the compiler, so the section is removed,
  but it is in the code, inlined, just as some of the functions, so you
  won't find it in the listing.
*/
asm("  .section .mystrings\n");
const char FlashOk[] = "OK";
const char FlashE05[] = "E05";
const char FlashEmpty[] = "";
asm("  .section .text\n");

/* C version of placing the variables into separate section but doesn't work for
   multiple variables - would need one section for each var.
__attribute__((section(".mystrings"))) const char FlashOk[] = "OK";
__attribute__((section(".mystrings"))) const char FlashE05[] PROGMEM = "E05";
__attribute__((section(".mystrings"))) const char FlashEmpty[] PROGMEM = "";
*/

//const uint16_t optiboot_version __attribute__((section(".version"))) = OPTIBOOT_MAJVER*256 + OPTIBOOT_MINVER;

/* Convert a hexadecimal digit to a 4 bit nibble. */
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

/* UART routines could be reused from optiboot but these are complicated and its very little code anyway. */

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

static uint8_t parse_hex(const uint8_t *buff, uint32_t *hex)
{
	uint8_t nib, len;
	for (*hex = 0, len = 0; (nib = hex2nib(buff[len])) != 0xff; ++len)
		*hex = (*hex << 4) + nib;
	return len;
}

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

static void my_memcpy_P(uint8_t *dest, const uint8_t *src, size_t cnt) {
	uint8_t c;
	while(cnt-- > 0) {
		c = pgm_read_byte(src++);
		*dest++ = c;
	}
}

static uint8_t my_strlen_P(const uint8_t *str) {
	uint8_t len = 0;
	while (pgm_read_byte(str++) != 0x00)
		len++;
	return len;
}

static void gdb_send_reply(const char *reply)
{
	G_buff_sz = my_strlen_P((const uint8_t*)reply);
	if ( G_buff_sz > (AVR8_MAX_BUFF - 4))
		G_buff_sz = AVR8_MAX_BUFF - 4;

	my_memcpy_P(G_buff, (const uint8_t*)reply, G_buff_sz);
	gdb_send_buff(G_buff, G_buff_sz);

}

/* Convert the binary stream in BUF to memory.

   Gdb will escape $, #, and the escape char (0x7d).
   COUNT is the total number of bytes to write into
   memory.
   Code from gdb sample m32r-stub.c
   Modified to escape also 0x2A */
static unsigned char *bin2mem(unsigned char *buf, unsigned char *mem, int count) {
	int i;

	/* Note: It could be better not to be paranoid below and un-escape everything after 0x7d.
	   It took me some time to figure out that 0x2A was escaped also even though not documented */

	for (i = 0; i < count; i++) {
		/* Check for any escaped characters. Be paranoid and
		 only unescape chars that should be escaped. */
		if (*buf == 0x7d) {
			switch (*(buf + 1)) {
			case 0x3: /* # */
			case 0x4: /* $ */
			case 0x5d: /* escape char */
			case 0x0a: /* * (0x2A) is also escaped by avr-gdb */
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
/** Support for binary load of program */
static void gdb_write_binary(const uint8_t *buff /*, uint16_t size*/) {
	uint32_t addr, sz;
	uint8_t* end;

	buff += parse_hex(buff, &addr);
	/* skip 'xxx,' */
	buff += parse_hex(buff + 1, &sz);
	/* skip , and : delimiters */
	buff += 2;

	if ((addr & MEM_SPACE_MASK) == FLASH_OFFSET) {
		/* Write to flash. GDB sends binary data; not characters in hex */
		end = bin2mem((unsigned char *)buff, (unsigned char *)tmp_buff, sz);
		sz = (end - (uint8_t*)tmp_buff);	/* bin2mem returns address of buffer at the end */
		addr &= ~MEM_SPACE_MASK;
		/* to words */
		addr >>= 1;

		dboot_safe_pgm_write(tmp_buff, (uint16_t)addr, sz );

	} else {
		/* We do not support writing to RAM or EEPROM. Report posix EIO error */
		gdb_send_reply(FlashE05);
		return;
	}
	gdb_send_reply(FlashOk);

}


/* Run the user app by triggering watchdog reset.
 * Jumping to reset vector 0 does not work so easy, proably needs to
 * set SP and the MCU is not in the same state as after reset. */
void run_user_app(void) {
	 watchdogConfig(WATCHDOG_16MS);    /* shorten WD timeout */
	 while (1)			      /* and busy-loop so that WD causes */
	   ;				      /*  a reset and app start. */

}


/* This is the main "dispatcher" of commands from GDB
 * If returns false, the debugged program continues execution */
static bool_t gdb_parse_packet(const uint8_t *buff)
{

	switch (*buff) {
	case 'X':
		gdb_write_binary(buff + 1);
		break;

		/* Write PC command we just ignore it and respond ok */
	case 'P':
	case 'G':	/* write registers */
		gdb_send_reply(FlashOk);
		/* Restart the APP */
		run_user_app();
		break;

		/* step command - run the application. Probably not needed, if
		 we report that we support P, gdb will just set PC.  */
	case 's':
	case 'c':
		gdb_send_reply(FlashOk);
		/* Restart the APP */
		run_user_app();
		break;

	default:
		gdb_send_reply(FlashEmpty);  /* not supported */
		/* Restart the APP ? */
		run_user_app();
		break;
	}

	return TRUE;
}


/*
  This is exported to bootloader API. The GDB stub in user application
  calls this when GDB asks for binary load (X packet).
  GDB sends X0,0 first to check whether the stub supports binary load.
  The stub in app responds OK and calls this bootloader code which handles the
  load and restarts the MCU.
 */
__attribute__ ((noinline))
void dboot_handle_xload(void)
{
	uint8_t checksum, pkt_checksum;
	uint8_t b;

	while (1) {
		b = getDebugChar();

		switch(b) {
		case '$':
			/* Read everything to buffer */
			G_buff_sz = 0;
			for (pkt_checksum = 0, b = getDebugChar();
				 b != '#'; b = getDebugChar())
			{
				G_buff[G_buff_sz++] = b;
				pkt_checksum += b;
			}
			G_buff[G_buff_sz] = 0;

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
			/* Need real size because with binary data cannot use 0 as terminator. */
			if (gdb_parse_packet(G_buff))
				continue;
			/* leave the trap, continue execution */
			run_user_app();
			break;	/* never executed */

		case '-':  /* NACK, repeat previous reply */
			gdb_send_buff(G_buff, G_buff_sz);
			break;
		case '+':  /* ACK, great */
			break;
		case 0x03:
			/* user in.terrupt by Ctrl-C, send current state and
			   continue reading in normal stub, but we do not support this,
			   so just restart and let the normal stub take control */
			run_user_app();
			break;
		default:
			gdb_send_reply(FlashEmpty); /* not supported */
			break;
		}	/* switch */
	}	/* while */

}

