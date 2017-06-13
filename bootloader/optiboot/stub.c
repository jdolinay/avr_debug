/*
 * stub.c
 * Code for the GDB stub which should be in bootloader section.
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

# define cli()  __asm__ __volatile__ ("cli" ::: "memory")

/* Exports */
void dboot_handle_xload(void);


/* todo: memory definitions from avr8-stub.c - could be merged to header? */

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


/* todo: this is copy from optiboot.c, move to header? */
#if defined(__AVR_ATmega168__)
#define RAMSTART (0x100)
#define NRWWSTART (0x3800)
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#define MY_RAMSTART (0x100)
/*#define NRWWSTART (0x7000)*/
#elif defined (__AVR_ATmega644P__)
#define RAMSTART (0x100)
#define NRWWSTART (0xE000)
#elif defined(__AVR_ATtiny84__)
#define RAMSTART (0x100)
#define NRWWSTART (0x0000)
#elif defined(__AVR_ATmega1280__)
#define RAMSTART (0x200)
#define NRWWSTART (0xE000)
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)
#define RAMSTART (0x100)
#define NRWWSTART (0x1800)
#endif

/** Size of the buffer we use for receiving messages from gdb. */
#define AVR8_MAX_BUFF   	(130)


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

const char FlashOk[] PROGMEM = "OK";
const char FlashE05[] PROGMEM = "E05";
const char FlashEmpty[] PROGMEM = "";


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
   Code from gdb sample m32r-stub.c */
static unsigned char *bin2mem(unsigned char *buf, unsigned char *mem, int count) {
	int i;

	/* todo: It could be better not to be paranoid below and un-escape everything after 0x7d.
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
	char cSREG;

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

		/* disable interrupts */
		/* Not needed, the API wrapper disables interrupts.
		cSREG = SREG;
		cli();*/
		dboot_safe_pgm_write(tmp_buff, (uint16_t)addr, sz );
		/* enable interrupts (restore) */
		/*SREG = cSREG; */

		/*}*/
	} else {
		/* posix EIO error */
		gdb_send_reply(FlashE05);
		return;
	}
	gdb_send_reply(FlashOk);

}


/* Run the user app by jumping to reset vector 0 */
void run_user_app(void) {
	/* Wait a while */
	uint16_t cnt = 50000;
	while( cnt-- > 0 )
		;

	/* Jump to RST vector */
	__asm__ __volatile__ (
			"clr r30\n"
			"clr r31\n"
			"ijmp\n"
	);
}


/* This is the main "dispatcher" of commands from GDB
 * If returns false, the debugged program continues execution */
__attribute__((optimize("-Os")))
static bool_t gdb_parse_packet(const uint8_t *buff)
{

	switch (*buff) {
	case 'X':
		gdb_write_binary(buff + 1);
		break;

		/* Write PC command we just ignore it and respond ok */
	case 'P':
		gdb_send_reply(FlashOk);
		/* Jump to RST vector */
		/*run_user_app();*/
		break;

		/* step command - run the application. Probably not needed, if
		 we report that we support P, gdb will just set PC.  */
	case 's':
	case 'c':
		gdb_send_reply(FlashOk);
		/* Jump to RST vector */
		run_user_app();
		break;

	default:
		gdb_send_reply(FlashEmpty);  /* not supported */
		break;
	}

	return TRUE;
}


/*
Plan: app stub pokud dostane X0 tj. test GDB yda podporuje bin load skoci sem,
 pak zpracuje load a restartuje app...
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
			   continue reading */
			//gdb_send_state(GDB_SIGINT);
			break;
		default:
			gdb_send_reply(FlashEmpty); /* not supported */
			break;
		}	/* switch */
	}	/* while */

}

