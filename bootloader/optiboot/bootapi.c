/*
 * bootapi.c
 *
 * Define the API functions for the bootloader API
 * This file is for the bootloader only!
 * The user app should include app_api.h and add to project app_api.c
 *
 * To build the bootloader add to the linker command line:
 * -Wl,--section-start=.text=0x7800 -Wl,--section-start=.version=0x7ffe
 * -Wl,--section-start=.opti_api=0x7ff0 -Wl,--relax -Wl,--gc-sections
 * -nostartfiles -nostdlib -Wl,--undefined=api_functions
 *
 * The address of section .opti_api must be in sync with the JUMP_TABLE_LOCATION
 * defined in app_api.c. See
 * The section .version is used by optiboot bootloader to store its version.
 *
 * NOTE:
 * The functions from this file should not be called directly. They are
 * called from wrapper functions defined in app_api.c
 *
 *  Created on: 16. 3. 2017
 *      Author: jan dolinay
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>

#include "pin_defs.h"
#include "bootapi.h"


/* ================ START BOOTLOADER API ==================== */
/* AVR_DEBUG API
  Define the API for user applications.
  Based on XBOOT project https://github.com/alexforencich/xboot

 Add to linker command line:
 --section-start=.opti_api=0x7fe8
 and
 -Wl,--undefined=api_functions
 without the "undefined" the linker will remove the section as unused, the word unused alone is not enough.

 The start of section .text=0x7800 can be moved to lower address (and the bootloader size set by fuses increased
 accordingly. Possible values:
 0x7c00 - bootloader size 512 words (1 kB)
 0x7800 - 1024 words, 2 kB
 0x7000 - 2048 words, 4 kB.
 Note that the build tools use byte addresses while the AVR programming tools
 and docs work with word addresses. So if the bootloader size is e.g. 1024 words,
 it starts at address 0x7800 for build tools but when setting the fuses in the MCU
 you set the bootloader address to 0x3C00.

 The address of opti_api 0x7fe8 allows for the size of the api_functions structure 21 Bytes (because
 the end of memory (last available address) is at 0x7ffd. At 0x7ffe the version of optiboot is stored.
 IMPORTANT: if you change this address, change also JUMP_TABLE_LOCATION in app_api.c
 */


/* Version of bootloader */
#define AVRDBG_BOOT_VERSION_MAJOR 1
#define AVRDBG_BOOT_VERSION_MINOR 1

/* Version of the API expected by this code.
   Keep in sync with app_api.c */
#define	BOOT_API_VERSION	(2)

/* Define the API structure */
/* API Version: 2 */
/* ID: ABj */
/* Size of the struct: 3 B + 1 B + number_of_functions x 2 B,
 * currently, with 5 functions, size is 14 B */
struct avrdbgboot_jump_table_s api_functions __attribute((section(".opti_api"))) = {
        {'A', 'B', 'j'}, BOOT_API_VERSION,
        {
            /* API functions */
            (uint16_t)(dboot_get_version),
            (uint16_t)(dboot_led_init),
			(uint16_t)(dboot_led_toggle),
			(uint16_t)(dboot_safe_pgm_write),
			(uint16_t)(dboot_handle_xload),
        }
};

/* set LED pin as output */
void dboot_led_init(void)
{
	LED_DDR |= _BV(LED);
}

/* toggle the LED on Arduino pin */
void dboot_led_toggle(void)
{
	LED_PIN |= _BV(LED);
}

/* Obtain the version of the bootloader */
uint8_t dboot_get_version(uint16_t *ver) {
	*ver = (AVRDBG_BOOT_VERSION_MAJOR << 8) | (AVRDBG_BOOT_VERSION_MINOR);
	return 0;
}


#if (SPM_PAGESIZE & (SPM_PAGESIZE - 1))
#error SPM_PAGESIZE is not power of two! Impossible!
#endif

#define SPM_PAGESIZE_W (SPM_PAGESIZE>>1)
#define ROUNDUP(x, s) (((x) + (s) - 1) & ~((s) - 1))
#define ROUNDDOWN(x, s) ((x) & ~((s) - 1))

static uint16_t safe_pgm_read_word(uint32_t rom_addr_b)
{
#ifdef pgm_read_word_far
	if (rom_addr_b >= (1l<<16))
		return pgm_read_word_far(rom_addr_b);
	else
#endif
		return pgm_read_word(rom_addr_b);
}


/* Write to program memory.
 * Supports writing multiple pages; it's possible to
 * write virtually buffer of any size.
 * rom_addr - in words,
 * sz - in bytes and must be multiple of two.
   NOTE: interrupts must be disabled before call of this func */
__attribute__ ((noinline))
void dboot_safe_pgm_write(const void *ram_addr,
						   uint16_t rom_addr,
						   uint16_t sz)
{
	uint16_t *ram = (uint16_t*)ram_addr;

	/* Sz must be valid and be multiple of two */
	if (!sz || (sz & 1))
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
/* ================ END BOOTLOADER API ==================== */



