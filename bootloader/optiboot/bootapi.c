/*
 * bootapi.c
 *
 * Define the API functions for the bootloader API
 * This file is for the bootloader only!
 * The user app should include app_api.h and add to project app_api.c
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
 --section-start=.opti_api=0x7ff0
 and
 -Wl,--undefined=api_functions
 without the "undefined" the linker will remove the section as unused, the word unused alone is not enough.
 */
/* Version */
#define AVRDBG_BOOT_VERSION_MAJOR 1
#define AVRDBG_BOOT_VERSION_MINOR 1


/* Define the API structure */
/* API Version: 1 */
/* ID: ABj */
struct avrdbgboot_jump_table_s api_functions __attribute((section(".opti_api"))) = {
        {'A', 'B', 'j'}, 1,
        {
            /* API functions */
            (uint16_t)(boot_get_version),
            (uint16_t)(boot_led_init),
			(uint16_t)(boot_led_toggle),
			(uint16_t)(dboot_safe_pgm_write),

        }
};


void boot_led_init(void)
{
	LED_DDR |= _BV(LED);
}

void boot_led_toggle(void)
{
	LED_PIN |= _BV(LED);
}

// bootloader version
uint8_t boot_get_version(uint16_t *ver) {
	*ver = (AVRDBG_BOOT_VERSION_MAJOR << 8) | (AVRDBG_BOOT_VERSION_MINOR);
	return 0;
}

// ----------------------------------------
// Write to program memory
// from GDBServer project
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


/* rom_addr - in words, sz - in bytes and must be multiple of two.
   NOTE: interrupts must be disabled before call of this func */
// todo: support more than 128 KB flash
__attribute__ ((noinline))
void dboot_safe_pgm_write(const void *ram_addr,
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
/* ================ END BOOTLOADER API ==================== */



