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
/* ================ END BOOTLOADER API ==================== */



