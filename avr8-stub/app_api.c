/*
 * app_api.c
 *
 * Implementation of the bootloader API.
 * This file should be built with the user app.
 *
 * Usage:
 * Call the function as needed.
 * You can also call boot_init_api, but it is called automatically if you don't
 *
 *
 *  Created on: 16. 3. 2017
 *      Author: jan dolinay
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "app_api.h"

/* jump table struct */
struct avrdbgboot_jump_table_s {
        uint8_t id[3];
        uint8_t ver;
        uint16_t ptr[];
};

/* Counter to count writes to flash.
  You can view this variable in debugger to see how many writes
  there are when stepping through the code, inserting breakpoints etc.
  Normally the AVR8_STUB_DEBUG is defined in avr8-stub.h */
#ifdef AVR8_STUB_DEBUG
uint16_t g_boot_write_cnt;
#endif



#define	JUMP_TABLE_LOCATION	(0x7fe8)
#define JUMP_TABLE_INDEX(k) (JUMP_TABLE_LOCATION + 4UL + 2UL * (k))

#define PGM_READ_BYTE pgm_read_byte_near
#define PGM_READ_WORD pgm_read_word_near
#define PGM_READ_DWORD pgm_read_dword_near

uint8_t g_app_api_version = 0;

__attribute__((optimize("-Os")))
uint8_t dboot_init_api(void) {
	struct avrdbgboot_jump_table_s jp;

	if (g_app_api_version > 0)
		return BOOT_OK;	// already initialized

	*((uint32_t *) (&jp)) = PGM_READ_DWORD(JUMP_TABLE_LOCATION);

	if ((jp.id[0] == 'A') && (jp.id[1] == 'B') && (jp.id[2] == 'j')) {
		g_app_api_version = jp.ver;
#ifdef AVR8_STUB_DEBUG
		g_boot_write_cnt = 0;
#endif
		return BOOT_OK;
	}

	return BOOT_ID_INVALID;
}

__attribute__((optimize("-Os")))
uint8_t dboot_get_api_version(uint8_t *ver) {
	uint8_t ret = dboot_init_api();

	if (ret != BOOT_OK)
		return ret;

	/* boot_init_api reads the api version into g_app_api_version */
	*ver = g_app_api_version;
	return BOOT_OK;

}

__attribute__((optimize("-Os")))
uint8_t dboot_get_version(uint16_t *ver) {
	uint8_t ret = dboot_init_api();
	uint16_t ptr;

	if (ret != 0)
		return ret;

	if (g_app_api_version == BOOT_API_VERSION) {
		ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(0));
		if (ptr == 0 || ptr == 0xffff)
			return BOOT_FUNCTION_INVALID;

		ret = ((uint8_t (*)(uint16_t *)) ptr)(ver);
		return ret;
	}

	return BOOT_VERSION_INVALID;
}

__attribute__((optimize("-Os")))
uint8_t dboot_led_init(void) {
	uint8_t ret = dboot_init_api();
		uint16_t ptr;

		if (ret != 0)
			return ret;

		if (g_app_api_version == BOOT_API_VERSION) {
			ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(1));
			if (ptr == 0 || ptr == 0xffff)
				return BOOT_FUNCTION_INVALID;

			// call the function
			((void (*)(void)) ptr)();
			return BOOT_OK;	// ok
		}

		return BOOT_VERSION_INVALID;
}

__attribute__((optimize("-Os")))
uint8_t dboot_led_toggle(void) {
	uint8_t ret = dboot_init_api();
	uint16_t ptr;

	if (ret != 0)
		return ret;

	if (g_app_api_version == BOOT_API_VERSION) {
		ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(2));
		if (ptr == 0 || ptr == 0xffff)
			return BOOT_FUNCTION_INVALID;

		// call the function
		((void (*)(void)) ptr)();
		return BOOT_OK;	// ok
		//           ret = ( (uint8_t(*)(uint32_t)) ptr )();
		//                 return ret;
	}

	return BOOT_VERSION_INVALID;
}

/* write to flash */
__attribute__((optimize("-Os")))
uint8_t dboot_safe_pgm_write(const void *ram_addr, uint16_t rom_addr, uint16_t sz) {
	uint8_t ret = dboot_init_api();
	uint16_t ptr;
	char cSREG;

	if (ret != 0)
		return ret;

	if (g_app_api_version == BOOT_API_VERSION) {
		ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(3));
		if (ptr == 0 || ptr == 0xffff)
			return BOOT_FUNCTION_INVALID;

		/* disable interrupts */
		cSREG = SREG;// store SREG value
		cli();

		/* call the function */
		((void (*)(const void *, uint16_t,uint16_t)) ptr)(ram_addr, rom_addr, sz);

		/* enable interrupts (restore) */
		SREG = cSREG;// restore SREG value (I-bit)

		/* debug info - number of write cycles */
#ifdef AVR8_STUB_DEBUG
		g_boot_write_cnt++;
#endif

		return BOOT_OK;
	}

	return BOOT_VERSION_INVALID;
}

/* Receive new program from GDB using the remote communication protocol,
   binary load packets (X). */
__attribute__((optimize("-Os")))
uint8_t dboot_handle_xload(void) {
	uint8_t ret = dboot_init_api();
	uint16_t ptr;
	char cSREG;

	if (ret != 0)
		return ret;

	if (g_app_api_version == BOOT_API_VERSION) {
		ptr = PGM_READ_WORD(JUMP_TABLE_INDEX(4));
		if (ptr == 0 || ptr == 0xffff)
			return BOOT_FUNCTION_INVALID;

		// disable interrupts
		cSREG = SREG;		// store SREG value
		cli();

		// call the function
		((void (*)(void)) ptr)();

		// enable interrupts (restore)
		SREG = cSREG;		// restore SREG value (I-bit)

		return BOOT_OK;
	}

	return BOOT_VERSION_INVALID;
}




