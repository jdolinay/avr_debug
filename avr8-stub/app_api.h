/*
 * app_api.h
 *
 * Define the API for the user applications to call the bootloader.
 * This file is used in the user application code only!
 * User bootloader should include bootapi.h file only.

 * NOTE to implementors:
 * The app_api.h and bootapi.h files should be kept in sync (the jump table structure)
 * but it is not good to have a single file as the app version will have some extra functions
 * and the functions are actually different in app than in bootloader.
 *
 *  Created on: 16. 3. 2017
 *      Author: jan dolinay
 */

#ifndef APP_API_H_
#define APP_API_H_


#ifdef __cplusplus
extern "C" {
#endif


/* Error codes */
#define	BOOT_OK					(0)	/* no error, success */
#define	BOOT_ERROR				(1)	/* unspecified error */
#define BOOT_FUNCTION_INVALID	(2)	/* The function address in jump table was not valid */
#define	BOOT_VERSION_INVALID	(3)	/* The version of the jump table in bootloader does not match the version of this code */
#define	BOOT_ID_INVALID			(4) /* The ID of the bootloader API does not match the ID expected by this code */


#define	BOOT_API_VERSION	(2)		/* Version of the API expected by this code. */
	/* See the "ver" field in jump table struct avrdbgboot_jump_table_s in app_api.c  */


/* Helper functions */
/**
 * Must be called to initialize the API and verify that it is OK before
 * using any other function of this API.
 */
uint8_t dboot_init_api(void);


/* Bootloader API functions - the functions in the bootloader */

/**
 * Read the verison of the API.
 * This is the value of the "ver" field in the jump table struct avrdbgboot_jump_table_s
 * @param ver (out) - the version of the API
 * @return BOOT_OK on success.
 * */
uint8_t dboot_get_api_version(uint8_t *ver);

/**
 * Read the version of the bootloader from bootloader memory.
 * This is the version of the bootloader, not the version of the API.
 * The bootloader verison is (AVRDBG_BOOT_VERSION_MAJOR << 8) | (AVRDBG_BOOT_VERSION_MINOR)
 * as defined in bootapi.c
 * @param ver (out) - the version of the bootloader
 * @return BOOT_OK on success.
 * */
uint8_t dboot_get_version(uint16_t *ver);


/**
 * Toggle the on-board LED
 * This is only available if AVR8_STUB_DEBUG is defined. If not defined,
 * it just returns BOOT_VERSION_INVALID.
 * @return BOOT_OK on success.
 */
uint8_t dboot_led_toggle(void);


/**
 * Initialize the pin for on-board LED (PB5)
 * This is only available if AVR8_STUB_DEBUG is defined. If not defined,
 * it just returns BOOT_VERSION_INVALID.
 * @return BOOT_OK on success.
 */
uint8_t dboot_led_init(void);

/**
 * Write to program memory from RAM buffer.
 * rom_addr - in words,
 * sz - in bytes and must be multiple of two.
 */
uint8_t dboot_safe_pgm_write(const void *ram_addr, uint16_t rom_addr, uint16_t sz);

/**
 * Handle X load command from GDB that is load new application to memory
 */
uint8_t dboot_handle_xload(void);



#ifdef __cplusplus
}
#endif


#endif /* APP_API_H_ */
