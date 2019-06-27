/*
 * bootapi.h
 *
 * Define the API for the user applications to call the bootloader.
 * This file is used in the bootloader code only!
 * User app should include app_api.h file only.

 * NOTE to implementors:
 * The app_api.h and bootapi.h files should be kept in sync (the jump table structure)
 * but it is not good to have a single file as the app version will have some extra functions
 * and the functions are actually different in app than in bootloader.
 *
 *  Created on: 16. 3. 2017
 *      Author: jan dolinay
 */

#ifndef BOOTAPI_H_
#define BOOTAPI_H_

/* Prototypes of the bootloader API functions */
void dboot_led_toggle(void);
void dboot_led_init(void);
uint8_t dboot_get_version(uint16_t *ver);
void dboot_safe_pgm_write(const void *ram_addr, uint16_t rom_addr, uint16_t sz);
void dboot_handle_xload(void);


/* jump table struct
   KEEP this in sync with app_api.h! */
struct avrdbgboot_jump_table_s {
        uint8_t id[3];
        uint8_t ver;
        uint16_t ptr[];
};


/* Defines used internally but shared between optiboot.c and stub.c */
#if defined(__AVR_ATmega168__)
#define RAMSTART (0x100)
#define NRWWSTART (0x3800)
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#define MY_RAMSTART (0x100)
#define NRWWSTART (0x7000)
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




#endif /* BOOTAPI_H_ */
