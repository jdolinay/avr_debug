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
void boot_led_toggle(void);
void boot_led_init(void);
uint8_t boot_get_version(uint16_t *ver);

// jump table struct
struct avrdbgboot_jump_table_s {
        uint8_t id[3];
        uint8_t ver;
        uint16_t ptr[];
};

#endif /* BOOTAPI_H_ */
