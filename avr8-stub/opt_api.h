/*
 * opt_api.h - functions for writing to flash memory using optiboot do_spm()
 * function.
 * This is modified version of the file optiboot.h provided with Optiboot
 * bootloader. For details see original comment below.
 * By June 2020 by Jan Dolinay
 */

/*------------------------ Optiboot header file ----------------------------|
 |                                                                          |
 | June 2015 by Marek Wodzinski, https://github.com/majekw                  |
 | Modified June 2016 by MCUdude, https://github.com/MCUdude                |
 | Released to public domain                                                |
 |                                                                          |
 | This header file gives possibility to use SPM instruction                |
 | from Optiboot bootloader memory.                                         |
 |                                                                          |
 | There are 5 convenient functions available here:                         |
 | * optiboot_page_erase - to erase a FLASH page                            |
 | * optiboot_page_fill - to put words into temporary buffer                |
 | * optiboot_page_write - to write contents of temporary buffer into FLASH |
 | * optiboot_readPage - higher level function to read a flash page and     |
 |                         store it in an array                             |
 | * optiboot_writePage - higher level function to write content to         |
 |                         a flash page                                     |
 |                                                                          |
 | For some hardcore users, you could use 'do_spm' as raw entry to          |
 | bootloader spm function.
 |
 |  |
 |-------------------------------------------------------------------------*/


#ifndef _OPT_API_AVR_DEBUG_H_
#define _OPT_API_AVR_DEBUG_H_ 1

#include <avr/boot.h>

/* 
 * Main 'magic' function - enter to bootloader do_spm function
 *
 * address - address to write (in bytes) but must be even number
 * command - one of __BOOT_PAGE_WRITE, __BOOT_PAGE_ERASE or __BOOT_PAGE_FILL
 * data - data to write in __BOOT_PAGE_FILL. In __BOOT_PAGE_ERASE or 
 *          __BOOT_PAGE_WRITE it control if boot_rww_enable is run
 *         (0 = run, !0 = skip running boot_rww_enable)
 *
 */

// 'typedef' (in following line) and 'const' (few lines below)
//   are a way to define external function at some arbitrary address
typedef void (*do_spm_t)(uint16_t address, uint8_t command, uint16_t data);


/*
 * Devices with more than 64KB of flash:
 * - have larger bootloader area (1KB) (they are BIGBOOT targets)
 * - have RAMPZ register :-) 
 * - need larger variable to hold address (pgmspace.h uses uint32_t)
 */
#ifdef RAMPZ
  typedef uint32_t optiboot_addr_t;
#else
  typedef uint16_t optiboot_addr_t;
#endif

#if FLASHEND > 65534
  const do_spm_t do_spm = (do_spm_t)((FLASHEND-1023+2)>>1);
#else
  const do_spm_t do_spm = (do_spm_t)((FLASHEND-511+2)>>1);
#endif


/*
 * The same as do_spm but with disable/restore interrupts state
 * required to succesfull SPM execution
 * 
 * On devices with more than 64kB flash, 16 bit address is not enough,
 * so there is also RAMPZ used in that case.
 */
void do_spm_cli(optiboot_addr_t address, uint8_t command, uint16_t data) {
  uint8_t sreg_save;

  sreg_save = SREG;  // save old SREG value
  asm volatile("cli");  // disable interrupts
#ifdef RAMPZ
  RAMPZ = (address >> 16) & 0xff;  // address bits 23-16 goes to RAMPZ
#ifdef EIND
  uint8_t eind = EIND;
  EIND = FLASHEND / 0x20000;
#endif
  // do_spm accepts only lower 16 bits of address
  do_spm((address & 0xffff), command, data);
#ifdef EIND
  EIND = eind;
#endif
#else
  // 16 bit address - no problems to pass directly
  do_spm(address, command, data);
#endif
  SREG = sreg_save; // restore last interrupts state
}


/**
 * Erase page in FLASH
 * Note that this function is placed into separate page and section
 * at the end of the program so that it is not erased when setting
 * a breakpoint into user code which would be in the same flash page.
 */
__attribute__((section(".avrdbg_flashwr")))
__attribute__ (( aligned(SPM_PAGESIZE) ))
void optiboot_page_erase(optiboot_addr_t address) {
  do_spm_cli(address, __BOOT_PAGE_ERASE, 0);
}


/** Write word into temporary buffer.
 * Note that this function is placed into separate page and section
 * at the end of the program so that it is not erased when setting
 * a breakpoint into user code which would be in the same flash page.
 */
__attribute__((section(".avrdbg_flashwr")))
__attribute__ (( aligned(SPM_PAGESIZE) ))
void optiboot_page_fill(optiboot_addr_t address, uint16_t data) {
  do_spm_cli(address, __BOOT_PAGE_FILL, data);
}


/** Write temporary buffer into FLASH.
 * Note that this function is placed into separate page and section
 * at the end of the program so that it is not erased when setting
 * a breakpoint into user code which would be in the same flash page.
 */
__attribute__((section(".avrdbg_flashwr")))
__attribute__ (( aligned(SPM_PAGESIZE) ))
void optiboot_page_write(optiboot_addr_t address) {
  do_spm_cli(address, __BOOT_PAGE_WRITE, 0);
}


#endif /* _OPT_API_AVR_DEBUG_H_ */
