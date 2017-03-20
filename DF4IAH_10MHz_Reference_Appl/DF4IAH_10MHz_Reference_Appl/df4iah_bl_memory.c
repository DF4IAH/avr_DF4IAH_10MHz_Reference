/*
 * df4iah_bl_memory.c
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */
// tabsize: 4

#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include "chipdef.h"
#include "df4iah_bl_usb.h"
#include "df4iah_bl_main.h"

#include "df4iah_bl_memory.h"


#define min(a,b) (a) < (b) ?  (a) : (b)


#ifdef RELEASE
__attribute__((section(".df4iah_bl_memory"), aligned(2)))
#endif
void memory_bl_eraseFlash(void)
{
	const uint32_t C_app_end = APP_END;
	uint32_t addr = 0;

	// erase only main section (bootloader protection)
	while (C_app_end >= addr) {
		boot_page_erase(addr);								// perform page erase
		boot_spm_busy_wait();								// wait until the memory is erased.
		addr += SPM_PAGESIZE;
	}
	boot_rww_enable();
}

#ifdef RELEASE
__attribute__((section(".df4iah_bl_memory"), aligned(2)))
#endif
void memory_bl_readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr)
{
	const uint32_t C_app_end = APP_END;
	uint16_t data;
	uint8_t idx = 0;

	while (size) {
#ifndef READ_PROTECT_BOOTLOADER
# warning "Bootloader not read-protected"
		if (true) {
#else
		// don't read bootloader
		if (baddr < C_app_end) {
#if defined(RAMPZ)
			data = pgm_read_word_far(baddr);
#else
			data = pgm_read_word_near(baddr);
#endif
		}
		else {
			data = 0xFFFF; 									// fake empty, no access to the bootloader
		}
#endif
		target[idx++] = data & 0xff;						// store LSB
		if (--size) {
			target[idx++] = data >> 8;						// store MSB
			baddr += 2;										// select next word in memory
			--size;											// subtract two bytes from number of bytes to read
		}
	}														// repeat until block has been read
}

#ifdef RELEASE
__attribute__((section(".df4iah_bl_memory"), aligned(2)))
#endif
void memory_bl_readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr)
{
	uint8_t idx = 0;

	while (size) {
		target[idx++] = eeprom_read_byte((uint8_t*) baddr++);
		--size;												// decrease number of bytes to read, repeat until block has been read
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_bl_memory"), aligned(2)))
#endif
void memory_bl_writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr)
{
	pagebuf_t sourceIdx = 0;

	while (size) {
		/* calculate */
		const uint32_t C_app_end = APP_END;
		const uint8_t  pageoffs  = baddr % SPM_PAGESIZE;
		const uint32_t pagestart = baddr - pageoffs;
		const uint8_t  len       = min(SPM_PAGESIZE - pageoffs, min(size, C_app_end - baddr));
		uint8_t  verifyCnt = 5;

		if (baddr >= C_app_end) {
			return;											// short-cut
		}

		/* on each new page erase it first */
		if (!pageoffs) {
			boot_page_erase(pagestart);						// perform page erase
			boot_spm_busy_wait();
		}

		/* after erasing - write up to 5 times the content of data that should '0'ing the bits */
		while (verifyCnt--) {
			const uint32_t C_word_mask = 0xfffffffe;
			uint16_t data = 0xffff;

			/* fill buffer with content */
			for (uint8_t idx = 0; idx < len; ++idx) {
				const uint16_t ptraddr = baddr + idx;
				data = (ptraddr & 1) ?  ((data & 0x00ff) | (((uint16_t) source[sourceIdx + idx]) << 8))
											:   (0xff00  |  ((uint16_t) source[sourceIdx + idx]));
				if (ptraddr & 1) {
					boot_page_fill(ptraddr & C_word_mask, data);	// call asm routine
				}
			}

			/* write the page */
			boot_page_write(pagestart);
			boot_spm_busy_wait();
			boot_rww_enable();								// re-enable the RWW section

			/* verify data, compare source data segment only */
			uint8_t isValid = 1;
			for (uint8_t idx = 0; idx < len; ++idx) {
				const uint8_t ptrdata = (pgm_read_word_near((baddr + idx) & 0xfffe) >> (((baddr + idx) & 1) ?  8 : 0));
				if (ptrdata != source[sourceIdx + idx]) {
					isValid = 0;
					break;									// test failed
				}
			}
			if (isValid) {
				++verifyCnt;
				break;										// verify OK
			}
		}
		if (!verifyCnt) {
			return;											// abort to write this page again
		}

		/* move pointers ahead */
		baddr     += len;
		size      -= len;
		sourceIdx += len;
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_bl_memory"), aligned(2)))
#endif
void memory_bl_writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr)
{
	uint8_t idx = 0;

	while (size--) {										// decrease number of bytes to write
		eeprom_write_byte((uint8_t*) baddr, source[idx++]);
		baddr++;											// select next byte
	}														// loop until all bytes written

	// eeprom_busy_wait();
}
