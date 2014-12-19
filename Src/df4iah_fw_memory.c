/*
 * df4iah_commands.c
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#include <avr/eeprom.h>

#include "df4iah_bl_memory.h"
#include "df4iah_fw_memory.h"
#include "main.h"


#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
uint8_t memory_fw_isEepromValid(void)
{
	return 1;			// TODO calculate CRC32
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void memory_fw_eraseFlash(void)
{
	memory_bl_eraseFlash();
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void memory_fw_readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr)
{
	memory_bl_readFlashPage(target, size, baddr);
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void memory_fw_readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr)
{
	uint8_t idx = 0;

	while (size) {
		target[idx++] = eeprom_read_byte((uint8_t*) baddr++);
		--size;										// decrease number of bytes to read, repeat until block has been read
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void memory_fw_writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr)
{
	memory_bl_writeFlashPage(source, size, baddr);
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_memory"), aligned(2)))
#endif
void memory_fw_writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr)
{
	uint8_t idx = 0;

	while (size--) {								// decrease number of bytes to write
		eeprom_write_byte((uint8_t*) baddr, source[idx++]);
		baddr++;									// select next byte
	}												// loop until all bytes written

	// eeprom_busy_wait();
}

// -- 8< --


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
#ifdef RELEASE
__attribute__((section(".eeprom"), aligned(2)))
#endif
eeprom_layout_t eeprom_content = { 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,	// reserved
		0xabcd,										// crc_df4iah
		((141 << 8) | (204)),						// version	MSB: yr*10 + month / 10,  LSB: month % 10 + day
		0x8000,										// pwm_pull_avg

		0xffff,										// unused
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff,
		0xffff
};
#pragma GCC diagnostic push
