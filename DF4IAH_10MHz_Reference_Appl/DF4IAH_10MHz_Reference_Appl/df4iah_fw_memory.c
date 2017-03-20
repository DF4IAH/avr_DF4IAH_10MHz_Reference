/*
 * df4iah_fw_memory.c
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */
// tabsize: 4

#include <string.h>
#include <stdbool.h>
#include <avr/eeprom.h>

#include "df4iah_fw_main.h"

#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_bl_memory.h"
#include "df4iah_fw_memory.h"
#include "df4iah_fw_serial.h"

#include "df4iah_fw_memory_eepromData.h"


/* only to silence Eclipse */
#ifndef DEFAULT_PWM_COUNT
# define DEFAULT_PWM_COUNT 									90
#endif


extern eeprom_defaultValues_layout_t eeprom_defaultValues_content;
extern uchar mainFormatBuffer[MAIN_FORMAT_BUFFER_SIZE];


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
#ifdef RELEASE
__attribute__((section(".eeprom"), aligned(2)))
#endif
eeprom_layout_t eeprom_content = {
		EEPROM_DEFAULT_CONTENT_B00,
		EEPROM_DEFAULT_CONTENT_B01,
		EEPROM_DEFAULT_CONTENT_B02,
		EEPROM_DEFAULT_CONTENT_B03,
		EEPROM_DEFAULT_CONTENT_B04,
		EEPROM_DEFAULT_CONTENT_B05,
		EEPROM_DEFAULT_CONTENT_B06,
		EEPROM_DEFAULT_CONTENT_B07,

		EEPROM_DEFAULT_CONTENT_FILL
};
#pragma GCC diagnostic pop


void* memory_fw_copyBuffer(uint8_t isPgm, void* destPtr, const void* srcPtr, size_t len)
{
	if (!isPgm) {
		return memcpy(destPtr, srcPtr, len);

	} else {
		for (int idx = 0; idx < len; ++idx) {
			*((uchar*) destPtr + idx) = pgm_read_byte_near(srcPtr + idx);
		}
		return destPtr;
	}
}

uint16_t memory_fw_calcBlockCrc(uint8_t* block)
{
	uint16_t crc = CRC_SALT_VALUE;

	return crc;
}

uint8_t memory_fw_isEepromBlockValid(uint8_t blockNr)	// initializes eepromBlockCopy
{
	if (blockNr < BLOCK_COUNT) {
		/* read block */
		memory_fw_readEEpromPage((uint8_t*) &mainFormatBuffer, sizeof(eeprom_b00_t), blockNr << 5);

		uint16_t localCrcBlock = mainFormatBuffer[30] | (mainFormatBuffer[31] << 8);
		uint16_t localCrcCalc  = memory_fw_calcBlockCrc(mainFormatBuffer);

		if (localCrcBlock == localCrcCalc) {
			// block valid
			return true;

		} else {
			// block CRC verified OK
			return false;
		}
	}

	// bad usage
	return false;
}

uint16_t memory_fw_getSealMarker(uint8_t blockNr)
{
	return (0xb00c | (blockNr << 4));
}

uint8_t memory_fw_makeEepromBlockValid(uint8_t* block, uint8_t blockNr)
{
	if (blockNr < BLOCK_COUNT) {
		uint16_t oldCrcBlock = block[30] | (block[31] << 8);
		uint16_t oldCrcCalc  = memory_fw_calcBlockCrc(block);

		if (oldCrcBlock != oldCrcCalc) {
			if (oldCrcBlock == memory_fw_getSealMarker(blockNr)) {
				/* initial CRC calc marker found, seal the content */
				block[30] = (oldCrcCalc & 0xff);
				block[31] = (oldCrcCalc >> 8);

			} else {
				/* any recalculation of the CRC is counted */
				uint16_t counter = block[28] | (block[29] << 8);

				counter++;
				block[28] = (counter & 0xff);
				block[29] = (counter >> 8);

				/* re-calc the CRC */
				uint16_t newCrcCalc = memory_fw_calcBlockCrc(block);

				block[30] = (newCrcCalc & 0xff);
				block[31] = (newCrcCalc >> 8);
			}
		}
		// block valid
		return true;
	}

	// bad usage
	return false;
}

uint8_t memory_fw_writeEepromBlockMakeValid(uint8_t* source, uint8_t blockNr)
{
	if (blockNr < BLOCK_COUNT) {
		memory_fw_makeEepromBlockValid(source, blockNr);
		memory_fw_writeEEpromPage(source, 1 << 5, blockNr << 5);
		return true;
	}

	// bad usage
	return false;
}

uint8_t memory_fw_readEepromValidBlock(uint8_t* target, uint8_t blockNr)
{
	if (blockNr < BLOCK_COUNT) {
		if (memory_fw_isEepromBlockValid(blockNr)) {
			if (target != mainFormatBuffer) {
				// eepromBlockCopy is already loaded, other targets have to
				memory_fw_readEEpromPage(target, 1 << 5, blockNr << 5);
			}
			return true;
		}
	}

	// bad usage
	return false;
}

uint8_t memory_fw_checkAndInitBlock(uint8_t blockNr)
{
	if (blockNr < BLOCK_COUNT) {
		if (!memory_fw_isEepromBlockValid(blockNr)) {  // HINT: memory_fw_isEepromBlockValid() preloads eepromBlockCopy
			/* the block is non-valid, reload the EEPROM block with the default data */
			uint16_t oldCrcBlock = mainFormatBuffer[30] | (mainFormatBuffer[31] << 8);

			if (oldCrcBlock != memory_fw_getSealMarker(blockNr)) {
				/* reading default values if not CRC calc marker for sealing is found */
				memory_fw_readFlashPage(mainFormatBuffer,
						(1 << 5) - 4,								// load default data, without counter and special marked CRC value
						(((uint16_t) ((void*) &eeprom_defaultValues_content)) + (blockNr << 5)));
			}

			memory_fw_writeEepromBlockMakeValid(mainFormatBuffer, blockNr);
			return 1;

		} else {
			// no correction made
			return 0;
		}
	}

	// bad usage
	return 0;
}

uint8_t memory_fw_checkAndInitAllBlocks()
{
	uint8_t ret = 0;

	for (int blockIdx = 0; blockIdx < BLOCK_COUNT; ++blockIdx) {
		ret += memory_fw_checkAndInitBlock(blockIdx);
	}

	// count of block that needed reloading of default values
	return ret;
}

void memory_fw_eraseFlash(void)
{
	memory_bl_eraseFlash();
}

void memory_fw_readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr)
{
	memory_bl_readFlashPage(target, size, baddr);
}

void memory_fw_readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr)
{
	uint8_t idx = 0;

	while (size) {
		target[idx++] = eeprom_read_byte((uint8_t*) baddr++);
		--size;										// decrease number of bytes to read, repeat until block has been read
	}
}

void memory_fw_writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr)
{
	memory_bl_writeFlashPage(source, size, baddr);
}

void memory_fw_writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr)
{
	uint8_t idx = 0;

	while (size--) {								// decrease number of bytes to write
		eeprom_write_byte((uint8_t*) baddr, source[idx++]);
		baddr++;									// select next byte
	}												// loop until all bytes written

	// eeprom_busy_wait();
}