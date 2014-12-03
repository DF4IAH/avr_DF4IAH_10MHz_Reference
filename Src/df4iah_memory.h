/*
 * df4iah_commands.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_MEMORY_H_
#define DF4IAH_MEMORY_H_


/*
 * define the following if the bootloader should not output
 * itself at flash read (will fake an empty boot-section)
 */
#define READ_PROTECT_BOOTLOADER


#include <stdint.h>

#include "chipdef.h"


typedef struct eeprom_layout {
	uint16_t		rc_cal;
	uint16_t		reserved[7];

	uint16_t		crc_df4iah;
	uint16_t		version;
	uint16_t		pwm_pull_avg;

	uint16_t		reserved_df4iah[512 - 8 - 3];
} eeprom_layout_t;


void eraseFlash(void);

void readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr);
void readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr);
void writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr);
void writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr);


#endif /* DF4IAH_MEMORY_H_ */
