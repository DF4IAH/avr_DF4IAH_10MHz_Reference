/*
 * df4iah_commands.h
 *
 *  Created on: 01.11.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_MEMORY_H_
#define DF4IAH_FW_MEMORY_H_

#include <stdint.h>

#include "chipdef.h"


typedef struct eeprom_layout {
	uint16_t		reserved[8];

	uint16_t		crc_df4iah;
	uint16_t		version;
	uint16_t		pwm_pull_avg;

	uint16_t		reserved_df4iah[512 - 8 - 3];
} eeprom_layout_t;


uint8_t memory_fw_isEepromValid(void);

void memory_fw_eraseFlash(void);
void memory_fw_readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr);
void memory_fw_readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr);
void memory_fw_writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr);
void memory_fw_writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr);

#endif /* DF4IAH_FW_MEMORY_H_ */
