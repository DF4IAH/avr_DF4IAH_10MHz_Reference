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

#endif /* DF4IAH_FW_MEMORY_H_ */
