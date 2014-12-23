/*
 * df4iah_commands.h
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_MEMORY_H_
#define DF4IAH_FW_MEMORY_H_

#include <stdint.h>

#include "chipdef.h"


typedef struct eeprom_layout {
	/* BLOCK_00:    HEADER-INFO */
	uint8_t			b00_header[16];
	uint16_t		b00_version;
	uint16_t		b00_device_serial;
	uint16_t		b00_reserved[4];
	uint16_t		b00_regen_ctr;
	uint16_t		b00_crc;

	/* BLOCK_01:    REFERENCE OSCILLATOR */
	uint16_t		b01_temp_25c_adc_offset;
	uint16_t		b01_temp_25c_adc_factor;
	uint16_t		b01_temp_drift_factor;
	uint16_t		b01_pwm_25c_pull;
	uint16_t		b01_pwm_pull_ppm_factor;
	uint16_t		b01_reserved[9];
	uint16_t		b01_regen_ctr;
	uint16_t		b01_crc;

	/* BLOCK_02:    GPS */
	uint16_t		b02_serial_baud;
	uint16_t		b02_serial_bitsParityStopbits;
	uint16_t		b02_gps_comm_mode;
	uint16_t		b02_last_fix;
	uint16_t		b02_reserved[10];
	uint16_t		b02_regen_ctr;
	uint16_t		b02_crc;

	/* BLOCK_03:    KEY */
	uint16_t		b03_device_key;
	uint16_t		b03_device_activations;
	uint16_t		b03_reserved[12];
	uint16_t		b03_regen_ctr;
	uint16_t		b03_crc;

	/* unassigned BLOCKS */
	uint16_t		bxx_reserved[512 - 48];
} eeprom_layout_t;


uint8_t memory_fw_isEepromValid(void);

void memory_fw_eraseFlash(void);
void memory_fw_readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr);
void memory_fw_readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr);
void memory_fw_writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr);
void memory_fw_writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr);

#endif /* DF4IAH_FW_MEMORY_H_ */
