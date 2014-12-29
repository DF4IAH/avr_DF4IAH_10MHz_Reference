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

	/* BLOCK_01:    MEASURING */
	float			b01_ref_AREF_V;
	float			b01_ref_1V1_V;
	float			b01_temp_ofs_adc_25C_steps;
	float			b01_temp_k_p1step_adc_1K;
	uint16_t		b01_reserved[6];
	uint16_t		b01_regen_ctr;
	uint16_t		b01_crc;

	/* BLOCK_02:    REFERENCE OSCILLATOR */
	float			b02_qrg_ofs_0v_25C_Hz;
	float			b02_qrg_ofs_5v_25C_Hz;
	float			b02_qrg_ofs_10MHz_25C_V;
	float			b02_qrg_ofs_0v_drift_1K_Hz;
	float			b02_qrg_k_p1v_25C_Hz;
	float			b02_qrg_k_p1v_drift_1K;
	uint16_t		b02_reserved[2];
	uint16_t		b02_regen_ctr;
	uint16_t		b02_crc;

	/* BLOCK_03:    GPS */
	uint16_t		b03_serial_baud;
	uint16_t		b03_serial_bitsParityStopbits;
	uint16_t		b03_gps_comm_mode;
	uint16_t		b03_last_fix;
	uint16_t		b03_reserved[10];
	uint16_t		b03_regen_ctr;
	uint16_t		b03_crc;

	/* BLOCK_04:    KEY */
	uint16_t		b04_device_key;
	uint16_t		b04_device_activations;
	uint16_t		b04_reserved[12];
	uint16_t		b04_regen_ctr;
	uint16_t		b04_crc;

	/* unassigned BLOCKS */
	uint16_t		bxx_reserved[512 - (5 * 16)];
} eeprom_layout_t;


uint8_t memory_fw_isEepromValid(void);

void memory_fw_eraseFlash(void);
void memory_fw_readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr);
void memory_fw_readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr);
void memory_fw_writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr);
void memory_fw_writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr);

#endif /* DF4IAH_FW_MEMORY_H_ */
