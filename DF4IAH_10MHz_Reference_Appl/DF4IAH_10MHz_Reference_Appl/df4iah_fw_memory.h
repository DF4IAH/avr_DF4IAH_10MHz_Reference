/*
 * df4iah_fw_memory.h
 *
 *  Created on: 01.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */

#ifndef DF4IAH_FW_MEMORY_H_
#define DF4IAH_FW_MEMORY_H_

#include <stdint.h>

#include "chipdef.h"


#define CRC_SALT_VALUE										0b1010010110100101		// DF4IAH's special taste


enum BLOCK_NR_t {
	BLOCK_HEADER_NR = 0,
	BLOCK_MEASURING_NR,
	BLOCK_REFOSC_NR,
	BLOCK_GPS_NR,
	BLOCK_KEYS_NR,
	BLOCK_B05_NR,
	BLOCK_B06_NR,
	BLOCK_B07_NR,

	BLOCK_COUNT
};


typedef struct eeprom_b00 {
	/* BLOCK_00:    HEADER-INFO */

	uint8_t			b00_header[16];
	uint16_t		b00_version;
	uint16_t		b00_device_serial;
	uint8_t			b00_lcdLedMode;
	uint8_t			b00_reserved2;
	uint16_t		b00_reserved[3];
	uint16_t		b00_regen_ctr;
	uint16_t		b00_crc;
} eeprom_b00_t;

typedef struct eeprom_b01 {
	/* BLOCK_01:    MEASURING */

	float			b01_ref_AREF_V;
	float			b01_ref_1V1_V;
	float			b01_temp_ofs_adc_25C_steps;
	float			b01_temp_k_p1step_adc_K;
	uint16_t		b01_reserved[6];
	uint16_t		b01_regen_ctr;
	uint16_t		b01_crc;
} eeprom_b01_t;

typedef struct eeprom_b02 {
	/* BLOCK_02:    REFERENCE OSCILLATOR (REFOSC) */

	float			b02_qrg_ofs_minV_25C_ppm;
	float			b02_qrg_ofs_maxV_25C_ppm;
	float			b02_qrg_k_pPwmStep_25C_ppm;
	uint16_t		b02_reserved[3];
	float			b02_pwm_minV_V;
	float			b02_pwm_maxV_V;
	uint8_t			b02_pwm_initial;
	uint8_t			b02_pwm_initial_sub;
	uint16_t		b02_regen_ctr;
	uint16_t		b02_crc;
} eeprom_b02_t;

typedef struct eeprom_b03 {
	/* BLOCK_03:    GPS */

	uint16_t		b03_serial_baud;
	uint16_t		b03_serial_bitsParityStopbits;
	uint16_t		b03_gps_comm_mode;
	uint16_t		b03_last_fix;
	uint16_t		b03_reserved[10];
	uint16_t		b03_regen_ctr;
	uint16_t		b03_crc;
} eeprom_b03_t;

typedef struct eeprom_b04 {
	/* BLOCK_04:    KEYS */

	uint16_t		b04_device_key;
	uint16_t		b04_device_activations;
	uint16_t		b04_reserved[12];
	uint16_t		b04_regen_ctr;
	uint16_t		b04_crc;
} eeprom_b04_t;

typedef struct eeprom_b05 {
	/* unassigned BLOCK */

	uint16_t		b05_reserved[14];
	uint16_t		b05_regen_ctr;
	uint16_t		b05_crc;
} eeprom_b05_t;

typedef struct eeprom_b06 {
	/* unassigned BLOCK */

	uint16_t		b06_reserved[14];
	uint16_t		b06_regen_ctr;
	uint16_t		b06_crc;
} eeprom_b06_t;

typedef struct eeprom_b07 {
	/* unassigned BLOCK */

	uint16_t		b07_reserved[14];
	uint16_t		b07_regen_ctr;
	uint16_t		b07_crc;
} eeprom_b07_t;

typedef struct eeprom_b08 {
	/* unassigned BLOCK */

	uint16_t		b08_reserved[14];
	uint16_t		b08_regen_ctr;
	uint16_t		b08_crc;
} eeprom_b08_t;

typedef struct eeprom_b09 {
	/* unassigned BLOCK */

	uint16_t		b09_reserved[14];
	uint16_t		b09_regen_ctr;
	uint16_t		b09_crc;
} eeprom_b09_t;

typedef struct eeprom_b10 {
	/* unassigned BLOCK */

	uint16_t		b10_reserved[14];
	uint16_t		b10_regen_ctr;
	uint16_t		b10_crc;
} eeprom_b10_t;

typedef struct eeprom_b11 {
	/* unassigned BLOCK */

	uint16_t		b11_reserved[14];
	uint16_t		b11_regen_ctr;
	uint16_t		b11_crc;
} eeprom_b11_t;

typedef struct eeprom_b12 {
	/* unassigned BLOCK */

	uint16_t		b12_reserved[14];
	uint16_t		b12_regen_ctr;
	uint16_t		b12_crc;
} eeprom_b12_t;

typedef struct eeprom_b13 {
	/* unassigned BLOCK */

	uint16_t		b13_reserved[14];
	uint16_t		b13_regen_ctr;
	uint16_t		b13_crc;
} eeprom_b13_t;

typedef struct eeprom_b14 {
	/* unassigned BLOCK */

	uint16_t		b14_reserved[14];
	uint16_t		b14_regen_ctr;
	uint16_t		b14_crc;
} eeprom_b14_t;

typedef struct eeprom_b15 {
	/* unassigned BLOCK */

	uint16_t		b15_reserved[14];
	uint16_t		b15_regen_ctr;
	uint16_t		b15_crc;
} eeprom_b15_t;


typedef struct eeprom_layout {

	/* BLOCK_00:    HEADER-INFO */
	eeprom_b00_t b00;

	/* BLOCK_01:    MEASURING */
	eeprom_b01_t b01;

	/* BLOCK_02:    REFERENCE OSCILLATOR (REFOSC) */
	eeprom_b02_t b02;

	/* BLOCK_03:    GPS */
	eeprom_b03_t b03;

	/* BLOCK_04:    KEYS */
	eeprom_b04_t b04;

	/* BLOCK_05:    unassigned */
	eeprom_b05_t b05;

	/* BLOCK_06:    unassigned */
	eeprom_b06_t b06;

	/* BLOCK_07:    unassigned */
	eeprom_b07_t b07;


	/* unassigned memory */
	uint16_t		unassigned1_reserved[512 - (BLOCK_COUNT * 16) - 3];
	uint16_t		bootMarker;						// EEPROM Address = 0x3fa..0x3fb
	uint16_t		unassigned2_reserved[2];

} eeprom_layout_t;

typedef struct eeprom_defaultValues_layout {

	/* BLOCK_00:    HEADER-INFO */
	eeprom_b00_t b00;

	/* BLOCK_01:    MEASURING */
	eeprom_b01_t b01;

	/* BLOCK_02:    REFERENCE OSCILLATOR (REFOSC) */
	eeprom_b02_t b02;

	/* BLOCK_03:    GPS */
	eeprom_b03_t b03;

	/* BLOCK_04:    KEYS */
	eeprom_b04_t b04;

} eeprom_defaultValues_layout_t;


void* memory_fw_copyBuffer(uint8_t isPgm, void* destPtr, const void* srcPtr, size_t len);

uint16_t memory_fw_getSealMarker(uint8_t blockNr);
uint16_t memory_fw_calcBlockCrc(uint8_t* block);
uint8_t  memory_fw_isEepromBlockValid(uint8_t blockNr);
uint8_t  memory_fw_readEepromValidBlock(uint8_t* target, uint8_t blockNr);
uint8_t  memory_fw_writeEepromBlockMakeValid(uint8_t* source, uint8_t blockNr);
uint8_t  memory_fw_manageBlock(uint8_t blockNr);
uint8_t  memory_fw_manageNonVolatileData();

void memory_fw_eraseFlash(void);
void memory_fw_readFlashPage(uint8_t target[], pagebuf_t size, uint32_t baddr);
void memory_fw_readEEpromPage(uint8_t target[], pagebuf_t size, uint16_t baddr);
void memory_fw_writeFlashPage(uint8_t source[], pagebuf_t size, uint32_t baddr);
void memory_fw_writeEEpromPage(uint8_t source[], pagebuf_t size, uint16_t baddr);

#endif /* DF4IAH_FW_MEMORY_H_ */
