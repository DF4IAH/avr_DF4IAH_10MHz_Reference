/*
 * df4iah_fw_memory_eepromData.h
 *
 *  Created on: 31.12.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_MEMORY_EEPROMDATA_H_
#define DF4IAH_FW_MEMORY_EEPROMDATA_H_


#include "df4iah_bl_clkPullPwm.h"


#define DEFAULT_PARITY_N0_E2_O3_BITPOS						12
#define DEFAULT_PARITY_N0_E2_O3_MASK						0xf000
#define DEFAULT_STOPBITS_BITPOS								8
#define DEFAULT_STOPBITS_MASK								0x0300
#define DEFAULT_DATABITS_BITPOS								0
#define DEFAULT_DATABITS_MASK								0x000f


#define EEPROM_DEFAULT_CONTENT_B00	\
		'D', 'F',									/* b00_header */ 																		\
		'4', 'I',					\
		'A', 'H',					\
		' ', '1',					\
		'0', 'M',					\
		'h', 'z',					\
		'-', 'R',					\
		'e', 'f',					\
		(VERSION_HIGH<<8) | VERSION_LOW,			/* b00_version	MSB: yr*10 + month / 10,  LSB: month % 10 + day */						\
		0x0000,										/* b00_device_serial */ 																\
		0x01,										/* b00_lcdLedMode */																	\
		0xff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0x0000,										/* b00_regen_ctr */																		\
		0xb00c										/* b00_crc */

#define EEPROM_DEFAULT_CONTENT_B01	\
		4.4742f,									/* b01_ref_AREF_V					4.4742 V */											\
		1.085f,										/* b01_ref_1V1_V					1.085  V */											\
		351.0f,										/* b01_temp_ofs_adc_25C_steps		0351 = 25°C */										\
		1.0595703f,									/* b01_temp_k_p1step_adc_K			1mv / K  -->  abt.  1K / step */					\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0x0000,										/* b01_regen_ctr */																		\
		0xb01c										/* b01_crc */																			\

#define EEPROM_DEFAULT_CONTENT_B02	\
		-4.65f,										/* b02_qrg_ofs_minV_25C_ppm				-4.65ppm @000PWM_step (= 1.08V) */				\
		 1.52f,										/* b02_qrg_ofs_maxV_25C_ppm				+1.52ppm @255PWM_step (= 2.30V) */				\
		0.037f,										/* b02_qrg_k_pPwmStep_25C_ppm			(0.037ppm/PWM_Step) --> delta 0.370Hz/PWM_Step @ 10 MHz */	\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		1.08f,										/* b02_pwm_minV_V */																	\
		2.30f,										/* b02_pwm_maxV_V */																	\
		120,										/* b02_pwm_initial = DEFAULT_PWM_COUNT */												\
		0,											/* b02_pwm_initial_sub */																\
		0x0000,										/* b02_regen_ctr */																		\
		0xb02c										/* b02_crc */																			\

#define EEPROM_DEFAULT_CONTENT_B03	\
		DEFAULT_BAUDRATE,							/* b03_serial_baud */																	\
		(DEFAULT_PARITY_N0_E2_O3																											\
				<< DEFAULT_PARITY_N0_E2_O3_BITPOS)	|																						\
		(DEFAULT_STOPBITS																													\
				<< DEFAULT_STOPBITS_BITPOS)			|																						\
		DEFAULT_DATABITS																													\
				<< DEFAULT_DATABITS_BITPOS,			/* b03_serial_bitsParityStopbits */														\
		0x0000,										/* b03_gps_comm_mode */																	\
		0x0000,										/* b03_last_fix */																		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0x0000,										/* b03_regen_ctr */																		\
		0xb03c										/* b03_crc */

#define EEPROM_DEFAULT_CONTENT_B04	\
		0x0000,										/* b04_device_key */																	\
		0x0000,										/* b04_device_activations */															\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0x0000,										/* b04_regen_ctr */																		\
		0xb04c										/* b04_crc */

#define EEPROM_DEFAULT_CONTENT_B05	\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0x0000,										/* b05_regen_ctr */																		\
		0xb05c										/* b05_crc */

#define EEPROM_DEFAULT_CONTENT_B06	\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0x0000,										/* b06_regen_ctr */																		\
		0xb06c										/* b06_crc */

#define EEPROM_DEFAULT_CONTENT_B07	\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0x0000,										/* b07_regen_ctr */																		\
		0xb07c										/* b07_crc */


		/* since here the memory is not organized */

#define EEPROM_DEFAULT_CONTENT_FILL	\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,			/* this position is used for the BOOT_MARKER */																		\
		0xffff,						\
		0xffff

#endif /* DF4IAH_FW_MEMORY_EEPROMDATA_H_ */
