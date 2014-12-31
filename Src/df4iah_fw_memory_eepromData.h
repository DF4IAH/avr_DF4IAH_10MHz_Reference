/*
 * df4iah_fw_memory_eepromData.h
 *
 *  Created on: 31.12.2014
 *      Author: espero
 */

#ifndef DF4IAH_FW_MEMORY_EEPROMDATA_H_
#define DF4IAH_FW_MEMORY_EEPROMDATA_H_


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
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0xffff,						\
		0x0000,										/* b00_regen_ctr */																		\
		0xb00c										/* b00_crc */

#define EEPROM_DEFAULT_CONTENT_B01	\
		4.4742f,									/* b01_ref_AREF_V					4.4742 V */											\
		1.085f,										/* b01_ref_1V1_V					1.085  V */											\
		367.0f,										/* b01_temp_ofs_adc_25C_steps		0367 = 25°C */										\
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
		 9999520.0f,								/* b02_qrg_ofs_0v_25C_Hz				 9.999 520 MHz (-48 ppm) */						\
		10000240.0f,								/* b02_qrg_ofs_5v_25C_Hz				10.000 240 MHz (+24 ppm) */						\
		3.000f,										/* b02_qrg_ofs_10MHz_25C_V				3.000 V for 10 MHz */							\
		0.0f,										/* b02_qrg_ofs_0v_drift_1K_Hz */														\
		190.0f,										/* b02_qrg_k_p1v_25C_Hz					delta 190 Hz / delta 1 V   @ 10 MHz */			\
		0.0f,										/* b02_qrg_k_p1v_drift_1K */															\
		DEFAULT_PWM_COUNT,							/* b02_pwm_initial */																	\
		0xff,						\
		0xffff,						\
		0x0000,										/* b02_regen_ctr */																		\
		0xb02c										/* b02_crc */																			\

#define EEPROM_DEFAULT_CONTENT_B03	\
		0x0000,										/* b03_serial_baud */																	\
		0x0000,										/* b03_serial_bitsParityStopbits */														\
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
