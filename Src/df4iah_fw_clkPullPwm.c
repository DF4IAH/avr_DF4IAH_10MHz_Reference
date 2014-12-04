/*
 * df4iah_clkPullPwm.c
 *
 *  Created on: 24.11.2014
 *      Author: espero
 */


#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "chipdef.h"
#include "main.h"
#include "df4iah_bl_memory.h"
#include "df4iah_fw_memory.h"
#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_fw_clkPullPwm.h"

extern eeprom_layout_t eeprom_content;


#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_init()
{
	clkPullPwm_bl_init();

	if (memory_fw_isEepromValid()) {
		uint8_t dfltPwmValue[2] = { 0x00, 0x80 };

		memory_bl_readEEpromPage(&(dfltPwmValue[0]), sizeof(dfltPwmValue), (uint16_t) &(eeprom_content.pwm_pull_avg));
		clkPullPwm_bl_setRatio(dfltPwmValue[0] | (((uint16_t) dfltPwmValue[1]) << 8));
	}
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_close()
{
	clkPullPwm_bl_close();
}
