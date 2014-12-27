/*
 * df4iah_clkPullPwm.c
 *
 *  Created on: 24.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */


/* this modules uses the T1 timer/counter/pwm-generator of the AVR controller as 16-bit PWM generator */


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


/* already defined in df4iah_bl_clkPullPwm.h and not needed for compilation */
#ifndef DEFAULT_PWM_COUNT
# define DEFAULT_PWM_COUNT 									0xC000
#endif


extern uint16_t pwmVal;
extern eeprom_layout_t eeprom_content;


#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_init()
{
	/* power up this module */
	PRR &= ~(_BV(PRTIM1));

	clkPullPwm_bl_init();
	pwmVal = DEFAULT_PWM_COUNT;

#if 0
	if (memory_fw_isEepromValid()) {
		uint8_t dfltPwmValue[2] = { 0 };

		memory_fw_readEEpromPage(&(dfltPwmValue[0]), sizeof(dfltPwmValue), (uint16_t) &(eeprom_content.pwm_pull_avg));
		clkPullPwm_fw_setRatio(dfltPwmValue[0] | (((uint16_t) dfltPwmValue[1]) << 8));
	}
#endif
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_close()
{
	clkPullPwm_bl_close();

	/* no more power is needed for this module */
	PRR |= _BV(PRTIM1);
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_setRatio(uint16_t ratio)
{
	clkPullPwm_bl_setRatio(ratio);
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_togglePin()
{
	clkPullPwm_bl_togglePin();
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_endlessTogglePin()
{
	clkPullPwm_bl_endlessTogglePin();
}
