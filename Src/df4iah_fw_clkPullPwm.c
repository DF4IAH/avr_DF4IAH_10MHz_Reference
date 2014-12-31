/*
 * df4iah_clkPullPwm.c
 *
 *  Created on: 24.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */


/* this modules uses the T0 timer/counter/pwm-generator of the AVR controller as 8-bit PWM generator */


#include <stdint.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "chipdef.h"
#include "df4iah_fw_main.h"
#include "df4iah_bl_memory.h"
#include "df4iah_fw_memory.h"

#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_fw_clkPullPwm.h"


/* only to silence Eclipse */
#ifndef DEFAULT_PWM_COUNT
# define DEFAULT_PWM_COUNT 0
#endif


extern uint8_t pullCoef_b02_pwm_initial;
extern uint8_t pullPwmVal;
//extern uint8_t  eepromBlockCopy[sizeof(eeprom_b00_t)];


#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_init()
{
	/* power up this module */
	//PRR &= ~(_BV(PRTIM0));								// already done in clkPullPwm_bl_init()

	clkPullPwm_bl_init();

#if 1
	/* single memory access */
	if (memory_fw_isEepromBlockValid(BLOCK_REFOSC_NR)) {
		memory_fw_readEEpromPage((uint8_t*) &pullCoef_b02_pwm_initial, sizeof(uint8_t), offsetof(eeprom_layout_t, b02.b02_pwm_initial));
		pullPwmVal = pullCoef_b02_pwm_initial;
		clkPullPwm_fw_setRatio(pullPwmVal);
	}
#else
	/* block read memory access */
	if (memory_fw_readEepromValidBlock(eepromBlockCopy, BLOCK_REFOSC_NR)) {
		/* read PWM coefficient */
		eeprom_b02_t* b02 = (eeprom_b02_t*) &eepromBlockCopy;
		pullPwmVal = pullCoef_b02_pwm_initial = b02->b02_pwm_initial;
		clkPullPwm_fw_setRatio(pullPwmVal);
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
	//PRR |= _BV(PRTIM0);									// already done in clkPullPwm_bl_close()
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_setRatio(uint8_t ratio)
{
	clkPullPwm_bl_setRatio(ratio);
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
inline void clkPullPwm_fw_setPin(uint8_t isSet)
{
	if (isSet) {
		PWMTOGGLEPIN_PIN |=   _BV(PWMTOGGLEPIN_PNUM);

	} else {
		PWMTOGGLEPIN_PIN &= ~(_BV(PWMTOGGLEPIN_PNUM));
	}
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
