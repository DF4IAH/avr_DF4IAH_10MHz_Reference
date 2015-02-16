/*
 * df4iah_clkPullPwm.c
 *
 *  Created on: 24.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */


/* this modules uses the T0-OC0B (Pin 11) timer/counter/pwm-generator of the AVR controller as 8-bit PWM generator */


#include <stdint.h>
#include <stddef.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "chipdef.h"
#include "df4iah_fw_main.h"
#include "df4iah_bl_memory.h"
#include "df4iah_fw_memory.h"

#include "df4iah_fw_clkFastCtr.h"
#include "df4iah_bl_clkPullPwm.h"
#include "df4iah_fw_clkPullPwm.h"

extern uint8_t  fastPwmSubCnt;
extern uint8_t  fastPwmSubCmp;
extern uint8_t  pullPwmVal;


/* only to silence Eclipse */
#ifndef DEFAULT_PWM_COUNT
# define DEFAULT_PWM_COUNT 0
#endif


extern uint8_t pullCoef_b02_pwm_initial;
extern uint8_t pullPwmVal;


#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_init()
{
	/* power up this module */
	//PRR &= ~(_BV(PRTIM0));								// already done in clkPullPwm_bl_init()

	clkPullPwm_bl_init();

	/* single memory access */
	if (memory_fw_isEepromBlockValid(BLOCK_REFOSC_NR)) {
		memory_fw_readEEpromPage((uint8_t*) &pullCoef_b02_pwm_initial, sizeof(uint8_t), offsetof(eeprom_layout_t, b02.b02_pwm_initial));
		pullPwmVal = pullCoef_b02_pwm_initial;
		clkPullPwm_fw_setRatio(pullPwmVal);
	}

	/* init interrupt */
	TIFR0  |= _BV(TOV0);
	TIMSK0 |= _BV(TOIE0);
}

#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_fw_close()
{
	/* disable interrupts */
	TIMSK0 |= 0;

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


/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 5	push		2		10
 * 2	in			1		 2
 * 1	eor			1		 1
 * 3	lds			2		 6
 * 2	out			1		 2
 * 1	cp			1		 1
 * 1	brcc		2		 2
 * 1	subi		1		 1
 * 1	sei			1		 1
 *
 * = 26 clocks --> 1.30 µs until sei() is done
 */
#ifdef RELEASE
__attribute__((section(".df4iah_fw_clkpullpwm"), aligned(2)))
#endif
//void clkPullPwm_fw_ISR_T0_OVF() - __vector_16
ISR(TIMER0_OVF_vect, ISR_BLOCK)
{
	/* minimal Sub-PWM value for its FAST_PWM_SUB_BITCNT */
	const uint8_t localSubPwmInc = (1 << (8 - FAST_PWM_SUB_BITCNT));

	/* set the T0 compare B register with the current setting of the integer PWM value */
	OCR0B = pullPwmVal;

	/* increment if counter is lower than the sub-compare value to get a Sub-PWM (fractional part) */
	if (fastPwmSubCnt < fastPwmSubCmp) {
		OCR0B++;
	}

	sei();

	/* sub-counter increment */
	fastPwmSubCnt += localSubPwmInc;						// overflowing is intended
}
