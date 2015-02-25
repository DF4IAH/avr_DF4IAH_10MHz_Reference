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


/* only to silence Eclipse */
#ifndef DEFAULT_PWM_COUNT
# define DEFAULT_PWM_COUNT 									90
#endif


extern uint8_t  fastPwmLoopVal;
extern uint8_t  fastPwmSubLoopVal;

extern uint8_t  fastPwmSingleLoad;
extern uint8_t  fastPwmSingleVal;
extern uint8_t  fastPwmSubSingleVal;

extern uint8_t  fastPwmSubCmp;
extern uint8_t  fastPwmSubCnt;

extern uint8_t  pullCoef_b02_pwm_initial;
extern uint8_t  pullCoef_b02_pwm_initial_sub;


void clkPullPwm_fw_init()
{
	/* power up this module */
	//PRR &= ~(_BV(PRTIM0));								// already done in clkPullPwm_bl_init()

	clkPullPwm_bl_init();

	/* single memory access */
	if (memory_fw_isEepromBlockValid(BLOCK_REFOSC_NR)) {
		memory_fw_readEEpromPage((uint8_t*) &pullCoef_b02_pwm_initial, sizeof(uint8_t), offsetof(eeprom_layout_t, b02.b02_pwm_initial));
		memory_fw_readEEpromPage((uint8_t*) &pullCoef_b02_pwm_initial_sub, sizeof(uint8_t), offsetof(eeprom_layout_t, b02.b02_pwm_initial_sub));

		uint8_t sreg = SREG;
		cli();
		fastPwmLoopVal		= pullCoef_b02_pwm_initial;
		fastPwmSubLoopVal	= pullCoef_b02_pwm_initial_sub;
		SREG = sreg;
	}

	/* init interrupt */
	TIFR0  |= _BV(TOV0);
	TIMSK0 |= _BV(TOIE0);
}

void clkPullPwm_fw_close()
{
	/* disable interrupts */
	TIMSK0 |= 0;

	clkPullPwm_bl_close();

	/* no more power is needed for this module */
	//PRR |= _BV(PRTIM0);									// already done in clkPullPwm_bl_close()
}

inline void clkPullPwm_fw_setPin(uint8_t isSet)
{
	if (isSet) {
		PWMTOGGLEPIN_PIN |=   _BV(PWMTOGGLEPIN_PNUM);

	} else {
		PWMTOGGLEPIN_PIN &= ~(_BV(PWMTOGGLEPIN_PNUM));
	}
}


/*
 * x	Mnemonics	clocks	resulting clocks
 * ------------------------------------------------
 * 5	push		2		10
 * 1	in			1		 1
 * 1	eor			1		 1
 * 1	sei			1		 1
 *
 * = 13 clocks --> 0.65 µs until sei() is done
 */
//void clkPullPwm_fw_ISR_T0_OVF() - __vector_16
ISR(TIMER0_OVF_vect, ISR_BLOCK)
{
	sei();

	/* minimal Sub-PWM value for its FAST_PWM_SUB_BITCNT */
	const uint8_t localSubPwmInc = (1 << (8 - FAST_PWM_SUB_BITCNT));

	if (fastPwmSingleLoad) {
		cli();
		OCR0B			= fastPwmSingleVal;
		fastPwmSubCmp	= fastPwmSubSingleVal;
		sei();

		/* single value loaded */
		fastPwmSingleLoad = 0;

	} else {
		/* set the T0 compare B register with the current setting of the integer PWM value */
		cli();
		OCR0B			= fastPwmLoopVal;
		fastPwmSubCmp	= fastPwmSubLoopVal;
		sei();
	}

	/* increment if counter is lower than the sub-compare value to get a Sub-PWM (fractional part) */
	cli();
	if (fastPwmSubCnt < fastPwmSubCmp) {
		OCR0B++;
	}
	sei();

	/* sub-counter increment */
	fastPwmSubCnt += localSubPwmInc;						// overflowing is intended
}
