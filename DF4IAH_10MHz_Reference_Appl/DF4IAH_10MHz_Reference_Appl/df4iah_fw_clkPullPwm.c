/*
 * df4iah_fw_clkPullPwm.c
 *
 *  Created on: 24.11.2014
 *      Author: DF4IAH, Ulrich Habel
 */
// tabsize: 4

/* this modules uses the T0-OC0B (Pin 11) timer/counter/pwm-generator of the AVR controller as 8-bit PWM generator */


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "chipdef.h"
#include "df4iah_fw_main.h"
#include "df4iah_fw_memory.h"

#include "df4iah_fw_clkFastCtr.h"
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


void clkPullPwm_fw_init(void)
{
	// power up this module
	PRR &= ~(_BV(PRTIM0));

	// set the timer-0 counter to zero.
	TCNT0 = 0x00;

	// set the timer-0 PWM-B compare register
	OCR0B = DEFAULT_PWM_COUNT;

	// set the timer-0 mode of operation: 0x3 = Fast PWM, counting up, TOP := MAX [. . WGM01 WGM00]
	// set the timer-0 compare-B waveform generator to: PWM, 1 at >= match, 0 else
	TCCR0A = (0b10<<COM0B0) | (0b11<<WGM00);

	// set the timer-0 mode of operation: 0x3 = Fast PWM, counting up, TOP := MAX [WGM02 . .]
	// set the timer-0 clock source to 20 MHz XTAL.
	TCCR0B = (0b0<<WGM02) | (0b001<<CS00);

	// set the timer-0 PWM-B compare output: setting data port for output
	DDR_OC0B_REG |= _BV(DDR_OC0B);


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

#if 0
void clkPullPwm_fw_close(void)
{
	/* disable interrupts */
	TIMSK0 = 0;

	// reset timer-0 PWM-B compare output port
	DDR_OC0B_REG &= ~(_BV(DDR_OC0B));

	// stop timer-0
	TCCR0B = (0b00<<WGM02)  | (0b000<<CS00);

	// stop timer-0 compare output
	TCCR0A = (0b00<<COM0B0) | (0b00<<WGM00);

	// set the timer-0 compare-B value to zero.
	OCR0B = 0x00;

	// set the timer-0 counter to zero.
	TCNT0 = 0x00;

	// no more power is needed for this module
	PRR |= _BV(PRTIM0);
}
#endif

#if 0
void clkPullPwm_fw_setPin(uint8_t isSet)  // do not use when TWI is active
{
	if (isSet) {
		PWMTOGGLEPIN_PORT |=   _BV(PWMTOGGLEPIN_PNUM);

	} else {
		PWMTOGGLEPIN_PORT &= ~(_BV(PWMTOGGLEPIN_PNUM));
	}
}

void clkPullPwm_fw_setPin_ID(uint8_t id)  // do not use when TWI is active
{
	uint8_t sreg = SREG;
	cli();

	/* first: start bit */
	clkPullPwm_fw_setPin(false);

	clkPullPwm_fw_setPin(id & 0x01);
	clkPullPwm_fw_setPin(id & 0x02);
	clkPullPwm_fw_setPin(id & 0x04);
	clkPullPwm_fw_setPin(id & 0x08);
	clkPullPwm_fw_setPin(id & 0x10);
	clkPullPwm_fw_setPin(id & 0x20);
	clkPullPwm_fw_setPin(id & 0x40);
	clkPullPwm_fw_setPin(id & 0x80);

	/* last: 1 stop bit */
	clkPullPwm_fw_setPin(true);

	SREG = sreg;
}
#endif


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
