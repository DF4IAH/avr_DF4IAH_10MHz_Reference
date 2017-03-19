/*
 * df4iah_bl_clkPullPwm.c
 *
 *  Created on: 24.11.2014
 *      Author: espero
 */
// tabsize: 4

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "chipdef.h"
#include "df4iah_bl_main.h"
#include "df4iah_bl_clkPullPwm.h"


#ifdef RELEASE
__attribute__((section(".df4iah_bl_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_bl_init()
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


	// set the DEBUG port to output
	PWMTOGGLEPIN_PORT |= _BV(PWMTOGGLEPIN_PNUM);
	PWMTOGGLEPIN_DDR  |= _BV(PWMTOGGLEPIN_PNUM);
}

#ifdef RELEASE
__attribute__((section(".df4iah_bl_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_bl_close()
{
	// reset the DEBUG port
	PWMTOGGLEPIN_DDR &= ~(_BV(PWMTOGGLEPIN_PNUM));


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

#ifdef RELEASE
__attribute__((section(".df4iah_bl_clkpullpwm"), aligned(2)))
#endif
inline void clkPullPwm_bl_togglePin()
{
	PWMTOGGLEPIN_PIN = _BV(PWMTOGGLEPIN_PNUM);
}

#ifdef RELEASE
__attribute__((section(".df4iah_bl_clkpullpwm"), aligned(2)))
#endif
void clkPullPwm_bl_endlessTogglePin()
{
	for(;;)	{
		clkPullPwm_bl_togglePin();
		_delay_ms(1);
		wdt_reset();
	}
}
